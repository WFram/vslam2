/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <chrono>
#include <iostream>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <queue>
#include <thread>
#include <vector>

#include "ImuTypes.h"
#include "System.h"
#include <node.h>

using namespace std;

class ImuGrabber {
public:
    ImuGrabber() {};

    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue <sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class VIO : public node {
public:
    VIO(ORB_SLAM3::System::eSensor sensor,
        ros::NodeHandle &node_handle,
        image_transport::ImageTransport &image_transport,
        std::string strOutput = std::string());

    ~VIO();

    void GrabImage(const sensor_msgs::ImageConstPtr &msg);

    static cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);

    void SyncWithImu();

    cv::Mat M1l, M2l;

    double td;

    ImuGrabber *mpImuGb;

public:
    void setmbClahe(bool mbClahe);

    void setmbRectify(bool mbRectify);

    void SavingTrajectory();

    size_t get_num_image() { return num_img; }

private:
    queue <sensor_msgs::ImageConstPtr> imgBuf;
    std::mutex mBufMutex;

private:
    // Variables for preprocessing images before passing to ORB_SLAM3
    bool mbResize = false;
    bool mbClahe = false;
    bool mbRectify = false;
    size_t num_img = 0;
private:
    cv::Ptr <cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ORB_SLAM3");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport image_transport(nh);
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                   ros::console::levels::Info);

    // Some static parameters
    std::string strVocFile;
    std::string strSettingsFile;
    std::string strOutput;
    bool bEqual = false;
    bool bRect = true;
    double timeshift = 0.0;

    // Retrieve parameters
    nh.getParam("Params", strSettingsFile);
    nh.getParam("Do_Rectify", bRect);
    nh.getParam("Do_Equalize", bEqual);
    nh.getParam("Output_name", strOutput);
    nh.getParam("Timeshift", timeshift);

    auto *imugb = new ImuGrabber();

    VIO mVIO(
            ORB_SLAM3::System::eSensor::IMU_MONOCULAR, nh, image_transport, strOutput);

    mVIO.td = timeshift;

    mVIO.mpImuGb = imugb;

    // Prepare for Rectify
    if (bRect) {
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if (!fsSettings.isOpened()) {
            ROS_FATAL("ERROR: Wrong path to settings");
            ros::shutdown();
            return -1;
        }

        cv::Mat K_l, P_l, R_l, D_l;
        fsSettings["LEFT.K"] >> K_l;

        fsSettings["LEFT.P"] >> P_l;

        fsSettings["LEFT.R"] >> R_l;

        fsSettings["LEFT.D"] >> D_l;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];

        if (K_l.empty() || P_l.empty() ||
            R_l.empty() || D_l.empty() ||
            rows_l == 0 || cols_l == 0) {
            ROS_FATAL("ERROR: Calibration parameters to rectify stereo are missing!");
            ros::shutdown();
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,
                                    D_l,
                                    R_l,
                                    P_l.rowRange(0, 3).colRange(0, 3),
                                    cv::Size(cols_l, rows_l),
                                    CV_32F,
                                    mVIO.M1l,
                                    mVIO.M2l);
    }

    mVIO.setmbRectify(bRect);
    mVIO.setmbClahe(bEqual);

    mVIO.Init();

    // Maximum delay, 5 seconds
    ros::Subscriber sub_imu =
            nh.subscribe("/imu", 1000, &ImuGrabber::GrabImu, imugb);
    ros::Subscriber sub_img =
            nh.subscribe("/camera/image_raw", 100, &VIO::GrabImage, &mVIO);

    std::thread sync_thread(&VIO::SyncWithImu, &mVIO);
    std::thread thread([&]() {
        mVIO.ros_viewer_->run();
    });
    ros::spin();
    sync_thread.join();
    thread.join();
    // mVIO.SavingTrajectory();
    std::cout << mVIO.get_num_image();
    return 0;
}

void VIO::GrabImage(const sensor_msgs::ImageConstPtr &img_msg) {
    if (left_cam_frame_id_.empty())
        left_cam_frame_id_ = img_msg->header.frame_id;
    mBufMutex.lock();
    if (!imgBuf.empty()) imgBuf.pop();
    imgBuf.push(img_msg);
    mBufMutex.unlock();
}

cv::Mat VIO::GetImage(const sensor_msgs::ImageConstPtr &img_msg) {
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0) {
        return cv_ptr->image.clone();
    } else {
        std::cout << "Error type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void VIO::SyncWithImu() {
    while (ros::ok()) {
        cv::Mat image;
        if (!imgBuf.empty() && !mpImuGb->imuBuf.empty()) {
            const double tIm = imgBuf.front()->header.stamp.toSec() + td;
            if (tIm == 0) std::cout << "New image timestamp: " << tIm << std::endl;

            // wait until we have IMUs to process
            if (mpImuGb->imuBuf.empty()) continue;
            // wait until we've received all IMUs preceding this image
            if (tIm > mpImuGb->imuBuf.back()->header.stamp.toSec()) continue;

            {
                this->mBufMutex.lock();
                image = GetImage(imgBuf.front());
                imgBuf.pop();
                this->mBufMutex.unlock();
            }

            vector <ORB_SLAM3::IMU::Point> vImuMeas;
            vImuMeas.reserve(mpImuGb->imuBuf.size());
            mpImuGb->mBufMutex.lock();
            if (!mpImuGb->imuBuf.empty()) {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while (!mpImuGb->imuBuf.empty() &&
                       mpImuGb->imuBuf.front()->header.stamp.toSec() <= tIm) {
                    double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
                    cv::Point3f acc((float) mpImuGb->imuBuf.front()->linear_acceleration.x,
                                    (float) mpImuGb->imuBuf.front()->linear_acceleration.y,
                                    (float) mpImuGb->imuBuf.front()->linear_acceleration.z);
                    cv::Point3f gyr((float) mpImuGb->imuBuf.front()->angular_velocity.x,
                                    (float) mpImuGb->imuBuf.front()->angular_velocity.y,
                                    (float) mpImuGb->imuBuf.front()->angular_velocity.z);
                    // vImuMeas.emplace_back(acc, gyr, t);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    mpImuGb->imuBuf.pop();
                }
            }
            mpImuGb->mBufMutex.unlock();

            if (vImuMeas.empty() || vImuMeas.size() == 1) {
                ROS_WARN("IMU vector is empty or only 1 measurement");
                continue;
            }
            if (mbClahe) {
                mClahe->apply(image, image);
            }
            if (mbRectify) {
                cv::remap(image, image, M1l, M2l, cv::INTER_LINEAR);
            }
            auto Tcw = mORB_SLAM3->TrackMonocular(image, tIm,
                                                  vImuMeas); // From here we can get a KeyFrame and project all the points on it
            ORB_SLAM3::KeyFrame *pRefKF = mORB_SLAM3->GetTracker()->mCurrentFrame.mpReferenceKF;
            Update(Tcw, tIm);
            ros_viewer_->pRefKF = pRefKF;
            num_img++;
            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}

VIO::VIO(ORB_SLAM3::System::eSensor sensor,
         ros::NodeHandle &node_handle,
         image_transport::ImageTransport &image_transport,
         std::string strOutput)
        : node(sensor, node_handle, image_transport, strOutput) {}

void VIO::setmbClahe(bool bClahe) { VIO::mbClahe = bClahe; }
void VIO::setmbRectify(bool bRectify) { VIO::mbRectify = bRectify; }

VIO::~VIO() {
    // Release memory
    delete mORB_SLAM3;
    delete mpLocalMapping;
    delete mpMapDrawer;
    delete mpAtlas;
}

void VIO::SavingTrajectory() {
    // Stop all threads
    mORB_SLAM3->Shutdown();
    ROS_INFO("Saving trajectory...");
    // Save trajectory
    if (mSensor == ORB_SLAM3::System::MONOCULAR ||
        mSensor == ORB_SLAM3::System::IMU_MONOCULAR) {
        mORB_SLAM3->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt"); // then this file should be save also
    } else {
        if (strOutputFile.empty()) {
            mORB_SLAM3->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
            mORB_SLAM3->SaveTrajectoryTUM("FrameTrajectory.txt");
        } else {
            mORB_SLAM3->SaveKeyFrameTrajectoryTUM("kf_" + strOutputFile + ".txt"); // this
            mORB_SLAM3->SaveTrajectoryTUM("f_" + strOutputFile + ".txt"); //this ahh oke that
        }
    }
    ROS_INFO("Saved trajectory!"); // i check with stdout
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg) {
    mBufMutex.lock();
    imuBuf.push(imu_msg);
    mBufMutex.unlock();
    return;
}
