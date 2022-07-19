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

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>
#include <opencv2/core/core.hpp>

#include "System.h"
#include <node.h>

using namespace std;

class ImageGrabber {
 public:
  ImageGrabber(ORB_SLAM3::System* pSLAM) : mpSLAM(pSLAM) {}

  void GrabImage(const sensor_msgs::ImageConstPtr& msg);

  ORB_SLAM3::System* mpSLAM;
};

class VIO : public node {
 public:
  VIO(ORB_SLAM3::System::eSensor sensor,
      ros::NodeHandle& node_handle,
      image_transport::ImageTransport& image_transport,
      std::string strOutput = std::string());

  ~VIO();

  void GrabImage(const sensor_msgs::ImageConstPtr& msg);

  static cv::Mat GetImage(const sensor_msgs::ImageConstPtr& img_msg);

  void Track();

 public:
  void setmbClahe(bool mbClahe);

  void SavingTrajectory();

 private:
  queue<sensor_msgs::ImageConstPtr> imgBuf;
  std::mutex mBufMutex;

 private:
  // Variables for preprocessing images before passing to ORB_SLAM3
  bool mbResize = false;
  bool mbClahe = false;
  bool mbRectify = true;

 private:
  cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "Mono_ORB_SLAM3");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport image_transport(nh);
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Info);

//   if (argc != 3) {
//     cerr << endl
//          << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings"
//          << endl;
//     ros::shutdown();
//     return 1;
//   }
  // Some static parameters
  std::string strVocFile;
  std::string strSettingsFile;
  std::string strOutput;
  bool bEqual = false;

  // Retrieve parameters
  nh.getParam("Params", strSettingsFile);
  nh.getParam("Do_Equalize", bEqual);
  nh.getParam("Output_name", strOutput);

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
//   ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

//   ImageGrabber igb(&SLAM);
  VIO mVIO(
      ORB_SLAM3::System::eSensor::MONOCULAR, nh, image_transport, strOutput);
  mVIO.setmbClahe(bEqual);
  mVIO.Init();

  ros::Subscriber sub = nh.subscribe(
      "/camera/image_raw", 100, &VIO::GrabImage, &mVIO);

  std::thread sync_thread(&VIO::Track, &mVIO);

  ros::spin();

  mVIO.SavingTrajectory();

  // Stop all threads
//   SLAM.Shutdown();

  // Save camera trajectory
//   SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  ros::shutdown();

  return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());
}

void VIO::GrabImage(const sensor_msgs::ImageConstPtr& img_msg) {
  mBufMutex.lock();
  if (!imgBuf.empty()) imgBuf.pop();
  imgBuf.push(img_msg);
  mBufMutex.unlock();
}

cv::Mat VIO::GetImage(const sensor_msgs::ImageConstPtr& img_msg) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  if (cv_ptr->image.type() == 0) {
    return cv_ptr->image.clone();
  } else {
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}

VIO::VIO(ORB_SLAM3::System::eSensor sensor,
         ros::NodeHandle& node_handle,
         image_transport::ImageTransport& image_transport,
         std::string strOutput)
    : node(sensor, node_handle, image_transport, strOutput) {}

void VIO::setmbClahe(bool bClahe) { VIO::mbClahe = bClahe; }

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
    mORB_SLAM3->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
  } else {
    if (strOutputFile.empty()) {
      mORB_SLAM3->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
      mORB_SLAM3->SaveTrajectoryTUM("FrameTrajectory.txt");
    } else {
      mORB_SLAM3->SaveKeyFrameTrajectoryTUM("kf_" + strOutputFile + ".txt");
      mORB_SLAM3->SaveTrajectoryTUM("f_" + strOutputFile + ".txt");
    }
  }
  ROS_INFO("Saved trajectory!");
}

void VIO::Track() {
  const double maxTimeDiff = 0.01;
  while (ros::ok()) {
    cv::Mat image;
    if (!imgBuf.empty()) {
      const double tImage = imgBuf.front()->header.stamp.toSec();
      {
        this->mBufMutex.lock();
        image = GetImage(imgBuf.front());
        imgBuf.pop();
        this->mBufMutex.unlock();
      }
     
      if (mbClahe) {
        mClahe->apply(image, image);
      }
      auto Tcw = mORB_SLAM3->TrackMonocular(image, tImage);
      Update(Tcw, tImage);
      std::chrono::milliseconds tSleep(1);
      std::this_thread::sleep_for(tSleep);
    }
  }
}