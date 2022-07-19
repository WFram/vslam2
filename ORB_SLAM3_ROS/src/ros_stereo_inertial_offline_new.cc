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

// FROM ROS
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

// FROM System
#include <chrono>
#include <iostream>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <queue>
#include <vector>

// FROM ORB_SLAM3
#include <System.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <node.h>

using namespace std;

void LoadImages(const string& strPathFolder,
                vector<string>& vstrImageLeft,
                vector<string>& vstrImageRight,
                vector<double>& vTimeStamps);
void LoadIMU(const string& strImuPath,
             vector<double>& vTimeStamps,
             vector<cv::Point3f>& vAcc,
             vector<cv::Point3f>& vGyro);
class VIO : public node {
 public:
  VIO(ORB_SLAM3::System::eSensor sensor,
      ros::NodeHandle& node_handle,
      image_transport::ImageTransport& image_transport,
      std::string strOutput = std::string());
  ~VIO();
  void GrabStereo(cv::Mat& imLeft, cv::Mat& imRight,vector<ORB_SLAM3::IMU::Point>& vImuMeas, double tframe);

  cv::Mat M1l, M2l, M1r, M2r;

 public:
  void setmbClahe(bool mbClahe);
  void setmbRectify(bool mbRectify);
  void setmbResize(bool mbResize);
  void setmnWidth(int mnWidth);
  void setmnHeight(int mnHeight);
  void SavingTrajectory();

 private:
  queue<sensor_msgs::ImageConstPtr> imgLeftBuf, imgRightBuf;
  std::mutex mBufMutexLeft, mBufMutexRight;

 private:
  // Variables for preprocessing images before passing to ORB_SLAM3
  bool mbResize = false;
  bool mbClahe = false;
  bool mbRectify = true;
  int mnWidth, mnHeight;

 private:
  cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ORB_SLAM3");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport image_transport(nh);
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Info);

  // Some static parameters
  std::string strDataType;
  std::string strDataPath;
  std::string strVocFile;
  std::string strSettingsFile;
  std::string strOutput;
  bool bEqual = false;
  bool bRect = true;
  bool bResize = false;

  // Retrieve parameters
  nh.getParam("data_type", strDataType);
  nh.getParam("data_path", strDataPath);
  nh.getParam("Params", strSettingsFile);
  nh.getParam("Do_Rectify", bRect);
  nh.getParam("Do_Equalize", bEqual);
  nh.getParam("Do_Resize", bResize);
  nh.getParam("Output_name", strOutput);

  VIO mVIO(ORB_SLAM3::System::eSensor::STEREO, nh, image_transport, strOutput);

  // Load all sequences:
  int seq;
  int num_seq = 1;
  vector<vector<string> > vstrImageLeft;
  vector<vector<string> > vstrImageRight;
  vector<vector<double> > vTimestampsCam;

  vector<vector<cv::Point3f> > vAcc, vGyro;
  vector<vector<double> > vTimestampsImu;
  vector<int> nImages;
  vector<int> nImu;
  vector<int> first_imu(num_seq, 0);

  vstrImageLeft.resize(num_seq);
  vstrImageRight.resize(num_seq);
  vTimestampsCam.resize(num_seq);
  vAcc.resize(num_seq);
  vGyro.resize(num_seq);
  vTimestampsImu.resize(num_seq);
  nImages.resize(num_seq);
  nImu.resize(num_seq);

  int tot_images = 0;
  for (seq = 0; seq < num_seq; seq++) {
    cout << "Loading images for sequence " << seq << "...";

    string pathImu = strDataPath + "/zed2_imu.txt";

    LoadImages(strDataPath,
               vstrImageLeft[seq],
               vstrImageRight[seq],
               vTimestampsCam[seq]);
    cout << "LOADED!" << endl;

    cout << "Loading IMU for sequence " << seq << "...";
    LoadIMU(pathImu, vTimestampsImu[seq], vAcc[seq], vGyro[seq]);
    cout << "LOADED!" << endl;

    nImages[seq] = vstrImageLeft[seq].size();
    tot_images += nImages[seq];

    if ((nImages[seq] <= 0) || (nImages[seq] != vstrImageRight[seq].size())) {
      cerr << "ERROR: Failed to load images or difference between left and "
              "right camera"
           << seq << endl;
      return EXIT_FAILURE;
    }

    // Find first imu to be considered, supposing imu measurements start first
    while (vTimestampsImu[seq][first_imu[seq]] <= vTimestampsCam[seq][0])
      first_imu[seq]++;
    first_imu[seq]--;  // first imu measurement to be considered
  }

  // Read rectification parameters
  cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    cerr << "ERROR: Wrong path to settings" << endl;
    return -1;
  }

  mVIO.Init();

  cv::Mat imLeft, imRight;
  for (seq = 0; seq < num_seq; seq++) {
    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    // Seq loop
    int proccIm = 0;
    for (int ni = 0; ni < nImages[seq]; ni++, proccIm++) {
      // Read left and right images from file
      imLeft = cv::imread(vstrImageLeft[seq][ni], cv::IMREAD_UNCHANGED);
      imRight = cv::imread(vstrImageRight[seq][ni], cv::IMREAD_UNCHANGED);

      if (imLeft.empty()) {
        cerr << endl
             << "Failed to load image at: " << string(vstrImageLeft[seq][ni])
             << endl;
        return 1;
      }
      double tframe = vTimestampsCam[seq][ni];
      if (imRight.empty()) {
        cerr << endl
             << "Failed to load image at: " << string(vstrImageRight[seq][ni])
             << endl;
        return 1;
      }

      // Load imu measurements from previous frame
      vImuMeas.clear();

      if (ni > 0)
        while (
            vTimestampsImu[seq][first_imu[seq]] <=vTimestampsCam[seq][ni]) {
              vImuMeas.push_back(
                ORB_SLAM3::IMU::Point(vAcc[seq][first_imu[seq]].x,
                                    vAcc[seq][first_imu[seq]].y,
                                    vAcc[seq][first_imu[seq]].z,
                                    vGyro[seq][first_imu[seq]].x,
                                    vGyro[seq][first_imu[seq]].y,
                                    vGyro[seq][first_imu[seq]].z,
                                    vTimestampsImu[seq][first_imu[seq]]));
            first_imu[seq]++;
        }


      mVIO.GrabStereo(imLeft,imRight,vImuMeas,tframe);
    }
  }
  mVIO.SavingTrajectory();
  nh.shutdown();
  return 0;
}

void VIO::GrabStereo(cv::Mat& imLeft, cv::Mat& imRight, vector<ORB_SLAM3::IMU::Point>& vImuMeas, double tframe) {
    mORB_SLAM3->TrackStereo(imLeft, imRight, tframe, vImuMeas);
}
VIO::VIO(ORB_SLAM3::System::eSensor sensor,
         ros::NodeHandle& node_handle,
         image_transport::ImageTransport& image_transport,
         std::string strOutput)
    : node(sensor, node_handle, image_transport, strOutput) {}

void VIO::setmbClahe(bool bClahe) { VIO::mbClahe = bClahe; }
void VIO::setmbRectify(bool bRectify) { VIO::mbRectify = bRectify; }
VIO::~VIO() {
  // Release memory
  delete mORB_SLAM3;
  delete mpTracking;
  delete mpLocalMapping;
  //  delete mpLoopClosing;
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
      SaveTrajectoryBag("/home/ami/test.bag");
      mORB_SLAM3->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
      mORB_SLAM3->SaveTrajectoryTUM("FrameTrajectory.txt");
    } else {
      SaveTrajectoryBag("/home/ami/test.bag");
      mORB_SLAM3->SaveKeyFrameTrajectoryTUM("kf_" + strOutputFile + ".txt");
      mORB_SLAM3->SaveTrajectoryTUM("f_" + strOutputFile + ".txt");
    }
  }
  ROS_INFO("Saved trajectory!");
}
void VIO::setmbResize(bool bResize) { VIO::mbResize = bResize; }
void VIO::setmnWidth(int nWidth) { VIO::mnWidth = nWidth; }
void VIO::setmnHeight(int nHeight) { VIO::mnHeight = nHeight; }

void LoadImages(const string& strPathFolder,
                vector<string>& vstrImageLeft,
                vector<string>& vstrImageRight,
                vector<double>& vTimeStamps) {
  ifstream fTimes;
  string strPathTimesLeft = strPathFolder + "/zed2_left.txt";
  string strPathTimesRight = strPathFolder + "/zed2_right.txt";
  fTimes.open(strPathTimesLeft.c_str());
  fTimes.good() ? std::cout << "Left timestamp path exist\n"
                : std::cerr << "Left timestamp path doesn't exist\n";
  vTimeStamps.reserve(5000);
  vstrImageLeft.reserve(5000);
  vstrImageRight.reserve(5000);
  while (!fTimes.eof()) {
    string s;
    getline(fTimes, s);
    if (s[0] == '#') continue;
    if (!s.empty()) {
      stringstream ss;
      ss << s;
      double t;
      ss >> t;
      vTimeStamps.push_back(t);
      string strLeft;
      ss >> strLeft;
      vstrImageLeft.push_back(strPathFolder + "/" + strLeft);
    }
  }
  fTimes.close();
  fTimes.open(strPathTimesRight.c_str());
  fTimes.good() ? std::cout << "Right timestamp path exist\n"
                : std::cerr << "Right timestamp path doesn't exist\n";
  while (!fTimes.eof()) {
    string s;
    getline(fTimes, s);
    if (s[0] == '#') continue;
    if (!s.empty()) {
      stringstream ss;
      ss << s;
      double t;
      ss >> t;
      vTimeStamps.push_back(t);
      string strRight;
      ss >> strRight;
      vstrImageRight.push_back(strPathFolder + "/" + strRight);
    }
  }
}
void LoadIMU(const string& strImuPath,
             vector<double>& vTimeStamps,
             vector<cv::Point3f>& vAcc,
             vector<cv::Point3f>& vGyro) {
  ifstream fImu;
  fImu.open(strImuPath.c_str());
  vTimeStamps.reserve(5000);
  vAcc.reserve(5000);
  vGyro.reserve(5000);
  fImu.good() ? std::cout << "IMU path exist\n"
              : std::cerr << "IMU path doesn't exist\n";
  while (!fImu.eof()) {
    string s;
    getline(fImu, s);
    if (s[0] == '#') continue;

    if (!s.empty()) {
      string item;
      size_t pos = 0;
      double data[7];
      int count = 0;
      while ((pos = s.find(' ')) != string::npos) {
        item = s.substr(0, pos);
        data[count++] = stod(item);
        s.erase(0, pos + 1);
      }
      item = s.substr(0, pos);
      data[6] = stod(item);

      vTimeStamps.push_back(data[0]);
      vAcc.push_back(cv::Point3f(data[4], data[5], data[6]));
      vGyro.push_back(cv::Point3f(data[1], data[2], data[3]));
    }
  }
}
