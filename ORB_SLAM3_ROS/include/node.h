//
// Created by vuong on 16/12/2021.
//

#ifndef VSLAM_NODE_H
#define VSLAM_NODE_H

// From ORB_SLAM3
#include <LocalMapping.h>
#include <System.h>
// From ROS
//  Core
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <rosbag/bag.h>
//  tf2
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
//  Msg Types
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// From Eigen & Sophus
#include <eigen3/Eigen/Dense>
#include <sophus/se3.hpp>

// From OpenCV
#include <opencv2/core/eigen.hpp>

#include "utils/Utils.h"
#include "viewer.h"

// From TBB
#include <tbb/parallel_for.h>

class node {
 public:
  node(ORB_SLAM3::System::eSensor sensor,
       ros::NodeHandle& node_handle,
       image_transport::ImageTransport& image_transport,
       std::string strOutput = std::string());
  ~node();

  // Advertise publish topics
  void Init();

  // Saved trajectory and map as bag file
  bool SaveTrajectoryBag(const std::string& file_path);
  
  viewer* ros_viewer_ = nullptr;

 protected:
  // Update state and publish data
  void Update(Sophus::SE3f Tcw, double timestamp = 0);
  // Pointer to ORB_SLAM3 threads
  // System
  ORB_SLAM3::System* mORB_SLAM3;
  // Tracking
  ORB_SLAM3::Tracking* mpTracking{};
  // Local Mapping
  ORB_SLAM3::LocalMapping* mpLocalMapping;  // Used
  // Loop Closing
  //  ORB_SLAM3::LoopClosing* mpLoopClosing{};
  // Map Drawer
  ORB_SLAM3::MapDrawer* mpMapDrawer{};  // Used
  // Atlas
  ORB_SLAM3::Atlas* mpAtlas{};  // Used

  // ORB_SLAM3 variables
  ORB_SLAM3::System::eSensor mSensor;
  // Path variables
  std::string strOutputFile;

  std::string left_cam_frame_id_;
 private:
  // Functions for Publish
  void PublishPoseAsTransform(const Sophus::SE3f& Twc, double timestamp);
  void PublishPoseAsOdometry(const Sophus::SE3d& Twc, double timestamp);
  void PublishMapPointsAsPCL2(std::vector<ORB_SLAM3::MapPoint*> vpMapPoints,
                              double timestamp);
  void PublishKF(ORB_SLAM3::KeyFrame* pKF);
  void ParseCamInfo(sensor_msgs::CameraInfo& msg) const;
  void GetCamInfo(cv::FileStorage& fSettings);
  

  ros::NodeHandle nh_;
  
  double transform_tolerance_;

  // Node's name
  std::string mNodeName;

  // Are we using IMU?
  bool mbIMU;
  bool should_start_publish_ = false;

  // Frame IDs for Odometry Publish
  std::string world_frame_id_;
  
  std::string point_cloud_frame_id_;
  std::string target_frame_id_;
  std::string camera_link_id_;

  // Radius for drawing
  const float r = 5;

  std::string strVocFile;
  std::string strSettingsFile;
  bool mbViewer;

  // Publish variables
  // Map
  image_transport::ImageTransport image_transport_;
  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  image_transport::Publisher mDebugImagePub;
  ros::Publisher mPosePub;
  ros::Publisher mMapPointsPub;

  // KF for Depth Estimation
  ros::Publisher mKFPosePub;
  image_transport::Publisher mKFDebugImagePub;
  ros::Publisher mMPsObsbyKFPub;
  ros::Publisher mKFsFeaturesPub;
  ros::Publisher mKFsCamInfoPub;
  //
  ros::Publisher ready_pub_;

  //  Saving a copy of camera parameters in case that we need it for depth est.
  int camWidth, camHeight;
  double fx, fy, cx, cy;
  double k1, k2, t1, t2;
  rosbag::Bag bag;

  Eigen::Matrix4f T_ROS_ORB,T_ORB_ROS;
  Sophus::SE3f spT_ROS_ORB,spT_ORB_ROS;

  // Robot state
  double mTimestamp;
  Sophus::SE3f spTwc;
  // Drawer
  cv::Scalar standardColor = cv::Scalar(0, 255, 0);

  Eigen::AngleAxisd rot_ros_to_cv_map_frame_;
};

#endif  // VSLAM_NODE_H
