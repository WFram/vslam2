//
// Created by vuong on 4/27/22.
//

#ifndef ORB_SLAM3_ROS_VIEWER_H
#define ORB_SLAM3_ROS_VIEWER_H

// std
#include <string>
#include <vector>
//ros
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//ORB_SLAM3
#include <LocalMapping.h>
#include <System.h>
#include <MapPoint.h>

//eigen
#include <Eigen/Geometry>
#include <sophus/se3.hpp>


    class viewer {
    public:
        /**
         * Constructor
         * @param nh
         * @param frame_publisher
         * @param map_publisher
         */
        viewer(const ros::NodeHandle &nh, ORB_SLAM3::LocalMapping* pLocalMapping, ORB_SLAM3::FrameDrawer* pFrameDrawer,
                ORB_SLAM3::MapDrawer* pMapDrawer, bool is_imu = false);
        
        /**
         * Main loop update visualization
         */
        void run();

        /**
         * Publish path by stack all keyframe's pose
         * @param stamp
         */
        void publish_path(ros::Time &stamp);

        /**
         * Publish all map points
         * @param stamp
         */
        void publish_map_point(ros::Time &stamp);

        /**
         * Publish local map points
         * @param stamp
         */
        void publish_local_map_point(ros::Time &stamp);

        /**
         * Publish local map points
         * @param stamp
         */
        void publish_debug_image(ros::Time &stamp);

        /**
         * Should we start to update
         * @param isStart
         */
        void setIsStart(bool isStart);

        /**
         * Update timestamp for publishing
         * @param stamp
         */
        void setStamp(const ros::Time &stamp);

        /**
         * Request to terminate the viewer
         * @note this function does not wait for terminate
         */
        void request_terminate();

        /**
         * Check if the viewer is terminated or not
         * @return whether the viewer is terminated or not
         */
        bool is_terminated();

        /**
         * Set frame id and parameters
         */
        void set_params();

        /**
          * Advertising topics
          */
        void set_advertising();

        Sophus::SE3f T_ros_cam_se3_;

        ORB_SLAM3::KeyFrame* pRefKF;

    private:
        // moment when we update
        ros::Time stamp_ = ros::Time::now();
        // should we start to update
        bool is_start_ = false;

        // ORB_SLAM3 pointers
        //! is used IMU
        bool is_imu_ = false;
        //! frame publisher
        ORB_SLAM3::FrameDrawer* mpFrameDrawer_ = nullptr;
        ORB_SLAM3::LocalMapping* mpLocalMapping_ = nullptr;  // Used
        //! map publisher
        ORB_SLAM3::MapDrawer* mpMapDrawer_ = nullptr;  // Used

        // ros pointers
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;

        //! Publisher
        // frame with features
        image_transport::Publisher debug_frame_publisher;
        std::string debug_frame_topic = "debug_image";

        // path publisher
        ros::Publisher path_publisher;
        std::string path_topic = "path";
        nav_msgs::Path path;

        // map points publisher
        ros::Publisher map_points_publisher;
        std::string map_points_topic = "map_points";
        ros::Publisher local_map_points_publisher;
        std::string local_map_points_topic = "local_map_points";

        // frame id
        std::string map_frame_id_ = "map";

        int min_observations_;
        bool only_quality_observations_;
        int min_local_points_;

    private:
        //! Transform robot coord to camera coord
        // transform from camera to ros world frame
        // clang-format off
        Eigen::Matrix4f T_ros_cam = (Eigen::Matrix4f() <<   0, 0, 1, 0,
                                                            -1, 0, 0, 0,
                                                            0,-1, 0, 0,
                                                            0, 0, 0, 1).finished();
        // clang-format on
        g2o::SE3Quat T_ros_cam_se3;
        

        //-----------------------------------------
        // management for terminate process

        //! mutex for access to terminate procedure
        mutable std::mutex mtx_terminate_;

        /**
         * Check if termination is requested or not
         * @return
         */
        bool terminate_is_requested();

        /**
         * Raise the flag which indicates the main loop has been already terminated
         */
        void terminate();

        //! flag which indicates termination is requested or not
        bool terminate_is_requested_ = false;
        //! flag which indicates whether the main loop is terminated or not
        bool is_terminated_ = true;
    };

#endif//ORB_SLAM3_ROS_VIEWER_H
