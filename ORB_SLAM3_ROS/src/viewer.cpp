#include "viewer.h"

viewer::viewer(const ros::NodeHandle &nh, ORB_SLAM3::LocalMapping *pLocalMapping, ORB_SLAM3::FrameDrawer *pFrameDrawer,
               ORB_SLAM3::MapDrawer *pMapDrawer, bool is_imu) :
        nh_(nh), it_(nh), mpLocalMapping_(pLocalMapping), mpFrameDrawer_(pFrameDrawer), mpMapDrawer_(pMapDrawer),
        is_imu_(is_imu), pRefKF(nullptr) {

    // set parameters and variable
    set_params();

    // broadcasting
    set_advertising();

    // publish reference map
    if (use_reference_cloud_)
        publish_reference_cloud();
}

void viewer::set_params() {
    // Retrieve frame id parameters
    ros::param::get("map_frame_id", map_frame_id_);
    std::cout << "map_frame_id: " << map_frame_id_ << std::endl;
    ros::param::get("left_camera_frame_id", left_camera_frame_id_);
    std::cout << "left_camera_frame_id: " << left_camera_frame_id_ << std::endl;

    ros::param::get("use_reference_cloud", use_reference_cloud_);
    std::cout << "use_reference_cloud: " << use_reference_cloud_ << std::endl;
    ros::param::get("reference_cloud_path", reference_cloud_path_);
    std::cout << "reference_cloud_path: " << reference_cloud_path_ << std::endl;

    pcl::io::loadPCDFile(reference_cloud_path_, reference_cloud);

    T_ros_cam.block<3, 3>(0, 0) = Eigen::Quaternionf(T_ros_cam.block<3, 3>(0, 0))
            .normalized()
            .toRotationMatrix();
    T_ros_cam_se3_ = Sophus::SE3f(T_ros_cam);
}


void viewer::set_advertising() {

    debug_frame_publisher = it_.advertise(debug_frame_topic, 1);

    path_publisher = nh_.advertise<nav_msgs::Path>(path_topic, 1);

    map_points_publisher =
            nh_.advertise<sensor_msgs::PointCloud2>(map_points_topic, 1);

    local_map_points_publisher =
            nh_.advertise<sensor_msgs::PointCloud2>(local_map_points_topic, 1);

    reference_map_points_publisher =
            nh_.advertise<sensor_msgs::PointCloud2>(reference_map_points_topic, 1, true);

    global_map_saver = nh_.advertiseService<std_srvs::Empty::Request,
            std_srvs::Empty::Response>("/vslam2/save_global_map", srv_cbk);
}

void viewer::publish_reference_cloud() {
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(reference_cloud, cloud_msg);
    cloud_msg.header.frame_id = map_frame_id_;
    cloud_msg.header.stamp = ros::Time::now();

    reference_map_points_publisher.publish(cloud_msg);
    ROS_INFO("Publish reference cloud as latched");
}

void viewer::run() {
    is_terminated_ = false;
    ros::Rate rate(30);
    while (ros::ok()) {
        if (is_start_) {
            publish_local_map_point(stamp_);
//            publish_map_point(stamp_);
            publish_debug_image(stamp_);
            setIsStart(false);
        }
        // check termination flag
        if (terminate_is_requested()) {
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }
    terminate();
}

void viewer::publish_debug_image(ros::Time &stamp) {
    cv::Mat im = mpFrameDrawer_->DrawFrame(1);
    if (im.empty()) {
        return;
    }
    sensor_msgs::ImagePtr img_msg;
    sensor_msgs::Image std_img_msg;
    std_img_msg.header.stamp = stamp;
    img_msg = cv_bridge::CvImage(std_img_msg.header, "bgr8", im).toImageMsg();
    debug_frame_publisher.publish(img_msg);
}

void viewer::publish_path(ros::Time &stamp) {
    ORB_SLAM3::Map *pActiveMap = mpMapDrawer_->mpAtlas->GetCurrentMap();
    if (!pActiveMap) return;

    const vector<ORB_SLAM3::KeyFrame *> vpKFs = pActiveMap->GetAllKeyFrames();
    path.header.stamp = stamp;
    path.header.frame_id = map_frame_id_;
    path.poses.clear();
    for (const auto &keyfrm: vpKFs) {
        if (!keyfrm) {
            continue;
        }

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = map_frame_id_;
        const auto Twc = T_ros_cam_se3_ * keyfrm->GetPoseInverse();
        pose.pose.orientation.x = Twc.unit_quaternion().x();
        pose.pose.orientation.y = Twc.unit_quaternion().y();
        pose.pose.orientation.z = Twc.unit_quaternion().z();
        pose.pose.orientation.w = Twc.unit_quaternion().w();

        pose.pose.position.x = Twc.translation().x();
        pose.pose.position.y = Twc.translation().y();
        pose.pose.position.z = Twc.translation().z();

        path.poses.push_back(pose);
    }
    path_publisher.publish(path);
}

void viewer::setStamp(const ros::Time &stamp) {
    stamp_ = stamp;
}

void viewer::setIsStart(bool isStart) {
    is_start_ = isStart;
}

void viewer::request_terminate() {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    terminate_is_requested_ = true;
}

bool viewer::is_terminated() {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    return is_terminated_;
}

bool viewer::terminate_is_requested() {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    return terminate_is_requested_;
}

void viewer::terminate() {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    is_terminated_ = true;
    is_start_ = false;
}

void viewer::publish_local_map_point(ros::Time &stamp) {
    ORB_SLAM3::Map *pActiveMap = mpMapDrawer_->mpAtlas->GetCurrentMap();
    if (!pActiveMap) return;

    if (pRefKF == nullptr) {
        ROS_WARN("Reference KF is null");
        return;
    }

    // From one side, we have too few points. From another one, we need only stable ones.
    // How to know if a point position is stable?
    // 1) From the result of BA? How many times the point position has been optimized? Counter?
    //      in the Local Mapping LocalBA? LocalInertialBA? It doesn't seem to change the meaning a lot
    // 2) Scale checking? Knowing that the point is the same, is its coordinates change a lot?
    // 3) If initial positions for the points are bad, can we make the conditions for triangulation more strict?

    const std::set<ORB_SLAM3::MapPoint *> &spLocalMPs = pRefKF->GetMapPoints();

    if (spLocalMPs.empty()) return;

    // TODO (3D): Now he publishes the same points. We need to have a look at the vector and keyframe graph.
    // Maybe new KFs are not showing up. Or just there should be another way to get the most recent KF

    // Declare and initialize red, green, blue component values
    // Dark color for inactive map points
    PointCloudXYZ::Ptr p_local_cloud(new PointCloudXYZ);

    for (auto spLocalMP: spLocalMPs) {
        if (spLocalMP->isBad()) continue;

        Eigen::Vector3f pos_w, pos_cam;
        if (is_imu_) {
            pos_w = spLocalMP->GetWorldPos();
            pos_cam = T_ros_cam_se3_ * mTcw * spLocalMP->GetWorldPos();
        } else
            pos_w = T_ros_cam_se3_ * spLocalMP->GetWorldPos();

        pcl::PointXYZ point;
        point.x = pos_cam(0);
        point.y = pos_cam(1);
        point.z = pos_cam(2);
        p_local_cloud->push_back(point);
    }

    if (p_local_cloud->size() < 1)
        return;

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*p_local_cloud, cloud_msg);
    cloud_msg.header.frame_id = left_camera_frame_id_;
    cloud_msg.header.stamp = stamp;

    local_map_points_publisher.publish(cloud_msg);
}

void viewer::publish_map_point(ros::Time &stamp) {
    ORB_SLAM3::Map *pActiveMap = mpMapDrawer_->mpAtlas->GetCurrentMap();
    if (!pActiveMap) return;

    const std::vector<ORB_SLAM3::MapPoint *> &vpMPs = pActiveMap->GetAllMapPoints();

    const std::vector<ORB_SLAM3::MapPoint *> &vpRefMPs = pActiveMap->GetReferenceMapPoints();

    set<ORB_SLAM3::MapPoint *> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if (vpMPs.empty()) return;

    PointCloudXYZ::Ptr p_global_cloud(new PointCloudXYZ);

    for (size_t i = 0; i < vpMPs.size(); i++) {
        const auto lm = vpMPs.at(i);
        if (lm->isBad()) continue;

        Eigen::Vector3f pos_w;
        if (is_imu_)
            pos_w = lm->GetWorldPos();
        else
            pos_w = T_ros_cam_se3_ * lm->GetWorldPos();
        pcl::PointXYZ point(pos_w.x(), pos_w.y(), pos_w.z());
        p_global_cloud->push_back(point);
    }

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*p_global_cloud, cloud_msg);
    cloud_msg.header.frame_id = map_frame_id_;
    cloud_msg.header.stamp = stamp;

    map_points_publisher.publish(cloud_msg);
}