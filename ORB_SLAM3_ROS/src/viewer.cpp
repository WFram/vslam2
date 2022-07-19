#include "viewer.h"

viewer::viewer(const ros::NodeHandle &nh, ORB_SLAM3::LocalMapping* pLocalMapping, ORB_SLAM3::FrameDrawer* pFrameDrawer, ORB_SLAM3::MapDrawer* pMapDrawer, bool is_imu): 
    nh_(nh), it_(nh), mpLocalMapping_(pLocalMapping), mpFrameDrawer_(pFrameDrawer), mpMapDrawer_(pMapDrawer), is_imu_(is_imu) {
        // set parameters and variable
        set_params();
        // broadcasting
        set_advertising();
}
void viewer::set_params() {
    // Retrieve frame id parameters
    if (nh_.hasParam("map_frame"))
        nh_.getParam("map_frame", map_frame_id_);

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
}
void viewer::run() {
    is_terminated_ = false;
    ros::Rate rate(30);
    while (ros::ok()) {
        if (is_start_) {
            publish_local_map_point(stamp_);
            // publish_map_point(stamp_);
            publish_debug_image(stamp_);
            // publish_path(stamp_);
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

void viewer::publish_debug_image(ros::Time& stamp) {
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

void viewer::publish_path(ros::Time& stamp) {
    ORB_SLAM3::Map* pActiveMap = mpMapDrawer_->mpAtlas->GetCurrentMap();
    if (!pActiveMap) return;

    const vector<ORB_SLAM3::KeyFrame*> vpKFs = pActiveMap->GetAllKeyFrames();
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
void viewer::publish_map_point(ros::Time& stamp) {
    ORB_SLAM3::Map* pActiveMap = mpMapDrawer_->mpAtlas->GetCurrentMap();
    if (!pActiveMap) return;

    const std::vector<ORB_SLAM3::MapPoint*>& vpMPs = pActiveMap->GetAllMapPoints();
    
    const std::vector<ORB_SLAM3::MapPoint*>& vpRefMPs = pActiveMap->GetReferenceMapPoints();

    set<ORB_SLAM3::MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if (vpMPs.empty()) return;

    //declare and initialize red, green, blue component values
    // dark color for inactive map points
    uint8_t r(15), g(15), b(15);

    sensor_msgs::PointCloud2 map_point_msg;
    map_point_msg.header.frame_id = map_frame_id_;
    map_point_msg.header.stamp = stamp;
    int num_channels = 4;// x y z, rgb
    map_point_msg.height = 1;
    map_point_msg.width = vpMPs.size();
    map_point_msg.is_bigendian = false;
    map_point_msg.is_dense = false;
    map_point_msg.point_step = num_channels * sizeof(float);
    map_point_msg.row_step = map_point_msg.point_step * map_point_msg.width;
    map_point_msg.fields.resize(num_channels);

    std::string channel_id[] = {"x", "y", "z", "rgb"};

    for (int i = 0; i < num_channels; i++) {
        map_point_msg.fields[i].name = channel_id[i];
        map_point_msg.fields[i].offset = i * sizeof(float);
        map_point_msg.fields[i].count = 1;
        map_point_msg.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }
    map_point_msg.data.resize(map_point_msg.row_step * map_point_msg.height);

    unsigned char *cloud_data_ptr = &(map_point_msg.data[0]);

    float data_array[num_channels];

    for (size_t i = 0; i < vpMPs.size(); i++) {
        const auto lm = vpMPs.at(i);
        if (lm->isBad() || spRefMPs.count(lm)) continue;
        // color is encoded strangely, but efficiently.  Stored as a 4-byte "float", but
        // interpreted as individual byte values for 3 colors
        // bits 0-7 are blue value, bits 8-15 are green, bits 16-23 are red;
        // Can build the rgb encoding with bit-level operations:
        uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                        static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
        // and encode these bits as a single-precision (4-byte) float:
        float rgb_float = *reinterpret_cast<float *>(&rgb);
        Eigen::Vector3f pos_w;
        if(is_imu_)
            pos_w = lm->GetWorldPos();
        else 
            pos_w = T_ros_cam_se3_ * lm->GetWorldPos();
        data_array[0] = pos_w.x(); // x.
        data_array[1] = pos_w.y(); // y.
        data_array[2] = pos_w.z(); // z.
        data_array[3] = rgb_float;
        memcpy(cloud_data_ptr + (i * map_point_msg.point_step),
                data_array,
                num_channels * sizeof(float));
    }
    
    map_points_publisher.publish(map_point_msg);
}
void viewer::publish_local_map_point(ros::Time &stamp) {
    ORB_SLAM3::Map* pActiveMap = mpMapDrawer_->mpAtlas->GetCurrentMap();
    if (!pActiveMap) return;
    
    const std::vector<ORB_SLAM3::MapPoint*>& vpRefMPs = pActiveMap->GetReferenceMapPoints();

    set<ORB_SLAM3::MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if (spRefMPs.empty()) return;

    //declare and initialize red, green, blue component values
    // dark color for inactive map points
    uint8_t r(15), g(15), b(15);
    int num_channels = 4;// x y z, rgb
    std::string channel_id[] = {"x", "y", "z", "rgb"};

    sensor_msgs::PointCloud2 local_map_point_msg;
    local_map_point_msg.header.frame_id = map_frame_id_;
    local_map_point_msg.header.stamp = stamp;
    local_map_point_msg.height = 1;
    local_map_point_msg.width = spRefMPs.size();
    local_map_point_msg.is_dense = false;
    local_map_point_msg.is_bigendian = false;
    local_map_point_msg.point_step = num_channels * sizeof(float);
    local_map_point_msg.row_step = local_map_point_msg.point_step * local_map_point_msg.width;
    local_map_point_msg.fields.resize(num_channels);

    for (int i = 0; i < num_channels; i++) {
        local_map_point_msg.fields[i].name = channel_id[i];
        local_map_point_msg.fields[i].offset = i * sizeof(float);
        local_map_point_msg.fields[i].count = 1;
        local_map_point_msg.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }
    local_map_point_msg.data.resize(local_map_point_msg.row_step * local_map_point_msg.height);

    unsigned char *cloud_data_ptr = &(local_map_point_msg.data[0]);

    float data_array[num_channels];

    size_t counter = 0;
    for (auto spRefMP : vpRefMPs) {
        if (spRefMP->isBad()) continue;
        
        // color is encoded strangely, but efficiently.  Stored as a 4-byte "float", but
        // interpreted as individual byte values for 3 colors
        // bits 0-7 are blue value, bits 8-15 are green, bits 16-23 are red;
        // Can build the rgb encoding with bit-level operations:
        uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                        static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
        // and encode these bits as a single-precision (4-byte) float:
        float rgb_float = *reinterpret_cast<float *>(&rgb);

        Eigen::Vector3f pos_w;
        if(is_imu_)
            pos_w = spRefMP->GetWorldPos();
        else 
            pos_w = T_ros_cam_se3_ * spRefMP->GetWorldPos();
        
        data_array[0] = (float) pos_w.x();// x.
        data_array[1] = (float) pos_w.y();// y.
        data_array[2] = (float) pos_w.z();// z.
        data_array[3] = rgb_float;

        memcpy(cloud_data_ptr + (counter * local_map_point_msg.point_step),
                data_array,
                num_channels * sizeof(float));
        counter++;
    }
    local_map_points_publisher.publish(local_map_point_msg);
}