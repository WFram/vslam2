#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"

#include "Eigen/Dense"

double a_x;
double a_y;
double a_z;
double g_x;
double g_y;
double g_z;

Eigen::MatrixXd X = Eigen::MatrixXd(6, 1);

using namespace std;

class kalman_filter {

public:
    kalman_filter();

    Eigen::MatrixXd update(Eigen::MatrixXd x, Eigen::MatrixXd z);

    ~kalman_filter();

private:
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd P;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd H;

    bool isinitized = false;
};

Eigen::MatrixXd kalman_filter::update(Eigen::MatrixXd x, Eigen::MatrixXd z) {
     if (!isinitized) {
          double p_var_acc = 0.0433;
          double p_var_gyro = 0.0020;
          P = Eigen::MatrixXd(6, 6);
          P << p_var_acc, 0,         0,         0,          0,          0,
               0,         p_var_acc, 0,         0,          0,          0,
               0,         0,         p_var_acc, 0,          0,          0,
               0,         0,         0,         p_var_gyro, 0,          0,
               0,         0,         0,         0,          p_var_gyro, 0,
               0,         0,         0,         0,          0,          p_var_gyro;
          isinitized = true;
     }

     Eigen::MatrixXd K = P * (H.transpose()) * ((H * P * (H.transpose()) + R).inverse());
     x = x + K * (z - H * x);
     int x_size = x.size();
     Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
     P = (I - K * H) * P;
     return x;
}

kalman_filter::kalman_filter() {
     H = Eigen::MatrixXd(6, 6);
     H << 1, 0, 0, 0, 0, 0,
          0, 1, 0, 0, 0, 0,
          0, 0, 1, 0, 0, 0,
          0, 0, 0, 1, 0, 0,
          0, 0, 0, 0, 1, 0,
          0, 0, 0, 0, 0, 1;
          
     double r_var_acc = 0.0433;
     double r_var_gyro = 0.0020;
     R = Eigen::MatrixXd(6, 6);
     R << r_var_acc, 0,         0,         0,          0,          0,
          0,         r_var_acc, 0,         0,          0,          0,
          0,         0,         r_var_acc, 0,          0,          0,
          0,         0,         0,         r_var_gyro, 0,          0,
          0,         0,         0,         0,          r_var_gyro, 0,
          0,         0,         0,         0,          0,          r_var_gyro;
}

kalman_filter::~kalman_filter() {}

class SubscribeAndPublish {
public:
  SubscribeAndPublish() {
   imu_info_sub = n.subscribe("/imu/data_raw", 10, &SubscribeAndPublish::imuInfoCallback, this);
   IMU_kalman_pub = n.advertise<sensor_msgs::Imu>("/imu/data", 10);
  }
 

 void imuInfoCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    if (first) {
        double a_x_m = msg->linear_acceleration.x;
        double a_y_m = msg->linear_acceleration.y;
        double a_z_m = msg->linear_acceleration.z;
        double g_x_m = msg->angular_velocity.x;
        double g_y_m = msg->angular_velocity.y;
        double g_z_m = msg->angular_velocity.z;
        X << a_x_m, a_y_m, a_z_m, g_x_m, g_y_m, g_z_m;
        
        sensor_msgs::Imu imu_msg;
        imu_msg.linear_acceleration.x = a_x_m;
        imu_msg.linear_acceleration.y = a_y_m;
        imu_msg.linear_acceleration.z = a_z_m;
        imu_msg.angular_velocity.x = g_x_m;
        imu_msg.angular_velocity.y = g_y_m;
        imu_msg.angular_velocity.z = g_z_m;
        imu_msg.header = msg->header;

        IMU_kalman_pub.publish(imu_msg);
        first = false;
    }
    else {
        double a_x_m = msg->linear_acceleration.x;
        double a_y_m = msg->linear_acceleration.y;
        double a_z_m = msg->linear_acceleration.z;
        double g_x_m = msg->angular_velocity.x;
        double g_y_m = msg->angular_velocity.y;
        double g_z_m = msg->angular_velocity.z;

        Eigen::MatrixXd z;
        z = Eigen::MatrixXd(6, 1);
        z << a_x_m, a_y_m, a_z_m, g_x_m, g_y_m, g_z_m;

        kalman_filter kf;
        
        Eigen::MatrixXd x_new = kf.update(X, z);
        X = z;
   
        a_x = x_new(0, 0);
        a_y = x_new(1, 0);
        a_z = x_new(2, 0);
        g_x = x_new(3, 0);
        g_y = x_new(4, 0);
        g_z = x_new(5, 0);

        sensor_msgs::Imu imu_msg;
        imu_msg.linear_acceleration.x = a_x;
        imu_msg.linear_acceleration.y = a_y;
        imu_msg.linear_acceleration.z = a_z;
        imu_msg.angular_velocity.x = g_x;
        imu_msg.angular_velocity.y = g_y;
        imu_msg.angular_velocity.z = g_z;
        imu_msg.header = msg->header;
        IMU_kalman_pub.publish(imu_msg);
    }  
}
 
private:
  ros::NodeHandle n; 
  ros::Publisher IMU_kalman_pub;
  ros::Subscriber imu_info_sub;
  bool first = true;
};
 
int main(int argc, char **argv) {
  ros::init(argc, argv, "KF");
 
  SubscribeAndPublish SAPObject;
 
  ros::spin();
 
  return 0;
}
