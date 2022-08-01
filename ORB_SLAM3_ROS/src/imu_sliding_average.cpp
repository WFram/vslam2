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
Eigen::MatrixXd M = Eigen::MatrixXd(6, 6);

using namespace std;

class SubscribeAndPublish {
public:
  SubscribeAndPublish() {
   imu_info_sub = nh.subscribe("/imu/data_raw", 10, &SubscribeAndPublish::imuInfoCallback, this);
   IMU_sliding_average_pub = nh.advertise<sensor_msgs::Imu>("/imu/data", 10);
  }
 

 void imuInfoCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    if (n < N) {
        double a_x_m = msg->linear_acceleration.x;
        double a_y_m = msg->linear_acceleration.y;
        double a_z_m = msg->linear_acceleration.z;
        double g_x_m = msg->angular_velocity.x;
        double g_y_m = msg->angular_velocity.y;
        double g_z_m = msg->angular_velocity.z;
        Eigen::MatrixXd temp = Eigen::MatrixXd(6, 1); 
        temp << a_x_m, a_y_m, a_z_m, g_x_m, g_y_m, g_z_m;
        M.block<6, 1>(0, n) = temp;
        
        sensor_msgs::Imu imu_msg;

        if (n == N - 1) {
          Eigen::MatrixXd num = Eigen::MatrixXd(6, 1);
          for (int i = 0; i < 6; i++) {
               for (int j = 0; j < N; j++)
                    num(i, 0) += M(i, j);
          }
          imu_msg.linear_acceleration.x = num(0, 0) / N;
          imu_msg.linear_acceleration.y = num(1, 0) / N;
          imu_msg.linear_acceleration.z = num(2, 0) / N;
          imu_msg.angular_velocity.x = num(3, 0) / N;
          imu_msg.angular_velocity.y = num(4, 0) / N;
          imu_msg.angular_velocity.z = num(5, 0) / N;
          imu_msg.header = msg->header;
        }
        else {
          imu_msg.linear_acceleration.x = a_x_m;
          imu_msg.linear_acceleration.y = a_y_m;
          imu_msg.linear_acceleration.z = a_z_m;
          imu_msg.angular_velocity.x = g_x_m;
          imu_msg.angular_velocity.y = g_y_m;
          imu_msg.angular_velocity.z = g_z_m;
          imu_msg.header = msg->header;
        }

        IMU_sliding_average_pub.publish(imu_msg);
        n++;
    }
    else {
        double a_x_m = msg->linear_acceleration.x;
        double a_y_m = msg->linear_acceleration.y;
        double a_z_m = msg->linear_acceleration.z;
        double g_x_m = msg->angular_velocity.x;
        double g_y_m = msg->angular_velocity.y;
        double g_z_m = msg->angular_velocity.z;

        sensor_msgs::Imu imu_msg;

        for (int j = 0; j < N; j++) {
          for (int i = 0; i < 6; i++) {
               if (j == N - 1) {
                    Eigen::MatrixXd temp = Eigen::MatrixXd(6, 1);
                    temp << a_x_m, a_y_m, a_z_m, g_x_m, g_y_m, g_z_m;
                    M.block<6, 1>(0, j) = temp;
               }
               else
                    M(i, j) = M(i, j + 1);
          }
        }

        Eigen::MatrixXd num = Eigen::MatrixXd(6, 1);
          for (int i = 0; i < 6; i++) {
               for (int j = 0; j < N; j++)
                    num(i, 0) += M(i, j);
          }
        
        imu_msg.linear_acceleration.x = num(0, 0) / N;
        imu_msg.linear_acceleration.y = num(1, 0) / N;
        imu_msg.linear_acceleration.z = num(2, 0) / N;
        imu_msg.angular_velocity.x = num(3, 0) / N;
        imu_msg.angular_velocity.y = num(4, 0) / N;
        imu_msg.angular_velocity.z = num(5, 0) / N;
        imu_msg.header = msg->header;

        IMU_sliding_average_pub.publish(imu_msg);
    }  
}

private:
  ros::NodeHandle nh; 
  ros::Publisher IMU_sliding_average_pub;
  ros::Subscriber imu_info_sub;
  int n = 0;
  int N = 5;
};
 
int main(int argc, char **argv) {
  ros::init(argc, argv, "sliding_average");
 
  SubscribeAndPublish SAPObject;
 
  ros::spin();
 
  return 0;
}
