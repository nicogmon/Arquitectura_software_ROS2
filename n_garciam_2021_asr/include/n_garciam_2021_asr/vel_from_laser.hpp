#ifndef N__GARCIAM__2021__ASR__VEL_FROM_LASER_NODE
#define N__GARCIAM__2021_ASR__VEL_FROM_LASER_NODE

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace n_garciam_2021_asr
{    

class VelFromLaserNode : public rclcpp::Node
{

public:
  VelFromLaserNode();
  

private: 

  void vel_callback(geometry_msgs::msg::Twist::UniquePtr msg);
  void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);
  
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
  geometry_msgs::msg::Twist msg_;
  sensor_msgs::msg::LaserScan::UniquePtr last_scan_;
  geometry_msgs::msg::Twist::UniquePtr last_vel_;
};
}
#endif