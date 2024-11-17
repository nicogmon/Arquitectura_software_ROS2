#include "n_garciam_2021_asr/vel_from_laser.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"


namespace n_garciam_2021_asr
{
VelFromLaserNode::VelFromLaserNode()
: Node("vel_from_laser")
{
  vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/teleop_cmd_vel",rclcpp::QoS(10), std::bind(&VelFromLaserNode::vel_callback, this, std::placeholders::_1));
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan",rclcpp::QoS(10), std::bind(&VelFromLaserNode::scan_callback, this, std::placeholders::_1));
  //timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&VelFromLaserNode::timer_callback, this));
  
}

void 
VelFromLaserNode::vel_callback(geometry_msgs::msg::Twist::UniquePtr msg)
{
  last_vel_ = std::move(msg);

  if (last_scan_->ranges[0] > 1){
    vel_pub_->publish(msg_);
  }
  else if(last_scan_->ranges[0] < 1 && last_scan_->ranges[0] > 0.75f){
    last_vel_->linear.x = last_vel_->linear.x * 0.75;
    last_vel_->angular.z =  last_vel_->angular.z *0.75;
  }
  else if (last_scan_->ranges[0] < 0.75f && last_scan_->ranges[0] > 0.5f){
    last_vel_->linear.x = last_vel_->linear.x * 0.5;
    last_vel_->angular.z =  last_vel_->angular.z *0.5;
  }
  else if (last_scan_->ranges[0] < 0.5f){
    last_vel_->linear.x = 0;
    last_vel_->angular.z = 0;
  }
  
}


void
VelFromLaserNode::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  last_scan_ = std::move(msg);
}

} // namespace vel_pub_sub
