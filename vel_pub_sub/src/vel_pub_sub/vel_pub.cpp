#include "vel_pub_sub/vel_pub.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"


namespace vel_pub_sub
{
VelPubNode::VelPubNode()
: Node("vel_pub_node")
{
  vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&VelPubNode::timer_callback, this));
  msg_.linear.x = 0.3;
}

void 
VelPubNode::timer_callback()
{
  vel_pub_->publish(msg_);
}
} // namespace vel_pub_sub

