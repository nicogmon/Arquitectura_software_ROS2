
#include "vel_pub_sub/vel_sub.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"


namespace vel_pub_sub
{
VelSubNode::VelSubNode()
: Node("vel_pub_node")
{

  vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",rclcpp::QoS(10), std::bind(&VelSubNode::vel_callback, this, std::placeholders::_1));
   
}

void VelSubNode::vel_callback(const geometry_msgs::msg::Twist::ConstSharedPtr & msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->linear.x);
}
} // namespace vel_pub_sub