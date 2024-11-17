#ifndef VEL_PUB_SUB__VEL_SUB_HPP_
#define VEL_PUB_SUB__VEL_SUB_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace vel_pub_sub
{
class VelSubNode : public rclcpp::Node
{

public:
  VelSubNode();

private:

  void vel_callback(const geometry_msgs::msg::Twist::ConstSharedPtr & msg);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
  
  geometry_msgs::msg::Twist msg_;
  void timer_callback();
};
}  // namespace vel_pub_sub

#endif  // VEL_PUB_SUB__VEL_SUB_HPP_