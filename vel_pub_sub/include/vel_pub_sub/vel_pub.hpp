#ifndef VEL_PUB_SUB__VEL_PUB_HPP_
#define VEL_PUB_SUB__VEL_PUB_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace vel_pub_sub
{
class VelPubNode : public rclcpp::Node
{

public:
  VelPubNode();
  

private:
  void timer_callback();
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Twist msg_;
};
}  // namespace vel_pub_sub

#endif  // VEL_PUB_SUB__VEL_PUB_HPP_