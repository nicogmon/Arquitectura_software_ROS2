#include <string>
#include <utility>

#include "seekandcapture/AvoidPersonNode.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"



#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <iostream>
#include <iomanip>
#include <numbers>
#include <memory>
#include <cmath>
   
namespace seekandcapture
{

using namespace std::chrono_literals;
using namespace std::placeholders;

AvoidPersonNode::AvoidPersonNode(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf) 
: BT::ActionNodeBase(xml_tag_name, conf)
{
  

  config().blackboard->get("node", node_);
  RCLCPP_INFO(node_->get_logger(), "AvoidPersonNode constructor");
  
  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
   geometry_msgs::msg::Twist vel;

  
}

BT::NodeStatus
AvoidPersonNode::tick()
{
 if (status() == BT::NodeStatus::IDLE) {
    start_time_ = node_->now();
  }

  geometry_msgs::msg::Twist vel_msgs;
  vel_msgs.angular.z = 0.5;
  vel_pub_->publish(vel_msgs);

  auto elapsed = node_->now() - start_time_;

  if (elapsed < 3s) {
    return BT::NodeStatus::RUNNING;
  } else {
    return BT::NodeStatus::SUCCESS;
  }

}

}
 // namespace br2_bt_bumpgo

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<seekandcapture::AvoidPersonNode>("AvoidPersonNode");
}