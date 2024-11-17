#include <string>
#include <utility>

#include "recepcionist/ButtonNode.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include <memory>

#include <utility>

#include "rclcpp/rclcpp.hpp"



namespace recepcionist
{

using namespace std::chrono_literals;
using namespace std::placeholders;

ButtonNode::ButtonNode(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf) 
: BT::ConditionNode(xml_tag_name, conf)
{


button_sub_ = node_->create_subscription<kobuki_ros_interfaces::msg::ButtonEvent>(
    "input_button", rclcpp::SensorDataQoS(),
    std::bind(&AvoidNode::button_callback, this, _1));
}

void
AvoidNode::button_callback(kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr button_msg)
{
  state_button_ = std::move(button_msg);  
}

BT::NodeStatus
DetectedPersonNode::tick()
{
  if (state_button_->button == 0 && state_button_->state == 1) {
    return BT::NodeStatus::SUCCESS;
  }
}
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist::DetectedPersonNode>("DetectedPersonNode");
}