#ifndef RECEPCIONIST_BUTTONNODE_HPP
#define RECEPCIONIST_BUTTONNODE_HPP

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "kobuki_ros_interfaces/msg/button_event.hpp"

namespace recepcionist
{

class ButtonNode : public BT::ConditionNode
{
public:
  explicit ButtonNode(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::OutputPort<std::string>("button"),
      });
  }


private:
  void button_callback(kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr button_msg);
  rclcpp::Subscription<kobuki_ros_interfaces::msg::ButtonEvent>::SharedPtr button_sub_;
};
}

#endif