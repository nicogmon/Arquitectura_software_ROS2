#ifndef SEEKANDCAPTURE__PURSUEPERSONNODE_HPP_
#define SEEKANDCAPTURE__PURSUEPERSONNODE_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"


#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "seekandcapture/PIDController.hpp"

#include "kobuki_ros_interfaces/msg/sound.hpp"
#include "kobuki_ros_interfaces/msg/led.hpp"

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace seekandcapture
{

class PursuePersonNode : public BT::ActionNodeBase
{
public:
  explicit PursuePersonNode(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);
  void halt() {}
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({
        BT::InputPort<std::string>("objective"),
    });
  }
  
private:

  static const int OFF = 0;
  static const int GREEN = 1;
  static const int ORANGE = 2;
  static const int RED = 3;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::Led>::SharedPtr led_pub_;

  rclcpp::Publisher<kobuki_ros_interfaces::msg::Sound>::SharedPtr sound_pub_;
  
  rclcpp::TimerBase::SharedPtr timer_;

  
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  PIDController lin_pid_, ang_pid_;
  
  int x = 0;
};
  
};  // namespace

#endif 