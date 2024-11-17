#include <string>
#include <utility>

#include "seekandcapture/PursuePersonNode.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"



#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "kobuki_ros_interfaces/msg/sound.hpp"

#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>


#include "seekandcapture/PIDController.hpp"

#include <iostream>
#include <iomanip>
#include <numbers>
#include <memory>
#include <cmath>
   
namespace seekandcapture
{

using namespace std::chrono_literals;
using namespace std::placeholders;

PursuePersonNode::PursuePersonNode(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf) 
: BT::ActionNodeBase(xml_tag_name, conf),
  tf_buffer_(),
  tf_listener_(tf_buffer_),
  lin_pid_(0.0, 5.0, 0.0, 0.3),
  ang_pid_(0.0, M_PI / 2, 0.5, 1.2)
{
  

  config().blackboard->get("node", node_);
  RCLCPP_INFO(node_->get_logger(), "PursuePersonNode constructor");
  
  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
   geometry_msgs::msg::Twist vel;

  led_pub_ = node_->create_publisher<kobuki_ros_interfaces::msg::Led>("output_led", 0);

  sound_pub_ = node_->create_publisher<kobuki_ros_interfaces::msg::Sound>("output_sound", 10);
  
}

BT::NodeStatus
PursuePersonNode::tick()
{
  RCLCPP_INFO(node_->get_logger(), "Pursuing person 1");
  lin_pid_.set_pid(0.6, 0.05, 0.35);
  ang_pid_.set_pid(0.6, 0.08, 0.32);


  kobuki_ros_interfaces::msg::Led out_led;
  kobuki_ros_interfaces::msg::Sound out_sound;

  geometry_msgs::msg::Twist vel;
  geometry_msgs::msg::TransformStamped person_msg;

  try {
    person_msg = tf_buffer_.lookupTransform(
      "base_link", "person",
      tf2::timeFromSec(rclcpp::Time(person_msg.header.stamp).seconds()));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "Obstacle transform not found: %s", ex.what());
    return BT::NodeStatus::RUNNING;
  }

  const auto & x = person_msg.transform.translation.x;
  const auto & y = person_msg.transform.translation.y;


  double angulo = atan2(y, x);
  double distancia = sqrt(pow(x, 2) + pow(y, 2));

  if (distancia <= 0.9) {
    vel.linear.x = 0;
    vel.angular.z = 0;
    vel_pub_->publish(vel);
    out_sound.value = kobuki_ros_interfaces::msg::Sound::ON;
    sound_pub_->publish(out_sound);
    out_led.value = GREEN;
    led_pub_->publish(out_led);
    RCLCPP_INFO(node_->get_logger(), "DETECTADAAAAAAAAAAAA");
    return BT::NodeStatus::SUCCESS;
    }
  
  out_led.value = RED;
  led_pub_->publish(out_led);

  vel.angular.z = ang_pid_.get_output(angulo);
  
  RCLCPP_INFO( 
    node_->get_logger(), "Velocidad angular(x:%f,y:%f =%f): %f", x, y,
    atan2(y, x), vel.angular.z);

  vel.linear.x = lin_pid_.get_output(distancia);
  RCLCPP_INFO(node_->get_logger(), "Velocidad lineal: %f", vel.linear.x);

  vel_pub_->publish(vel);

  return BT::NodeStatus::RUNNING;
  
}

}
 // namespace br2_bt_bumpgo

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<seekandcapture::PursuePersonNode>("PursuePersonNode");
}