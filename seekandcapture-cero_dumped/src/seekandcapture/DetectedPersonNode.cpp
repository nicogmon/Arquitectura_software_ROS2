#include <string>
#include <utility>

#include "seekandcapture/DetectedPersonNode.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

#include <memory>

#include "seekandcapture/TfDetectionNode.hpp"

#include "vision_msgs/msg/detection3_d_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"



namespace seekandcapture
{

using namespace std::chrono_literals;
using namespace std::placeholders;

DetectedPersonNode::DetectedPersonNode(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf) 
: BT::ConditionNode(xml_tag_name, conf)
{
  
  config().blackboard->get("node", node_);
  RCLCPP_INFO(node_->get_logger(), "DetectedPersonNode constructor");

  detection3d_sub_ = node_->create_subscription<vision_msgs::msg::Detection3DArray>(
    "output_detection_3d", rclcpp::SensorDataQoS().reliable(),
    std::bind(&DetectedPersonNode::detection3d_callback, this, _1));

  tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*node_);
  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

void 
DetectedPersonNode::detection3d_callback(vision_msgs::msg::Detection3DArray::UniquePtr msg)
{
  RCLCPP_INFO(node_->get_logger(), "DETECTION CALLBACK");

  last_msg_ = std::move(msg); 
}

BT::NodeStatus
DetectedPersonNode::tick()
{
  
  tf2::Transform camera2obj;
  camera2obj.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  camera2obj.setRotation(tf2::Quaternion(0, 0, 0, 1));

  geometry_msgs::msg::TransformStamped odom2camera_msg;
  tf2::Stamped<tf2::Transform> odom2camera;
  

  try
  {
    if (last_msg_ == nullptr) {
      throw std::runtime_error("LAST MSG IS NULL");
    }
    float len = std::size(last_msg_->detections);
    
    for (int i = 0; i < len; i++) {
      if (last_msg_->detections[i].results[0].hypothesis.class_id == "person") {
        RCLCPP_INFO(node_->get_logger(), "DETECTED PERSON");
        return BT::NodeStatus::SUCCESS;
      }
    }
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';

  }

  RCLCPP_INFO(node_->get_logger(), "NO DETECTED PERSON");

  geometry_msgs::msg::Twist vel_msg;
  vel_msg.angular.z = 0.5;

  vel_pub_->publish(vel_msg);

 
  return BT::NodeStatus::RUNNING;
  }
}  
// namespace br2_bt_bumpgo

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<seekandcapture::DetectedPersonNode>("DetectedPersonNode");
}