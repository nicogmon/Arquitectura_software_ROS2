#include <string>
#include <utility>

#include "recepcionist/DetectedPersonNode.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

#include <memory>

#include "vision_msgs/msg/detection3_d_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include <utility>

#include "rclcpp/rclcpp.hpp"



namespace recepcionist
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
    "/output_detection_3d", rclcpp::SensorDataQoS(),
    std::bind(&DetectedPersonNode::detection3d_callback, this, _1));

  tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*node_);
}

void 
DetectedPersonNode::detection3d_callback(vision_msgs::msg::Detection3DArray::UniquePtr msg)
{
  RCLCPP_INFO(node_->get_logger(), "DETECTION CALLBACK");

  last_msg_ = std::move(msg); 
  msg = nullptr;
  if (last_msg_ != nullptr){
    RCLCPP_ERROR(node_->get_logger(),"HAY MENSAJEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE");
  }
}

BT::NodeStatus
DetectedPersonNode::tick()
{
  
  tf2::Transform camera2obj;
  camera2obj.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  camera2obj.setRotation(tf2::Quaternion(0, 0, 0, 1));

  geometry_msgs::msg::TransformStamped odom2camera_msg;
  tf2::Stamped<tf2::Transform> odom2camera;
  rclcpp::spin_some(node_);
  try
  {
    if (last_msg_ == nullptr) {
      throw std::runtime_error("LAST MSG IS NULL");
    }
    float len = std::size(last_msg_->detections);

    for (auto detection : last_msg_->detections) {
      if (detection.results[0].hypothesis.class_id.compare("person") == 0) {
        if (detection.bbox.center.position.z < 1.5) {
          return BT::NodeStatus::SUCCESS;
        }
  
      }
    }
    // for (int i = 0; i < len; i++) {
    //   if (last_msg_->detections[i].results[0].hypothesis.class_id == "person") {
    //     RCLCPP_INFO(node_->get_logger(), "DETECTED PERSON");
    //     last_msg_ = nullptr;
    //     return BT::NodeStatus::SUCCESS;
    //   }
    // }
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';

  }

  RCLCPP_INFO(node_->get_logger(), "NO DETECTED PERSON");


 
  return BT::NodeStatus::RUNNING;
  }
}  
// namespace br2_bt_bumpgo

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist::DetectedPersonNode>("DetectedPersonNode");
}