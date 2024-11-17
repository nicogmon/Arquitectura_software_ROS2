#ifndef RECEPCIONIST__EMPTYCHAIR_HPP_
#define RECEPCIONIST__EMPTYCHAIR_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"


#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

#include "vision_msgs/msg/detection3_d_array.hpp"
 
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"



namespace recepcionist
{

class EmptyChair : public BT::ConditionNode
{
public:
  explicit EmptyChair(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::OutputPort<std::string>("silla_vacia"),
      });
  }

void 
detection3d_callback(vision_msgs::msg::Detection3DArray::UniquePtr msg);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection3d_sub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  vision_msgs::msg::Detection3DArray::UniquePtr last_msg_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  int x = 0;
};

}  // namespace

#endif 