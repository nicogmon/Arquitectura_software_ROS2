// Copyright 2021 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SEEKANDCAPTURE__TF_DETECTION_NODE_HPP_
#define SEEKANDCAPTURE__TF_DETECTION_NODE_HPP_

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

#include "vision_msgs/msg/detection3_d_array.hpp"

#include "rclcpp/rclcpp.hpp"

namespace seekandcapture
{

class TfDetectionNode : public rclcpp::Node
{
public:
  TfDetectionNode();

private:
  void detection3d_callback(vision_msgs::msg::Detection3DArray::UniquePtr msg);

  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection3d_sub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  vision_msgs::msg::Detection3DArray::UniquePtr last_msg_;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace seekandcapture
#endif  // SEEKANDCAPTURE__TF_DETECTION_NODE_HPP_
