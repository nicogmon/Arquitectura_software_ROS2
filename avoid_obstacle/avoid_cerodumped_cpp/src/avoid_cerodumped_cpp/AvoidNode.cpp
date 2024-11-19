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

#include <utility>
#include "avoid_cerodumped_cpp/AvoidNode.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "kobuki_ros_interfaces/msg/button_event.hpp"

#include "rclcpp/rclcpp.hpp"

namespace avoid_cerodumped_cpp
{

using namespace std::chrono_literals;
using std::placeholders::_1;

AvoidNode::AvoidNode()
: Node("bump_go"),
  state_(START)
{
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", rclcpp::SensorDataQoS(),
    std::bind(&AvoidNode::scan_callback, this, _1));

  button_sub_ = create_subscription<kobuki_ros_interfaces::msg::ButtonEvent>(
    "input_button", rclcpp::SensorDataQoS(),
    std::bind(&AvoidNode::button_callback, this, _1));

  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);
  timer_ = create_wall_timer(50ms, std::bind(&AvoidNode::control_cycle, this));

  led_pub_ = create_publisher<kobuki_ros_interfaces::msg::Led>("output_led", 0);

  state_ts_ = now();
}

void
AvoidNode::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  last_scan_ = std::move(msg);
}

void
AvoidNode::button_callback(kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr button_msg)
{
  state_button_ = std::move(button_msg);

  if (state_button_->button == 0 && state_button_->state == 1) {
    button_pressed = true;
  }
  if (state_button_->button == 1 && state_button_->state == 1) {
    button_pressed = false;
  }
}

void
AvoidNode::control_cycle()
{
  // Do nothing until the first sensor read
  if (last_scan_ == nullptr) {
    return;
  }
  geometry_msgs::msg::Twist out_vel;
  kobuki_ros_interfaces::msg::Led out_led;

  switch (state_) {
    case START:
      out_led.value = RED;
      if (check_start_2_forward()) {
        out_led.value = OFF;
        go_state(FORWARD);
      }

      break;
    case FORWARD:
      out_vel.linear.x = SPEED_LINEAR;

      if (!check_start_2_forward()) {
        go_state(START);
      }

      if (check_forward_2_stop()) {
        go_state(STOP);
      }

      if (check_forward_2_turn()) {
        go_state(TURN);
      }
      break;
    case ARCH:
      out_vel.linear.x = SPEED_ARCH;
      out_vel.angular.z = -SPEED_ARCH_ANGULAR;
      out_led.value = OFF;

      if (!check_start_2_forward()) {
        go_state(START);
      }

      if (check_forward_2_turn()) {
        go_state(TURN);
      }

      if (check_arch_2_turn()) {
        go_state(TURN2);
      }
      break;
    case TURN:
      out_vel.angular.z = SPEED_ANGULAR;
      out_led.value = ORANGE;

      if (!check_start_2_forward()) {
        go_state(START);
      }

      if (check_turn_2_forward()) {
        go_state(ARCH);
      }

      break;

    case TURN2:
      out_vel.angular.z = SPEED_ANGULAR;
      out_led.value = GREEN;

      if (!check_start_2_forward()) {
        go_state(START);
      }

      if (check_turn_2_forward()) {
        go_state(FORWARD);
      }

      break;
    case STOP:
      if (!check_start_2_forward()) {
        go_state(START);
      }

      if (check_stop_2_forward()) {
        go_state(FORWARD);
      }
      break;
  }

  vel_pub_->publish(out_vel);
  led_pub_->publish(out_led);
}

void
AvoidNode::go_state(int new_state)
{
  state_ = new_state;
  state_ts_ = now();
}

bool
AvoidNode::check_forward_2_turn()
{
  // going forward when deteting an obstacle
  // at 0.5 meters with the front laser read

  return last_scan_->ranges[0] < OBSTACLE_DISTANCE;
}

bool
AvoidNode::check_forward_2_stop()
{
  // Stop if no sensor readings for 1 second
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed > SCAN_TIMEOUT;
}

bool
AvoidNode::check_stop_2_forward()
{
  // Going forward if sensor readings are available
  // again
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed < SCAN_TIMEOUT;
}

bool
AvoidNode::check_arch_2_turn()
{
  // Going back for 2 seconds
  return (now() - state_ts_) > ARCHING_TIME;
}

bool
AvoidNode::check_turn_2_forward()
{
  // Turning for 2 seconds
  return (now() - state_ts_) > TURNING_TIME;
}

bool
AvoidNode::check_start_2_forward()
{
  // Turning for 2 seconds
  return button_pressed;
}

}  // namespace avoid_cerodumped_cpp
