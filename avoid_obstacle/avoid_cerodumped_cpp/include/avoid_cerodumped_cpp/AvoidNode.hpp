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

#ifndef AVOID_CERODUMPED_CPP__AVOIDNODE_HPP_
#define AVOID_CERODUMPED_CPP__AVOIDNODE_HPP_

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "kobuki_ros_interfaces/msg/button_event.hpp"

#include "kobuki_ros_interfaces/msg/led.hpp"

#include "rclcpp/rclcpp.hpp"

namespace avoid_cerodumped_cpp
{

using namespace std::chrono_literals;  // NOLINT

class AvoidNode : public rclcpp::Node
{
public:
  AvoidNode();

private:
  void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);
  void button_callback(kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr button_msg);
  void control_cycle();

  static const int FORWARD = 0;
  static const int ARCH = 1;
  static const int TURN = 2;
  static const int TURN2 = 3;
  static const int STOP = 4;
  static const int START = 5;

  static const int OFF = 0;
  static const int GREEN = 1;
  static const int ORANGE = 2;
  static const int RED = 3;

  int state_;
  rclcpp::Time state_ts_;

  void go_state(int new_state);
  bool check_start_2_forward();
  bool check_forward_2_turn();
  bool check_forward_2_stop();
  bool check_arch_2_turn();
  bool check_turn_2_forward();
  bool check_stop_2_forward();


  const rclcpp::Duration TURNING_TIME {4.2s};
  const rclcpp::Duration ARCHING_TIME {8.9s};
  const rclcpp::Duration SCAN_TIMEOUT {1s};

  static constexpr float SPEED_LINEAR = 0.3f;
  static constexpr float SPEED_ARCH = 0.30f;
  static constexpr float SPEED_ANGULAR = 0.5f;
  static constexpr float SPEED_ARCH_ANGULAR = 0.50f;
  static constexpr float OBSTACLE_DISTANCE = 0.8f;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<kobuki_ros_interfaces::msg::ButtonEvent>::SharedPtr button_sub_;

  rclcpp::Publisher<kobuki_ros_interfaces::msg::Led>::SharedPtr led_pub_;

  kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr state_button_;
  sensor_msgs::msg::LaserScan::UniquePtr last_scan_;

  bool button_pressed = false;
};

}  // namespace avoid_cerodumped_cpp

#endif  // AVOID_CERODUMPED_CPP__AVOIDNODE_HPP_
