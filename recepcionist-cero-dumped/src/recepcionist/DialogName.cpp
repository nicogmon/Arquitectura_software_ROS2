#include <string>
#include <iostream>

#include <string>



#include "gb_dialog/DialogInterface.hpp"
#include "sound_play.hpp"

#include "recepcionist/DialogName.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
// #include "kobuki_ros_interfaces/msg/led.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
namespace ph = std::placeholders;

namespace recepcionist
{

DialogName::DialogName(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
    config().blackboard->get("node", node_);
    dialog_.DialogInterface::registerCallback(
        std::bind(&DialogName::welcomeIntentCB, this, ph::_1),
        "Default Welcome Intent");

    dialog_.DialogInterface::registerCallback(
        std::bind(&DialogName::DialogNameCB, this, ph::_1),
        "RequestName");
    
    led_pub_ = node_->create_publisher<kobuki_ros_interfaces::msg::Led>("output_led", 0);
}

void
DialogName::halt()
{
  std::cout << "DialogName halt" << std::endl;
}

BT::NodeStatus
DialogName::tick()
{

  if (status() == BT::NodeStatus::IDLE) {
    name_ = "";
    start_time_ = node_->now();
    dialog_.DialogInterface::speak("Hello, What is your name?"); 
  }
  auto elapsed_time = node_->now() - start_time_;

  if (elapsed_time > 3s){
    if(name_ != "" ){
        
        return BT::NodeStatus::SUCCESS;
    }
    // out_led.value = RED;
    // led_pub_->publish(out_led);

    dialog_.DialogInterface::listen();
    rclcpp::spin_some(dialog_.get_node_base_interface());
  }
  
  return BT::NodeStatus::RUNNING;
}

void 
DialogName::welcomeIntentCB(dialogflow_ros2_interfaces::msg::DialogflowResult result)
{
    RCLCPP_INFO(node_->get_logger(), "[ExampleDF] welcomeIntentCB: intent [%s]", result.intent.c_str());
    dialog_.DialogInterface::speak(result.fulfillment_text);
}


void 
DialogName::DialogNameCB(dialogflow_ros2_interfaces::msg::DialogflowResult result)
{
  RCLCPP_INFO(node_->get_logger(), "[ExampleDF] DialogIntentCB: intent [%s]", result.intent.c_str());
  dialog_.DialogInterface::speak(result.fulfillment_text);
  name_ = result.parameters[0].value[0];
  RCLCPP_INFO(node_->get_logger(), "[ExampleDF] DialogIntentCB: NAME [%s]", name_.c_str());
  setOutput("name_", name_);

  // out_led.value = OFF;
  // led_pub_->publish(out_led);
  // name_ = "";
}



}  // namespace recepcionist

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist::DialogName>("DialogName");
}