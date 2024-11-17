#include <string>
#include <iostream>

#include <string>



#include "gb_dialog/DialogInterface.hpp"
#include "sound_play.hpp"

#include "recepcionist/DialogDrink.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
namespace ph = std::placeholders;

namespace recepcionist
{

DialogDrink::DialogDrink(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
    config().blackboard->get("node", node_);
    dialog_.DialogInterface::registerCallback(
        std::bind(&DialogDrink::GetDrinkCB, this, ph::_1),
        "GetDrink");
}

void
DialogDrink::halt()
{
  std::cout << "DialogDrink halt" << std::endl;
}

BT::NodeStatus
DialogDrink::tick()
{
  if (status() == BT::NodeStatus::IDLE ) {
    drink_ = "";
    start_time_ = node_->now();
    dialogflow_ros2_interfaces::msg::DialogflowResult result;
    getInput("name_", name_);
    //RCLCPP_INFO(node_->get_logger(), "[ExampleDF] DialogDrinkCB: Name [%s]", name_.c_str());
    result.fulfillment_text = "Hello everyone, this is                                   " + name_ + "                                What do you want to drink?";
    dialog_.DialogInterface::speak(result.fulfillment_text); 
  }

  auto elapsed_time = node_->now() - start_time_;

  if (elapsed_time > 5s){
    if(drink_ != "" ){
        return BT::NodeStatus::SUCCESS;
    }
    dialog_.DialogInterface::listen();
    rclcpp::spin_some(dialog_.get_node_base_interface());
  }
  
  return BT::NodeStatus::RUNNING;
}



void 
DialogDrink::GetDrinkCB(dialogflow_ros2_interfaces::msg::DialogflowResult result)
{
  RCLCPP_INFO(node_->get_logger(), "[ExampleDF] DialogIntentCB: intent [%s]", result.intent.c_str());
  dialog_.DialogInterface::speak(result.fulfillment_text);
  drink_ = result.parameters[0].value[0];
  RCLCPP_INFO(node_->get_logger(), "[ExampleDF] DialogIntentCB: Drink [%s]", drink_.c_str());
  setOutput("drink", drink_);

}

}  // namespace recepcionist

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist::DialogDrink>("DialogDrink");
}