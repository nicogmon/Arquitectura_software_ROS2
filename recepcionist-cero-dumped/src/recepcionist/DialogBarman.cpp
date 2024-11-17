#include <string>
#include <iostream>

#include <string>



#include "gb_dialog/DialogInterface.hpp"
#include "sound_play.hpp"

#include "recepcionist/DialogBarman.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
namespace ph = std::placeholders;

namespace recepcionist
{

DialogBarman::DialogBarman(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
    config().blackboard->get("node", node_);
    dialog_.DialogInterface::registerCallback(
        std::bind(&DialogBarman::GetServedCB, this, ph::_1),
        "GetServed");
}

void
DialogBarman::halt()
{
  std::cout << "DialogBarman halt" << std::endl;
}

BT::NodeStatus
DialogBarman::tick()
{
  if (status() == BT::NodeStatus::IDLE ) {
    start_time_ = node_->now();
    dialogflow_ros2_interfaces::msg::DialogflowResult result;
    getInput("drink", drink_ );
    RCLCPP_INFO(node_->get_logger(), "[ExampleDF] DialogBarmanCB: DRINKTOBAR [%s]", drink_.c_str());
    result.fulfillment_text = "The guest would have a " + drink_ ;
    dialog_.DialogInterface::speak(result.fulfillment_text); 
  }

  auto elapsed_time = node_->now() - start_time_;

  if (elapsed_time>3s){
    if(served_){
        return BT::NodeStatus::SUCCESS;
    }
    dialog_.DialogInterface::listen();
    rclcpp::spin_some(dialog_.get_node_base_interface());
  }
  
  return BT::NodeStatus::RUNNING;
}



void 
DialogBarman::GetServedCB(dialogflow_ros2_interfaces::msg::DialogflowResult result)
{
  RCLCPP_INFO(node_->get_logger(), "[ExampleDF] DialogIntentCB: intent [%s]", result.intent.c_str());
  dialog_.DialogInterface::speak(result.fulfillment_text);
  served_ = true;
  

}

}  // namespace recepcionist

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist::DialogBarman>("DialogBarman");
}