#include <string>
#include <iostream>

#include <string>



#include "gb_dialog/DialogInterface.hpp"
#include "sound_play.hpp"

#include "recepcionist/GiveDrink.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
namespace ph = std::placeholders;

namespace recepcionist
{

GiveDrink::GiveDrink(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
    config().blackboard->get("node", node_);
}

void
GiveDrink::halt()
{
  std::cout << "GiveDrink halt" << std::endl;
}

BT::NodeStatus
GiveDrink::tick()
{
  if (status() == BT::NodeStatus::IDLE ) {
    start_time_ = node_->now();
    dialogflow_ros2_interfaces::msg::DialogflowResult result;
    
    //RCLCPP_INFO(node_->get_logger(), "[ExampleDF] DialogDrinkCB: Name [%s]", name_.c_str());
    result.fulfillment_text = "Take your drink please!";
    dialog_.DialogInterface::speak(result.fulfillment_text); 

  }

  auto elapsed_time = node_->now() - start_time_;

  if (elapsed_time>3s){
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}


}  // namespace recepcionist

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist::GiveDrink>("GiveDrink");
}