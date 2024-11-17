#ifndef RECEPCIONIST_DIALOGDRINK_HPP
#define RECEPCIONIST_DIALOGDRINK_HPP

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace recepcionist
{

class DialogDrink : public BT::ActionNodeBase
{
public:
  explicit DialogDrink(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();


  static BT::PortsList providedPorts()
  {
    return BT::PortsList({
        BT::OutputPort<std::string>("drink"),
        BT::InputPort<std::string>("name_")
    });
  }

  void 
  GetDrinkCB(dialogflow_ros2_interfaces::msg::DialogflowResult);

  private:
    rclcpp::Node::SharedPtr node_;
    std::string name_ = ""; 
    std::string drink_ = "";
    rclcpp::Time start_time_;
    gb_dialog::DialogInterface dialog_;
};

}  // namespace recepcionist

#endif  // RECEPCIONIST_DIALOG_HPP