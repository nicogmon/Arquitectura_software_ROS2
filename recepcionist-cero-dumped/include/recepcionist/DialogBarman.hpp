#ifndef RECEPCIONIST_DIALOGBARMAN_HPP
#define RECEPCIONIST_DIALOGBARMAN_HPP

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace recepcionist
{

class DialogBarman : public BT::ActionNodeBase
{
public:
  explicit DialogBarman(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();


  static BT::PortsList providedPorts()
  {
    return BT::PortsList({
        BT::InputPort<std::string>("drink"),
        
    });
  }

  void 
  GetServedCB(dialogflow_ros2_interfaces::msg::DialogflowResult);

  private:
    rclcpp::Node::SharedPtr node_;
    std::string drink_ = "";
    bool served_ = false;
    rclcpp::Time start_time_;
    gb_dialog::DialogInterface dialog_;
};

}  // namespace recepcionist

#endif  // RECEPCIONIST_DIALOG_HPP