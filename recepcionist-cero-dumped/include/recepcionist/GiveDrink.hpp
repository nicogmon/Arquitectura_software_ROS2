#ifndef RECEPCIONIST_GIVEDRINK_HPP
#define RECEPCIONIST_GIVEDRINK_HPP

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace recepcionist
{

class GiveDrink : public BT::ActionNodeBase
{
public:
  explicit GiveDrink(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);
  
  void halt();
  BT::NodeStatus tick();
  
  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }

  private:
    rclcpp::Node::SharedPtr node_;
    gb_dialog::DialogInterface dialog_;
    rclcpp::Time start_time_;
};

}  // namespace recepcionist

#endif  // RECEPCIONIST_DIALOG_HPP