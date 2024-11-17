#ifndef RECEPCIONIST_DIALOGNAME_HPP
#define RECEPCIONIST_DIALOGNAME_HPP

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "kobuki_ros_interfaces/msg/led.hpp"

namespace recepcionist
{

class DialogName : public BT::ActionNodeBase
{
public:
  explicit DialogName(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);
  void halt();
  BT::NodeStatus tick();


  static BT::PortsList providedPorts()
  {
    return BT::PortsList({
        BT::OutputPort<std::string>("name_")
    });
  }

  void 
  welcomeIntentCB(dialogflow_ros2_interfaces::msg::DialogflowResult);

  void 
  DialogNameCB(dialogflow_ros2_interfaces::msg::DialogflowResult);
  

  private:
    kobuki_ros_interfaces::msg::Led out_led;
    rclcpp::Publisher<kobuki_ros_interfaces::msg::Led>::SharedPtr led_pub_;

    static const int OFF = 0;
    static const int GREEN = 1;
    static const int RED = 3;

    rclcpp::Node::SharedPtr node_;
    std::string name_ = "";    
    rclcpp::Time start_time_;
    gb_dialog::DialogInterface dialog_;
};

}  // namespace recepcionist

#endif  // RECEPCIONIST_DIALOG_HPP