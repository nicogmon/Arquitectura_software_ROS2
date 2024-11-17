#include <memory>
#include "vel_pub_sub/vel_sub.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node_vel_sub = std::make_shared<vel_pub_sub::VelSubNode>();

  rclcpp::spin(node_vel_sub);

  rclcpp::shutdown();
  return 0;

}