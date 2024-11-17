#include <memory>
#include "vel_pub_sub/vel_pub.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node_vel_pub = std::make_shared<vel_pub_sub::VelPubNode>();

  rclcpp::spin(node_vel_pub);

  rclcpp::shutdown();
  return 0;

}