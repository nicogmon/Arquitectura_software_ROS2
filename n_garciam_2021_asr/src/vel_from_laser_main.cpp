#include <memory>
#include "n_garciam_2021_asr/vel_from_laser.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node_vel_pub = std::make_shared<n_garciam_2021_asr::VelFromLaserNode>();

  rclcpp::spin(node_vel_pub);

  rclcpp::shutdown();
  return 0;

}