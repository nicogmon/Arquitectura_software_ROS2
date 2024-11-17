#include <string>
#include <memory>


#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "rclcpp/rclcpp.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("bt_recepcionist_node");

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  //factory.registerFromPlugin(loader.getOSName("detected_person_node"));
  // factory.registerFromPlugin(loader.getOSName("empty_chair_bt_node"));
  factory.registerFromPlugin(loader.getOSName("br2_move_bt_node"));
  factory.registerFromPlugin(loader.getOSName("detected_person_bt_node"));
  factory.registerFromPlugin(loader.getOSName("dialogName_bt_node"));
  factory.registerFromPlugin(loader.getOSName("dialogDrink_bt_node"));
  factory.registerFromPlugin(loader.getOSName("dialogBarman_bt_node"));
  factory.registerFromPlugin(loader.getOSName("dialogGive_bt_node"));

  std::string pkgpath = ament_index_cpp::get_package_share_directory("recepcionist");
  // std::string xml_file = pkgpath + "/behavior_tree_xml/recepcionist.xml";
  std::string xml_file = pkgpath + "/behavior_tree_xml/move.xml";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  geometry_msgs::msg::PoseStamped wp;
  tf2::Quaternion myQuaternion;
  wp.header.frame_id = "map";
  wp.pose.orientation.w = 1.0;

  // inicio
  wp.pose.position.x = 0.0;
  wp.pose.position.y = 0.0;
  

  // puerta
  wp.pose.position.x = 6.4;
  wp.pose.position.y = -0.95;
  myQuaternion.setRPY(0.0,0.0,(-3.50/2));
  myQuaternion=myQuaternion.normalize();
  wp.pose.orientation.x = myQuaternion[0];
  wp.pose.orientation.y = myQuaternion[1];
  wp.pose.orientation.z = myQuaternion[2];
  wp.pose.orientation.w = myQuaternion[3];
  blackboard->set("door",wp );
  
  // silla 1
  // wp.pose.position.x = 0.0;
  // wp.pose.position.y = 0.0;
  wp.pose.position.x = 0.32;
  wp.pose.position.y = 3.43;
  myQuaternion.setRPY(0.0,0.0,(-3.1/4));
  myQuaternion=myQuaternion.normalize();
  wp.pose.orientation.x = myQuaternion[0];
  wp.pose.orientation.y = myQuaternion[1];
  wp.pose.orientation.z = myQuaternion[2];
  wp.pose.orientation.w = myQuaternion[3];
  blackboard->set("sillas",wp );

  // silla 2
  wp.pose.position.x = 0.32;
  wp.pose.position.y = 3.43;
  myQuaternion.setRPY(0.0,0.0,(-3.64/2));
  myQuaternion=myQuaternion.normalize();
  wp.pose.orientation.x = myQuaternion[0];
  wp.pose.orientation.y = myQuaternion[1];
  wp.pose.orientation.z = myQuaternion[2];
  wp.pose.orientation.w = myQuaternion[3];
  blackboard->set("barra",wp );
  
  // // silla 3
  // wp.pose.position.x = -0.217;
  // wp.pose.position.y = 2.83;
  // blackboard->set("silla",wp );
  
  // // silla 4
  // wp.pose.position.x = -0.217;
  // wp.pose.position.y = 2.83;
  // blackboard->set("silla",wp );
  
  // mesa
  wp.pose.position.x = 0.248;
  wp.pose.position.y = 6.64;
  myQuaternion.setRPY(0.0,0.0,(3.1/2));
  myQuaternion=myQuaternion.normalize();
  wp.pose.orientation.x = myQuaternion[0];
  wp.pose.orientation.y = myQuaternion[1];
  wp.pose.orientation.z = myQuaternion[2];
  wp.pose.orientation.w = myQuaternion[3];
  blackboard->set("mesa", wp);
  

  auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 1666, 1667);

  rclcpp::Rate rate(10);

  bool finish = false;
  while (!finish && rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() != BT::NodeStatus::RUNNING;

    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
