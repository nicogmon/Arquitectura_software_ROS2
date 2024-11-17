cmake_minimum_required(VERSION 3.5)
project(recepcionist)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(action_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(gb_dialog REQUIRED)
find_package(dialogflow_ros2_interfaces REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)

find_package(ZMQ)
if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found.")
    add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_lifecycle
    rclcpp_action
    behaviortree_cpp_v3
    action_msgs
    lifecycle_msgs
    geometry_msgs
    nav2_msgs
    ament_index_cpp
    tf2_geometry_msgs
    tf2_ros
    vision_msgs
    gb_dialog
    dialogflow_ros2_interfaces
    kobuki_ros_interfaces
)

include_directories(include ${ZMQ_INCLUDE_DIRS})

add_library(br2_move_bt_node SHARED src/recepcionist/Move.cpp)
add_library(detected_person_bt_node SHARED src/recepcionist/DetectedPersonNode.cpp)
add_library(empty_chair_bt_node SHARED src/recepcionist/EmptyChair.cpp)
add_library(dialogName_bt_node SHARED src/recepcionist/DialogName.cpp)
add_library(dialogDrink_bt_node SHARED src/recepcionist/DialogDrink.cpp)
add_library(dialogBarman_bt_node SHARED src/recepcionist/DialogBarman.cpp)
add_library(dialogGive_bt_node SHARED src/recepcionist/GiveDrink.cpp)

list(APPEND plugin_libs
  br2_move_bt_node
  empty_chair_bt_node
  detected_person_bt_node
  dialogName_bt_node
  dialogDrink_bt_node
  dialogBarman_bt_node
  dialogGive_bt_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(patrolling_main src/patrolling_main.cpp)
ament_target_dependencies(patrolling_main ${dependencies})
target_link_libraries(patrolling_main ${ZMQ_LIBRARIES})

add_executable(bt_recepcionist_main src/bt_recepcionist_main.cpp)
ament_target_dependencies(bt_recepcionist_main ${dependencies})
target_link_libraries(bt_recepcionist_main ${ZMQ_LIBRARIES})


install(TARGETS
  ${plugin_libs}
  bt_recepcionist_main
  patrolling_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY behavior_tree_xml launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
