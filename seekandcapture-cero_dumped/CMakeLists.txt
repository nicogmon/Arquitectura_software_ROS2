cmake_minimum_required(VERSION 3.8)
project(seekandcapture)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(action_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(darknet_ros_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(image_geometry REQUIRED)
find_package(depth_image_proc REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)



find_package(ZMQ)
if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found.")
    add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

set(dependencies
kobuki_ros_interfaces
rclcpp
sensor_msgs
vision_msgs
darknet_ros_msgs
image_transport
cv_bridge
OpenCV
message_filters
pcl_conversions
pcl_ros
image_geometry
depth_image_proc
tf2_geometry_msgs
tf2_ros
geometry_msgs
rclcpp
rclcpp_lifecycle
rclcpp_action
behaviortree_cpp_v3
action_msgs
lifecycle_msgs
geometry_msgs
ament_index_cpp

)

include_directories(include)


add_library(${PROJECT_NAME} SHARED
  src/seekandcapture/TfDetectionNode.cpp
  src/seekandcapture/DarknetDetectionNode.cpp
  src/seekandcapture/DetectionTo3DfromDepthNode.cpp
  src/seekandcapture/PursuePersonNode.cpp
  src/seekandcapture/PIDController.cpp
  src/seekandcapture/DetectedPersonNode.cpp
  src/seekandcapture/AvoidPersonNode.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

include_directories(include ${ZMQ_INCLUDE_DIRS})

add_library(detected_person_node SHARED src/seekandcapture/DetectedPersonNode.cpp)
add_library(pursue_person_node SHARED src/seekandcapture/PursuePersonNode.cpp)
add_library(avoid_person_node SHARED src/seekandcapture/AvoidPersonNode.cpp)
target_link_libraries(pursue_person_node ${PROJECT_NAME})

list(APPEND plugin_libs
  
  detected_person_node
  pursue_person_node
  avoid_person_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()


add_executable(darknet_detection src/darknet_detection_main.cpp)
ament_target_dependencies(darknet_detection ${dependencies})
target_link_libraries(darknet_detection ${PROJECT_NAME})

add_executable(detection_2d_to_3d_depth src/detection_2d_to_3d_depth_main.cpp)
ament_target_dependencies(detection_2d_to_3d_depth ${dependencies})
target_link_libraries(detection_2d_to_3d_depth ${PROJECT_NAME})

add_executable(tf_detection src/tf_detection_main.cpp)
ament_target_dependencies(tf_detection ${dependencies})
target_link_libraries(tf_detection ${PROJECT_NAME})

add_executable(bt_perception src/bt_perception_main.cpp)
ament_target_dependencies(bt_perception ${dependencies})
target_link_libraries(bt_perception ${PROJECT_NAME})




install(TARGETS
  ${PROJECT_NAME}
  tf_detection
  darknet_detection
  detection_2d_to_3d_depth
  bt_perception
  detected_person_node
  pursue_person_node
  avoid_person_node
  
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY behavior_tree_xml
  DESTINATION share/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
