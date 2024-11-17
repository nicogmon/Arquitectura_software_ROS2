cmake_minimum_required(VERSION 3.8)
project(vel_pub_sub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies
  kobuki_ros_interfaces
  rclcpp
  geometry_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/vel_pub_sub/vel_pub.cpp
  src/vel_pub_sub/vel_sub.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(vel_pub src/vel_pub_main.cpp)
ament_target_dependencies(vel_pub ${dependencies})
target_link_libraries(vel_pub ${PROJECT_NAME})

add_executable(vel_sub src/vel_sub_main.cpp)
ament_target_dependencies(vel_sub ${dependencies})
target_link_libraries(vel_sub ${PROJECT_NAME})

install(TARGETS
  vel_pub
  vel_sub
  
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
