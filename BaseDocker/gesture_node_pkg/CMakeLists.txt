cmake_minimum_required(VERSION 3.8)
project(gesture_node_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(rosidl_default_generators REQUIRED)

# Generate ROS 2 interfaces
#rosidl_generate_interfaces(${PROJECT_NAME}
#  "msg/Gesture.msg"
#)


# Add the executable
add_executable(gesture_node src/gesture_node.cpp)
ament_target_dependencies(gesture_node rclcpp std_msgs)
#rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
#target_link_libraries(gesture_node ${typesupport_target})

# Install the executable
install(TARGETS gesture_node
  DESTINATION lib/${PROJECT_NAME})

# Enable testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Mark the package for ament
ament_package()
