#!/bin/bash
colcon build
source install/setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh 
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
