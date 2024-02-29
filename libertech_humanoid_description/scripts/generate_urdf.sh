#!/bin/bash
# generate_urdf.sh

xacro_file="$1"
output_file="$2"

# Run xacro to generate the URDF
ros2 run xacro xacro "$xacro_file" > "$output_file"
# ros2 run xacro xacro /home/cyz/libertech_robot/src/libertech_ros2_robots/libertech_humanoid_description/urdf/robot.xacro > /home/cyz/libertech_robot/src/libertech_ros2_robots/libertech_humanoid_description/urdf/robot.urdf

