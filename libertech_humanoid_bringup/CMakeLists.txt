cmake_minimum_required(VERSION 3.8)
project(libertech_humanoid_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  ament_cmake_python
  Boost
  control_msgs
  Eigen3
  interactive_markers
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_servo
  # moveit_task_constructor_core
  moveit_visual_tools
  pluginlib
  rclcpp
  rclcpp_action
  rviz_visual_tools
  tf2_geometry_msgs
  tf2_geometry_msgs
  tf2_ros
  tf2
)

# find dependencies
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
    include
    ${THIS_PACKAGE_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(move_group_interface_direct
    src/move_group_interface_direct.cpp)
target_include_directories(move_group_interface_direct
    PUBLIC include)
ament_target_dependencies(move_group_interface_direct
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

add_executable(move_group_interface_joint
    src/move_group_interface_joint.cpp)
target_include_directories(move_group_interface_joint
    PUBLIC include)
ament_target_dependencies(move_group_interface_joint
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

add_executable(move_group_interface_cartesian
    src/move_group_interface_cartesian.cpp)
target_include_directories(move_group_interface_cartesian
    PUBLIC include)
ament_target_dependencies(move_group_interface_cartesian
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

add_executable(move_group_interface_direct_dual
    src/move_group_interface_direct_dual.cpp)
target_include_directories(move_group_interface_direct_dual
    PUBLIC include)
ament_target_dependencies(move_group_interface_direct_dual
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)


install(TARGETS 
move_group_interface_direct
move_group_interface_joint
move_group_interface_cartesian
move_group_interface_direct_dual
DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/test_huge_joint_space_jump.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  config
  world
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

ament_package()
