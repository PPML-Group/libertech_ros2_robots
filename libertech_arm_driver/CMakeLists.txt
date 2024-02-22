cmake_minimum_required(VERSION 3.8)
project(libertech_arm_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  Eigen3
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
)
link_directories(libs)

# 外部导出给其他功能包用
add_library(arm_driver SHARED
  src/ArmDriver.cpp
)
target_link_libraries(arm_driver controlcan.so pthread)

############# ros control ####################

add_library(
  arm_hardware_interface
  SHARED
  src/hardware_interface.cpp
)
target_compile_features(arm_hardware_interface PUBLIC cxx_std_17)
# target_link_libraries(arm_hardware_interface arm_driver)
ament_target_dependencies(
  arm_hardware_interface
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(arm_hardware_interface arm_driver)

pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

#############################

add_executable(demo_arm_driver src/main.cpp)
target_link_libraries(demo_arm_driver arm_driver)

add_executable(demo_arm_calibration src/calibration.cpp)
target_link_libraries(demo_arm_calibration arm_driver)

add_executable(demo_arm_hello src/node_hello.cpp)
ament_target_dependencies(demo_arm_hello 
  rclcpp sensor_msgs
)
target_link_libraries(demo_arm_hello arm_driver)

add_executable(test_hex src/test_hex.cpp)

install(TARGETS
  arm_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install(TARGETS arm_hardware_interface
#   EXPORT export_arm_hardware_interface
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

install(TARGETS
  demo_arm_driver
  test_hex
  demo_arm_hello
  demo_arm_calibration
  DESTINATION lib/${PROJECT_NAME}
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

# ament_export_include_directories(
#   include
# )
# ament_export_targets(export_arm_hardware_interface HAS_LIBRARY_TARGET)
# ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} arm_driver)
ament_package()
