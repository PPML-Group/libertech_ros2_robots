cmake_minimum_required(VERSION 3.8)
project(libertech_arm_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
)
link_directories(libs)

# 外部导出给其他功能包用
add_library(arm_driver SHARED
  src/ArmDriver.cpp
)
# ament_target_dependencies(arm_driver 
#   rclcpp 
# )

add_executable(demo_arm_driver src/main.cpp)
# ament_target_dependencies(demo_arm_driver 
#   rclcpp 
# )
target_link_libraries(demo_arm_driver arm_driver controlcan.so pthread)

add_executable(test_hex src/test_hex.cpp)

install(TARGETS
  arm_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  demo_arm_driver
  test_hex
  DESTINATION lib/${PROJECT_NAME}
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
