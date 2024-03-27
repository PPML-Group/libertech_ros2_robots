cmake_minimum_required(VERSION 3.8)
project(libertech_hand_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(Python3 REQUIRED Development)
find_package(Python3 REQUIRED Interpreter)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
 
# Install Python executables
install(PROGRAMS
  scripts/test_ik.py
  scripts/hand_ros_driver.py
  scripts/hand_demo.py
  scripts/hand_retarget_mediapipe.py

  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # # the following line skips the linter which checks for copyrights
  # # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # # the following line skips cpplint (only works in a git repo)
  # # comment the line when this package is in a git repo and when
  # # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)

  set(_pytest_tests
    tests/test_hand_model.py
    # tests/test_b.py
    # Add other test files here
  )
  foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 60
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
  endforeach()

  # ament_add_pytest_test(test_hand_model ${CMAKE_CURRENT_SOURCE_DIR}/test/test_hand_model.py)

endif()

ament_package()

