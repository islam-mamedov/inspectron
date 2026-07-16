cmake_minimum_required(VERSION 3.16)
project(inspectron_perception_bridge)

option(
  INSPECTRON_ROS_WARNINGS_AS_ERRORS
  "Accepted for consistency with the ROS 2 control-stack build"
  OFF
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(inspectron_safety_supervisor REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_python_install_package(inspectron_perception_bridge)

ament_python_install_package(
  inspectron
  PACKAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../src/inspectron"
)

install(
  PROGRAMS scripts/perception_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_pytest_test(
    test_conversion
    test/test_conversion.py
    TIMEOUT 60
  )

  add_launch_test(
    test/test_perception_bridge_launch.py
    TIMEOUT 30
  )
endif()

ament_package()
