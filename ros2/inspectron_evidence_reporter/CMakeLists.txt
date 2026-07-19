cmake_minimum_required(VERSION 3.16)
project(inspectron_evidence_reporter)

option(
  INSPECTRON_ROS_WARNINGS_AS_ERRORS
  "Accepted for consistency with the Inspectron ROS 2 stack"
  OFF
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(inspectron_evidence_msgs REQUIRED)
find_package(inspectron_mission_msgs REQUIRED)
find_package(inspectron_safety_supervisor REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_srvs REQUIRED)

ament_python_install_package(inspectron_evidence_reporter)

install(
  PROGRAMS scripts/evidence_reporter_node
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
    test_reporting
    test/test_reporting.py
    TIMEOUT 60
  )

  add_launch_test(
    test/test_evidence_reporter_launch.py
    TIMEOUT 40
  )
endif()

ament_package()
