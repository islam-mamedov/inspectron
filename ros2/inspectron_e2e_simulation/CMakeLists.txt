cmake_minimum_required(VERSION 3.16)
project(inspectron_e2e_simulation)

option(
  INSPECTRON_ROS_WARNINGS_AS_ERRORS
  "Accepted for consistency with the Inspectron ROS 2 stack"
  OFF
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(inspectron_evidence_msgs REQUIRED)
find_package(inspectron_mission_msgs REQUIRED)
find_package(inspectron_safety_supervisor REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

ament_python_install_package(inspectron_e2e_simulation)

install(
  PROGRAMS scripts/scenario_simulator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_pytest_test(
    test_scenario_logic
    test/test_scenario_logic.py
    TIMEOUT 60
  )

  ament_add_pytest_test(
    test_fault_logic
    test/test_fault_logic.py
    TIMEOUT 60
  )

  ament_add_pytest_test(
    test_scenarios
    test/test_scenarios.py
    TIMEOUT 60
  )

  add_launch_test(
    test/test_e2e_safe_mission_launch.py
    TIMEOUT 180
  )

  add_launch_test(
    test/test_e2e_hazard_stop_launch.py
    TIMEOUT 120
  )

  add_launch_test(
    test/test_fault_watchdog_orchestrator_launch.py
    TIMEOUT 120
  )

  add_launch_test(
    test/test_fault_watchdog_supervisor_launch.py
    TIMEOUT 120
  )

  add_launch_test(
    test/test_fault_recovery_launch.py
    TIMEOUT 180
  )

  add_launch_test(
    test/test_fault_authority_launch.py
    TIMEOUT 180
  )
endif()

ament_package()
