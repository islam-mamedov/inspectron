cmake_minimum_required(VERSION 3.16)
project(inspectron_motion_controller LANGUAGES CXX)

option(
  INSPECTRON_ROS_WARNINGS_AS_ERRORS
  "Treat compiler warnings as errors"
  OFF
)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(inspectron_mission_msgs REQUIRED)
find_package(inspectron_safety_supervisor REQUIRED)
find_package(rclcpp REQUIRED)

function(inspectron_enable_warnings target)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(
      ${target}
      PRIVATE
        -Wall
        -Wextra
        -Wpedantic
    )

    if(INSPECTRON_ROS_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE -Werror)
    endif()
  elseif(MSVC)
    target_compile_options(${target} PRIVATE /W4)

    if(INSPECTRON_ROS_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE /WX)
    endif()
  endif()
endfunction()

add_library(
  motion_policy
  src/motion_policy.cpp
)

target_compile_features(motion_policy PUBLIC cxx_std_17)

target_include_directories(
  motion_policy
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

inspectron_enable_warnings(motion_policy)

add_executable(
  motion_controller_node
  src/motion_controller_node.cpp
)

target_compile_features(motion_controller_node PRIVATE cxx_std_17)
target_link_libraries(motion_controller_node motion_policy)

ament_target_dependencies(
  motion_controller_node
  builtin_interfaces
  geometry_msgs
  inspectron_mission_msgs
  inspectron_safety_supervisor
  rclcpp
)

inspectron_enable_warnings(motion_controller_node)

install(
  TARGETS
    motion_policy
    motion_controller_node
  EXPORT export_inspectron_motion_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(
  export_inspectron_motion_controller
  HAS_LIBRARY_TARGET
)

ament_export_include_directories(include)

ament_export_dependencies(
  builtin_interfaces
  geometry_msgs
  inspectron_mission_msgs
  inspectron_safety_supervisor
  rclcpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_gtest(
    test_motion_policy
    test/test_motion_policy.cpp
  )

  target_link_libraries(test_motion_policy motion_policy)
  inspectron_enable_warnings(test_motion_policy)

  add_launch_test(
    test/test_motion_controller_launch.py
    TIMEOUT 30
  )
endif()

ament_package()
