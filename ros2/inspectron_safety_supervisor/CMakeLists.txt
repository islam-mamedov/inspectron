cmake_minimum_required(VERSION 3.16)

project(inspectron_safety_supervisor VERSION 0.1.0 LANGUAGES C CXX)

option(
  INSPECTRON_ROS_WARNINGS_AS_ERRORS
  "Treat warnings in Inspectron ROS 2 targets as errors"
  ON
)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SceneAssessment.msg"
  "msg/PolicyDecision.msg"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

get_filename_component(
  INSPECTRON_REPOSITORY_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../.."
  ABSOLUTE
)

set(
  INSPECTRON_CORE_SOURCE
  "${INSPECTRON_REPOSITORY_ROOT}/cpp/src/safety_policy.cpp"
)
set(
  INSPECTRON_CORE_INCLUDE
  "${INSPECTRON_REPOSITORY_ROOT}/cpp/include"
)

if(NOT EXISTS "${INSPECTRON_CORE_SOURCE}")
  message(FATAL_ERROR "Inspectron C++ safety core was not found")
endif()

function(inspectron_enable_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4)
    if(INSPECTRON_ROS_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE /WX)
    endif()
  else()
    target_compile_options(
      ${target}
      PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Wconversion
        -Wsign-conversion
    )
    if(INSPECTRON_ROS_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE -Werror)
    endif()
  endif()
endfunction()

add_library(inspectron_safety_core STATIC "${INSPECTRON_CORE_SOURCE}")
target_include_directories(
  inspectron_safety_core
  PUBLIC "${INSPECTRON_CORE_INCLUDE}"
)
target_compile_features(inspectron_safety_core PUBLIC cxx_std_20)
inspectron_enable_warnings(inspectron_safety_core)

rosidl_get_typesupport_target(
  inspectron_typesupport_cpp
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

add_executable(
  safety_supervisor_node
  src/safety_supervisor_node.cpp
)
target_include_directories(
  safety_supervisor_node
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_compile_features(safety_supervisor_node PRIVATE cxx_std_20)
target_link_libraries(
  safety_supervisor_node
  inspectron_safety_core
  "${inspectron_typesupport_cpp}"
)
ament_target_dependencies(safety_supervisor_node rclcpp)
inspectron_enable_warnings(safety_supervisor_node)

install(
  TARGETS safety_supervisor_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(
    test_message_conversion
    test/test_message_conversion.cpp
  )
  if(TARGET test_message_conversion)
    target_include_directories(
      test_message_conversion
      PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
    )
    target_compile_features(test_message_conversion PRIVATE cxx_std_20)
    target_link_libraries(
      test_message_conversion
      inspectron_safety_core
      "${inspectron_typesupport_cpp}"
    )
    inspectron_enable_warnings(test_message_conversion)
  endif()
endif()

ament_package()
