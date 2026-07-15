cmake_minimum_required(VERSION 3.20)

project(
  inspectron_safety
  VERSION 0.1.0
  DESCRIPTION "Deterministic C++ safety policy for Inspectron"
  LANGUAGES CXX
)

option(INSPECTRON_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" ON)

add_library(inspectron_safety STATIC
  src/safety_policy.cpp
)

add_library(inspectron::safety ALIAS inspectron_safety)

target_include_directories(
  inspectron_safety
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(inspectron_safety PUBLIC cxx_std_20)

if(MSVC)
  target_compile_options(inspectron_safety PRIVATE /W4)
  if(INSPECTRON_WARNINGS_AS_ERRORS)
    target_compile_options(inspectron_safety PRIVATE /WX)
  endif()
else()
  target_compile_options(
    inspectron_safety
    PRIVATE
      -Wall
      -Wextra
      -Wpedantic
      -Wconversion
      -Wsign-conversion
  )
  if(INSPECTRON_WARNINGS_AS_ERRORS)
    target_compile_options(inspectron_safety PRIVATE -Werror)
  endif()
endif()

include(CTest)

if(BUILD_TESTING)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  add_executable(inspectron_safety_tests tests/test_safety_policy.cpp)
  target_link_libraries(inspectron_safety_tests PRIVATE inspectron::safety GTest::gtest_main)
  target_compile_features(inspectron_safety_tests PRIVATE cxx_std_20)

  include(GoogleTest)
  gtest_discover_tests(inspectron_safety_tests)
endif()
