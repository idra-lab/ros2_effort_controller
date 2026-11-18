cmake_minimum_required(VERSION 3.5)
project(force_impedance_controller)

set(CMAKE_CXX_STANDARD 17)
set(ADDITIONAL_COMPILE_OPTIONS -Wall -Wextra -Wpedantic -Wno-unused-parameter)

# Optional parameter to enable logging, only if MatLogger2 is found
# Refer to https://github.com/ADVRHumanoids/MatLogger2 to install MatLogger2
# output file is /tmp/force_impedance_log<timestamp>.mat
option(LOGGING "Enable logging (allocates in update; disable for experiments)" OFF)
add_compile_options(${ADDITIONAL_COMPILE_OPTIONS})

# Build in release mode if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
add_definitions(-DEIGEN_MPL2_ONLY)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(effort_controller_base REQUIRED)
find_package(debug_msg REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)

# Try to find MatLogger2 if logging is enabled
if (LOGGING)
  find_package(matlogger2 QUIET)
  if (matlogger2_FOUND)
    message(STATUS "MatLogger2 found: Logging enabled")
    set(LOGGING_ENABLED TRUE)
  else()
    message(WARNING "MatLogger2 not found: Logging disabled")
    set(LOGGING_ENABLED FALSE)
  endif()
else()
  set(LOGGING_ENABLED FALSE)
endif()

# Convenience variable for dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
        rclcpp
        realtime_tools
        effort_controller_base
        Eigen3
        debug_msg
        geometry_msgs
        std_msgs
        pluginlib
)

if (LOGGING_ENABLED)
  list(APPEND THIS_PACKAGE_INCLUDE_DEPENDS matlogger2)
  add_compile_definitions(LOGGING)
endif()

ament_export_dependencies(
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

#--------------------------------------------------------------------------------
# Libraries
#--------------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/force_impedance_controller.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME}
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

if (LOGGING_ENABLED)
  target_link_libraries(${PROJECT_NAME} matlogger2::matlogger2)
endif()

#--------------------------------------------------------------------------------
# Install and export
#--------------------------------------------------------------------------------

pluginlib_export_plugin_description_file(controller_interface force_impedance_controller_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  find_package(cartesian_impedance_controller REQUIRED)
  add_executable(controller_lifecycle_test test/controller_lifecycle_test.cpp)
  target_link_libraries(controller_lifecycle_test ${PROJECT_NAME})
  ament_target_dependencies(controller_lifecycle_test ${THIS_PACKAGE_INCLUDE_DEPENDS})
  add_test(NAME controller_lifecycle_test COMMAND controller_lifecycle_test)
  add_executable(plugin_separation_test test/plugin_separation_test.cpp)
  ament_target_dependencies(plugin_separation_test controller_interface pluginlib)
  add_test(NAME plugin_separation_test COMMAND plugin_separation_test)
endif()

ament_package()
