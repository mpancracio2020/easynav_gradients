cmake_minimum_required(VERSION 3.20)
project(easynav_gradients)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(easynav_common REQUIRED)
find_package(easynav_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(navmap_core REQUIRED)
find_package(navmap_ros REQUIRED)
find_package(navmap_ros_interfaces REQUIRED)

# Gradient planner library
add_library(${PROJECT_NAME} SHARED
  src/gradient_planner.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${grid_map_ros_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  easynav_common::easynav_common
  easynav_core::easynav_core
  navmap_core::navmap_core
  navmap_ros::navmap_ros
  pluginlib::pluginlib
  ${navmap_ros_interfaces_TARGETS}
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${rclcpp_TARGETS}
  ${grid_map_msgs_TARGETS}
  ${grid_map_ros_LIBRARIES}
)

# Example test node
add_executable(simple_test
  src/nodes/simple_test.cpp
)

ament_target_dependencies(simple_test
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
)

install(TARGETS simple_test
  DESTINATION lib/${PROJECT_NAME}
  
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(tests)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

# Install configuration and plugin metadata
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(FILES
  easynav_gradients_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)


pluginlib_export_plugin_description_file(easynav_core easynav_gradients_plugins.xml)

ament_export_dependencies(
  easynav_common
  easynav_core
  pluginlib
  nav_msgs
  geometry_msgs
  visualization_msgs
  rclcpp
  grid_map_ros
  grid_map_msgs
  navmap_core
  navmap_ros
  navmap_ros_interfaces
)

ament_package()