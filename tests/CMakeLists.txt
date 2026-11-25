ament_add_gtest(test_plugin_loading test_plugin_loading.cpp test_gradient_planner.cpp)
if(TARGET test_plugin_loading)
  # NavMap exports Eigen plugin macros that require Grid Map headers.
  # Match the include paths used by the production planner without linking it directly.
  target_include_directories(test_plugin_loading PRIVATE
    ${grid_map_ros_INCLUDE_DIRS}
  )
  target_link_libraries(test_plugin_loading
    easynav_core::easynav_core
    pluginlib::pluginlib
    easynav_common::easynav_common
    navmap_core::navmap_core
    ${nav_msgs_TARGETS}
    ${rclcpp_TARGETS}
    ${grid_map_ros_LIBRARIES}
    ${grid_map_msgs_TARGETS}
  )
  add_dependencies(test_plugin_loading ${PROJECT_NAME})
endif()
