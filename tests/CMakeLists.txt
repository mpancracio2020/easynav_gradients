ament_add_gtest(test_plugin_loading test_plugin_loading.cpp)
if(TARGET test_plugin_loading)
  target_link_libraries(test_plugin_loading
    easynav_core::easynav_core
    pluginlib::pluginlib
    ${rclcpp_TARGETS}
  )
  add_dependencies(test_plugin_loading ${PROJECT_NAME})
endif()
