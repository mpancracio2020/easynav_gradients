// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <string>

#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include "easynav_core/PlannerMethodBase.hpp"

namespace {
constexpr char kPlugin[] = "easynav_navmap_gradients/GradientPlanner";

class PluginLoadingTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(PluginLoadingTest, DiscoversRegisteredPlanner) {
  pluginlib::ClassLoader<easynav::PlannerMethodBase> loader(
      "easynav_core", "easynav::PlannerMethodBase");
  const auto classes = loader.getDeclaredClasses();
  EXPECT_NE(std::find(classes.begin(), classes.end(), kPlugin), classes.end());
  EXPECT_TRUE(loader.isClassAvailable(kPlugin));
}

TEST_F(PluginLoadingTest, InstantiatesPlanner) {
  pluginlib::ClassLoader<easynav::PlannerMethodBase> loader(
      "easynav_core", "easynav::PlannerMethodBase");
  ASSERT_TRUE(loader.isClassAvailable(kPlugin));
  // Keep the instance's lifetime shorter than the loader's lifetime.
  const auto planner = loader.createSharedInstance(kPlugin);
  ASSERT_NE(planner, nullptr);
}
}  // namespace
