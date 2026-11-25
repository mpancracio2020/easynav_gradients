// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <memory>

#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav_msgs/msg/goals.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include "easynav_core/PlannerMethodBase.hpp"
#include "navmap_core/NavMap.hpp"

namespace {
// Three unit squares, each split into two triangles. Removing the middle
// square disconnects the two ends without moving either endpoint.
navmap::NavMap make_map(bool disconnected = false) {
  navmap::NavMap map;
  const auto surface = map.create_surface("map");
  for (int x = 0; x <= 3; ++x) {
    map.add_vertex(Eigen::Vector3f(static_cast<float>(x), 0.0f, 0.0f));
    map.add_vertex(Eigen::Vector3f(static_cast<float>(x), 1.0f, 0.0f));
  }
  for (unsigned x = 0; x < 3; ++x) {
    if (disconnected && x == 1) {
      continue;
    }
    const auto a = 2 * x;
    map.add_navcel_to_surface(surface, map.add_navcel(a, a + 2, a + 3));
    map.add_navcel_to_surface(surface, map.add_navcel(a, a + 3, a + 1));
  }
  map.rebuild_geometry_accels();
  return map;
}

class GradientPlannerTest : public ::testing::Test {
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

  void SetUp() override {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("gradients.continuous_replan", false),
      rclcpp::Parameter("gradients.robot_width", 0.8)
    });
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "gradient_planner_test", options);
    loader_ = std::make_unique<pluginlib::ClassLoader<easynav::PlannerMethodBase>>(
      "easynav_core", "easynav::PlannerMethodBase");
    planner_ = loader_->createSharedInstance("easynav_navmap_gradients/GradientPlanner");
    planner_->initialize(node_, "gradients");

    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "map";
    odom.pose.pose.position.x = 0.2;
    odom.pose.pose.position.y = 0.3;
    odom.pose.pose.orientation.w = 1.0;
    state_.set("robot_pose", odom);
    goals_.header.frame_id = "map";
    goals_.goals.resize(1);
    goals_.goals.front().pose.position.x = 2.8;
    goals_.goals.front().pose.position.y = 0.3;
    goals_.goals.front().pose.orientation.w = 1.0;
    state_.set("goals", goals_);
    state_.set("map.navmap", make_map());
  }

  void TearDown() override {
    planner_.reset();
    loader_.reset();
    node_.reset();
  }

  nav_msgs::msg::Path plan() {
    planner_->force_update(state_);
    return state_.get<nav_msgs::msg::Path>("path");
  }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::unique_ptr<pluginlib::ClassLoader<easynav::PlannerMethodBase>> loader_;
  std::shared_ptr<easynav::PlannerMethodBase> planner_;
  easynav::NavState state_;
  nav_msgs::msg::Goals goals_;
};

TEST_F(GradientPlannerTest, InitializesParameters) {
  EXPECT_FALSE(node_->get_parameter("gradients.continuous_replan").as_bool());
  EXPECT_DOUBLE_EQ(node_->get_parameter("gradients.robot_width").as_double(), 0.8);
  EXPECT_EQ(node_->get_parameter("gradients.max_iterations").as_int(), 500);
}

TEST_F(GradientPlannerTest, ReachesGoalOnConnectedMesh) {
  const auto path = plan();
  ASSERT_GE(path.poses.size(), 2u);
  EXPECT_EQ(path.header.frame_id, "map");
  EXPECT_NEAR(path.poses.front().pose.position.x, 0.2, 1e-5);
  EXPECT_NEAR(path.poses.front().pose.position.y, 0.3, 1e-5);
  EXPECT_NEAR(path.poses.back().pose.position.x, 2.8, 1e-5);
  EXPECT_NEAR(path.poses.back().pose.position.y, 0.3, 1e-5);
  for (const auto & waypoint : path.poses) {
    EXPECT_TRUE(std::isfinite(waypoint.pose.position.x));
    EXPECT_GE(waypoint.pose.position.x, -1e-5);
    EXPECT_LE(waypoint.pose.position.x, 3.0 + 1e-5);
    EXPECT_GE(waypoint.pose.position.y, -1e-5);
    EXPECT_LE(waypoint.pose.position.y, 1.0 + 1e-5);
  }
}

TEST_F(GradientPlannerTest, RejectsDisconnectedGoal) {
  state_.set("map.navmap", make_map(true));
  EXPECT_TRUE(plan().poses.empty());
}

TEST_F(GradientPlannerTest, ReusesPathForUnchangedInputs) {
  const auto first = plan();
  ASSERT_FALSE(first.poses.empty());
  EXPECT_EQ(plan(), first);
}

TEST_F(GradientPlannerTest, InvalidatesPathAfterTopologyChange) {
  ASSERT_FALSE(plan().poses.empty());
  state_.set("map.navmap", make_map(true));
  EXPECT_TRUE(plan().poses.empty());
  state_.set("map.navmap", make_map());
  EXPECT_FALSE(plan().poses.empty());
}

TEST_F(GradientPlannerTest, PreservesChangedGoalOrientation) {
  ASSERT_FALSE(plan().poses.empty());
  auto & orientation = goals_.goals.front().pose.orientation;
  orientation.z = std::sqrt(0.5);
  orientation.w = std::sqrt(0.5);
  state_.set("goals", goals_);
  const auto path = plan();
  ASSERT_FALSE(path.poses.empty());
  EXPECT_DOUBLE_EQ(path.poses.back().pose.orientation.z, orientation.z);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.orientation.w, orientation.w);
}
}  // namespace
