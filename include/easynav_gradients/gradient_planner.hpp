// Copyright 2025 Intelligent Robotics Lab (original EasyNav A* planner)
// Copyright 2026 Marvin Pancracio Manso (gradient planner modifications)
//
// Adapted from the EasyNav NavMap A* planner.
// Modified to implement gradient-based planning, caching and Elastic Band smoothing.
// Upstream attribution: https://github.com/EasyNavigation/easynav_plugins
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

/// \file
/// \brief Declaration of GradientPlanner: gradient-based path planner using
/// NavMap.

#ifndef EASYNAV_NAVMAP_PLANNER__GRADIENT_PLANNER_HPP_
#define EASYNAV_NAVMAP_PLANNER__GRADIENT_PLANNER_HPP_

#include <cstddef>
#include <fstream>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_core/PlannerMethodBase.hpp"
#include "navmap_core/NavMap.hpp"
#include <navmap_ros_interfaces/msg/nav_map_layer.hpp>

namespace easynav {
namespace navmap {

/// \brief Planner based on gradient descent using NavMap navigation data.
class GradientPlanner : public PlannerMethodBase {
public:
  explicit GradientPlanner();
  ~GradientPlanner();

  void on_initialize() override;
  void update(NavState &nav_state) override;

protected:
  // ── Parameters ──────────────────────────────────────────────────────────
  bool continuous_replan_{true};
  int max_iterations_{500};
  double smoothing_factor_{0.4}; ///< Gradient field smoothing
  double robot_width_{0.0};

  // Change detection.
  double goal_change_epsilon_{1e-3};

  // Neighbor selection.
  double potential_drop_weight_{0.75};

  // Elastic Band.
  double elastic_w_data_{0.10};
  double elastic_w_smooth_{0.35};
  int elastic_max_iterations_{15};
  double elastic_convergence_threshold_{2e-3};

  // ── Planner state ───────────────────────────────────────────────────────
  nav_msgs::msg::Path current_path_;
  geometry_msgs::msg::Pose current_goal_;
  ::navmap::NavMap navmap_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<navmap_ros_interfaces::msg::NavMapLayer>::SharedPtr
      potential_layer_pub_;

  // ── Internal structures ─────────────────────────────────────────────────
  std::vector<double> potentials_;
  std::vector<Eigen::Vector2d> gradients_;

  // ── NavMap-derived caches ───────────────────────────────────────────────
  // Raw neighbor topology.
  mutable std::vector<std::vector<::navmap::NavCelId>> neighbors_cache_;
  mutable bool neighbors_cached_ = false;

  // Static geometry and transitions satisfying robot_width_.
  std::vector<Eigen::Vector3f> centroids_cache_;
  std::vector<std::vector<::navmap::NavCelId>> traversable_neighbors_cache_;
  bool geometry_cached_{false};

  // ── NavMap signature ────────────────────────────────────────────────────
  // Allows cache invalidation when geometry or topology changes,
  // even if the number of NavCels remains unchanged.
  std::size_t navmap_signature_{0};
  bool navmap_signature_initialized_{false};

  // ── Internal methods ────────────────────────────────────────────────────

  /// \brief Full planning: potential field, gradient field and descent.
  std::vector<geometry_msgs::msg::Pose>
  compute_gradient_path(::navmap::NavMap &navmap,
                        const geometry_msgs::msg::Pose &start,
                        const geometry_msgs::msg::Pose &goal);

  /// \brief Propagate the potential field from the goal using Dijkstra.
  void compute_potential_field(::navmap::NavMap &navmap,
                               const geometry_msgs::msg::Pose &goal);

  /// \brief Compute the gradient field from the potential field.
  void compute_gradient_field(const ::navmap::NavMap &navmap);

  /// \brief Smooth the gradient field by averaging neighboring gradients.
  void smooth_gradients(const ::navmap::NavMap &navmap);

  /// \brief Compute the gradient vector for a given cell.
  Eigen::Vector2d
  compute_gradient_at_cell(::navmap::NavCelId cell_id,
                           const std::vector<double> &potentials,
                           const ::navmap::NavMap &navmap) const;

  /// \brief Locate the NavCel containing a given point.
  /// \param allow_closest_fallback If true, use closest_navcel() when the
  /// point is not contained in any NavCel.
  ::navmap::NavCelId locate_cell(const ::navmap::NavMap &navmap,
                                 const geometry_msgs::msg::Point &point,
                                 bool allow_closest_fallback) const;

  /// \brief Return the cached neighbors of a cell.
  /// \note Returns a const reference; avoid copying the vector.
  const std::vector<::navmap::NavCelId> &
  get_neighbors(const ::navmap::NavMap &navmap,
                ::navmap::NavCelId cell_id) const;

  /// \brief Build the complete neighbor cache for the current NavMap.
  void build_neighbors_cache(const ::navmap::NavMap &navmap) const;

  /// \brief Precompute centroids and traversable neighbors for the current NavMap.
  void build_geometry_cache(const ::navmap::NavMap &navmap);

  /// \brief Length of the shared edge between two NavCels.
  /// \return Length in meters; 0.0 if the cells do not share exactly one edge.
  double shared_edge_width(::navmap::NavCelId cell_a, ::navmap::NavCelId cell_b,
                           const ::navmap::NavMap &navmap) const;

  /// \brief Check whether the robot can traverse the shared edge between
  /// two NavCels.
  bool is_transition_traversable(::navmap::NavCelId from, ::navmap::NavCelId to,
                                 const ::navmap::NavMap &navmap) const;

  /// \brief Return the traversable neighbor with the lowest potential below
  ///               that of the current cell. Return current_cell if none exists.
  ::navmap::NavCelId
  find_lower_potential_neighbor(::navmap::NavCelId current_cell,
                                const ::navmap::NavMap &navmap) const;

  /// \brief Create a Pose from the centroid of a cell.
  geometry_msgs::msg::Pose create_pose_from_cell(
      ::navmap::NavCelId cell_id, const ::navmap::NavMap &navmap,
      const geometry_msgs::msg::Quaternion &orientation) const;

  /// \brief Reset the current path without invalidating cached fields.
  ///               Cached potential and gradient fields are not modified.
  void reset_path_state();

  /// \brief Apply Elastic Band smoothing to the generated path.
  void elastic_band_smoothing(nav_msgs::msg::Path &path);

  /// \brief Report diagnostics for a stuck cell.
  void debug_stuck_cell(::navmap::NavCelId cell_id,
                        const ::navmap::NavMap &navmap);

  std::ofstream metrics_file_;

  // ── Potential and gradient cache ────────────────────────────────────────
  /// \brief Indicates whether the potential and gradient fields are valid
  ///               for the current goal and map.
  bool potential_field_valid_{false};

  /// \brief Indicates whether at least one goal has been received.
  bool goal_initialized_{false};

  std::vector<::navmap::NavCelId> current_path_cells_;
};

} // namespace navmap
} // namespace easynav

#endif // EASYNAV_NAVMAP_PLANNER__GRADIENT_PLANNER_HPP_