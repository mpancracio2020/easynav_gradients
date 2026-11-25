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
#include "easynav_gradients/gradient_planner.hpp"
#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "navmap_core/NavMap.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <mutex>
#include <navmap_ros_interfaces/msg/nav_map_layer.hpp>
#include <queue>
#include <sstream>
namespace easynav {
namespace navmap {
// Tipos internos
struct QueueElement {
  double cost;
  ::navmap::NavCelId cell_id;
  bool operator>(const QueueElement &other) const { return cost > other.cost; }
};
// Profiling por función
// Escribe en /tmp/gradient_planner_profile.csv una fila por cada llamada a una
// función instrumentada con PROFILE_FUNCTION():
// timestamp_us,funcion,duracion_ms El fichero se abre una única vez
// (thread-safe gracias a "magic statics" de C++11) y cada escritura va
// protegida por un mutex para evitar líneas entrelazadas si el nodo tiene
// varios hilos (callbacks, timers, etc.).
namespace {
// Internal numerical constants. These values are implementation details,
// not runtime tuning parameters.
constexpr double kZeroGradientThreshold = 1e-4;
constexpr double kGoalOrientationDotTolerance = 1e-6;
constexpr double kGeometryEpsilon = 1e-6;
constexpr double kGradientWeightEpsilon = 1e-3;
std::ofstream &profile_stream() {
  static std::ofstream file("/tmp/gradient_planner_profile.csv", std::ios::app);
  static bool header_written = []() {
    file << "timestamp_us,function,duration_ms\n";
    file.flush();
    return true;
  }();
  (void)header_written;
  return file;
}
std::mutex &profile_mutex() {
  static std::mutex m;
  return m;
}
class ScopedTimer {
public:
  explicit ScopedTimer(const char *name)
      : name_(name), t0_(std::chrono::high_resolution_clock::now()) {}
  ~ScopedTimer() {
    const auto t1 = std::chrono::high_resolution_clock::now();
    const double ms =
        std::chrono::duration<double, std::milli>(t1 - t0_).count();
    const auto us = std::chrono::duration_cast<std::chrono::microseconds>(
                        t1.time_since_epoch())
                        .count();
    std::lock_guard<std::mutex> lock(profile_mutex());
    auto &f = profile_stream();
    f << us << "," << name_ << "," << ms << "\n";
    f.flush();
  }

private:
  const char *name_;
  std::chrono::high_resolution_clock::time_point t0_;
};
// __func__ da el nombre crudo de la función (sin namespace/clase), que es
// suficiente para diferenciarlas en este fichero. Si se prefiere diferenciar
// overloads o namespaces, se puede pasar __PRETTY_FUNCTION__ en su lugar.
#define PROFILE_FUNCTION() ScopedTimer _scoped_timer_##__LINE__(__func__)
std::size_t compute_navmap_signature(const ::navmap::NavMap &navmap) {
  PROFILE_FUNCTION();
  // Lightweight 64-bit FNV-1a style signature. We hash the raw IEEE-754 bits
  // of every vertex coordinate and every triangle vertex index, avoiding the
  // repeated std::hash<float> calls used by the previous implementation.
  // The information covered by the signature is unchanged: geometry + topology.
  constexpr std::uint64_t kOffsetBasis = 1469598103934665603ULL;
  std::uint64_t hash = kOffsetBasis;
  const auto mix = [&hash](std::uint64_t value) {
    hash ^= value;
    hash *= 1099511628211ULL;
  };
  const auto float_bits = [](float value) {
    std::uint32_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value),
                  "Unexpected float representation size");
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
  };
  // Geometry: vertex count and exact coordinate bit patterns.
  const std::size_t position_count = navmap.positions.size();
  mix(static_cast<std::uint64_t>(position_count));
  for (std::size_t i = 0; i < position_count; ++i) {
    const auto &p = navmap.positions.at(i);
    mix(static_cast<std::uint64_t>(float_bits(p.x())));
    mix(static_cast<std::uint64_t>(float_bits(p.y())));
    mix(static_cast<std::uint64_t>(float_bits(p.z())));
  }
  // Topology: NavCel count and the three vertex indices of every triangle.
  const std::size_t navcel_count = navmap.navcels.size();
  mix(static_cast<std::uint64_t>(navcel_count));
  for (std::size_t i = 0; i < navcel_count; ++i) {
    const auto &cell = navmap.navcels[i];
    mix(static_cast<std::uint64_t>(cell.v[0]));
    mix(static_cast<std::uint64_t>(cell.v[1]));
    mix(static_cast<std::uint64_t>(cell.v[2]));
  }
  return static_cast<std::size_t>(hash);
}
} // namespace
// Utilidades libres
static double compute_path_length(const nav_msgs::msg::Path &path) {
  PROFILE_FUNCTION();
  double total = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto &a = path.poses[i - 1].pose.position;
    const auto &b = path.poses[i].pose.position;
    const double dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
    total += std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  return total;
}
// Constructor
GradientPlanner::GradientPlanner() {
  NavState::register_printer<nav_msgs::msg::Path>(
      [](const nav_msgs::msg::Path &path) {
        std::ostringstream ret;
        ret << "Gradient Path with " << path.poses.size()
            << " poses and length " << compute_path_length(path) << " m.";
        return ret.str();
      });
}
GradientPlanner::~GradientPlanner() {
  if (metrics_file_.is_open()) {
    metrics_file_.close();
  }
}
// on_initialize
void GradientPlanner::on_initialize() {
  PROFILE_FUNCTION();
  auto node = get_node();
  const auto &pn = get_plugin_name();
  node->declare_parameter<bool>(pn + ".continuous_replan", continuous_replan_);
  node->declare_parameter<int>(pn + ".max_iterations", max_iterations_);
  node->declare_parameter<double>(pn + ".smoothing_factor", smoothing_factor_);
  node->declare_parameter<double>(pn + ".robot_width", robot_width_);
  node->declare_parameter<double>(pn + ".goal_change_epsilon",
                                  goal_change_epsilon_);
  node->declare_parameter<double>(pn + ".potential_drop_weight",
                                  potential_drop_weight_);
  node->declare_parameter<double>(pn + ".elastic_w_data", elastic_w_data_);
  node->declare_parameter<double>(pn + ".elastic_w_smooth", elastic_w_smooth_);
  node->declare_parameter<int>(pn + ".elastic_max_iterations",
                               elastic_max_iterations_);
  node->declare_parameter<double>(pn + ".elastic_convergence_threshold",
                                  elastic_convergence_threshold_);
  node->get_parameter(pn + ".continuous_replan", continuous_replan_);
  node->get_parameter(pn + ".max_iterations", max_iterations_);
  node->get_parameter(pn + ".smoothing_factor", smoothing_factor_);
  node->get_parameter(pn + ".robot_width", robot_width_);
  node->get_parameter(pn + ".goal_change_epsilon", goal_change_epsilon_);
  node->get_parameter(pn + ".potential_drop_weight", potential_drop_weight_);
  node->get_parameter(pn + ".elastic_w_data", elastic_w_data_);
  node->get_parameter(pn + ".elastic_w_smooth", elastic_w_smooth_);
  node->get_parameter(pn + ".elastic_max_iterations", elastic_max_iterations_);
  node->get_parameter(pn + ".elastic_convergence_threshold",
                      elastic_convergence_threshold_);
  // -----------------------------------------------------------------------
  // Parameter validation.
  // Invalid values are corrected once during initialization so the planner
  // never runs with an inconsistent numerical configuration.
  // -----------------------------------------------------------------------
  if (max_iterations_ <= 0) {
    RCLCPP_WARN(node->get_logger(),
                "Invalid max_iterations=%d. Using default value 500.",
                max_iterations_);
    max_iterations_ = 500;
  }
  if (!std::isfinite(smoothing_factor_)) {
    RCLCPP_WARN(node->get_logger(),
                "Invalid smoothing_factor. Using default value 0.4.");
    smoothing_factor_ = 0.4;
  } else {
    const double validated = std::clamp(smoothing_factor_, 0.0, 1.0);
    if (validated != smoothing_factor_) {
      RCLCPP_WARN(node->get_logger(),
                  "smoothing_factor=%.6f is outside [0, 1]. Clamping to %.6f.",
                  smoothing_factor_, validated);
      smoothing_factor_ = validated;
    }
  }
  if (!std::isfinite(robot_width_) || robot_width_ < 0.0) {
    RCLCPP_WARN(
        node->get_logger(),
        "Invalid robot_width=%.6f. Using 0.0 m (width filter disabled).",
        robot_width_);
    robot_width_ = 0.0;
  }
  if (!std::isfinite(goal_change_epsilon_) || goal_change_epsilon_ <= 0.0) {
    RCLCPP_WARN(
        node->get_logger(),
        "Invalid goal_change_epsilon=%.6f. Using default value 0.001 m.",
        goal_change_epsilon_);
    goal_change_epsilon_ = 1e-3;
  }
  if (!std::isfinite(potential_drop_weight_) || potential_drop_weight_ < 0.0) {
    RCLCPP_WARN(node->get_logger(),
                "Invalid potential_drop_weight=%.6f. Using default value 0.75.",
                potential_drop_weight_);
    potential_drop_weight_ = 0.75;
  }
  if (!std::isfinite(elastic_w_data_) || elastic_w_data_ < 0.0) {
    RCLCPP_WARN(node->get_logger(),
                "Invalid elastic_w_data=%.6f. Using default value 0.10.",
                elastic_w_data_);
    elastic_w_data_ = 0.10;
  }
  if (!std::isfinite(elastic_w_smooth_) || elastic_w_smooth_ < 0.0) {
    RCLCPP_WARN(node->get_logger(),
                "Invalid elastic_w_smooth=%.6f. Using default value 0.35.",
                elastic_w_smooth_);
    elastic_w_smooth_ = 0.35;
  }
  if (elastic_max_iterations_ <= 0) {
    RCLCPP_WARN(node->get_logger(),
                "Invalid elastic_max_iterations=%d. Using default value 15.",
                elastic_max_iterations_);
    elastic_max_iterations_ = 15;
  }
  if (!std::isfinite(elastic_convergence_threshold_) ||
      elastic_convergence_threshold_ <= 0.0) {
    RCLCPP_WARN(node->get_logger(),
                "Invalid elastic_convergence_threshold=%.6f. Using default "
                "value 0.002 m.",
                elastic_convergence_threshold_);
    elastic_convergence_threshold_ = 2e-3;
  }
  path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
      node->get_fully_qualified_name() + std::string("/") + pn + "/path", 10);
  rclcpp::QoS qos(10);
  qos.transient_local();
  potential_layer_pub_ =
      node->create_publisher<navmap_ros_interfaces::msg::NavMapLayer>(
          "/navmap_layers", qos);
  RCLCPP_INFO(
      node->get_logger(),
      "GradientPlanner initialized: continuous_replan=%s, max_iterations=%d, "
      "robot_width=%.3f m, smoothing_factor=%.3f",
      continuous_replan_ ? "true" : "false", max_iterations_, robot_width_,
      smoothing_factor_);
  metrics_file_.open("/tmp/planner_metrics.csv", std::ios::app);
}
// update
static bool sameGoalPosition(const geometry_msgs::msg::Pose &a,
                             const geometry_msgs::msg::Pose &b,
                             double epsilon) {
  return std::abs(a.position.x - b.position.x) < epsilon &&
         std::abs(a.position.y - b.position.y) < epsilon &&
         std::abs(a.position.z - b.position.z) < epsilon;
}
static bool sameGoalOrientation(const geometry_msgs::msg::Quaternion &a,
                                const geometry_msgs::msg::Quaternion &b) {
  const double norm_a =
      std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z + a.w * a.w);
  const double norm_b =
      std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z + b.w * b.w);
  // Si alguna orientación no es un cuaternión válido, comparar componentes.
  if (norm_a < kGeometryEpsilon || norm_b < kGeometryEpsilon) {
    return std::abs(a.x - b.x) < kGeometryEpsilon &&
           std::abs(a.y - b.y) < kGeometryEpsilon &&
           std::abs(a.z - b.z) < kGeometryEpsilon &&
           std::abs(a.w - b.w) < kGeometryEpsilon;
  }
  // q y -q representan exactamente la misma orientación.
  const double dot =
      (a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w) / (norm_a * norm_b);
  return 1.0 - std::abs(dot) < kGoalOrientationDotTolerance;
}
void GradientPlanner::update(NavState &nav_state) {
  PROFILE_FUNCTION();
  RCLCPP_DEBUG(get_node()->get_logger(), "GradientPlanner::update called");
  if (!nav_state.has("goals") || !nav_state.has("robot_pose") ||
      !nav_state.has("map.navmap")) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Missing data: goals=%d, robot_pose=%d, map=%d",
                nav_state.has("goals"), nav_state.has("robot_pose"),
                nav_state.has("map.navmap"));
    return;
  }
  const auto goals = nav_state.get<nav_msgs::msg::Goals>("goals");
  if (goals.goals.empty()) {
    nav_state.set("path", current_path_);
    return;
  }
  if (goals.header.frame_id != "map") {
    RCLCPP_WARN(get_node()->get_logger(), "Goals frame is not 'map': %s",
                goals.header.frame_id.c_str());
    return;
  }
  const auto robot_pose = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");
  const auto &goal = goals.goals.front().pose;
  // Detectar cambios del goal.
  // La posición afecta al campo potencial/gradiente; la orientación sólo obliga
  // a regenerar el path para actualizar correctamente la pose final.
  const bool goal_position_changed =
      !goal_initialized_ ||
      !sameGoalPosition(goal, current_goal_, goal_change_epsilon_);
  const bool goal_orientation_changed =
      !goal_initialized_ ||
      !sameGoalOrientation(goal.orientation, current_goal_.orientation);
  const bool goal_changed = goal_position_changed || goal_orientation_changed;
  if (goal_changed) {
    current_goal_ = goal;
    goal_initialized_ = true;
    if (goal_position_changed) {
      potential_field_valid_ = false;
      RCLCPP_INFO(get_node()->get_logger(),
                  "New goal position detected (%.2f, %.2f, %.2f): recomputing "
                  "fields once",
                  goal.position.x, goal.position.y, goal.position.z);
    } else {
      RCLCPP_INFO(
          get_node()->get_logger(),
          "Goal orientation changed: regenerating path with cached fields");
    }
  }
  // Leer el NavMap ANTES de decidir si reutilizamos el path. Esto permite
  // detectar cambios del mapa incluso con continuous_replan=false.
  try {
    navmap_ = nav_state.get<::navmap::NavMap>("map.navmap");
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error getting NavMap: %s",
                 e.what());
    return;
  }
  if (navmap_.navcels.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "NavMap is empty!");
    return;
  }
  // Detectar cambios de geometría/topología del NavMap.
  const std::size_t current_navmap_signature =
      compute_navmap_signature(navmap_);
  const bool map_changed = !navmap_signature_initialized_ ||
                           current_navmap_signature != navmap_signature_;
  if (map_changed) {
    navmap_signature_ = current_navmap_signature;
    navmap_signature_initialized_ = true;
    // Todo lo derivado de la geometría/topología anterior deja de ser válido.
    neighbors_cached_ = false;
    neighbors_cache_.clear();
    geometry_cached_ = false;
    centroids_cache_.clear();
    traversable_neighbors_cache_.clear();
    potential_field_valid_ = false;
    potentials_.clear();
    gradients_.clear();
    current_path_cells_.clear();
    RCLCPP_INFO(get_node()->get_logger(),
                "NavMap change detected: invalidating topology, geometry, "
                "potential and gradient caches");
  }
  // Sin replanning continuo sólo reutilizamos el path si ni el objetivo ni
  // el NavMap han cambiado.
  if (!continuous_replan_ && !goal_changed && !map_changed &&
      !current_path_.poses.empty()) {
    nav_state.set("path", current_path_);
    if (path_pub_->get_subscription_count() > 0) {
      path_pub_->publish(current_path_);
    }
    return;
  }
  reset_path_state(); // limpiamos posible basura.
  if (!neighbors_cached_) {
    build_neighbors_cache(navmap_);
  }
  if (!geometry_cached_) {
    build_geometry_cache(navmap_);
  }
  auto t0 = std::chrono::high_resolution_clock::now();
  auto poses = compute_gradient_path(navmap_, robot_pose.pose.pose, goal);
  if (!poses.empty()) {
    current_path_.header.stamp = get_node()->now();
    current_path_.header.frame_id = goals.header.frame_id;
    current_path_.poses.reserve(poses.size());
    for (const auto &pose : poses) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = current_path_.header;
      ps.pose = pose;
      current_path_.poses.push_back(ps);
    }
    elastic_band_smoothing(current_path_); // suavizamos el path.
    if (path_pub_->get_subscription_count() > 0) {
      path_pub_->publish(current_path_);
    }
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Empty path generated!");
  }
  nav_state.set("path", current_path_);
  auto t1 = std::chrono::high_resolution_clock::now();
  double planning_time_ms =
      std::chrono::duration<double, std::milli>(t1 - t0).count();
  metrics_file_ << planning_time_ms << "," << current_path_.poses.size() << ","
                << compute_path_length(current_path_) << "\n";
  metrics_file_.flush();
}
// compute_gradient_path.
std::vector<geometry_msgs::msg::Pose>
GradientPlanner::compute_gradient_path(::navmap::NavMap &navmap,
                                       const geometry_msgs::msg::Pose &start,
                                       const geometry_msgs::msg::Pose &goal) {
  PROFILE_FUNCTION();
  // 1. Localizar start y goal antes de calcular los campos.
  //
  // La pose inicial puede usar closest_navcel() como fallback para tolerar
  // pequeñas discrepancias de localización. El goal, en cambio, debe estar
  // contenido realmente en el NavMap: no se proyecta automáticamente al
  // triángulo más cercano.
  const auto start_cell = locate_cell(navmap, start.position, true);
  const auto goal_cell = locate_cell(navmap, goal.position, false);
  if (start_cell == ::navmap::NavCelId(-1) ||
      goal_cell == ::navmap::NavCelId(-1)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to locate start/goal cell");
    return {};
  }
  // 2. Calcular potencial + gradiente UNA SOLA VEZ por goal.
  if (!potential_field_valid_) {
    compute_potential_field(navmap, goal);
    compute_gradient_field(navmap);
    potential_field_valid_ = true;
    RCLCPP_INFO(
        get_node()->get_logger(),
        "Potential and gradient fields computed and cached for current goal");
  }
  if (start_cell >= potentials_.size() || goal_cell >= potentials_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Cell indices out of range: start=%u, goal=%u, size=%zu",
                 start_cell, goal_cell, potentials_.size());
    return {};
  }
  if (std::isinf(potentials_[start_cell])) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Start cell %u has infinite potential - no path exists!",
                 start_cell);
    return {};
  }
  if (std::isinf(potentials_[goal_cell])) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Goal cell %u has infinite potential - unreachable!",
                 goal_cell);
    return {};
  }
  // 3. Inicializar path + correspondencia Pose <-> NavCel.
  std::vector<geometry_msgs::msg::Pose> path;
  path.reserve(max_iterations_ + 2);
  current_path_cells_.clear();
  current_path_cells_.reserve(max_iterations_ + 2);
  // Primer waypoint
  path.push_back(start);
  current_path_cells_.push_back(start_cell);
  auto current_cell = start_cell;
  // 4. Estado del descenso.
  int iterations_used = 0;
  // 5. Seguimiento del campo de gradiente.
  for (int step = 0; step < max_iterations_; ++step) {
    iterations_used = step + 1;
    // Goal alcanzado.
    if (current_cell == goal_cell) {
      // Evitar duplicar exactamente el goal si ya estuviera añadido.
      const auto &last = path.back().position;
      if (last.x != goal.position.x || last.y != goal.position.y ||
          last.z != goal.position.z) {
        path.push_back(goal);
        current_path_cells_.push_back(goal_cell);
      }
      break;
    }
    const double current_potential = potentials_[current_cell];
    // Validar índice del campo.
    if (current_cell >= gradients_.size()) {
      RCLCPP_WARN(get_node()->get_logger(), "Cell %u out of gradient range",
                  current_cell);
      break;
    }
    // Obtener gradiente de la celda actual.
    const Eigen::Vector2d raw_gradient = gradients_[current_cell];
    const double grad_magnitude = raw_gradient.norm();
    // Gradiente prácticamente nulo: usar directamente el vecino atravesable
    // con menor potencial. Dijkstra garantiza una dirección descendente para
    // cualquier celda alcanzable que no sea el goal.
    if (grad_magnitude < kZeroGradientThreshold) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Zero gradient at step %d, cell %u: using potential fallback",
                  step, current_cell);
      const auto fallback_cell =
          find_lower_potential_neighbor(current_cell, navmap);
      if (fallback_cell == current_cell) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Potential fallback failed at cell %u", current_cell);
        debug_stuck_cell(current_cell, navmap);
        break;
      }
      current_cell = fallback_cell;
      path.push_back(
          create_pose_from_cell(current_cell, navmap, start.orientation));
      current_path_cells_.push_back(current_cell);
      continue;
    }
    // Dirección del descenso dada directamente por el campo de gradiente.
    // El caso de norma prácticamente nula ya se ha tratado arriba.
    Eigen::Vector2d dir = raw_gradient.normalized();
    // Selección LOCAL de la siguiente NavCel
    //
    // Ya NO ejecutamos closest_navcel() global.
    // Sólo analizamos vecinos directos de current_cell.
    const auto centroid = centroids_cache_[current_cell];
    const auto &nbrs = traversable_neighbors_cache_[current_cell];
    ::navmap::NavCelId next_cell = current_cell;
    double best_score = -std::numeric_limits<double>::infinity();
    for (const auto nb : nbrs) {
      if (nb >= potentials_.size()) {
        continue;
      }
      if (std::isinf(potentials_[nb])) {
        continue;
      }
      const auto &nb_centroid = centroids_cache_[nb];
      Eigen::Vector2d to_neighbor(nb_centroid.x() - centroid.x(),
                                  nb_centroid.y() - centroid.y());
      const double neighbor_distance = to_neighbor.norm();
      if (neighbor_distance < kGeometryEpsilon) {
        continue;
      }
      to_neighbor /= neighbor_distance;
      // Alineación con el gradiente.
      const double alignment = dir.dot(to_neighbor);
      // Descenso de potencial.
      const double potential_drop = current_potential - potentials_[nb];
      // Sólo permitimos avanzar hacia menor potencial.
      if (potential_drop <= 0.0) {
        continue;
      }
      // Score combinado.
      const double score = alignment + potential_drop_weight_ * potential_drop;
      if (score > best_score) {
        best_score = score;
        next_cell = nb;
      }
    }
    // Aplicar transición a la siguiente NavCel.
    if (next_cell != current_cell) {
      current_cell = next_cell;
      const auto &next_centroid = centroids_cache_[current_cell];
      geometry_msgs::msg::Pose next_pose;
      next_pose.position.x = next_centroid.x();
      next_pose.position.y = next_centroid.y();
      next_pose.position.z = next_centroid.z();
      // Orientación según la dirección entre waypoints
      const auto &prev_pos = path.back().position;
      const double dx = next_pose.position.x - prev_pos.x;
      const double dy = next_pose.position.y - prev_pos.y;
      if (std::hypot(dx, dy) > kGeometryEpsilon) {
        const double yaw = std::atan2(dy, dx);
        next_pose.orientation.x = 0.0;
        next_pose.orientation.y = 0.0;
        next_pose.orientation.z = std::sin(yaw * 0.5);
        next_pose.orientation.w = std::cos(yaw * 0.5);
      } else {
        next_pose.orientation = start.orientation;
      }
      // IMPORTANTE: pose y NavCel se añaden SIEMPRE juntas
      path.push_back(next_pose);
      current_path_cells_.push_back(current_cell);
    } else {
      // El gradiente no ha producido una transición válida. Usar el descenso
      // directo de potencial como recuperación determinista.
      const auto fallback_cell =
          find_lower_potential_neighbor(current_cell, navmap);
      if (fallback_cell == current_cell) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "No traversable lower-potential neighbor at cell %u",
                     current_cell);
        debug_stuck_cell(current_cell, navmap);
        break;
      }
      current_cell = fallback_cell;
      path.push_back(
          create_pose_from_cell(current_cell, navmap, start.orientation));
      current_path_cells_.push_back(current_cell);
    }
  }
  // 6. Verificar que el descenso ha alcanzado realmente la NavCel objetivo.
  if (current_cell != goal_cell) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Gradient path failed to reach goal cell. "
                 "Last=%u (pot=%.3f), Goal=%u (pot=%.3f)",
                 current_cell, potentials_[current_cell], goal_cell,
                 potentials_[goal_cell]);
    current_path_cells_.clear();
    return {};
  }
  // Ya estamos en la NavCel objetivo. Añadir las coordenadas exactas del goal
  // solo si todavía no coinciden con el último waypoint.
  const auto &last = path.back().position;
  if (last.x != goal.position.x || last.y != goal.position.y ||
      last.z != goal.position.z) {
    path.push_back(goal);
    current_path_cells_.push_back(goal_cell);
  }

  // La última pose representa el objetivo: conservar siempre la orientación
  // solicitada, incluso si su posición ya coincidía con el último waypoint.
  path.back().orientation = goal.orientation;

  // 7. Verificación crítica Pose <-> NavCel.
  if (path.size() != current_path_cells_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Path/Cell mismatch: poses=%zu cells=%zu", path.size(),
                 current_path_cells_.size());
  }
  // 8. Calcular correctamente la longitud.
  double path_length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    const auto &a = path[i - 1].position;
    const auto &b = path[i].position;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double dz = b.z - a.z;
    path_length += std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  // Debug.
  RCLCPP_INFO(get_node()->get_logger(),
              "Gradient path | iterations=%d/%d | poses=%zu | cells=%zu",
              iterations_used, max_iterations_, path.size(),
              current_path_cells_.size());
  RCLCPP_INFO(get_node()->get_logger(), "Path done: %zu poses, length=%.2f m",
              path.size(), path_length);
  return path;
}
// Create_pose_from_cell.
geometry_msgs::msg::Pose GradientPlanner::create_pose_from_cell(
    ::navmap::NavCelId cell_id, const ::navmap::NavMap & /*navmap*/,
    const geometry_msgs::msg::Quaternion &orientation) const {
  geometry_msgs::msg::Pose pose;
  const auto &c = centroids_cache_[cell_id];
  pose.position.x = c.x();
  pose.position.y = c.y();
  pose.position.z = c.z();
  pose.orientation = orientation;
  return pose;
}
// Compute_potential_field.
void GradientPlanner::compute_potential_field(
    ::navmap::NavMap &navmap, const geometry_msgs::msg::Pose &goal) {
  PROFILE_FUNCTION();
  const size_t n = navmap.navcels.size();
  // Reutilizar vector si el tamaño no cambia (evita realloc).
  if (potentials_.size() != n) {
    potentials_.assign(n, std::numeric_limits<double>::infinity());
  } else {
    std::fill(potentials_.begin(), potentials_.end(),
              std::numeric_limits<double>::infinity());
  }
  const auto goal_cell = locate_cell(navmap, goal.position, false);
  if (goal_cell == ::navmap::NavCelId(-1)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Goal cell not found");
    return;
  }
  // Dijkstra desde el goal (potencial = distancia mínima al objetivo).
  std::priority_queue<QueueElement, std::vector<QueueElement>,
                      std::greater<QueueElement>>
      pq;
  potentials_[goal_cell] = 0.0;
  pq.push({0.0, goal_cell});
  while (!pq.empty()) {
    const auto [cur_pot, cur_cell] = pq.top();
    pq.pop();
    if (cur_pot > potentials_[cur_cell]) {
      continue;
    }
    const auto &cur_centroid = centroids_cache_[cur_cell];
    for (const auto nb : traversable_neighbors_cache_[cur_cell]) {
      const auto &nb_centroid = centroids_cache_[nb];
      const double dx = nb_centroid.x() - cur_centroid.x();
      const double dy = nb_centroid.y() - cur_centroid.y();
      const double dz = nb_centroid.z() - cur_centroid.z();
      const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
      const double new_pot = potentials_[cur_cell] + dist;
      if (new_pot < potentials_[nb]) {
        potentials_[nb] = new_pot;
        pq.push({new_pot, nb});
      }
    }
  }
  // Publicar capa de potencial solo si hay suscriptores.
  if (potential_layer_pub_->get_subscription_count() == 0) {
    return;
  }
  if (!navmap.has_layer("potential")) {
    navmap.add_layer<float>("potential", "potential_field", "", 0.0f);
  }
  double min_p = std::numeric_limits<double>::infinity();
  double max_p = -std::numeric_limits<double>::infinity();
  for (double p : potentials_) {
    if (!std::isinf(p)) {
      min_p = std::min(min_p, p);
      max_p = std::max(max_p, p);
    }
  }
  const double range = std::max(max_p - min_p, kGeometryEpsilon);
  navmap_ros_interfaces::msg::NavMapLayer layer_msg;
  layer_msg.header.stamp = get_node()->now();
  layer_msg.header.frame_id = "map";
  layer_msg.name = "potential";
  layer_msg.type = 1;
  layer_msg.data_f32.resize(n);
  for (size_t i = 0; i < n; ++i) {
    const float v = std::isinf(potentials_[i])
                        ? 1.0f
                        : static_cast<float>((potentials_[i] - min_p) / range);
    navmap.layer_set<float>("potential", i, v);
    layer_msg.data_f32[i] = v;
  }
  potential_layer_pub_->publish(layer_msg);
}
// Compute_gradient_field.
void GradientPlanner::compute_gradient_field(const ::navmap::NavMap &navmap) {
  PROFILE_FUNCTION();
  const size_t n = navmap.navcels.size();
  if (gradients_.size() != n) {
    gradients_.assign(n, Eigen::Vector2d::Zero());
  } else {
    std::fill(gradients_.begin(), gradients_.end(), Eigen::Vector2d::Zero());
  }
  for (size_t i = 0; i < n; ++i) {
    if (!std::isinf(potentials_[i])) {
      gradients_[i] = compute_gradient_at_cell(i, potentials_, navmap);
    }
  }
  smooth_gradients(navmap);
}
// Compute_gradient_at_cell.
Eigen::Vector2d GradientPlanner::compute_gradient_at_cell(
    ::navmap::NavCelId cell_id, const std::vector<double> &potentials,
    const ::navmap::NavMap & /*navmap*/) const {
  const auto &nbrs = traversable_neighbors_cache_[cell_id];
  if (nbrs.empty() || std::isinf(potentials[cell_id])) {
    return Eigen::Vector2d::Zero();
  }
  const auto centroid = centroids_cache_[cell_id];
  Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
  int valid_count = 0;
  for (auto nb : nbrs) {
    if (nb >= potentials.size() || std::isinf(potentials[nb])) {
      continue;
    }
    const auto &nb_centroid = centroids_cache_[nb];
    Eigen::Vector2d dir(nb_centroid.x() - centroid.x(),
                        nb_centroid.y() - centroid.y());
    const double dist = dir.norm();
    if (dist < kGeometryEpsilon) {
      continue;
    }
    dir /= dist; // normalizar.
    const double w = 1.0 / (dist + kGradientWeightEpsilon);
    gradient += ((potentials[nb] - potentials[cell_id]) * w) * dir;
    ++valid_count;
  }
  if (valid_count == 0) {
    RCLCPP_WARN(get_node()->get_logger(),
                "No valid neighbors for gradient at cell %u", cell_id);
    return Eigen::Vector2d::Zero();
  }
  gradient /= valid_count;
  return -gradient; // apunta hacia el descenso del potencial.
}
// Smooth_gradients.
void GradientPlanner::smooth_gradients(const ::navmap::NavMap & /*navmap*/) {
  PROFILE_FUNCTION();
  std::vector<Eigen::Vector2d> smoothed = gradients_;
  for (size_t i = 0; i < gradients_.size(); ++i) {
    if (std::isinf(potentials_[i])) {
      continue;
    }
    const auto &nbrs = traversable_neighbors_cache_[i];
    if (nbrs.empty()) {
      continue;
    }
    Eigen::Vector2d sum = gradients_[i];
    int cnt = 1;
    for (const auto nb : nbrs) {
      if (nb < gradients_.size() && !std::isinf(potentials_[nb])) {
        sum += gradients_[nb];
        ++cnt;
      }
    }
    if (cnt > 1) {
      smoothed[i] = (1.0 - smoothing_factor_) * gradients_[i] +
                    smoothing_factor_ * (sum / cnt);
    }
  }
  gradients_ = std::move(smoothed);
}
// Locate_cell.
::navmap::NavCelId
GradientPlanner::locate_cell(const ::navmap::NavMap &navmap,
                             const geometry_msgs::msg::Point &point,
                             bool allow_closest_fallback) const {
  PROFILE_FUNCTION();
  const Eigen::Vector3f pos(point.x, point.y, point.z);
  size_t surface_idx;
  ::navmap::NavCelId cell_id;
  Eigen::Vector3f bary, hit;
  if (navmap.locate_navcel(pos, surface_idx, cell_id, bary, &hit)) {
    return cell_id;
  }
  if (!allow_closest_fallback) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Point (%.2f, %.2f, %.2f) is outside the NavMap", point.x,
                point.y, point.z);
    return ::navmap::NavCelId(-1);
  }
  Eigen::Vector3f closest;
  float dist_sq;
  if (navmap.closest_navcel(pos, surface_idx, cell_id, closest, dist_sq)) {
    return cell_id;
  }
  RCLCPP_WARN(get_node()->get_logger(),
              "Could not locate point (%.2f, %.2f, %.2f) in NavMap", point.x,
              point.y, point.z);
  return ::navmap::NavCelId(-1);
}
// Caché de vecinos.
void GradientPlanner::build_neighbors_cache(
    const ::navmap::NavMap &navmap) const {
  PROFILE_FUNCTION();
  const size_t n = navmap.navcels.size();
  neighbors_cache_.resize(n);
  for (size_t i = 0; i < n; ++i) {
    auto ids = navmap.navcel_neighbors(i);
    neighbors_cache_[i].assign(ids.begin(), ids.end());
  }
  neighbors_cached_ = true;
  RCLCPP_DEBUG(get_node()->get_logger(), "Neighbors cache built: %zu cells", n);
}
const std::vector<::navmap::NavCelId> &
GradientPlanner::get_neighbors(const ::navmap::NavMap &navmap,
                               ::navmap::NavCelId cell_id) const {
  if (!neighbors_cached_) {
    build_neighbors_cache(navmap);
  }
  return neighbors_cache_[cell_id];
}
// Caché geométrica derivada del NavMap.
void GradientPlanner::build_geometry_cache(const ::navmap::NavMap &navmap) {
  PROFILE_FUNCTION();
  if (!neighbors_cached_) {
    build_neighbors_cache(navmap);
  }
  const size_t n = navmap.navcels.size();
  centroids_cache_.resize(n);
  traversable_neighbors_cache_.clear();
  traversable_neighbors_cache_.resize(n);
  // Los centroides sólo dependen de la geometría del NavMap.
  for (size_t i = 0; i < n; ++i) {
    const auto c = navmap.navcel_centroid(i);
    centroids_cache_[i] = Eigen::Vector3f(c.x(), c.y(), c.z());
  }
  // Precalcular una única vez las transiciones válidas para robot_width_.
  // Dijkstra, gradiente, smoothing y fallback reutilizan esta lista.
  for (size_t i = 0; i < n; ++i) {
    auto &traversable = traversable_neighbors_cache_[i];
    const auto &neighbors = neighbors_cache_[i];
    traversable.reserve(neighbors.size());
    for (const auto nb : neighbors) {
      if (nb >= n) {
        continue;
      }
      if (is_transition_traversable(i, nb, navmap)) {
        traversable.push_back(nb);
      }
    }
  }
  geometry_cached_ = true;
  RCLCPP_DEBUG(get_node()->get_logger(), "Geometry cache built: %zu centroids",
               n);
}
// Shared edge width.
double
GradientPlanner::shared_edge_width(::navmap::NavCelId cell_a,
                                   ::navmap::NavCelId cell_b,
                                   const ::navmap::NavMap &navmap) const {
  if (cell_a >= navmap.navcels.size() || cell_b >= navmap.navcels.size()) {
    return 0.0;
  }
  const auto &a = navmap.navcels[cell_a];
  const auto &b = navmap.navcels[cell_b];
  std::array<std::size_t, 2> shared_vertices{};
  std::size_t shared_count = 0;
  for (const auto va : a.v) {
    for (const auto vb : b.v) {
      if (va == vb) {
        if (shared_count < shared_vertices.size()) {
          shared_vertices[shared_count] = static_cast<std::size_t>(va);
        }
        ++shared_count;
        break;
      }
    }
  }
  // Dos NavCels triangulares vecinas deben compartir exactamente una arista,
  // es decir, dos vértices. Si no es así, no consideramos válida la transición.
  if (shared_count != 2) {
    return 0.0;
  }
  const auto &p0 = navmap.positions.at(shared_vertices[0]);
  const auto &p1 = navmap.positions.at(shared_vertices[1]);
  return static_cast<double>((p1 - p0).norm());
}
// Transition traversability.
bool GradientPlanner::is_transition_traversable(
    ::navmap::NavCelId from, ::navmap::NavCelId to,
    const ::navmap::NavMap &navmap) const {
  if (from == to) {
    return true;
  }
  if (robot_width_ <= 0.0) {
    return true;
  }
  const double edge_width = shared_edge_width(from, to, navmap);
  return edge_width + kGeometryEpsilon >= robot_width_;
}
// Lower-potential fallback.
::navmap::NavCelId GradientPlanner::find_lower_potential_neighbor(
    ::navmap::NavCelId current_cell,
    const ::navmap::NavMap & /*navmap*/) const {
  PROFILE_FUNCTION();
  if (current_cell >= potentials_.size()) {
    return current_cell;
  }
  const auto &nbrs = traversable_neighbors_cache_[current_cell];
  ::navmap::NavCelId best_nb = current_cell;
  double min_potential = potentials_[current_cell];
  for (const auto nb : nbrs) {
    if (nb >= potentials_.size()) {
      continue;
    }
    if (std::isinf(potentials_[nb])) {
      continue;
    }
    if (potentials_[nb] < min_potential) {
      min_potential = potentials_[nb];
      best_nb = nb;
    }
  }
  return best_nb;
}
// Stuck-cell diagnostics.
void GradientPlanner::debug_stuck_cell(::navmap::NavCelId cell_id,
                                       const ::navmap::NavMap &navmap) {
  PROFILE_FUNCTION();
  if (cell_id >= potentials_.size()) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Debug stuck cell: cell %u outside potential field (size=%zu)",
                cell_id, potentials_.size());
    return;
  }
  const auto centroid = navmap.navcel_centroid(cell_id);
  const auto &nbrs = get_neighbors(navmap, cell_id);
  Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
  if (cell_id < gradients_.size()) {
    gradient = gradients_[cell_id];
  }
  RCLCPP_WARN(
      get_node()->get_logger(),
      "Stuck cell debug | cell=%u | pos=(%.3f, %.3f, %.3f) | pot=%.6f | "
      "grad=(%.6f, %.6f) | neighbors=%zu",
      cell_id, centroid.x(), centroid.y(), centroid.z(), potentials_[cell_id],
      gradient.x(), gradient.y(), nbrs.size());
  std::ostringstream oss;
  oss << "Stuck cell neighbors:";
  for (const auto nb : nbrs) {
    oss << " [" << nb << ": ";
    if (nb >= potentials_.size()) {
      oss << "out_of_range";
    } else if (std::isinf(potentials_[nb])) {
      oss << "inf";
    } else {
      oss << potentials_[nb];
    }
    oss << "]";
  }
  RCLCPP_WARN(get_node()->get_logger(), "%s", oss.str().c_str());
}
// Reset_path_state.
void GradientPlanner::reset_path_state() {
  PROFILE_FUNCTION();
  // Solo resetea el estado de planificación (potenciales, gradientes, path).
  // La caché de vecinos se mantiene si el mapa no ha cambiado.
  // No hacemos clear()+resize() — se reutilizan los vectores en compute_*
  // mediante std::fill, lo cual evita realloc si el tamaño no cambia.
  current_path_.poses.clear();
}
// Elastic_band_smoothing.
void GradientPlanner::elastic_band_smoothing(nav_msgs::msg::Path &path) {
  PROFILE_FUNCTION();
  if (path.poses.size() < 3) {
    return;
  }
  const size_t N = path.poses.size();

  // La banda elástica recalcula las orientaciones intermedias a partir de la
  // tangente del camino, pero debe preservar la orientación solicitada en el
  // goal.
  const auto goal_orientation = path.poses.back().pose.orientation;

  // 1. Preparar puntos y reutilizar NavCel IDs del GradientPlanner.
  std::vector<Eigen::Vector3f> original_points(N);
  std::vector<Eigen::Vector3f> current_points(N);
  std::vector<Eigen::Vector3f> next_points(N);
  if (current_path_cells_.size() != N) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Elastic band: Path/Cell mismatch: poses=%zu cells=%zu", N,
                 current_path_cells_.size());
    return;
  }
  const auto &cell_ids = current_path_cells_;
  for (size_t i = 0; i < N; ++i) {
    const auto &p = path.poses[i].pose.position;
    const Eigen::Vector3f point(static_cast<float>(p.x),
                                static_cast<float>(p.y),
                                static_cast<float>(p.z));
    original_points[i] = point;
    current_points[i] = point;
    next_points[i] = point;
  }
  // 2. Precalcular los vértices de la NavCel asociada a cada waypoint.
  // Evita repetir accesos a navmap_.positions en cada iteración del Elastic
  // Band.
  std::vector<std::array<Eigen::Vector3f, 3>> cell_vertices(N);
  std::vector<bool> valid_cells(N, false);
  for (size_t i = 0; i < N; ++i) {
    const auto cell_id = cell_ids[i];
    if (cell_id >= navmap_.navcels.size()) {
      continue;
    }
    const auto &triangle = navmap_.navcels[cell_id];
    cell_vertices[i] = {navmap_.positions.at(triangle.v[0]),
                        navmap_.positions.at(triangle.v[1]),
                        navmap_.positions.at(triangle.v[2])};
    valid_cells[i] = true;
  }
  // 3. Elastic Band iterativo con convergencia.
  int iterations_used = 0;
  const auto t_smoothing_start = std::chrono::high_resolution_clock::now();
  for (int iteration = 0; iteration < elastic_max_iterations_; ++iteration) {
    iterations_used = iteration + 1;
    float max_displacement = 0.0f;
    // Los extremos permanecen fijos.
    next_points.front() = current_points.front();
    next_points.back() = current_points.back();
    for (size_t i = 1; i + 1 < N; ++i) {
      if (!valid_cells[i]) {
        next_points[i] = current_points[i];
        continue;
      }
      const Eigen::Vector3f &xi = current_points[i];
      const Eigen::Vector3f &xi_original = original_points[i];
      const Eigen::Vector3f &xm = current_points[i - 1];
      const Eigen::Vector3f &xp = current_points[i + 1];
      Eigen::Vector3f candidate = xi;
      candidate.x() += static_cast<float>(
          elastic_w_data_ * (xi_original.x() - xi.x()) +
          elastic_w_smooth_ * (xm.x() + xp.x() - 2.0 * xi.x()));
      candidate.y() += static_cast<float>(
          elastic_w_data_ * (xi_original.y() - xi.y()) +
          elastic_w_smooth_ * (xm.y() + xp.y() - 2.0 * xi.y()));
      // Mantener el waypoint dentro de SU NavCel.
      const auto &vertices = cell_vertices[i];
      const Eigen::Vector3f constrained = ::navmap::closest_point_on_triangle(
          candidate, vertices[0], vertices[1], vertices[2]);
      const float displacement = (constrained - current_points[i]).norm();
      max_displacement = std::max(max_displacement, displacement);
      next_points[i] = constrained;
    }
    current_points.swap(next_points);
    // Convergencia.
    if (max_displacement < elastic_convergence_threshold_) {
      break;
    }
  }
  const auto t_smoothing_end = std::chrono::high_resolution_clock::now();
  const double smoothing_total_ms = std::chrono::duration<double, std::milli>(
                                        t_smoothing_end - t_smoothing_start)
                                        .count();
  RCLCPP_INFO(get_node()->get_logger(),
              "Elastic band | iterations=%d/%d | poses=%zu | smoothing=%.3f ms",
              iterations_used, elastic_max_iterations_, N, smoothing_total_ms);
  // 4. Copiar puntos suavizados al Path.
  for (size_t i = 0; i < N; ++i) {
    path.poses[i].pose.position.x = current_points[i].x();
    path.poses[i].pose.position.y = current_points[i].y();
    path.poses[i].pose.position.z = current_points[i].z();
  }
  // 5. Recalcular orientación.
  for (size_t i = 0; i + 1 < N; ++i) {
    const auto current = path.poses[i].pose.position;
    const auto &next = path.poses[i + 1].pose.position;
    const double dx = next.x - current.x;
    const double dy = next.y - current.y;
    if (std::hypot(dx, dy) < kGeometryEpsilon) {
      continue;
    }
    const double yaw = std::atan2(dy, dx);
    auto &q = path.poses[i].pose.orientation;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
  }
  // Mantener en la última pose la orientación solicitada en el objetivo.
  path.poses.back().pose.orientation = goal_orientation;
}
} // namespace navmap
} // namespace easynav
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::navmap::GradientPlanner,
                       easynav::PlannerMethodBase)