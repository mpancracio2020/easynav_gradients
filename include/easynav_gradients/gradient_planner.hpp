// Copyright 2026 Marvin Pancracio Manso
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.
//
// Easy Navigation program is free software: you can redistribute it and/or
// modify it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

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
  // ── Parámetros ────────────────────────────────────────────────────────────
  bool continuous_replan_{true};
  int max_iterations_{500};
  double smoothing_factor_{0.4}; ///< Suavizado del campo de gradiente
  double robot_width_{0.0};

  // Detección de cambios.
  double goal_change_epsilon_{1e-3};

  // Selección de vecinos.
  double potential_drop_weight_{0.75};

  // Elastic Band.
  double elastic_w_data_{0.10};
  double elastic_w_smooth_{0.35};
  int elastic_max_iterations_{15};
  double elastic_convergence_threshold_{2e-3};

  // ── Estado del planificador ───────────────────────────────────────────────
  nav_msgs::msg::Path current_path_;
  geometry_msgs::msg::Pose current_goal_;
  ::navmap::NavMap navmap_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<navmap_ros_interfaces::msg::NavMapLayer>::SharedPtr
      potential_layer_pub_;

  // ── Estructuras internas ─────────────────────────────────────────────────
  std::vector<double> potentials_;
  std::vector<Eigen::Vector2d> gradients_;

  // ── Cachés derivadas del NavMap ───────────────────────────────────────────
  // Topología bruta de vecindad.
  mutable std::vector<std::vector<::navmap::NavCelId>> neighbors_cache_;
  mutable bool neighbors_cached_ = false;

  // Geometría estática y transiciones que cumplen robot_width_.
  std::vector<Eigen::Vector3f> centroids_cache_;
  std::vector<std::vector<::navmap::NavCelId>> traversable_neighbors_cache_;
  bool geometry_cached_{false};

  // ── Firma del NavMap ──────────────────────────────────────────────────────
  // Permite invalidar las cachés si cambia la geometría o la topología,
  // incluso cuando el número de NavCels permanece constante.
  std::size_t navmap_signature_{0};
  bool navmap_signature_initialized_{false};

  // ── Métodos privados ──────────────────────────────────────────────────────

  /// \brief Planificación completa: campo potencial + gradiente + seguimiento.
  std::vector<geometry_msgs::msg::Pose>
  compute_gradient_path(::navmap::NavMap &navmap,
                        const geometry_msgs::msg::Pose &start,
                        const geometry_msgs::msg::Pose &goal);

  /// \brief Propaga el campo potencial desde el goal con Dijkstra.
  void compute_potential_field(::navmap::NavMap &navmap,
                               const geometry_msgs::msg::Pose &goal);

  /// \brief Calcula el campo de gradientes a partir del campo potencial.
  void compute_gradient_field(const ::navmap::NavMap &navmap);

  /// \brief Suaviza el campo de gradientes mediante promedio de vecinos.
  void smooth_gradients(const ::navmap::NavMap &navmap);

  /// \brief Calcula el vector gradiente en una celda concreta.
  Eigen::Vector2d
  compute_gradient_at_cell(::navmap::NavCelId cell_id,
                           const std::vector<double> &potentials,
                           const ::navmap::NavMap &navmap) const;

  /// \brief Localiza la NavCel que contiene un punto dado.
  /// \param allow_closest_fallback Si es true, usa closest_navcel() cuando el
  /// punto no está contenido en ninguna NavCel.
  ::navmap::NavCelId locate_cell(const ::navmap::NavMap &navmap,
                                 const geometry_msgs::msg::Point &point,
                                 bool allow_closest_fallback) const;

  /// \brief Devuelve los vecinos de una celda (con caché).
  /// \note Devuelve referencia const — no copiar el vector.
  const std::vector<::navmap::NavCelId> &
  get_neighbors(const ::navmap::NavMap &navmap,
                ::navmap::NavCelId cell_id) const;

  /// \brief Construye la caché completa de vecinos para el NavMap actual.
  void build_neighbors_cache(const ::navmap::NavMap &navmap) const;

  /// \brief Precalcula centroides y vecinos atravesables para el NavMap actual.
  void build_geometry_cache(const ::navmap::NavMap &navmap);

  /// \brief Longitud de la arista compartida entre dos NavCels.
  /// \return Longitud en metros; 0.0 si no comparten exactamente una arista.
  double shared_edge_width(::navmap::NavCelId cell_a, ::navmap::NavCelId cell_b,
                           const ::navmap::NavMap &navmap) const;

  /// \brief Comprueba si el robot puede atravesar la arista compartida entre
  /// dos NavCels.
  bool is_transition_traversable(::navmap::NavCelId from, ::navmap::NavCelId to,
                                 const ::navmap::NavMap &navmap) const;

  /// \brief Devuelve el vecino atravesable con menor potencial que la celda
  ///        actual. Devuelve current_cell si no existe ninguno.
  ::navmap::NavCelId
  find_lower_potential_neighbor(::navmap::NavCelId current_cell,
                                const ::navmap::NavMap &navmap) const;

  /// \brief Crea una Pose a partir del centroide de una celda.
  geometry_msgs::msg::Pose create_pose_from_cell(
      ::navmap::NavCelId cell_id, const ::navmap::NavMap &navmap,
      const geometry_msgs::msg::Quaternion &orientation) const;

  /// \brief Resetea el estado local del seguimiento del gradiente y el path.
  ///        No modifica los campos potencial ni de gradientes cacheados.
  void reset_path_state();

  /// \brief Suavizado elastic-band sobre el path generado.
  void elastic_band_smoothing(nav_msgs::msg::Path &path);

  /// \brief Diagnóstico de celda estancada (debug).
  void debug_stuck_cell(::navmap::NavCelId cell_id,
                        const ::navmap::NavMap &navmap);

  std::ofstream metrics_file_;

  // ── Caché de potencial y gradiente ────────────────────────────────────────
  /// \brief Indica si el campo potencial y el campo de gradientes
  ///        corresponden al objetivo actual.
  bool potential_field_valid_{false};

  /// \brief Indica si ya se ha recibido al menos un objetivo.
  bool goal_initialized_{false};

  std::vector<::navmap::NavCelId> current_path_cells_;
};

} // namespace navmap
} // namespace easynav

#endif // EASYNAV_NAVMAP_PLANNER__GRADIENT_PLANNER_HPP_