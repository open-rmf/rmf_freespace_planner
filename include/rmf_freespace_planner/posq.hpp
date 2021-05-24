/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef RMF_FREESPACE_PLANNER__POSQ_HPP
#define RMF_FREESPACE_PLANNER__POSQ_HPP

#include "rmf_freespace_planner/kinodynamic_rrt_star.hpp"

namespace rmf_freespace_planner {
namespace kinodynamic_rrt_star {
class Posq : public KinodynamicRRTStar::LocalPlanner
{
public:
  struct Parameters
  {
    double vmax = 1.0;

    double base = 0.4;

    double k_v = 3.8;

    double k_rho = 1.0;

    double k_alpha = 6.0;

    double k_theta = -1.0;

    double rho_end = 0.00510;

    bool forward = true;

    double sample_time = 0.1;
  } parameters;

  Posq() = default;

  Posq(Parameters parameters);

private:
  std::optional<KinodynamicRRTStar::ComputedTrajectory> compute_trajectory(
    const std::shared_ptr<KinodynamicRRTStar::Vertex>& start,
    const std::shared_ptr<KinodynamicRRTStar::Vertex>& end) const override;

  struct PosqState
  {
    const Eigen::Vector3d goal;

    const Parameters parameters;

    double theta;

    bool eot;

    struct
    {
      double forward;
      double rotational;
    } velocity;

    void step(const Eigen::Vector3d& start);
  };

  static double norm_angle(double theta, double start = 0.0);
};
}
}

#endif //RMF_FREESPACE_PLANNER__POSQ_HPP
