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

#include "rmf_freespace_planner/posq.hpp"

namespace rmf_freespace_planner {
namespace kinodynamic_rrt_star {

Posq::Posq(Parameters parameters)
: parameters(parameters)
{
}

std::optional<KinodynamicRRTStar::ComputedTrajectory> Posq::compute_trajectory(
  const std::shared_ptr<KinodynamicRRTStar::Vertex>& start,
  const std::shared_ptr<KinodynamicRRTStar::Vertex>& end) const
{
  Eigen::Vector3d x0(start->state.position);

  double t = 0.0;

  double sl = 0.0;
  double sr = 0.0;
  double old_sl = 0.0;
  double old_sr = 0.0;

  PosqState posq_state{end->state.position, parameters, 0, false};

  double cost = 0.0;

  rmf_traffic::Trajectory trajectory;
  trajectory.insert(start->trajectory.back());

  while (!posq_state.eot)
  {
    double dSl = sl - old_sl;
    double dSr = sr - old_sr;
    double dSm = (dSl + dSr) / 2.0;
    double dSd = (dSr - dSl) / parameters.base;

    x0(0) = x0(0) + dSm * cos(x0(2) + dSd / 2.0);
    x0(1) = x0(1) + dSm * sin(x0(2) + dSd / 2.0);
    x0(2) = norm_angle(x0(2) + dSd, -M_PI);

    posq_state.step(x0);
    double vl = posq_state.velocity.forward -
      posq_state.velocity.rotational * parameters.base / 2.0;
    vl = std::clamp(vl, -parameters.vmax, parameters.vmax);

    double vr = posq_state.velocity.forward +
      posq_state.velocity.rotational * parameters.base / 2.0;
    vr = std::clamp(vr, -parameters.vmax, parameters.vmax);

    t = t + parameters.sample_time;

    old_sl = sl;
    old_sr = sr;
    sl += vl * parameters.sample_time;
    sr += vr * parameters.sample_time;

    trajectory.insert(
      rmf_traffic::time::apply_offset(
        trajectory.back().time(), parameters.sample_time),
      x0,
      Eigen::Vector3d(
        posq_state.velocity.forward,
        0.0,
        posq_state.velocity.rotational));
    cost += parameters.sample_time;
  }

  if (trajectory.size() < 2)
  {
    return std::nullopt;
  }
  return KinodynamicRRTStar::ComputedTrajectory{trajectory, cost};
}

void Posq::PosqState::step(const Eigen::Vector3d& start)
{
  double dx = goal(0) - start(0);
  double dy = goal(1) - start(1);
  double rho = std::hypot(dx, dy);
  double f_rho = std::min(rho, parameters.vmax / parameters.k_rho);

  double alpha = norm_angle(atan2(dy, dx) - start(2), -M_PI);
  if (parameters.forward)
  {
    if (alpha > M_PI_2)
    {
      f_rho = -f_rho;
      alpha = alpha - M_PI;
    }
    else if (alpha <= -M_PI_2)
    {
      f_rho = -f_rho;
      alpha = alpha + M_PI;
    }
  }
  else
  {
    f_rho = -f_rho;
    alpha = alpha + M_PI;
    if (alpha > M_PI)
    {
      alpha = alpha - 2 * M_PI;
    }
  }

  double phi = goal(2) - start(2);
  phi = norm_angle(phi, -M_PI);
  double angle = norm_angle(phi - alpha, -M_PI);
  if (abs(theta - angle) > M_PI)
  {
    angle = theta;
  }
  theta = angle;

  eot = (rho < parameters.rho_end);

  double vm = parameters.k_rho * tanh(f_rho * parameters.k_v);
  double vd = (parameters.k_alpha * alpha + parameters.k_theta * theta);
  velocity = {vm, vd};
}

double Posq::norm_angle(double theta, double start)
{
  const double inf = std::numeric_limits<double>::infinity();
  if (-inf < theta && theta < inf)
  {
    while (theta >= start + 2 * M_PI)
    {
      theta -= 2 * M_PI;
    }
    while (theta < start)
    {
      theta += 2 * M_PI;
    }
    return theta;
  }

  throw std::runtime_error(
          "[rmf_freespace_planner::Posq::norm_angle] Received infinite theta");
}
}
}
