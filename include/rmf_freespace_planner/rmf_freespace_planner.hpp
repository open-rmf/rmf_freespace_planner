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

#ifndef RMF_FREESPACE_PLANNER__RMF_FREESPACE_PLANNER_HPP
#define RMF_FREESPACE_PLANNER__RMF_FREESPACE_PLANNER_HPP

#include <rmf_traffic/agv/VehicleTraits.hpp>
#include <rmf_traffic/Route.hpp>

namespace rmf_freespace_planner {

class FreespacePlanner
{
public:
  struct Obstacle
  {
    Eigen::Vector2d position;

    double radius;
  };

  struct Start
  {
    Eigen::Vector3d position;

    rmf_traffic::Time time;
  };

  struct Goal
  {
    Eigen::Vector3d position;
  };

  class Planner
  {
  public:
    virtual std::vector<rmf_traffic::Route> plan(
      const Start& start,
      const Goal& goal,
      const rmf_traffic::agv::VehicleTraits& traits,
      const std::optional<std::vector<Obstacle>>& obstacles,
      const std::string& map) = 0;
  };
};
}

#endif //RMF_FREESPACE_PLANNER__RMF_FREESPACE_PLANNER_HPP
