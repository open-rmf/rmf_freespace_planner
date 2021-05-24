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

#include <rmf_traffic/agv/RouteValidator.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/schedule/Database.hpp>

#include "rmf_freespace_planner/posq.hpp"

#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <thread>

bool done = false;

void get_estimated_cost(
  const std::shared_ptr<rmf_freespace_planner::kinodynamic_rrt_star::KinodynamicRRTStar>& kinodynamic_rrt_star)
{
  while (!done)
  {
    std::cout << "Estimated cost " <<
      kinodynamic_rrt_star->get_estimated_total_cost() <<
      std::endl;
  }
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_freespace_planner");
  auto freespace_path_pub =
    node->create_publisher<geometry_msgs::msg::PoseArray>("freespace_path",
      rclcpp::SystemDefaultsQoS());
  auto route_waypoints_pub =
    node->create_publisher<geometry_msgs::msg::PoseArray>("route_waypoints",
      rclcpp::SystemDefaultsQoS());

  rmf_traffic::agv::VehicleTraits traits{
    {0.5, 2.0}, {0.75, 1.5},
    rmf_traffic::Profile{
      rmf_traffic::geometry::make_final_convex(
        rmf_traffic::geometry::Circle(0.2))
    }
  };

  const auto database = std::make_shared<rmf_traffic::schedule::Database>();
  std::vector<rmf_traffic::schedule::Participant> obstacles;

  auto start_time = rmf_traffic::Time::clock::now();

  auto new_obstacle = rmf_traffic::schedule::make_participant(
    rmf_traffic::schedule::ParticipantDescription{
      "obstacle_" + std::to_string(0),
      "obstacles",
      rmf_traffic::schedule::ParticipantDescription::Rx::Unresponsive,
      traits.profile()
    }, database);

  auto obstacle_trajectory = rmf_traffic::Trajectory();
  obstacle_trajectory.insert(start_time, {0, 0, 0}, {0, 0, 0});
  obstacle_trajectory.insert(rmf_traffic::time::apply_offset(start_time,
    5), {1, 1, 0}, {0, 0, 0});
  auto obstacle_route = rmf_traffic::Route("test_map", obstacle_trajectory);
  std::vector<rmf_traffic::Route> obstacle_routes;
  obstacle_routes.push_back(obstacle_route);
  new_obstacle.set(obstacle_routes);

  auto new_obstacle2 = rmf_traffic::schedule::make_participant(
    rmf_traffic::schedule::ParticipantDescription{
      "obstacle_" + std::to_string(1),
      "obstacles",
      rmf_traffic::schedule::ParticipantDescription::Rx::Unresponsive,
      traits.profile()
    }, database);

  auto obstacle_trajectory2 = rmf_traffic::Trajectory();
  obstacle_trajectory2.insert(start_time, {-4, -3, 0}, {0, 0, 0});
  obstacle_trajectory2.insert(rmf_traffic::time::apply_offset(start_time,
    10), {0, 0, 0}, {0, 0, 0});
  auto obstacle_route2 = rmf_traffic::Route("test_map", obstacle_trajectory2);
  std::vector<rmf_traffic::Route> obstacle_routes2;
  obstacle_routes2.push_back(obstacle_route2);
  new_obstacle2.set(obstacle_routes2);

  const auto obstacle_validator =
    rmf_traffic::agv::ScheduleRouteValidator::make(
    database, std::numeric_limits<std::size_t>::max(), traits.profile());

  auto kinodynamic_rrt_star =
    std::make_shared<rmf_freespace_planner::kinodynamic_rrt_star::KinodynamicRRTStar>(
    obstacle_validator,
    database,
    std::nullopt,
    std::make_unique<rmf_freespace_planner::kinodynamic_rrt_star::Posq>());

  std::thread estimated_cost_thread(get_estimated_cost, kinodynamic_rrt_star);

  auto start_timing = std::chrono::steady_clock::now();
  std::vector<rmf_traffic::Route> freespace_routes;

  struct Task
  {
    rmf_freespace_planner::FreespacePlanner::Start start;

    rmf_freespace_planner::FreespacePlanner::Goal goal;
  };

  Task tasks[2];

  tasks[0].start = {{1, 1, M_PI + M_PI_4}, start_time};
  tasks[0].goal = {{0, 0, M_PI + M_PI_4}};

  tasks[1].start = {{0, 0, M_PI + M_PI_4},
    rmf_traffic::time::apply_offset(start_time, 5)};
  tasks[1].goal = {{-4, -3, M_PI + M_PI_4}};

  for (const auto& task : tasks)
  {
    auto freespace_route =
      kinodynamic_rrt_star->plan(
      task.start,
      task.goal,
      traits,
      std::nullopt,
      "test_map");
    freespace_routes.insert(freespace_routes.end(),
      freespace_route.begin(), freespace_route.end());
  }

  auto end_timing = std::chrono::steady_clock::now();
  done = true;
  std::cout << "-------------------------" << std::endl;
  std::cout << "Time taken for make_plan " <<
    std::chrono::duration_cast<std::chrono::milliseconds>(
      end_timing - start_timing).count() / 1000.0 << "s" << std::endl;
  std::cout << "-------------------------" << std::endl;

  geometry_msgs::msg::PoseArray freespace_path;
  freespace_path.header.stamp = node->get_clock()->now();
  freespace_path.header.frame_id = "test_map";

  for (const auto& freespace_route : freespace_routes)
  {
    for (const auto& waypoint : freespace_route.trajectory())
    {
      geometry_msgs::msg::Pose pose;
      pose.position.x = waypoint.position().x();
      pose.position.y = waypoint.position().y();
      tf2::Quaternion quat;
      quat.setRPY(0, 0, waypoint.position().z());
      pose.orientation = tf2::toMsg(quat);
      freespace_path.poses.push_back(pose);
    }
  }

  for (int pub_index = 0; pub_index < 3; pub_index++) //Loop required otherwise rviz misses the messages sometimes
  {
    freespace_path_pub->publish(freespace_path);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header.stamp = node->get_clock()->now();
  pose_array.header.frame_id = "test_map";

  for (const auto& task : tasks)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = task.start.position.x();
    pose.position.y = task.start.position.y();
    pose.position.z = 0;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, task.start.position.z());
    pose.orientation = tf2::toMsg(quat);
    pose_array.poses.push_back(pose);

    pose.position.x = task.goal.position.x();
    pose.position.y = task.goal.position.y();
    pose.position.z = 0;
    quat.setRPY(0, 0, task.goal.position.z());
    pose.orientation = tf2::toMsg(quat);
    pose_array.poses.push_back(pose);
  }

  for (int pub_index = 0; pub_index < 3; pub_index++)
  {
    route_waypoints_pub->publish(pose_array);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  estimated_cost_thread.join();
  rclcpp::shutdown();
  return 0;
}