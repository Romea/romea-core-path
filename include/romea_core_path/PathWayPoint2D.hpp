// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROMEA_CORE_PATH__PATHWAYPOINT2D_HPP_
#define ROMEA_CORE_PATH__PATHWAYPOINT2D_HPP_

// Eigen
#include <Eigen/Core>

// std
#include <iostream>

namespace romea
{

struct PathWayPoint2D
{
  PathWayPoint2D();
  explicit PathWayPoint2D(const Eigen::Vector2d & position);

  Eigen::Vector2d position;
  double desired_speed;
};

std::ostream & operator<<(std::ostream & os, const PathWayPoint2D & wayPoint);

}  // namespace romea

#endif  // ROMEA_CORE_PATH__PATHWAYPOINT2D_HPP_
