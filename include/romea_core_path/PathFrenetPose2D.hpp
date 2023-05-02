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

#ifndef ROMEA_CORE_PATH__PATHFRENETPOSE2D_HPP_
#define ROMEA_CORE_PATH__PATHFRENETPOSE2D_HPP_

// eigen
#include <Eigen/Core>

// std
#include <ostream>

namespace romea
{

struct PathFrenetPose2D
{
  PathFrenetPose2D();

  double curvilinearAbscissa;
  double lateralDeviation;
  double courseDeviation;
  Eigen::Matrix3d covariance;
};

std::ostream & operator<<(std::ostream & os, const PathFrenetPose2D & frenetPose);

PathFrenetPose2D reverse(const PathFrenetPose2D & frenetPose);

}  // namespace romea

#endif  // ROMEA_CORE_PATH__PATHFRENETPOSE2D_HPP_
