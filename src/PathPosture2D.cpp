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

#include "romea_core_path/PathPosture2D.hpp"
#include "romea_core_common/math/EulerAngles.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
PathPosture2D::PathPosture2D()
: position(Eigen::Vector2d::Zero()),
  course(0.),
  curvature(0.),
  dotCurvature(0.)
{
}

//-----------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const PathPosture2D & posture)
{
  os << "Posture " << std::endl;
  os << " x = " << posture.position.x() << std::endl;
  os << " y = " << posture.position.y() << std::endl;
  os << " course = " << posture.course << std::endl;
  os << " curvature " << posture.curvature << std::endl;
  os << " dot curvature " << posture.dotCurvature << std::endl;
  return os;
}

//-----------------------------------------------------------------------------
PathPosture2D reverse(const PathPosture2D & posture)
{
  PathPosture2D reversedPosture = posture;
  reversedPosture.course = between0And2Pi(posture.course + M_PI);
  reversedPosture.curvature = -posture.curvature;
  reversedPosture.dotCurvature = -posture.dotCurvature;
  return reversedPosture;
}

}  // namespace core
}  // namespace romea
