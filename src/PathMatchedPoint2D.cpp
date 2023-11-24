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

// std
#include <limits>
#include <ostream>

// romea
#include "romea_core_path/PathMatchedPoint2D.hpp"
#include "romea_core_common/math/EulerAngles.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
PathMatchedPoint2D::PathMatchedPoint2D()
: pathPosture(),
  frenetPose(),
  futureCurvature(0.),
  desiredSpeed(std::numeric_limits<double>::quiet_NaN()),
  sectionIndex(std::numeric_limits<size_t>::max()),
  curveIndex(std::numeric_limits<size_t>::max())
{
}

//-----------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const PathMatchedPoint2D & matchedPoint)
{
  os << "Matched point " << std::endl;
  os << matchedPoint.pathPosture;
  os << matchedPoint.frenetPose;
  os << "speed : " << matchedPoint.desiredSpeed << std::endl;
  os << "section index : " << matchedPoint.sectionIndex << std::endl;
  os << "curve index : " << matchedPoint.curveIndex;
  return os;
}


//-----------------------------------------------------------------------------
bool isOrderRespected(const PathMatchedPoint2D & p1, const PathMatchedPoint2D & p2)
{
  if (p1.sectionIndex < p2.sectionIndex) {
    return p1.frenetPose.curvilinearAbscissa < p2.frenetPose.curvilinearAbscissa;
  } else {
    return p1.frenetPose.curvilinearAbscissa > p2.frenetPose.curvilinearAbscissa;
  }
}

}  // namespace core
}  // namespace romea
