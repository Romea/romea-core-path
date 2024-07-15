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

#ifndef ROMEA_CORE_PATH__PATHMATCHEDPOINT2D_HPP_
#define ROMEA_CORE_PATH__PATHMATCHEDPOINT2D_HPP_

// std
#include <optional>
#include <ostream>
#include <vector>

// romea
#include "romea_core_path/PathPosture2D.hpp"
#include "romea_core_path/PathFrenetPose2D.hpp"

namespace romea
{
namespace core
{

struct PathMatchedPoint2D
{
  PathMatchedPoint2D();

  PathPosture2D pathPosture;
  PathFrenetPose2D frenetPose;
  double futureCurvature;
  double desiredSpeed;
  size_t sectionIndex;
  double sectionMinimalCurvilinearAbscissa;
  double sectionMaximalCurvilinearAbscissa;
  size_t curveIndex;
};

std::ostream & operator<<(std::ostream & os, const PathMatchedPoint2D & matchedPoint);

double direction(const PathMatchedPoint2D & matchedPoint);

std::optional<PathMatchedPoint2D> findMatchedPointBySectionIndex(
  const std::vector<PathMatchedPoint2D> matchedPoints,
  const size_t & sectionIndex);


}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_PATH__PATHMATCHEDPOINT2D_HPP_
