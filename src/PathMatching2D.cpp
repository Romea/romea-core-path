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
#include <cassert>
#include <iterator>
#include <iostream>
#include <list>
#include <limits>
#include <vector>

// romea
#include "romea_core_path/PathMatching2D.hpp"
#include "romea_core_path/PathSectionMatching2D.hpp"
#include "romea_core_common/math/EulerAngles.hpp"
#include "romea_core_common/math/Algorithm.hpp"


namespace
{

//----------------------------------------------------------------------------
void match_impl(
  const romea::core::Path2D & path,
  const romea::core::Pose2D & vehiclePose,
  const double & vehicleSpeed,
  const double & time_horizon,
  const double & researchRadius,
  std::list<romea::core::PathMatchedPoint2D> & matchedPoints)
{
  for (size_t n = 0; n < path.size(); ++n) {
    // std::cout << "\n global match section " << n << std::endl;
    auto matchedPoint = match(
      path.getSection(n),
      vehiclePose,
      vehicleSpeed,
      time_horizon,
      researchRadius);

    if (matchedPoint.has_value()) {
      matchedPoint->sectionIndex = n;
      matchedPoints.push_back(*matchedPoint);
    }
  }
}

// //-----------------------------------------------------------------------------
// void match_impl(const romea::Path2D & path,
//                 const romea::Pose2D & vehiclePose,
//                 const double & vehicleSpeed,
//                 const size_t & sectionIndex,
//                 const size_t & curveIndex,
//                 const romea::Interval<double> & curvilinearAbscissaInterval,
//                 const double & time_horizon,
//                 const double &researchRadius,
//                 std::list<romea::PathMatchedPoint2D> & matchedPoints)
// {
//   const auto & section = path.getSection(sectionIndex);

//   auto matched_point = match(section,
//                              vehiclePose,
//                              vehicleSpeed,
//                              curveIndex,
//                              curvilinearAbscissaInterval,
//                              time_horizon,
//                              researchRadius);

//   if (matched_point.has_value())
//   {
//     matched_point->sectionIndex = sectionIndex;
//     matchedPoints.push_back(*matched_point);
//   }
// }


//-----------------------------------------------------------------------------
void match_impl(
  const romea::core::Path2D & path,
  const romea::core::Pose2D & vehiclePose,
  const double & vehicleSpeed,
  const size_t & sectionIndex,
  const size_t & curveIndex,
  const romea::core::Interval<double> & curvilinearAbscissaResearchInterval,
  const double & time_horizon,
  const double & researchRadius,
  std::list<romea::core::PathMatchedPoint2D> & matchedPoints)
{
  // std::cout << "\n\n local" << std::endl;
  const auto & section = path.getSection(sectionIndex);

  auto matched_point = match(
    section,
    vehiclePose,
    vehicleSpeed,
    curveIndex,
    curvilinearAbscissaResearchInterval,
    time_horizon,
    researchRadius);

  if (matched_point.has_value()) {
    matched_point->sectionIndex = sectionIndex;
    matched_point->sectionMinimalCurvilinearAbscissa =
      section.getCurvilinearAbscissa().initialValue();
    matched_point->sectionMaximalCurvilinearAbscissa =
      section.getCurvilinearAbscissa().finalValue();
    matchedPoints.push_back(*matched_point);
  }

  romea::core::Interval<size_t> rangeIndex = section.
    findIntervalBoundIndexes(curveIndex, curvilinearAbscissaResearchInterval);

  if (rangeIndex.lower() == 0 && sectionIndex != 0 &&
    curvilinearAbscissaResearchInterval.lower() < section.getCurvilinearAbscissa().initialValue())
  {
    const auto & previousSection = path.getSection(sectionIndex - 1);
    const size_t previousCurveIndex = path.getSection(sectionIndex - 1).size() - 1;

    auto previousMatchedPoint = match(
      previousSection,
      vehiclePose,
      vehicleSpeed,
      previousCurveIndex,
      curvilinearAbscissaResearchInterval,
      time_horizon,
      researchRadius);

    if (previousMatchedPoint.has_value()) {
      previousMatchedPoint->sectionIndex = sectionIndex - 1;
      previousMatchedPoint->sectionMinimalCurvilinearAbscissa =
        previousSection.getCurvilinearAbscissa().initialValue();
      previousMatchedPoint->sectionMaximalCurvilinearAbscissa =
        previousSection.getCurvilinearAbscissa().finalValue();
      matchedPoints.push_front(*previousMatchedPoint);
    }
  }

  if (rangeIndex.upper() == section.size() - 1 && sectionIndex != path.size() - 1 &&
    curvilinearAbscissaResearchInterval.upper() > section.getCurvilinearAbscissa().finalValue())
  {
    const auto & nextSection = path.getSection(sectionIndex + 1);

    auto nextMatchedPoint = match(
      nextSection,
      vehiclePose,
      vehicleSpeed,
      0,
      curvilinearAbscissaResearchInterval,
      time_horizon,
      researchRadius);

    if (nextMatchedPoint.has_value()) {
      nextMatchedPoint->sectionIndex = sectionIndex + 1;
      nextMatchedPoint->sectionMinimalCurvilinearAbscissa =
        nextSection.getCurvilinearAbscissa().initialValue();
      nextMatchedPoint->sectionMaximalCurvilinearAbscissa =
        nextSection.getCurvilinearAbscissa().finalValue();
      matchedPoints.push_back(*nextMatchedPoint);
    }
  }

  // reorder
  matchedPoints.sort(
    [vehicleSpeed](
      const romea::core::PathMatchedPoint2D & first,
      const romea::core::PathMatchedPoint2D & second)
    {
      double firstlateralDeviation = std::abs(first.frenetPose.lateralDeviation);
      if (std::signbit(first.desiredSpeed) != std::signbit(vehicleSpeed)) {
        firstlateralDeviation += 1000;
      }
      double secondlateralDeviation = std::abs(second.frenetPose.lateralDeviation);
      if (std::signbit(second.desiredSpeed) != std::signbit(vehicleSpeed)) {
        secondlateralDeviation += 1000;
      }

      return firstlateralDeviation < secondlateralDeviation;
    });
}

}  // namespace

namespace romea
{
namespace core
{

//----------------------------------------------------------------------------
std::vector<PathMatchedPoint2D> match(
  const Path2D & path,
  const Pose2D & vehiclePose,
  const double & vehicleSpeed,
  const double & time_horizon,
  const double & researchRadius)
{
  std::list<PathMatchedPoint2D> matchedPoints;

  match_impl(
    path,
    vehiclePose,
    vehicleSpeed,
    time_horizon,
    researchRadius,
    matchedPoints);

  return std::vector<PathMatchedPoint2D>(matchedPoints.begin(), matchedPoints.end());
}

//----------------------------------------------------------------------------
std::vector<PathMatchedPoint2D> match(
  const Path2D & path,
  const Pose2D & vehiclePose,
  const double & vehicleSpeed,
  const PathMatchedPoint2D & previousMatchedPoint,
  const double & expectedTravelledDistance,
  const double & time_horizon,
  const double & researchRadius)
{
  double s = previousMatchedPoint.frenetPose.curvilinearAbscissa;
  double mins = s - expectedTravelledDistance / 2.;
  double maxs = s + expectedTravelledDistance / 2.;

  return match(
    path,
    vehiclePose,
    vehicleSpeed,
    previousMatchedPoint.sectionIndex,
    previousMatchedPoint.curveIndex,
    Interval<double>(mins, maxs),
    time_horizon,
    researchRadius);
}


//----------------------------------------------------------------------------
std::vector<PathMatchedPoint2D> match(
  const Path2D & path,
  const Pose2D & vehiclePose,
  const double & vehicleSpeed,
  const size_t & previousSectionIndex,
  const size_t & previousCurveIndex,
  const Interval<double> & curvilinearAbscissaResearchInterval,
  const double & time_horizon,
  const double & researchRadius)
{
  std::list<PathMatchedPoint2D> matchedPoints;

  match_impl(
    path,
    vehiclePose,
    vehicleSpeed,
    previousSectionIndex,
    previousCurveIndex,
    curvilinearAbscissaResearchInterval,
    time_horizon,
    researchRadius,
    matchedPoints);

  return std::vector<PathMatchedPoint2D>(matchedPoints.begin(), matchedPoints.end());
}

}  // namespace core
}  // namespace romea
