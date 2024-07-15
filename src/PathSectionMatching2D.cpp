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
#include <optional>
#include <algorithm>
#include <cassert>
#include <iterator>
#include <iostream>
#include <list>

// romea
#include "romea_core_path/PathSectionMatching2D.hpp"
#include "romea_core_common/math/EulerAngles.hpp"
#include "romea_core_common/math/Algorithm.hpp"


namespace
{

//-----------------------------------------------------------------------------
size_t findNearestCurveIndex(
  const romea::core::PathSection2D & section,
  const Eigen::Vector2d & vehiclePosition,
  const romea::core::Interval<size_t> indexRange,
  const double & researchRadius)
{
  assert(indexRange.upper() < section.size());

  const auto X = section.getX();
  const auto Y = section.getY();

  size_t nearestPointIndex = section.size();
  double minimalDistance = researchRadius;

  for (size_t n = indexRange.lower(); n <= indexRange.upper(); ++n) {
    double distance = (vehiclePosition - Eigen::Vector2d(X[n], Y[n])).norm();

    if (distance < minimalDistance) {
      minimalDistance = distance;
      nearestPointIndex = n;
    }
  }

  return nearestPointIndex;
}


//-----------------------------------------------------------------------------
std::optional<romea::core::PathMatchedPoint2D> match_impl(
  const romea::core::PathSection2D & section,
  const romea::core::Pose2D & vehiclePose,
  const double & vehicleSpeed,
  const double & time_horizon,
  const romea::core::Interval<size_t> & rangeIndex,
  const double & researchRadius)
{
  std::optional<romea::core::PathMatchedPoint2D> matchedPoint;

  size_t nearestCurveIndex = findNearestCurveIndex(
    section,
    vehiclePose.position,
    rangeIndex,
    researchRadius);

  if (nearestCurveIndex != section.size()) {
    matchedPoint = match(
      section.getCurve(nearestCurveIndex),
      vehiclePose,
      section.getSpeeds()[nearestCurveIndex]);
  }

  if (matchedPoint.has_value()) {
    matchedPoint->curveIndex = findNearestCurveIndex(
      section, matchedPoint->pathPosture.position,
      section.getCurve(nearestCurveIndex).getIndexInterval(),
      researchRadius);

    matchedPoint->desiredSpeed = section.getSpeeds()[matchedPoint->curveIndex];

    double futureCurvilinearAbscissa = matchedPoint->frenetPose.curvilinearAbscissa +
      std::abs(vehicleSpeed) * time_horizon;

    size_t futureCurveIndex = section.findIndex(
      futureCurvilinearAbscissa,
      matchedPoint->curveIndex);

    matchedPoint->futureCurvature = section.getCurve(futureCurveIndex).
      computeCurvature(futureCurvilinearAbscissa);
  }

  return matchedPoint;
}


//-----------------------------------------------------------------------------
std::optional<romea::core::PathMatchedPoint2D> match_impl(
  const romea::core::PathSection2D & section,
  const romea::core::Pose2D & vehiclePose,
  const double & vehicleSpeed,
  const double & time_horizon,
  const double & researchRadius)
{
  return match_impl(
    section,
    vehiclePose,
    vehicleSpeed,
    time_horizon,
    romea::core::Interval<size_t>(0, section.size() - 1),
    researchRadius);
}


}  // namespace

namespace romea
{

namespace core
{

//-----------------------------------------------------------------------------
std::optional<PathMatchedPoint2D> match(
  const PathSection2D & section,
  const Pose2D & vehiclePose,
  const double & vehicleSpeed,
  const double & time_horizon,
  const double & researchRadius)
{
  return match_impl(section, vehiclePose, vehicleSpeed, time_horizon, researchRadius);
}


//-----------------------------------------------------------------------------
std::optional<PathMatchedPoint2D> match(
  const PathSection2D & section,
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
    section,
    vehiclePose,
    vehicleSpeed,
    previousMatchedPoint.curveIndex,
    Interval<double>(mins, maxs),
    time_horizon,
    researchRadius);
}

//-----------------------------------------------------------------------------
std::optional<PathMatchedPoint2D> match(
  const PathSection2D & section,
  const Pose2D & vehiclePose,
  const double & vehicleSpeed,
  const size_t & previousCurveIndex,
  const Interval<double> & curvilinearAbscissaInterval,
  const double & time_horizon,
  const double & researchRadius)
{
  Interval<size_t> rangeIndex = section.
    findIntervalBoundIndexes(previousCurveIndex, curvilinearAbscissaInterval);

  return match_impl(
    section,
    vehiclePose,
    vehicleSpeed,
    time_horizon,
    rangeIndex,
    researchRadius);
}


//-----------------------------------------------------------------------------
std::optional<PathMatchedPoint2D> match(
  const PathCurve2D & curve,
  const Pose2D & vehiclePose,
  const double & desiredSpeed)
{
  // std::cout << " search nearest curvilinear abscissa " << std::endl;
  auto nearestCurvilinearAbscissa = curve.findNearestCurvilinearAbscissa(vehiclePose.position);

  if (nearestCurvilinearAbscissa.has_value()) {
    // std::cout << " has nearest curvilinear abscissa" << std::endl;
    double xp = curve.computeX(nearestCurvilinearAbscissa.value());
    double yp = curve.computeY(nearestCurvilinearAbscissa.value());
    double tangent = curve.computeTangent(nearestCurvilinearAbscissa.value());
    double curvature = curve.computeCurvature(nearestCurvilinearAbscissa.value());

    const double & xv = vehiclePose.position.x();
    const double & yv = vehiclePose.position.y();
    const double & o = vehiclePose.yaw;
    const double & cost = std::cos(tangent);
    const double & sint = std::sin(tangent);

    double courseDeviation = betweenMinusPiAndPi(o - tangent);
    double lateralDeviation = (yv - yp) * cost - (xv - xp) * sint;

    double tangentOffset = desiredSpeed < 0 ? M_PI : 0;
    if (std::abs(betweenMinusPiAndPi(courseDeviation + tangentOffset)) < M_PI_2) {
      Eigen::Matrix3d J = Eigen::Matrix3d::Identity();
      J.block<2, 2>(0, 0) = eulerAngleToRotation2D(courseDeviation);
      Eigen::Matrix3d frenetPoseCovariance = J * vehiclePose.covariance * J.transpose();

      // Singularity
      if ((std::abs(curvature) > 10e-6) &&
        (std::abs(lateralDeviation - (1 / curvature)) <= 10e-6))
      {
        curvature = 0;
      }

      PathMatchedPoint2D matchedPoint;
      matchedPoint.pathPosture.position.x() = xp;
      matchedPoint.pathPosture.position.y() = yp;
      matchedPoint.pathPosture.course = tangent;
      matchedPoint.pathPosture.curvature = curvature;
      matchedPoint.frenetPose.curvilinearAbscissa = nearestCurvilinearAbscissa.value();
      matchedPoint.frenetPose.lateralDeviation = lateralDeviation;
      matchedPoint.frenetPose.courseDeviation = courseDeviation;
      matchedPoint.frenetPose.covariance = frenetPoseCovariance;
      return matchedPoint;
    }
    //    else
    //    {
    //      std::cout <<" cap rejection "<< std::endl;
    //    }
  }

  return {};
}


}  // namespace core
}  // namespace romea
