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

#ifndef ROMEA_CORE_PATH__PATHSECTION2D_HPP_
#define ROMEA_CORE_PATH__PATHSECTION2D_HPP_

// std
#include <optional>
#include <atomic>
#include <vector>

// romea
#include "romea_core_common/math/Interval.hpp"
#include "romea_core_common/containers/Eigen/VectorOfEigenVector.hpp"
#include "romea_core_common/containers/Eigen/DequeOfEigenVector.hpp"
#include "romea_core_path/CumulativeSum.hpp"
#include "romea_core_path/PathCurve2D.hpp"
#include "romea_core_path/PathWayPoint2D.hpp"


namespace romea
{


// TODO(jean) séparer la version static et online
class PathSection2D
{
public:
  using Vector = std::vector<double, Eigen::aligned_allocator<double>>;
  using CurvilinearAbscissa = CumulativeSum<double, Eigen::aligned_allocator<double>>;

public:
  PathSection2D(
    const double & interpolationWindowLength,
    const double & initialCurvilinearAbcissa = 0);

  void addWayPoint(const PathWayPoint2D & wayPoint);

  void addWayPoints(const std::vector<PathWayPoint2D> & wayPoints);

  const PathCurve2D & getCurve(const size_t & pointIndex) const;

  const Vector & getX()const;

  const Vector & getY()const;

  const CurvilinearAbscissa & getCurvilinearAbscissa()const;

  const Vector & getSpeeds() const;

  const double & getLength()const;

  void reserve(size_t n);

  size_t size()const;

  void clear();


  size_t findIndex(const double & value) const;

  size_t findIndex(
    const double & value,
    const size_t & startSearchIndex) const;

  Interval<size_t> findIntervalBoundIndexes(
    const size_t & intervalCenterIndex,
    const double & intervalWidth)const;

  Interval<size_t> findIntervalBoundIndexes(
    const size_t & intervalCenterIndex,
    const Interval<double> & interval)const;

private:
  void incrementCurvilinearAbscissa_();

  void computePathCurve_(const size_t & pointIndex)const;

  Interval<double> computeCurvilinearAbscissaInterval_(
    const size_t & intervalCenterIndex,
    const double & intervalWidth) const;

private:
  Vector X_;
  Vector Y_;
  CurvilinearAbscissa curvilinearAbscissa_;

  mutable std::vector<std::optional<PathCurve2D>> curves_;
  Vector speeds_;

  double interpolationWindowLength_;
  double length_;
};

}  // namespace romea

#endif  // ROMEA_CORE_PATH__PATHSECTION2D_HPP_
