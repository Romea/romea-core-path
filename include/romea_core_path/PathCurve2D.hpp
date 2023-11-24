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

#ifndef ROMEA_CORE_PATH__PATHCURVE2D_HPP_
#define ROMEA_CORE_PATH__PATHCURVE2D_HPP_

// std
#include <optional>
#include <vector>

// romea
#include "PathPosture2D.hpp"
#include "PathFrenetPose2D.hpp"
#include "romea_core_common/math/Interval.hpp"

namespace romea
{
namespace core
{


class PathCurve2D
{
public:
  using Vector = std::vector<double, Eigen::aligned_allocator<double>>;

public:
  PathCurve2D();

  bool estimate(
    const Vector & X,
    const Vector & Y,
    const Vector & S,
    const Interval<size_t> & indexInterval,
    const Interval<double> & curvilinearAbscissaInterval);

  std::optional<double> findNearestCurvilinearAbscissa(
    const Eigen::Vector2d & vehiclePosition) const;

  double computeX(const double & curvilinearAbscissa)const;

  double computeY(const double & curvilinearAbscissa)const;

  double computeTangent(const double & curvilinearAbscissa)const;

  double computeCurvature(const double & curvilinearAbscissa)const;

  const Interval<double> & getCurvilinearAbscissaInterval()const;

  const Interval<size_t> & getIndexInterval()const;

private:
  Eigen::Array3d fxPolynomCoefficient_;
  Eigen::Array3d fyPolynomCoefficient_;

  Interval<size_t> indexInterval_;
  Interval<double> curvilinearAbscissaInterval_;
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_PATH__PATHCURVE2D_HPP_
