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

#ifndef ROMEA_CORE_PATH__PATH2D_HPP_
#define ROMEA_CORE_PATH__PATH2D_HPP_

// std
#include <vector>

#include "PathSection2D.hpp"
#include "CumulativeSum.hpp"

namespace romea
{

class Path2D
{
public:
  using WayPoints = std::vector<std::vector<PathWayPoint2D>>;
  using CurvilinearAbscissa = CumulativeSum<double, Eigen::aligned_allocator<double>>;

public:
  Path2D(const WayPoints & wayPoints, const double & interpolationWindowLength);

  const PathSection2D & getSection(const size_t & sectionIndex)const;

  const CurvilinearAbscissa & getCurvilinearAbscissa()const;

  const double & getLength()const;

  size_t size()const;

private:
  std::vector<PathSection2D> sections_;
  CurvilinearAbscissa curvilinearAbscissa_;
  double length_;
};

}  // namespace romea


#endif  // ROMEA_CORE_PATH__PATH2D_HPP_
