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
#include <map>
#include <vector>

// romea
#include "CumulativeSum.hpp"
#include "PathAnnotation.hpp"
#include "PathSection2D.hpp"

namespace romea
{

class Path2D
{
public:
  using WayPoints = std::vector<std::vector<PathWayPoint2D>>;
  using CurvilinearAbscissa = CumulativeSum<double, Eigen::aligned_allocator<double>>;
  using Sections = std::vector<PathSection2D>;
  using Annotations = std::multimap<std::size_t, PathAnnotation>;
  using AnnotationList = std::vector<PathAnnotation>;

public:
  Path2D(
    const WayPoints & wayPoints,
    const double & interpolationWindowLength);

  Path2D(
    const WayPoints & wayPoints,
    const double & interpolationWindowLength,
    const Annotations & annotations);

  const PathSection2D & getSection(const size_t & sectionIndex) const;

  const CurvilinearAbscissa & getCurvilinearAbscissa() const;

  double getLength() const;

  size_t size() const;

  PathSection2D & addEmptySection();

  const Sections & getSections() const {return sections_;}

  void setAnnotations(const Annotations & annotations);

  const Annotations & getAnnotations() const {return annotations_;}

private:
  Sections sections_;
  CurvilinearAbscissa curvilinearAbscissa_;
  double length_;
  double interpolationWindowLength_;
  Annotations annotations_;
};

}  // namespace romea

#endif  // ROMEA_CORE_PATH__PATH2D_HPP_
