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

#include "romea_core_path/Path2D.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
Path2D::Path2D(
  const WayPoints & wayPoints,
  const double & interpolationWindowLength)
: sections_(),
  curvilinearAbscissa_(0),
  length_(0),
  interpolationWindowLength_(interpolationWindowLength)
{
  sections_.reserve(wayPoints.size());

  size_t global_point_index = 0;
  for (const auto & sectionWayPoints : wayPoints) {
    if (sections_.empty()) {
      sections_.emplace_back(interpolationWindowLength, 0, 0);
      sections_.back().addWayPoints(sectionWayPoints);
    } else {
      curvilinearAbscissa_.increment(sections_.back().getLength());

      double finalValue = curvilinearAbscissa_.finalValue();
      sections_.emplace_back(interpolationWindowLength, finalValue, global_point_index);
      sections_.back().addWayPoints(sectionWayPoints);
    }

    length_ += sections_.back().getLength();
    global_point_index += sections_.back().size();
  }

  for (size_t i = 0; i < sections_.size(); ++i) {
    for (size_t j = 0; j < sections_[i].size(); ++j) {
      sections_[i].getCurve(j);
    }
  }
}

Path2D::Path2D(
  const WayPoints & wayPoints,
  const double & interpolationWindowLength,
  const Annotations & annotations)
: Path2D(wayPoints, interpolationWindowLength)
{
  setAnnotations(annotations);

  // std::cout << "annotations:\n";
  // for(auto const & [i, a] : annotations_) {
  //   std::printf("  - index: %4lu, abscissa: %7.3f, type: %-12s, value: %s\n",
  //     i, a.abscissa, a.type.c_str(), a.value.c_str());
  // }
}

//-----------------------------------------------------------------------------
const PathSection2D & Path2D::getSection(const size_t & sectionIndex)const
{
  return sections_[sectionIndex];
}

//-----------------------------------------------------------------------------
double Path2D::getLength()const
{
  return sections_.empty() ? 0 : sections_.back().getCurvilinearAbscissa().finalValue();
}

//-----------------------------------------------------------------------------
size_t Path2D::size()const
{
  return sections_.size();
}

//-----------------------------------------------------------------------------
const Path2D::CurvilinearAbscissa & Path2D::getCurvilinearAbscissa()const
{
  return curvilinearAbscissa_;
}

//-----------------------------------------------------------------------------
PathSection2D & Path2D::addEmptySection()
{
  double abscissa = 0;
  size_t initial_index = 0;
  if (sections_.size()) {
    auto const & last_section = sections_.back();
    abscissa = last_section.getCurvilinearAbscissa().finalValue();
    initial_index = last_section.getInitialPointIndex() + last_section.size();
  }

  return sections_.emplace_back(interpolationWindowLength_, abscissa, initial_index);
}

void Path2D::setAnnotations(Annotations const & annotations)
{
  annotations_ = annotations;

  // set abscissa member of each annotation
  auto annotations_it = begin(annotations_);
  auto annotations_end = end(annotations_);
  for (auto const & section : sections_) {
    auto const & section_initial_index = section.getInitialPointIndex();
    auto const & abscissa = section.getCurvilinearAbscissa();

    for (std::size_t i = 0; i < section.size(); ++i) {
      if (annotations_it == annotations_end) {
        return;
      }

      while (annotations_it->first == section_initial_index + i) {
        annotations_it->second.abscissa = abscissa[i];
        ++annotations_it;
      }
    }
  }
}

}  // namespace core
}  // namespace romea
