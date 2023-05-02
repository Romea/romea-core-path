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


//-----------------------------------------------------------------------------
Path2D::Path2D(
  const WayPoints & wayPoints,
  const double & interpolationWindowLength)
: sections_(),
  curvilinearAbscissa_(0),
  length_(0)
{
  sections_.reserve(wayPoints.size());

  for (const auto & sectionWayPoints : wayPoints) {
    if (sections_.empty()) {
      sections_.emplace_back(interpolationWindowLength, 0);
      sections_.back().addWayPoints(sectionWayPoints);
    } else {
      curvilinearAbscissa_.increment(sections_.back().getLength());
      sections_.emplace_back(interpolationWindowLength, curvilinearAbscissa_.finalValue());
      sections_.back().addWayPoints(sectionWayPoints);
    }

    length_ += sections_.back().getLength();
  }

  for (size_t i = 0; i < sections_.size(); ++i) {
    for (size_t j = 0; j < sections_[i].size(); ++j) {
      sections_[i].getCurve(j);
    }
  }
}

//-----------------------------------------------------------------------------
const PathSection2D & Path2D::getSection(const size_t & sectionIndex)const
{
  return sections_[sectionIndex];
}

//-----------------------------------------------------------------------------
const double & Path2D::getLength()const
{
  return length_;
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

}  // namespace romea
