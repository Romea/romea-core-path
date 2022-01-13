#include "romea_core_path/Path2D.hpp"
#include <iostream>

namespace romea {


//-----------------------------------------------------------------------------
Path2D::Path2D(const WayPoints & wayPoints,
               const double & interpolationWindowLength):
  sections_(),
  curvilinearAbscissa_(0),
  length_(0)
{
  sections_.reserve(wayPoints.size());

  for(const auto & sectionWayPoints : wayPoints)
  {
    if(sections_.empty())
    {
      sections_.emplace_back(interpolationWindowLength,0);
      sections_.back().addWayPoints(sectionWayPoints);
    }
    else
    {
      curvilinearAbscissa_.increment(sections_.back().getLength());
      sections_.emplace_back(interpolationWindowLength,curvilinearAbscissa_.finalValue());
      sections_.back().addWayPoints(sectionWayPoints);
    }

    length_+=sections_.back().getLength();

  }

  for(size_t i=0; i < sections_.size();++i)
  {
    for(size_t j=0; j < sections_[i].size();++j)
    {
      sections_[i].getCurve(j);
    }
  }

//  std::cout << sections_.size() <<std::endl;
//  std::cout << sections_[0].size() <<std::endl;

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
const Path2D::CurvilinearAbscissa &Path2D::getCurvilinearAbscissa()const
{
  return curvilinearAbscissa_;
}


}
