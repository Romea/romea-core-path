#ifndef romea_Path2D_hpp
#define romea_Path2D_hpp

#include "PathSection2D.hpp"
#include "CumulativeSum.hpp"

namespace romea
{

class Path2D
{
public:

  using WayPoints = std::vector<std::vector<PathWayPoint2D>>;
  using CurvilinearAbscissa = CumulativeSum<double,Eigen::aligned_allocator<double>>;

public:

  Path2D(const WayPoints& wayPoints, const double & interpolationWindowLength);

  const PathSection2D & getSection(const size_t & sectionIndex)const;

  const CurvilinearAbscissa &getCurvilinearAbscissa()const;

  const double & getLength()const;

  size_t size()const;

private :

  std::vector<PathSection2D> sections_;
  CurvilinearAbscissa curvilinearAbscissa_;
  double length_;
};

}


#endif
