#ifndef ROMEA_CORE_PATH_PATH2D_HPP_
#define ROMEA_CORE_PATH_PATH2D_HPP_

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

}  // namespace romea


#endif  // ROMEA_CORE_PATH_PATH2D_HPP_
