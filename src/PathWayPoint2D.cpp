#include "romea_path/PathWayPoint2D.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
PathWayPoint2D::PathWayPoint2D():
  PathWayPoint2D(Eigen::Vector2d::Zero())
{

}

//-----------------------------------------------------------------------------
PathWayPoint2D::PathWayPoint2D(const Eigen::Vector2d & position):
  position(position),
  desired_speed(std::numeric_limits<double>::quiet_NaN())
{

}


//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream & os, const PathWayPoint2D & wayPoint)
{
  os << wayPoint.position.transpose() << wayPoint.desired_speed << std::endl;
  return os;
}


}
