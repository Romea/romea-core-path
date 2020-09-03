//romea
#include "romea_path/PathMatchedPoint2D.hpp"

namespace romea {

//-----------------------------------------------------------------------------
PathMatchedPoint2D::PathMatchedPoint2D():
  pathPosture(),
  frenetPose(),
  nearestPointIndex(0)
{

}




//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream & os, const PathMatchedPoint2D & matchedPoint)
{
  os << "Matched point "<< std::endl;
  os << matchedPoint.pathPosture;
  os << matchedPoint.frenetPose;
  os << "nearest point index " << matchedPoint.nearestPointIndex << std::endl;
  return os;
}


}//End of namespace romea


