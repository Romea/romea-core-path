// std
#include <ostream>

// romea
#include "romea_core_path/PathMatchedPoint2D.hpp"
#include <romea_core_common/math/EulerAngles.hpp>

namespace romea {

//-----------------------------------------------------------------------------
PathMatchedPoint2D::PathMatchedPoint2D():
  pathPosture(),
  frenetPose(),
  futureCurvature(0.),
  desiredSpeed(std::numeric_limits<double>::quiet_NaN()),
  sectionIndex(std::numeric_limits<size_t>::max()),
  curveIndex(std::numeric_limits<size_t>::max())
{
}

//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream & os, const PathMatchedPoint2D & matchedPoint)
{
  os << "Matched point "<< std::endl;
  os << matchedPoint.pathPosture;
  os << matchedPoint.frenetPose;
  os << "speed : "  << matchedPoint.desiredSpeed << std::endl;
  os << "section index : "<< matchedPoint.sectionIndex << std::endl;
  os << "curve index : "<< matchedPoint.curveIndex;
  return os;
}


//-----------------------------------------------------------------------------
bool isOrderRespected(const PathMatchedPoint2D & p1, const PathMatchedPoint2D & p2)
{
  if (p1.sectionIndex < p2.sectionIndex)
  {
    return p1.frenetPose.curvilinearAbscissa < p2.frenetPose.curvilinearAbscissa;
  } else {
    return p1.frenetPose.curvilinearAbscissa > p2.frenetPose.curvilinearAbscissa;
  }
}


}  // namespace romea


