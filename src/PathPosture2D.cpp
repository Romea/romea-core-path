#include "romea_core_path/PathPosture2D.hpp"
#include <romea_core_common/math/EulerAngles.hpp>

namespace romea {

//-----------------------------------------------------------------------------
PathPosture2D::PathPosture2D():
  position(Eigen::Vector2d::Zero()),
  course(0.),
  curvature(0.),
  dotCurvature(0.)
{
}

//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream & os, const PathPosture2D & posture)
{
  os<< "Posture "<< std::endl;
  os<< " x = " << posture.position.x() <<std::endl;
  os<< " y = " << posture.position.y() <<std::endl;
  os<< " course = " << posture.course <<std::endl;
  os<< " curvature " << posture.curvature <<std::endl;
  os<< " dot curvature " << posture.dotCurvature <<std::endl;
  return os;
}

//-----------------------------------------------------------------------------
PathPosture2D reverse(const PathPosture2D & posture)
{
  PathPosture2D reversedPosture = posture;
  reversedPosture.course = romea::between0And2Pi(posture.course+M_PI);
  reversedPosture.curvature = - posture.curvature;
  reversedPosture.dotCurvature = -posture.dotCurvature;
  return reversedPosture;
}

}  // namespace romea
