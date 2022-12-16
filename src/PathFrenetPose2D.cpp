// std
#include <ostream>

// romea
#include "romea_core_path/PathFrenetPose2D.hpp"
#include <romea_core_common/math/EulerAngles.hpp>


namespace romea {

//-----------------------------------------------------------------------------
PathFrenetPose2D::PathFrenetPose2D() :
  curvilinearAbscissa(0.),
  lateralDeviation(0.),
  courseDeviation(0.),
  covariance(Eigen::Matrix3d::Zero())
{
}

//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream & os, const PathFrenetPose2D & frenetPose)
{
  os << "Frenet pose "<< std::endl;
  os << " curvilinear abscissa = " << frenetPose.curvilinearAbscissa <<std::endl;
  os << " lateral deviation = " << frenetPose.lateralDeviation <<std::endl;
  os << " course deviation = " << frenetPose.courseDeviation <<std::endl;
  os << " Covariance " <<std::endl;
  os << frenetPose.covariance <<std::endl;
  return os;
}

//-----------------------------------------------------------------------------
PathFrenetPose2D reverse(const PathFrenetPose2D & frenetPose)
{
   PathFrenetPose2D reversedFrenetPose = frenetPose;
   reversedFrenetPose.lateralDeviation = -frenetPose.lateralDeviation;
   reversedFrenetPose.courseDeviation  = -romea::betweenMinusPiAndPi(
    frenetPose.courseDeviation+M_PI);
   return reversedFrenetPose;
}

}  // namespace romea
