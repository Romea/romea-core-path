#ifndef romea_PathFrenetPose2D_hpp
#define romea_PathFrenetPose2D_hpp

//eigen
#include <Eigen/Core>

namespace romea {

struct PathFrenetPose2D {

  PathFrenetPose2D();

  double curvilinearAbscissa;
  double lateralDeviation;
  double courseDeviation;
  Eigen::Matrix3d covariance;
};

std::ostream& operator<<(std::ostream & os, const PathFrenetPose2D & frenetPose);

PathFrenetPose2D reverse(const PathFrenetPose2D & frenetPose);

}

#endif
