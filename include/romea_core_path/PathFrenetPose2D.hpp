#ifndef ROMEA_CORE_PATH_PATHFRENETPOSE2D_HPP_
#define ROMEA_CORE_PATH_PATHFRENETPOSE2D_HPP_

// eigen
#include <Eigen/Core>

// std
#include <ostream>

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

}  // namespace romea

#endif  // ROMEA_CORE_PATH_PATHFRENETPOSE2D_HPP_
