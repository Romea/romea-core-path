#ifndef ROMEA_CORE_PATH_PATHPOSTURE2D_HPP_
#define ROMEA_CORE_PATH_PATHPOSTURE2D_HPP_

// eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// std
#include <ostream>

namespace romea {

struct PathPosture2D {
  PathPosture2D();

  Eigen::Vector2d position;
  double course;
  double curvature;
  double dotCurvature;
};

std::ostream& operator<<(std::ostream & os, const PathPosture2D & posture);

PathPosture2D reverse(const PathPosture2D & posture);

}  // namespace romea

#endif  // ROMEA_CORE_PATH_PATHPOSTURE2D_HPP_
