#ifndef romea_PathPosture2D_hpp
#define romea_PathPosture2D_hpp

//eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

//std
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

}

#endif
