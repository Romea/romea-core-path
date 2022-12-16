#ifndef ROMEA_CORE_PATH_PATHMATCHEDPOINT2D_HPP_
#define ROMEA_CORE_PATH_PATHMATCHEDPOINT2D_HPP_

// std
#include <ostream>

// romea
#include "romea_core_path/PathPosture2D.hpp"
#include "romea_core_path/PathFrenetPose2D.hpp"

namespace romea {

struct PathMatchedPoint2D {
  PathMatchedPoint2D();

  PathPosture2D pathPosture;
  PathFrenetPose2D frenetPose;
  double futureCurvature;
  double desiredSpeed;
  size_t sectionIndex;
  size_t curveIndex;
};

std::ostream& operator<<(std::ostream & os, const PathMatchedPoint2D & matchedPoint);

bool isOrderRespected(const PathMatchedPoint2D & p1, const PathMatchedPoint2D & p2);

}  // namespace romea

#endif // ROMEA_CORE_PATH_PATHMATCHEDPOINT2D_HPP_

