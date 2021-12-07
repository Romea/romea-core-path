#ifndef romea_PathMatchedPoint2D_hpp
#define romea_PathMatchedPoint2D_hpp

//romea
#include "PathPosture2D.hpp"
#include "PathFrenetPose2D.hpp"

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

}//End of namespace romea

#endif

