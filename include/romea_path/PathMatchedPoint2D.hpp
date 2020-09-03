#ifndef romea_PathMatchedPoint2D_hpp
#define romea_PathMatchedPoint2D_hpp

//romea
#include "PathPosture2D.hpp"
#include "PathFrenetPose2D.hpp"

//boost to replace by std optional
#include <boost/optional/optional.hpp>

namespace romea {

struct PathMatchedPoint2D {

public :

  using Opt = boost::optional<PathMatchedPoint2D> ;

  PathMatchedPoint2D();

  PathPosture2D pathPosture;
  PathFrenetPose2D frenetPose;
  size_t nearestPointIndex;

};

std::ostream& operator<<(std::ostream & os, const PathMatchedPoint2D & matchedPoint);

}//End of namespace romea

#endif

