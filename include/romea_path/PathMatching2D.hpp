#ifndef romea_ENUPathMatching2D_hpp
#define romea_ENUPathMatching2D_hpp

#include "Path2D.hpp"
#include "PathCurve2D.hpp"
#include "PathMatchedPoint2D.hpp"
#include <romea_localisation/PoseAndTwist2D.hpp>

namespace romea {

class PathMatching2D
{

public :

  PathMatching2D();

  PathMatching2D(const double & maximalResearchRadius);

  void setMaximalResearchRadius(const double & maximalResearchRadius);

  boost::optional<PathMatchedPoint2D> match(const Path2D & path,
                                            const Pose2D & vehiclePose);

  boost::optional<PathMatchedPoint2D> match(const Path2D & path,
                                            const Pose2D & vehiclePose,
                                            const PathMatchedPoint2D & previousMatchedPoint,
                                            const double & expectedTravelledDistance);

  double computeFutureCurvature(const Path2D & path,
                                const PathMatchedPoint2D & matchedPoint,
                                const double & linear_speed,
                                const double & time_horizon);

private :


  size_t findNearestPointIndex_(const Path2D & path,
                                const Eigen::Vector2d & vehiclePosition,
                                const Interval<size_t> indexRange)const;


  boost::optional<PathMatchedPoint2D> findMatchedPoint_(const Path2D & path,
                                                        const Pose2D &vehiclePose,
                                                        const size_t & nearestPointIndex);

private:

  double maximalResearchRadius_;

};

}

#endif
