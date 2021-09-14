#ifndef romea_PathMatching2D_hpp
#define romea_PathMatching2D_hpp

//romea
#include "Path2D.hpp"
#include "PathMatchedPoint2D.hpp"
#include <romea_common/geometry/PoseAndTwist2D.hpp>

//std
#include <vector>
#include <optional>

namespace romea {


std::vector<PathMatchedPoint2D> match(const Path2D & path,
                                      const Pose2D & vehiclePose,
                                      const double & vehicleSpeed,
                                      const double & time_horizon,
                                      const double & researchRadius);

std::vector<PathMatchedPoint2D> match(const Path2D & path,
                                      const Pose2D & vehiclePose,
                                      const double & vehicleSpeed,
                                      const PathMatchedPoint2D & previousMatchedPoint,
                                      const double & expectedTravelledDistance,
                                      const double & time_horizon,
                                      const double & researchRadius);

std::vector<PathMatchedPoint2D> match(const Path2D & path,
                                      const Pose2D & vehiclePose,
                                      const double & vehicleSpeed,
                                      const size_t & previousSectionIndex,
                                      const size_t & previousCurveIndex,
                                      const Interval<double> & curvilinearAbscissaInterval,
                                      const double & time_horizon,
                                      const double &researchRadius);

size_t bestMatchedPointIndex(const std::vector<PathMatchedPoint2D> & matchedPoints,
                             const double & vehicleSpeed);


std::optional<PathMatchedPoint2D> match(const PathSection2D & section,
                                        const Pose2D & vehiclePose,
                                        const double & vehicleSpeed,
                                        const double & time_horizon,
                                        const double & researchRadius);

std::optional<PathMatchedPoint2D> match(const PathSection2D & section,
                                        const Pose2D & vehiclePose,
                                        const double & vehicleSpeed,
                                        const PathMatchedPoint2D & previousMatchedPoint,
                                        const double & expectedTravelledDistance,
                                        const double & time_horizon,
                                        const double & researchRadius);

std::optional<PathMatchedPoint2D> match(const PathSection2D & section,
                                        const Pose2D & vehiclePose,
                                        const double & vehicleSpeed,
                                        const size_t & previousCurveIndex,
                                        const Interval<double> & curvilinearAbscissaInterval,
                                        const double & time_horizon,
                                        const double &researchRadius);

std::optional<PathMatchedPoint2D> match(const PathCurve2D & curve,
                                        const Pose2D & vehiclePose,
                                        const double & desiredSpeed);

}

#endif
