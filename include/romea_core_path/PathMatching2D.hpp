#ifndef ROMEA_CORE_PATH_PATHMATCHING2D_HPP_
#define ROMEA_CORE_PATH_PATHMATCHING2D_HPP_

// std
#include <vector>
#include <optional>

// romea
#include "romea_core_path/Path2D.hpp"
#include "romea_core_path/PathMatchedPoint2D.hpp"
#include <romea_core_common/geometry/PoseAndTwist2D.hpp>


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



}  // namespace romea

#endif  // ROMEA_CORE_PATH_PATHMATCHING2D_HPP_
