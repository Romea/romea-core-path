#ifndef ROMEA_CORE_PATH_PATHSECTIONMATCHING2D_HPP_
#define ROMEA_CORE_PATH_PATHSECTIONMATCHING2D_HPP_

// std
#include <vector>
#include <optional>

// romea
#include "romea_core_path/Path2D.hpp"
#include "romea_core_path/PathMatchedPoint2D.hpp"
#include <romea_core_common/geometry/PoseAndTwist2D.hpp>


namespace romea {


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

}  // namespace romea

#endif  // ROMEA_CORE_PATH_PATHSECTIONMATCHING2D_HPP_
