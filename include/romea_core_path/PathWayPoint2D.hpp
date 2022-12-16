#ifndef ROMEA_CORE_PATH_PATHWAYPOINT2D_HPP
#define ROMEA_CORE_PATH_PATHWAYPOINT2D_HPP

//Eigen
#include <Eigen/Core>

// std
#include <iostream>

namespace romea
{

struct PathWayPoint2D
{
   PathWayPoint2D();
   explicit PathWayPoint2D(const Eigen::Vector2d & position);

   Eigen::Vector2d position;
   double desired_speed;
};

std::ostream& operator<<(std::ostream & os, const PathWayPoint2D & wayPoint);

}  // namespace romea

#endif  // ROMEA_CORE_PATH_PATHWAYPOINT2D_HPP
