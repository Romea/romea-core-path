#ifndef romea_PathWayPoint2D_hpp
#define romea_PathWayPoint2D_hpp

//Eigen
#include <Eigen/Core>

//std
#include <iostream>

namespace romea
{

struct PathWayPoint2D
{
   PathWayPoint2D();
   PathWayPoint2D(const Eigen::Vector2d & position);

   Eigen::Vector2d position;
   double desired_speed;
};


std::ostream& operator<<(std::ostream & os, const PathWayPoint2D & wayPoint);


}

#endif
