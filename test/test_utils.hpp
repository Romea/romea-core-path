//std
#include <fstream>
#include "test_helper.h"
#include "PathWayPoint2D.hpp"

//-----------------------------------------------------------------------------
std::vector<romea::PathWayPoint2D> loadWayPoints(const std::string &filename)
{
  std::vector<romea::PathWayPoint2D> wayPoints;
  wayPoints.reserve(1000);

  std::string path = std::string(TEST_DIR);
  std::ifstream data(path +filename);


  romea::PathWayPoint2D wayPoint;
  romea::PathWayPoint2D previousWayPoint;
  previousWayPoint.position << std::numeric_limits<double>::max(),std::numeric_limits<double>::max();
  previousWayPoint.desired_speed = 0;

  while(!data.eof())
  {

    data >> wayPoint.position[0] >> wayPoint.position[1]>>wayPoint.desired_speed;
//    std::cout << p.transpose() <<" "<< (p-pp).norm()<<" "<< points.size()<< std::endl;

    if( (wayPoint.position-previousWayPoint.position).norm()> 0.01)
    {
//      std::cout << p.transpose() << std::endl;
      wayPoints.push_back(wayPoint);
    }
    else
    {
      break;
    }

    previousWayPoint=wayPoint;
  }

  std::cout << " wayPoints.size() " << wayPoints.size()<< std::endl;
  return wayPoints;
}
