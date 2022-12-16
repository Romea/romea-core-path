// std
#include <fstream>
#include <vector>
#include <string>
#include <limits>

// local
#include "test_helper.h"

// romea
#include "romea_core_path/PathWayPoint2D.hpp"

//-----------------------------------------------------------------------------
std::vector<romea::PathWayPoint2D> loadWayPoints(const std::string &filename)
{
  std::vector<romea::PathWayPoint2D> wayPoints;
  wayPoints.reserve(1000);

  std::string path = std::string(TEST_DIR);
  std::ifstream data(path +filename);

  romea::PathWayPoint2D wayPoint;
  romea::PathWayPoint2D previousWayPoint;
  previousWayPoint.position[0] = std::numeric_limits<double>::max();
  previousWayPoint.position[1] = std::numeric_limits<double>::max();
  previousWayPoint.desired_speed = 0;

  while (!data.eof())
  {
    data >> wayPoint.position[0] >> wayPoint.position[1] >> wayPoint.desired_speed;
    if ((wayPoint.position-previousWayPoint.position).norm() > 0.01)
    {
      wayPoints.push_back(wayPoint);
    } else {
      break;
    }
    previousWayPoint = wayPoint;
  }

  return wayPoints;
}
