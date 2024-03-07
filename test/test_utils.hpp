// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TEST_UTILS_HPP_
#define TEST_UTILS_HPP_

// std
#include <fstream>
#include <vector>
#include <string>
#include <limits>

// local
#include "../test/test_helper.h"

// romea
#include "romea_core_path/PathWayPoint2D.hpp"

//-----------------------------------------------------------------------------
std::vector<romea::core::PathWayPoint2D> loadWayPoints(const std::string & filename)
{
  std::vector<romea::core::PathWayPoint2D> wayPoints;
  wayPoints.reserve(1000);

  std::string path = std::string(TEST_DIR);
  std::ifstream data(path + filename);

  romea::core::PathWayPoint2D wayPoint;
  romea::core::PathWayPoint2D previousWayPoint;
  previousWayPoint.position[0] = std::numeric_limits<double>::max();
  previousWayPoint.position[1] = std::numeric_limits<double>::max();
  previousWayPoint.desired_speed = 0;

  while (!data.eof()) {
    data >> wayPoint.position[0] >> wayPoint.position[1] >> wayPoint.desired_speed;
    if ((wayPoint.position - previousWayPoint.position).norm() > 0.01) {
      // std::cout << wayPoint.position[0] << " " << wayPoint.position[1] << " " <<
      //   wayPoint.desired_speed << std::endl;
      wayPoints.push_back(wayPoint);
    } else {
      break;
    }
    previousWayPoint = wayPoint;
  }

  return wayPoints;
}


#endif  // TEST_UTILS_HPP_
