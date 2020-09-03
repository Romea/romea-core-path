//std
#include <fstream>
#include "test_helper.h"
#include <romea_common/containers/Eigen/VectorOfEigenVector.hpp>

//-----------------------------------------------------------------------------
romea::VectorOfEigenVector2d loadPath(const std::string &filename)
{
  romea::VectorOfEigenVector2d points;
  points.reserve(1000);

  std::string path = std::string(TEST_DIR);
  std::ifstream data(path +filename);

  Eigen::Vector2d p;
  while(!data.eof())
  {
    data >> p[0] >> p[1];
    points.push_back(p);
  }

  return points;
}
