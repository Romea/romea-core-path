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

// std
#include <vector>
#include <memory>

// gtest
#include "gtest/gtest.h"

// romea
#include "romea_core_path/PathMatching2D.hpp"

// local
#include "../test/test_helper.h"
#include "test_utils.hpp"


class TestPath : public ::testing::Test
{
public:
  TestPath() {}

  void SetUp() override
  {
    std::vector<std::vector<romea::core::PathWayPoint2D>> wayPoints(3);
    wayPoints[0] = loadWayPoints("/path11.txt");
    wayPoints[1] = loadWayPoints("/path12.txt");
    wayPoints[2] = loadWayPoints("/path13.txt");
    path = std::make_unique<romea::core::Path2D>(wayPoints, 3);
  }

  std::unique_ptr<romea::core::Path2D> path;
};


//-----------------------------------------------------------------------------
TEST_F(TestPath, isSizeOK)
{
  EXPECT_EQ(path->size(), 3);
  EXPECT_EQ(path->getSection(0).size(), 259);
  EXPECT_EQ(path->getSection(1).size(), 17);
  EXPECT_EQ(path->getSection(2).size(), 255);
}

//-----------------------------------------------------------------------------
TEST_F(TestPath, isLengthOK)
{
  EXPECT_NEAR(path->getSection(0).getLength(), 25.758932752740488, 0.001);
  EXPECT_NEAR(path->getSection(1).getLength(), 1.5998739194895477, 0.001);
  EXPECT_NEAR(path->getSection(2).getLength(), 25.296885714279526, 0.001);
  EXPECT_NEAR(path->getLength(), 52.655692386509557, 0.001);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
