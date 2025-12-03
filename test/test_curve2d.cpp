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
#include <memory>
#include <vector>

// gtest
#include "gtest/gtest.h"

// romea
#include "romea_core_path/PathCurve2D.hpp"
#include "romea_core_path/PathSection2D.hpp"

struct InfluenceOfLateralDeviationOnCurvilinearAbscissa : ::testing::Test
{
  void SetUp() override
  {
  }

};

TEST_F(InfluenceOfLateralDeviationOnCurvilinearAbscissa, lineX)
{
  romea::core::PathSection2D section{1.};
  for (double x = 0.; x < 100.01; x += .1) {
    section.addWayPoint({{x, 0.}});
  }

  Eigen::Vector2d pos{90., 0.};
  auto index = section.findIndex(pos.x());
  auto curve = section.getCurve(index);

  for (double y = -10.; y < 10.1; y += 1.) {
    pos.y() = y;
    auto abscissa = curve.findNearestCurvilinearAbscissa(pos);
    ASSERT_TRUE(abscissa);
    EXPECT_NEAR(*abscissa, pos.x(), 1e-2);
  }
}

TEST_F(InfluenceOfLateralDeviationOnCurvilinearAbscissa, lineY)
{
  romea::core::PathSection2D section{1.};
  for (double y = 0.; y < 100.01; y += .1) {
    section.addWayPoint({{y, 0.}});
  }

  Eigen::Vector2d pos{0., 90.};
  auto index = section.findIndex(pos.y());
  auto curve = section.getCurve(index);

  for (double x = -10.; x < 10.1; x += 1.) {
    pos.x() = x;
    auto abscissa = curve.findNearestCurvilinearAbscissa(pos);
    ASSERT_TRUE(abscissa);
    EXPECT_NEAR(*abscissa, pos.y(), 1e-2);
  }
}

TEST_F(InfluenceOfLateralDeviationOnCurvilinearAbscissa, diagonal)
{
  romea::core::PathSection2D section{1.};
  for (double x = 0.; x < 100.01; x += .07) {
    section.addWayPoint({{x, x}});
  }

  Eigen::Vector2d pos{90., 90.};
  double dist = pos.norm();
  auto index = section.findIndex(dist);
  auto curve = section.getCurve(index);

  for (double lat = -5.; lat < 5.1; lat += 1.) {
    Eigen::Vector2d p = pos + Eigen::Vector2d{lat, -lat};
    auto abscissa = curve.findNearestCurvilinearAbscissa(p);
    ASSERT_TRUE(abscissa);
    EXPECT_NEAR(*abscissa, dist, 1e-2);
  }
}

TEST_F(InfluenceOfLateralDeviationOnCurvilinearAbscissa, smallLineXFar)
{
  double start = 98.;
  double length = 4.;
  romea::core::PathSection2D section{1.};
  for (double x = start; x < start + length + 1e-2; x += .1) {
    section.addWayPoint({{x, 0.}});
  }

  Eigen::Vector2d pos{start + length / 2, 0.};
  auto index = section.findIndex(length / 2);
  auto curve = section.getCurve(index);

  for (double y = -10.; y < 10.1; y += 1.) {
    pos.y() = y;
    auto abscissa = curve.findNearestCurvilinearAbscissa(pos);
    ASSERT_TRUE(abscissa);
    EXPECT_NEAR(*abscissa, length / 2., 1e-2);
  }
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
