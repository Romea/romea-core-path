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
    section.addWayPoint({{0., y}});
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
  double start = 2846;
  double length = 4.;
  romea::core::PathSection2D section{1.};
  for (double x = start; x < start + length + 1e-2; x += .1) {
    section.addWayPoint({{x, -4928.}});
  }

  Eigen::Vector2d pos{start + length / 2, 0.};
  auto index = section.findIndex(length / 2);
  auto curve = section.getCurve(index);

  for (double offset = -10.; offset < 10.1; offset += 1.) {
    Eigen::Vector2d p = pos + Eigen::Vector2d{0., offset};
    auto abscissa = curve.findNearestCurvilinearAbscissa(p);
    ASSERT_TRUE(abscissa);
    EXPECT_NEAR(*abscissa, length / 2., 1e-2);
  }
}

TEST_F(InfluenceOfLateralDeviationOnCurvilinearAbscissa, smallLineYFar)
{
  double start = 873;
  double length = 12.;
  romea::core::PathSection2D section{1.};
  for (double y = start; y < start + length + 1e-2; y += .1) {
    section.addWayPoint({{439., y}});
  }

  Eigen::Vector2d pos{0., start + length / 2};
  auto index = section.findIndex(length / 2);
  auto curve = section.getCurve(index);

  for (double offset = -10.; offset < 10.1; offset += 1.) {
    Eigen::Vector2d p = pos + Eigen::Vector2d{offset, 0.};
    auto abscissa = curve.findNearestCurvilinearAbscissa(p);
    ASSERT_TRUE(abscissa);
    EXPECT_NEAR(*abscissa, length / 2., 1e-2);
  }
}

TEST_F(InfluenceOfLateralDeviationOnCurvilinearAbscissa, lineXNegative)
{
  double start = -100;
  double length = 7.;
  romea::core::PathSection2D section{1.};
  for (double x = start; x > start - length - 1e-2; x -= .1) {
    section.addWayPoint({{x, -20.}});
  }

  Eigen::Vector2d pos{start - length / 2, 0.};
  auto index = section.findIndex(length / 2);
  auto curve = section.getCurve(index);

  for (double offset = -10.; offset < 10.1; offset += 1.) {
    Eigen::Vector2d p = pos + Eigen::Vector2d{0., offset};
    auto abscissa = curve.findNearestCurvilinearAbscissa(p);
    ASSERT_TRUE(abscissa);
    EXPECT_NEAR(*abscissa, length / 2., 1e-2);
  }
}

TEST_F(InfluenceOfLateralDeviationOnCurvilinearAbscissa, circle)
{
  Eigen::Vector2d center{-4893., 3972};
  double radius = 6.;
  double step_angle = 0.1 / radius;
  romea::core::PathSection2D section{1.};
  for (double a = 0.; a < M_PI; a += step_angle) {
    section.addWayPoint({center + radius * Eigen::Vector2d{std::cos(a), std::sin(a)}});
  }

  Eigen::Vector2d pos = center + Eigen::Vector2d{0., radius};
  auto index = section.findIndex(radius * M_PI_2);
  auto curve = section.getCurve(index);

  for (double y = -3.; y < 3.01; y += 0.5) {
    pos.y() = y;
    auto abscissa = curve.findNearestCurvilinearAbscissa(pos);
    ASSERT_TRUE(abscissa);
    EXPECT_NEAR(*abscissa, radius * M_PI_2, 1e-2);
  }
}

//-----------------------------------------------------------------------------
struct CurvesAtExtremities : ::testing::Test
{
  void SetUp() override
  {
    Eigen::Vector2d start{246., -17.};
    double radius = 4.;
    for (double t = 0.; t < 20; t += 0.07 / radius) {
      section.addWayPoint({start + Eigen::Vector2d{radius * std::cos(t), radius * t}});
    }
  }

  romea::core::PathSection2D section{1.};
};

TEST_F(CurvesAtExtremities, beginning)
{
  std::size_t index = 0;
  Eigen::Vector2d pos = Eigen::Vector2d{section.getX()[index], section.getY()[index]};
  auto curve = section.getCurve(index);

  auto abscissa = curve.findNearestCurvilinearAbscissa(pos);
  ASSERT_TRUE(abscissa);
  EXPECT_NEAR(*abscissa, section.getCurvilinearAbscissa()[index], 1e-3);
}

TEST_F(CurvesAtExtremities, nearBeginning)
{
  std::size_t index = 4;
  Eigen::Vector2d pos = Eigen::Vector2d{section.getX()[index], section.getY()[index]};
  auto curve = section.getCurve(index);

  auto abscissa = curve.findNearestCurvilinearAbscissa(pos);
  ASSERT_TRUE(abscissa);
  EXPECT_NEAR(*abscissa, section.getCurvilinearAbscissa()[index], 1e-3);
}

TEST_F(CurvesAtExtremities, end)
{
  std::size_t index = section.size() - 1;
  Eigen::Vector2d pos = Eigen::Vector2d{section.getX()[index], section.getY()[index]};
  auto curve = section.getCurve(index);

  auto abscissa = curve.findNearestCurvilinearAbscissa(pos);
  ASSERT_TRUE(abscissa);
  EXPECT_NEAR(*abscissa, section.getCurvilinearAbscissa()[index], 1e-3);
}

TEST_F(CurvesAtExtremities, nearEnd)
{
  std::size_t index = section.size() - 6;
  Eigen::Vector2d pos = Eigen::Vector2d{section.getX()[index], section.getY()[index]};
  auto curve = section.getCurve(index);

  auto abscissa = curve.findNearestCurvilinearAbscissa(pos);
  ASSERT_TRUE(abscissa);
  EXPECT_NEAR(*abscissa, section.getCurvilinearAbscissa()[index], 1e-3);
}

TEST_F(CurvesAtExtremities, middle)
{
  std::size_t index = section.size() >> 1;
  Eigen::Vector2d pos = Eigen::Vector2d{section.getX()[index], section.getY()[index]};
  auto curve = section.getCurve(index);

  auto abscissa = curve.findNearestCurvilinearAbscissa(pos);
  ASSERT_TRUE(abscissa);
  EXPECT_NEAR(*abscissa, section.getCurvilinearAbscissa()[index], 1e-3);
}

TEST_F(CurvesAtExtremities, middle2)
{
  std::size_t index = 107 + (section.size() >> 1);
  Eigen::Vector2d pos = Eigen::Vector2d{section.getX()[index], section.getY()[index]};
  auto curve = section.getCurve(index);

  auto abscissa = curve.findNearestCurvilinearAbscissa(pos);
  ASSERT_TRUE(abscissa);
  EXPECT_NEAR(*abscissa, section.getCurvilinearAbscissa()[index], 1e-3);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
