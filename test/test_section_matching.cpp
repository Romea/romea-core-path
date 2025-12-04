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

// gtest
#include "gtest/gtest.h"

// romea
#include "romea_core_path/PathSectionMatching2D.hpp"

// local
#include "../test/test_helper.h"
#include "test_utils.hpp"

std::unique_ptr<romea::core::Path2D> makeUturnPath(
  double speed = std::numeric_limits<double>::quiet_NaN())
{
  std::vector<std::vector<romea::core::PathWayPoint2D>> wayPointLists;
  auto & waypoints = wayPointLists.emplace_back();

  // line (0,0) -> (4,0)
  for (double x = 0; x < 4.0; x += 0.1) {
    waypoints.emplace_back(Eigen::Vector2d{x, 0.}, speed);
  }
  // half circle (4,0) -> (4,2)
  for (double angle = -M_PI_2; angle < M_PI_2; angle += M_PI / 20.) {
    waypoints.emplace_back(Eigen::Vector2d{4. + std::cos(angle), 1. + std::sin(angle)}, speed);
  }
  // line (4,2) -> (0,2)
  for (double x = 4; x > -1e-3; x -= 0.1) {
    waypoints.emplace_back(Eigen::Vector2d{x, 2.}, speed);
  }

  auto path = std::make_unique<romea::core::Path2D>(wayPointLists, 2);
  return path;
}

class TestSectionMatching : public ::testing::Test
{
public:
  TestSectionMatching()
  : maximalRadiusResearch(10),
    time_horizon(1)
  {
  }

  void SetUp() override
  {
    path = std::make_unique<romea::core::PathSection2D>(3);
    path->addWayPoints(loadWayPoints("/section.txt"));
  }

  std::unique_ptr<romea::core::PathSection2D> path;
  double maximalRadiusResearch;
  double time_horizon;
};

//-----------------------------------------------------------------------------
TEST_F(TestSectionMatching, testGlobalMatchingOK)
{
  romea::core::Pose2D vehiclePose;
  vehiclePose.position.x() = -8.2;
  vehiclePose.position.y() = 16.1;
  vehiclePose.yaw = 120 / 180. * M_PI;
  double vehicleSpeed = 0;

  auto matchedPoint = match(
    *path,
    vehiclePose,
    vehicleSpeed,
    time_horizon,
    maximalRadiusResearch);

  ASSERT_EQ(matchedPoint.has_value(), true);
  EXPECT_NEAR(matchedPoint->pathPosture.position.x(), -8.10917, 0.001);
  EXPECT_NEAR(matchedPoint->pathPosture.position.y(), 16.2537, 0.001);
  EXPECT_NEAR(matchedPoint->pathPosture.course, 2.60782, 0.001);
  EXPECT_NEAR(matchedPoint->pathPosture.curvature, 0.0816672, 0.001);
  EXPECT_NEAR(matchedPoint->frenetPose.curvilinearAbscissa, 17.1109, 0.001);
  EXPECT_NEAR(matchedPoint->frenetPose.lateralDeviation, 0.17852, 0.001);
  EXPECT_NEAR(matchedPoint->frenetPose.courseDeviation, -0.51343, 0.001);
  EXPECT_EQ(matchedPoint->curveIndex, 177);
}

//-----------------------------------------------------------------------------
TEST_F(TestSectionMatching, testGlobalMatchingFailedWhenVehicleIsToFarFromPath)
{
  romea::core::Pose2D vehiclePose;
  vehiclePose.position.x() = -5.2;
  vehiclePose.position.y() = 30.1;
  vehiclePose.yaw = -20 / 180. * M_PI;
  double vehicleSpeed = 1;

  auto matchedPoint = match(
    *path,
    vehiclePose,
    vehicleSpeed,
    time_horizon,
    maximalRadiusResearch);

  ASSERT_EQ(matchedPoint.has_value(), false);
}

//-----------------------------------------------------------------------------
TEST_F(TestSectionMatching, testGlobalMatchingFailedWhenVehicleIsGoingInWrongDirection)
{
  romea::core::Pose2D vehiclePose;
  vehiclePose.position.x() = -8.2;
  vehiclePose.position.y() = 16.1;
  vehiclePose.yaw = -20 / 180. * M_PI;
  double vehicleSpeed = -1;

  auto matchedPoint = match(
    *path,
    vehiclePose,
    vehicleSpeed,
    time_horizon,
    maximalRadiusResearch);

  ASSERT_EQ(matchedPoint.has_value(), false);
}

//-----------------------------------------------------------------------------
TEST_F(TestSectionMatching, testGlobalMatchingOKWhenVehicleIsJustAfterEndOfSection)
{
  romea::core::Pose2D vehiclePose;
  vehiclePose.position.x() = -136.4;
  vehiclePose.position.y() = 8.4;
  vehiclePose.yaw = -100 / 180. * M_PI;
  double vehicleSpeed = 1;

  auto matchedPoint = match(
    *path,
    vehiclePose,
    vehicleSpeed,
    time_horizon,
    maximalRadiusResearch);

  ASSERT_EQ(matchedPoint.has_value(), true);
  ASSERT_EQ(matchedPoint->curveIndex, path->size() - 1);
  ASSERT_GT(matchedPoint->frenetPose.curvilinearAbscissa, path->getLength());
}

//-----------------------------------------------------------------------------
TEST_F(TestSectionMatching, testGlobalMatchingOKWhenVehicleIsJustBeforeBeginningOfSection)
{
  romea::core::Pose2D vehiclePose;
  vehiclePose.position.x() = 0;
  vehiclePose.position.y() = 2;
  vehiclePose.yaw = 120 / 180. * M_PI;
  double vehicleSpeed = 1;

  auto matchedPoint = match(
    *path,
    vehiclePose,
    vehicleSpeed,
    time_horizon,
    maximalRadiusResearch);

  ASSERT_EQ(matchedPoint.has_value(), true);
  ASSERT_EQ(matchedPoint->curveIndex, 0);
  ASSERT_LT(matchedPoint->frenetPose.curvilinearAbscissa, 0.);
}

//-----------------------------------------------------------------------------
TEST_F(TestSectionMatching, testLocalMatchingOK)
{
  romea::core::Pose2D firstVehiclePose;
  firstVehiclePose.position.x() = -8.2;
  firstVehiclePose.position.y() = 16.1;
  firstVehiclePose.yaw = 120 / 180. * M_PI;
  double firstVehicleSpeed = 1;

  romea::core::Pose2D secondVehiclePose;
  secondVehiclePose.position.x() = -9.3;
  secondVehiclePose.position.y() = 17.2;
  secondVehiclePose.yaw = 130 / 180. * M_PI;
  double secondVehicleSpeed = 1;

  auto firstMatchedPoint = match(
    *path,
    firstVehiclePose,
    firstVehicleSpeed,
    time_horizon,
    maximalRadiusResearch);

  auto secondMatchedPoint = match(
    *path,
    secondVehiclePose,
    secondVehicleSpeed,
    *firstMatchedPoint,
    10.,
    time_horizon,
    maximalRadiusResearch);

  ASSERT_EQ(firstMatchedPoint.has_value(), true);
  ASSERT_EQ(secondMatchedPoint.has_value(), true);
  EXPECT_NEAR(secondMatchedPoint->pathPosture.position.x(), -9.41664, 0.001);
  EXPECT_NEAR(secondMatchedPoint->pathPosture.position.y(), 16.9346, 0.001);
  EXPECT_NEAR(secondMatchedPoint->pathPosture.course, 2.72756, 0.001);
  EXPECT_NEAR(secondMatchedPoint->pathPosture.curvature, 0.104456, 0.001);
  EXPECT_NEAR(secondMatchedPoint->frenetPose.curvilinearAbscissa, 18.5847, 0.001);
  EXPECT_NEAR(secondMatchedPoint->frenetPose.lateralDeviation, -0.289936, 0.001);
  EXPECT_NEAR(secondMatchedPoint->frenetPose.courseDeviation, -0.458636, 0.001);
}

//-----------------------------------------------------------------------------
TEST_F(TestSectionMatching, testLocalMatchingOKWhenVehicleSpeedHasBeenReversed)
{
  romea::core::Pose2D firstVehiclePose;
  firstVehiclePose.position.x() = -8.2;
  firstVehiclePose.position.y() = 16.1;
  firstVehiclePose.yaw = 120 / 180. * M_PI;
  double firstVehicleSpeed = 1;

  romea::core::Pose2D secondVehiclePose;
  secondVehiclePose.position.x() = -9.3;
  secondVehiclePose.position.y() = 17.2;
  secondVehiclePose.yaw = 130 / 180. * M_PI;
  double secondVehicleSpeed = -1;


  auto firstMatchedPoint = match(
    *path,
    firstVehiclePose,
    firstVehicleSpeed,
    time_horizon,
    maximalRadiusResearch);

  auto secondMatchedPoint = match(
    *path,
    secondVehiclePose,
    secondVehicleSpeed,
    *firstMatchedPoint,
    10.,
    time_horizon,
    maximalRadiusResearch);

  ASSERT_EQ(firstMatchedPoint.has_value(), true);
  ASSERT_EQ(secondMatchedPoint.has_value(), true);
}

//-----------------------------------------------------------------------------
TEST_F(TestSectionMatching, localMatchingFailedWhenPositionIsTooFarFromTheLastMatchedPoint)
{
  romea::core::Pose2D firstVehiclePose;
  firstVehiclePose.position.x() = -8.2;
  firstVehiclePose.position.y() = 16.1;
  firstVehiclePose.yaw = 120 / 180. * M_PI;
  double firstVehicleSpeed = 1;

  romea::core::Pose2D secondVehiclePose;
  secondVehiclePose.position.x() = -11.3;
  secondVehiclePose.position.y() = 17.6;
  secondVehiclePose.yaw = 130 / 180. * M_PI;
  double secondVehicleSpeed = 1;

  auto firstMatchedPoint = match(
    *path,
    firstVehiclePose,
    firstVehicleSpeed,
    time_horizon,
    maximalRadiusResearch);


  auto secondMatchedPoint = match(
    *path,
    secondVehiclePose,
    secondVehicleSpeed,
    *firstMatchedPoint,
    0.5,
    time_horizon,
    maximalRadiusResearch);

  EXPECT_EQ(firstMatchedPoint.has_value(), true);
  EXPECT_EQ(secondMatchedPoint.has_value(), false);
}

TEST_F(TestSectionMatching, invertedRobotOrientationAtTheClosestPoint)
{
  auto path = makeUturnPath();
  const auto & section = path->getSections().front();
  romea::core::Pose2D pose;
  pose.position << 0.5, 0.99;  // the point is close to the first line of the path
  pose.yaw = -M_PI;
  double speed = 1.;
  double timeHorizon = 1.;

  auto matchedPoint = romea::core::match(section, pose, speed, timeHorizon, maximalRadiusResearch);
  ASSERT_TRUE(matchedPoint);

  std::size_t index = matchedPoint->curveIndex;
  EXPECT_NEAR(section.getX()[index], 0.5, 1e-3);
  EXPECT_NEAR(section.getY()[index], 2., 1e-3);
  EXPECT_GT(matchedPoint->frenetPose.lateralDeviation, 1.);
}

TEST_F(TestSectionMatching, invertedRobotOrientationAtTheClosestPointNegativeSpeedPath)
{
  auto path = makeUturnPath(-1.);
  const auto & section = path->getSections().front();

  // for (std::size_t i = 0; i < section.size(); ++i) {
  //   std::cout << section.getX()[i] << " " << section.getY()[i] << std::endl;
  // }

  romea::core::Pose2D pose;
  pose.position << 1., 1.01;  // the point is close to the second line of the path
  pose.yaw = -M_PI;
  double speed = -1.;
  double timeHorizon = 1.;

  auto matchedPoint = romea::core::match(section, pose, speed, timeHorizon, maximalRadiusResearch);
  ASSERT_TRUE(matchedPoint);

  std::size_t index = matchedPoint->curveIndex;
  EXPECT_NEAR(section.getX()[index], 1., 1e-3);
  EXPECT_NEAR(section.getY()[index], 0., 1e-3);
  EXPECT_GT(matchedPoint->frenetPose.lateralDeviation, 1.);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
