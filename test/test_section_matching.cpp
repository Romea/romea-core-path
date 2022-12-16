// gtest
#include <gtest/gtest.h>

// local
#include "test_helper.h"
#include "test_utils.hpp"

//romea
#include "romea_core_path/PathSectionMatching2D.hpp"

class TestSectionMatching : public ::testing::Test
{
public :

  TestSectionMatching():
    maximalRadiusResearch(10),
    time_horizon(1)
  {
  }

  void SetUp() override
  {
    path = std::make_unique<romea::PathSection2D>(3);
    path->addWayPoints(loadWayPoints("/section.txt"));
  }

  std::unique_ptr<romea::PathSection2D> path;
  double maximalRadiusResearch;
  double time_horizon;
};

//-----------------------------------------------------------------------------
TEST_F(TestSectionMatching, testGlobalMatchingOK)
{
  romea::Pose2D vehiclePose;
  vehiclePose.position.x() = -8.2;
  vehiclePose.position.y() = 16.1;
  vehiclePose.yaw = 120/180.*M_PI;
  double vehicleSpeed = 0;

  auto matchedPoint = match(*path,
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
  romea::Pose2D vehiclePose;
  vehiclePose.position.x() = -5.2;
  vehiclePose.position.y() = 30.1;
  vehiclePose.yaw = -20/180.*M_PI;
  double vehicleSpeed = 1;

  auto matchedPoint = match(*path,
                            vehiclePose,
                            vehicleSpeed,
                            time_horizon,
                            maximalRadiusResearch);

  ASSERT_EQ(matchedPoint.has_value(), false);
}

//-----------------------------------------------------------------------------
TEST_F(TestSectionMatching, testGlobalMatchingFailedWhenVehicleIsGoingInWrongDirection)
{
  romea::Pose2D vehiclePose;
  vehiclePose.position.x() = -8.2;
  vehiclePose.position.y() = 16.1;
  vehiclePose.yaw = -20/180.*M_PI;
  double vehicleSpeed = -1;

  auto matchedPoint = match(*path,
                            vehiclePose,
                            vehicleSpeed,
                            time_horizon,
                            maximalRadiusResearch);

  ASSERT_EQ(matchedPoint.has_value(), false);
}

//-----------------------------------------------------------------------------
TEST_F(TestSectionMatching, testGlobalMatchingOKWhenVehicleIsJustAfterEndOfSection)
{
  romea::Pose2D vehiclePose;
  vehiclePose.position.x() = -136.4;
  vehiclePose.position.y() = 8.4;
  vehiclePose.yaw = -100/180.*M_PI;
  double vehicleSpeed = 1;

  auto matchedPoint = match(*path,
                            vehiclePose,
                            vehicleSpeed,
                            time_horizon,
                            maximalRadiusResearch);

  ASSERT_EQ(matchedPoint.has_value(), true);
  ASSERT_EQ(matchedPoint->curveIndex, path->size()-1);
  ASSERT_GT(matchedPoint->frenetPose.curvilinearAbscissa, path->getLength());

}

//-----------------------------------------------------------------------------
TEST_F(TestSectionMatching, testGlobalMatchingOKWhenVehicleIsJustBeforeBeginningOfSection)
{
  romea::Pose2D vehiclePose;
  vehiclePose.position.x() = 0;
  vehiclePose.position.y() = 2;
  vehiclePose.yaw = 120/180.*M_PI;
  double vehicleSpeed = 1;

  auto matchedPoint = match(*path,
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
  romea::Pose2D firstVehiclePose;
  firstVehiclePose.position.x() = -8.2;
  firstVehiclePose.position.y() = 16.1;
  firstVehiclePose.yaw = 120/180.*M_PI;
  double firstVehicleSpeed = 1;

  romea::Pose2D secondVehiclePose;
  secondVehiclePose.position.x() = -9.3;
  secondVehiclePose.position.y() = 17.2;
  secondVehiclePose.yaw = 130/180.*M_PI;
  double secondVehicleSpeed = 1;

  auto firstMatchedPoint = match(*path,
                                 firstVehiclePose,
                                 firstVehicleSpeed,
                                 time_horizon,
                                 maximalRadiusResearch);

  auto secondMatchedPoint = match(*path,
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
  romea::Pose2D firstVehiclePose;
  firstVehiclePose.position.x() = -8.2;
  firstVehiclePose.position.y() = 16.1;
  firstVehiclePose.yaw = 120/180.*M_PI;
  double firstVehicleSpeed = 1;

  romea::Pose2D secondVehiclePose;
  secondVehiclePose.position.x() = -9.3;
  secondVehiclePose.position.y() = 17.2;
  secondVehiclePose.yaw = 130/180.*M_PI;
  double secondVehicleSpeed = -1;


  auto firstMatchedPoint = match(*path,
                                 firstVehiclePose,
                                 firstVehicleSpeed,
                                 time_horizon,
                                 maximalRadiusResearch);

  auto secondMatchedPoint = match(*path,
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
  romea::Pose2D firstVehiclePose;
  firstVehiclePose.position.x() = -8.2;
  firstVehiclePose.position.y() = 16.1;
  firstVehiclePose.yaw = 120/180.*M_PI;
  double firstVehicleSpeed = 1;

  romea::Pose2D secondVehiclePose;
  secondVehiclePose.position.x() = -11.3;
  secondVehiclePose.position.y() = 17.6;
  secondVehiclePose.yaw = 130/180.*M_PI;
  double secondVehicleSpeed = 1;

  auto firstMatchedPoint = match(*path,
                                 firstVehiclePose,
                                 firstVehicleSpeed,
                                 time_horizon,
                                 maximalRadiusResearch);


  std::cout << "\n\n second match "<< std::endl;
  auto secondMatchedPoint = match(*path,
                                  secondVehiclePose,
                                  secondVehicleSpeed,
                                  *firstMatchedPoint,
                                  0.5,
                                  time_horizon,
                                  maximalRadiusResearch);

  EXPECT_EQ(firstMatchedPoint.has_value(), true);
  EXPECT_EQ(secondMatchedPoint.has_value(), false);
}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
