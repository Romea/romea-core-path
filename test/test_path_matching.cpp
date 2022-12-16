// gtest
#include <gtest/gtest.h>

#include "test_helper.h"
#include "test_utils.hpp"

// romea
#include "romea_core_path/PathMatching2D.hpp"

class TestPathMatching : public ::testing::Test
{
public :

  TestPathMatching():
    maximalRadiusResearch(10),
    timeHorizon(0.2)
  {
  }

  void load(const std::string & path_name)
  {
    std::vector<std::vector<romea::PathWayPoint2D>> wayPoints(3);
    wayPoints[0] = loadWayPoints("/"+path_name+"1.txt");
    wayPoints[1] = loadWayPoints("/"+path_name+"2.txt");
    wayPoints[2] = loadWayPoints("/"+path_name+"3.txt");
    path = std::make_unique<romea::Path2D>(wayPoints, 3);
  }

  std::unique_ptr<romea::Path2D> path;
  double timeHorizon;
  double maximalRadiusResearch;
};

//-----------------------------------------------------------------------------
TEST_F(TestPathMatching, testGlobalMatchingOKWhenVehicleIsStopped)
{
  load("path1");
  romea::Pose2D vehiclePose;
  vehiclePose.position.x()= 18.3;
  vehiclePose.position.y()= -4;
  vehiclePose.yaw = 0.278;
  double vehicleSpeed = 0;

  auto matchedPoints = romea::match(*path,
                                    vehiclePose,
                                    vehicleSpeed,
                                    timeHorizon,
                                    maximalRadiusResearch);

  ASSERT_EQ(matchedPoints.size(), 1);
  EXPECT_EQ(matchedPoints.front().sectionIndex, 0);
  EXPECT_EQ(matchedPoints.front().curveIndex, 187);
}

//-----------------------------------------------------------------------------
TEST_F(TestPathMatching, testGlobalMatchingOKWhenVehicleGoesForward)
{
  load("path1");
  romea::Pose2D vehiclePose;
  vehiclePose.position.x()= 18.3;
  vehiclePose.position.y()= -4;
  vehiclePose.yaw = 0.278;
  double vehicleSpeed = 1.;

  auto matchedPoints = romea::match(*path,
                                    vehiclePose,
                                    vehicleSpeed,
                                    timeHorizon,
                                    maximalRadiusResearch);

  ASSERT_EQ(matchedPoints.size(), 1);
  EXPECT_EQ(matchedPoints.front().sectionIndex, 0);
  EXPECT_EQ(matchedPoints.front().curveIndex, 187);
}

//-----------------------------------------------------------------------------
TEST_F(TestPathMatching, testGlobalMatchingFailedWhenVehicleGoesBackward)
{
  load("path1");
  romea::Pose2D vehiclePose;
  vehiclePose.position.x()= 18.3;
  vehiclePose.position.y()= -4;
  vehiclePose.yaw = 0.278;
  double vehicleSpeed =-1.;

  auto matchedPoints = romea::match(*path,
                                    vehiclePose,
                                    vehicleSpeed,
                                    timeHorizon,
                                    maximalRadiusResearch);

  ASSERT_EQ(matchedPoints.size(), 1);
  EXPECT_EQ(matchedPoints.front().sectionIndex, 0);
  EXPECT_EQ(matchedPoints.front().curveIndex, 187);
}

//-----------------------------------------------------------------------------
TEST_F(TestPathMatching, testGlobalMatchingFailedWhenVehicleIsToFarFromPath)
{
  load("path1");
  romea::Pose2D vehiclePose;
  vehiclePose.position.x() = 18.3;
  vehiclePose.position.y() = -40;
  vehiclePose.yaw = -0.8;
  double vehicleSpeed = 1.;

  auto matchedPoints = match(*path,
                             vehiclePose,
                             vehicleSpeed,
                             timeHorizon,
                             maximalRadiusResearch);

  ASSERT_EQ(matchedPoints.empty(), true);
}


//-----------------------------------------------------------------------------
TEST_F(TestPathMatching, testLocalMatchingOK)
{
  load("path1");
  romea::Pose2D firstVehiclePose;
  firstVehiclePose.position.x()= 18.3;
  firstVehiclePose.position.y()= -4;
  firstVehiclePose.yaw = 0.278;
  double firstVehicleSpeed = 1.;

  romea::Pose2D secondVehiclePose;
  secondVehiclePose.position.x() = 22.1;
  secondVehiclePose.position.y() = -4;
  secondVehiclePose.yaw = -0.4;
  double secondVehicleSpeed = 1.;

  auto firstMatchedPoints = match(*path,
                                  firstVehiclePose,
                                  firstVehicleSpeed,
                                  timeHorizon,
                                  maximalRadiusResearch);

  auto secondMatchedPoints = match(*path,
                                   secondVehiclePose,
                                   secondVehicleSpeed,
                                   firstMatchedPoints.front(),
                                   10.,
                                   timeHorizon,
                                   maximalRadiusResearch);

  ASSERT_EQ(firstMatchedPoints.size(), 1);
  ASSERT_EQ(firstMatchedPoints.size(), 1);
  EXPECT_EQ(firstMatchedPoints.front().sectionIndex, 0);
  EXPECT_EQ(firstMatchedPoints.front().curveIndex, 187);
  EXPECT_EQ(secondMatchedPoints.front().sectionIndex, 0);
  EXPECT_EQ(secondMatchedPoints.front().curveIndex, 225);

  EXPECT_NEAR(secondMatchedPoints.front().pathPosture.position.x(), 22.1269, 0.001);
  EXPECT_NEAR(secondMatchedPoints.front().pathPosture.position.y(), -3.8858, 0.001);
  EXPECT_NEAR(secondMatchedPoints.front().pathPosture.course, -0.2320, 0.001);
  EXPECT_NEAR(secondMatchedPoints.front().pathPosture.curvature, -0.1340, 0.001);
  EXPECT_NEAR(secondMatchedPoints.front().frenetPose.curvilinearAbscissa, 22.4621, 0.001);
  EXPECT_NEAR(secondMatchedPoints.front().frenetPose.lateralDeviation, -0.1173, 0.001);
  EXPECT_NEAR(secondMatchedPoints.front().frenetPose.courseDeviation, -0.1679, 0.001);
}

//-----------------------------------------------------------------------------
TEST_F(TestPathMatching, testGlobalMatchingInBird)
{
  load("path2");
  romea::Pose2D pose;
  pose.position.x() =  -1.1099182296584995;
  pose.position.y() =  36.015842580693125;
  pose.yaw = 1.0595642624260422;
  double speed = 0.86174230688888409;

  auto matchedPoints = romea::match(*path,
                                    pose,
                                    speed,
                                    timeHorizon,
                                    maximalRadiusResearch);

  EXPECT_EQ(matchedPoints.size(), 2);
  EXPECT_EQ(matchedPoints.front().sectionIndex, 0);
  EXPECT_EQ(matchedPoints.front().curveIndex, 134);

  EXPECT_EQ(matchedPoints.back().sectionIndex, 1);
  EXPECT_EQ(matchedPoints.back().curveIndex, 7);

}

//-----------------------------------------------------------------------------
TEST_F(TestPathMatching, testLocalMatchingOKBirdDecelerrate)
{
  load("path1");
  romea::Pose2D firstVehiclePose;
  firstVehiclePose.position.x()= 22.1;
  firstVehiclePose.position.y()= -4;
  firstVehiclePose.yaw = -0.4;
  double firstVehicleSpeed = 1.;

  romea::Pose2D secondVehiclePose;
  secondVehiclePose.position.x() = 24.5;
  secondVehiclePose.position.y() = -6;
  secondVehiclePose.yaw = -1.2;
  double secondVehicleSpeed = 1.;

  auto firstMatchedPoints = match(*path,
                                  firstVehiclePose,
                                  firstVehicleSpeed,
                                  timeHorizon,
                                  maximalRadiusResearch);

  auto secondMatchedPoints = match(*path,
                                   secondVehiclePose,
                                   secondVehicleSpeed,
                                   firstMatchedPoints.front(),
                                   10.,
                                   timeHorizon,
                                   maximalRadiusResearch);

  ASSERT_EQ(firstMatchedPoints.size(), 2);
  EXPECT_EQ(firstMatchedPoints.front().sectionIndex, 0);
  EXPECT_EQ(firstMatchedPoints.front().curveIndex, 225);
  EXPECT_EQ(firstMatchedPoints.back().sectionIndex, 1);
  EXPECT_EQ(firstMatchedPoints.back().curveIndex, 11);

  EXPECT_LT(firstMatchedPoints.front().frenetPose.curvilinearAbscissa,
            firstMatchedPoints.back().frenetPose.curvilinearAbscissa);

  ASSERT_EQ(secondMatchedPoints.size(), 2);
  EXPECT_EQ(secondMatchedPoints.front().sectionIndex, 0);
  EXPECT_EQ(secondMatchedPoints.front().curveIndex, 258);
  EXPECT_EQ(secondMatchedPoints.back().sectionIndex, 1);
  EXPECT_EQ(secondMatchedPoints.back().curveIndex, 0);

  EXPECT_GT(secondMatchedPoints.front().frenetPose.curvilinearAbscissa,
            secondMatchedPoints.back().frenetPose.curvilinearAbscissa);
}


//-----------------------------------------------------------------------------
TEST_F(TestPathMatching, testLocalMatchingOKBirdAccelerate)
{
  load("path1");
  romea::Pose2D firstVehiclePose;
  firstVehiclePose.position.x()= 22.1;
  firstVehiclePose.position.y()= -4;
  firstVehiclePose.yaw = -0.4;
  double firstVehicleSpeed = 1.;

  romea::Pose2D secondVehiclePose;
  secondVehiclePose.position.x() = 24.5;
  secondVehiclePose.position.y() = -6;
  secondVehiclePose.yaw = -1.2;
  double secondVehicleSpeed = -1.;

  auto firstMatchedPoints = match(*path,
                                  firstVehiclePose,
                                  firstVehicleSpeed,
                                  timeHorizon,
                                  maximalRadiusResearch);

  auto secondMatchedPoints = match(*path,
                                   secondVehiclePose,
                                   secondVehicleSpeed,
                                   firstMatchedPoints.front(),
                                   10.,
                                   timeHorizon,
                                   maximalRadiusResearch);

  EXPECT_EQ(firstMatchedPoints.size(), 2);
  EXPECT_EQ(firstMatchedPoints.front().sectionIndex, 0);
  EXPECT_EQ(firstMatchedPoints.front().curveIndex, 225);
  EXPECT_EQ(firstMatchedPoints.back().sectionIndex, 1);
  EXPECT_EQ(firstMatchedPoints.back().curveIndex, 11);

  EXPECT_LT(firstMatchedPoints.front().frenetPose.curvilinearAbscissa,
            firstMatchedPoints.back().frenetPose.curvilinearAbscissa);

  EXPECT_EQ(secondMatchedPoints.size(), 2);
  EXPECT_EQ(secondMatchedPoints.front().sectionIndex, 0);
  EXPECT_EQ(secondMatchedPoints.front().curveIndex, 258);
  EXPECT_EQ(secondMatchedPoints.back().sectionIndex, 1);
  EXPECT_EQ(secondMatchedPoints.back().curveIndex, 0);

  EXPECT_GT(secondMatchedPoints.front().frenetPose.curvilinearAbscissa,
            secondMatchedPoints.back().frenetPose.curvilinearAbscissa);
}

//-----------------------------------------------------------------------------
TEST_F(TestPathMatching, localMatchingFailedWhenPositionIsTooFarFromTheLastMatchedPoint)
{
  load("path1");
  romea::Pose2D firstVehiclePose;
  firstVehiclePose.position.x() = 22.1;
  firstVehiclePose.position.y() = -4;
  firstVehiclePose.yaw = -0.4;
  double firstVehicleSpeed = 1.;

  romea::Pose2D secondVehiclePose;
  secondVehiclePose.position.x() = 24.5;
  secondVehiclePose.position.y() = -6;
  secondVehiclePose.yaw = -1.2;
  double secondVehicleSpeed = -11.;

  auto firstMatchedPoints = match(*path,
                                  firstVehiclePose,
                                  firstVehicleSpeed,
                                  timeHorizon,
                                  maximalRadiusResearch);

  auto secondMatchedPoints = match(*path,
                                   secondVehiclePose,
                                   secondVehicleSpeed,
                                   firstMatchedPoints.front(),
                                   0.5,
                                   timeHorizon,
                                   maximalRadiusResearch);

  EXPECT_EQ(firstMatchedPoints.size(), 2);
  EXPECT_EQ(secondMatchedPoints.empty(), true);
}

////-----------------------------------------------------------------------------
//TEST_F(TestPathMatching, testLocalMatchingOKBirdDecelerrate2)
//{
//  romea::Pose2D firstVehiclePose;
//  firstVehiclePose.position.x()= 24.5;
//  firstVehiclePose.position.y()= -6;
//  firstVehiclePose.yaw= -0.4;
//  double firstVehicleSpeed=-1.;

//  romea::Pose2D secondVehiclePose;
//  secondVehiclePose.position.x() = 24.5;
//  secondVehiclePose.position.y() = -4.2;
//  secondVehiclePose.yaw = -1.2;
//  double secondVehicleSpeed=-1.;

//  auto firstMatchedPoint = match(path,
//                                 firstVehiclePose,
//                                 firstVehicleSpeed,
//                                 maximalRadiusResearch);

//  auto secondMatchedPoint = match(path,
//                                  secondVehiclePose,
//                                  secondVehicleSpeed,
//                                  *firstMatchedPoint,
//                                  10.,
//                                  maximalRadiusResearch);

//  ASSERT_EQ(firstMatchedPoint.is_initialized(),true);
//  ASSERT_EQ(secondMatchedPoint.is_initialized(),true);
//  EXPECT_EQ(firstMatchedPoint->sectionIndex,1);
//  EXPECT_EQ(firstMatchedPoint->curveIndex,0);
//  EXPECT_EQ(secondMatchedPoint->sectionIndex,1);
//  EXPECT_EQ(secondMatchedPoint->curveIndex,16);
//  EXPECT_GT(secondMatchedPoint->frenetPose.curvilinearAbscissa,path.getCurvilinearAbscissa().vector()[2]);
//}


////-----------------------------------------------------------------------------
//TEST_F(TestPathMatching, testLocalMatchingOKBirdAccelerrate2)
//{
//  romea::Pose2D firstVehiclePose;
//  firstVehiclePose.position.x()= 24.5;
//  firstVehiclePose.position.y()= -6;
//  firstVehiclePose.yaw= -0.4;
//  double firstVehicleSpeed=-1.;

//  romea::Pose2D secondVehiclePose;
//  secondVehiclePose.position.x() = 24.5;
//  secondVehiclePose.position.y() = -4.2;
//  secondVehiclePose.yaw = -1.2;
//  double secondVehicleSpeed=1.;

//  auto firstMatchedPoint = match(path,
//                                 firstVehiclePose,
//                                 firstVehicleSpeed,
//                                 maximalRadiusResearch);

//  auto secondMatchedPoint = match(path,
//                                  secondVehiclePose,
//                                  secondVehicleSpeed,
//                                  *firstMatchedPoint,
//                                  10.,
//                                  maximalRadiusResearch);

//  ASSERT_EQ(firstMatchedPoint.is_initialized(),true);
//  ASSERT_EQ(secondMatchedPoint.is_initialized(),true);
//  EXPECT_EQ(firstMatchedPoint->sectionIndex,1);
//  EXPECT_EQ(firstMatchedPoint->curveIndex,0);
//  EXPECT_EQ(secondMatchedPoint->sectionIndex,2);
//  EXPECT_EQ(secondMatchedPoint->curveIndex,0);
//  EXPECT_LT(secondMatchedPoint->frenetPose.curvilinearAbscissa,path.getCurvilinearAbscissa().vector()[2]);
//}


//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
