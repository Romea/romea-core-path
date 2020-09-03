// gtest
#include <gtest/gtest.h>
#include "test_helper.h"

//romea
#include "romea_path/PathMatching2D.hpp"
#include "test_utils.hpp"

class TestPathMatching : public ::testing::Test
{
public :

  TestPathMatching(){}

  virtual void SetUp() override
  {
    path.setInterpolationWindowLength(3);
    path_matching.setMaximalResearchRadius(10);
    path.load(loadPath("/path.txt"));
  }

  romea::Path2D path;
  romea::PathMatching2D path_matching;
};

//-----------------------------------------------------------------------------
TEST_F(TestPathMatching, testGlobalMatchingOK)
{
  romea::Pose2D vehiclePose;
  vehiclePose.position.x() = -8.2;
  vehiclePose.position.y() = 16.1;
  vehiclePose.yaw = 120/180.*M_PI;

  auto matchedPoint = path_matching.match(path,vehiclePose);

  ASSERT_EQ(matchedPoint.is_initialized(),true);
  EXPECT_NEAR(matchedPoint->pathPosture.position.x(),-8.10917,0.001);
  EXPECT_NEAR(matchedPoint->pathPosture.position.y(),16.2537,0.001);
  EXPECT_NEAR(matchedPoint->pathPosture.course,2.60782,0.001);
  EXPECT_NEAR(matchedPoint->pathPosture.curvature,0.0816672,0.001);
  EXPECT_NEAR(matchedPoint->frenetPose.curvilinearAbscissa,17.1109,0.001);
  EXPECT_NEAR(matchedPoint->frenetPose.lateralDeviation,0.17852,0.001);
  EXPECT_NEAR(matchedPoint->frenetPose.courseDeviation,-0.51343,0.001);
  EXPECT_EQ(matchedPoint->nearestPointIndex,177);
}

//-----------------------------------------------------------------------------
TEST_F(TestPathMatching, testGlobalMatchingFailedWhenVehicleIsToFarFromPath)
{
  romea::Pose2D vehiclePose;
  vehiclePose.position.x() = -5.2;
  vehiclePose.position.y() = 30.1;
  vehiclePose.yaw = -20/180.*M_PI;

  auto matchedPoint = path_matching.match(path,vehiclePose);
  ASSERT_EQ(matchedPoint.is_initialized(),false);
}

//-----------------------------------------------------------------------------
TEST_F(TestPathMatching, testLocalMatchingOK)
{
  romea::Pose2D firstVehiclePose;
  firstVehiclePose.position.x() = -8.2;
  firstVehiclePose.position.y() = 16.1;
  firstVehiclePose.yaw = 120/180.*M_PI;

  romea::Pose2D secondVehiclePose;
  secondVehiclePose.position.x()=-9.3;
  secondVehiclePose.position.y()=17.2;
  secondVehiclePose.yaw=130/180.*M_PI;

  auto firstMatchedPoint = path_matching.match(path,firstVehiclePose);
  auto secondMatchedPoint = path_matching.match(path,secondVehiclePose,*firstMatchedPoint,10.)  ;

  ASSERT_EQ(secondMatchedPoint.is_initialized(),true);
  EXPECT_NEAR(secondMatchedPoint->pathPosture.position.x(),-9.41664,0.001);
  EXPECT_NEAR(secondMatchedPoint->pathPosture.position.y(),16.9346,0.001);
  EXPECT_NEAR(secondMatchedPoint->pathPosture.course,2.72756,0.001);
  EXPECT_NEAR(secondMatchedPoint->pathPosture.curvature,0.104456,0.001);
  EXPECT_NEAR(secondMatchedPoint->frenetPose.curvilinearAbscissa, 18.5847,0.001);
  EXPECT_NEAR(secondMatchedPoint->frenetPose.lateralDeviation,-0.289936,0.001);
  EXPECT_NEAR(secondMatchedPoint->frenetPose.courseDeviation,-0.458636,0.001);
}

//-----------------------------------------------------------------------------
TEST_F(TestPathMatching, localMatchingFailedWhenPositionIsTooFarFromTheLastMatchedPoint)
{
  romea::Pose2D firstVehiclePose;
  firstVehiclePose.position.x() = -8.2;
  firstVehiclePose.position.y() = 16.1;
  firstVehiclePose.yaw = 120/180.*M_PI;

  romea::Pose2D secondVehiclePose;
  secondVehiclePose.position.x()=-11.3;
  secondVehiclePose.position.y()=17.6;
  secondVehiclePose.yaw=130/180.*M_PI;

  auto firstMatchedPoint = path_matching.match(path,firstVehiclePose);
  auto secondMatchedPoint = path_matching.match(path,secondVehiclePose,*firstMatchedPoint,0.5);

  EXPECT_EQ(secondMatchedPoint.is_initialized(),false);
}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
