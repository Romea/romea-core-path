// gtest
#include <gtest/gtest.h>
#include "test_helper.h"

//romea
#include "PathCurve2D.hpp"
#include "test_utils.hpp"


class TestPathCurve : public ::testing::Test
{
public :

  TestPathCurve(){}

  virtual void SetUp() override
  {
    pathPoints=loadPath("path.txt");
  }

  romea::PathCurve2D pathCurve;
  romea::VectorOfEigenVector2d pathPoints;
};


//-----------------------------------------------------------------------------
TEST_F(TestPathCurve, isLoadedOK)
{
  pathCurve
}

//-----------------------------------------------------------------------------
TEST_F(TestPath, isLengthOK)
{
  EXPECT_DOUBLE_EQ(path.getLength(),246.80741112805143);
}

//-----------------------------------------------------------------------------
TEST_F(TestPath, isLengthInterpolcationWindowLenghtOK)
{
  EXPECT_DOUBLE_EQ(path.getInterpolationWindowLength(),3);
}

//-----------------------------------------------------------------------------
TEST_F(TestPath, testNumberOfCurvesOK)
{
  size_t numberOfCurves = 2427;
  EXPECT_EQ(path.getCurves().size(),numberOfCurves);
  EXPECT_EQ(path.getX().size(),numberOfCurves);
  EXPECT_EQ(path.getY().size(),numberOfCurves);
  EXPECT_EQ(path.getCurvilinearAbscissa().size(),numberOfCurves);
}

//-----------------------------------------------------------------------------
TEST_F(TestPath, testFindMinMaxIndexNearPathBegin)
{
  auto range = path.findMinMaxIndexes(1.,2.);

  EXPECT_EQ(range.lower(),0);
  EXPECT_EQ(range.upper(),22);
}

//-----------------------------------------------------------------------------
TEST_F(TestPath, testFindMinMaxIndexInCenterOfPahtPath)
{
  auto range = path.findMinMaxIndexes(path.getLength()/2.,2.);

  EXPECT_EQ(range.lower(),1224);
  EXPECT_EQ(range.upper(),1244);
}

//-----------------------------------------------------------------------------
TEST_F(TestPath, testFindMinMaxIndexNearPathEnd)
{
  auto range = path.findMinMaxIndexes(path.getLength()-1,2.);
  EXPECT_EQ(range.lower(),2404);
  EXPECT_EQ(range.upper(),2426);
}

//-----------------------------------------------------------------------------
TEST_F(TestPath, testFindNearestIndex)
{
  EXPECT_EQ(path.findNearestIndex(42.36),430);
}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
