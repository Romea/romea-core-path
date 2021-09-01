// gtest
#include <gtest/gtest.h>
#include "test_helper.h"

//romea
#include "PathMatching2D.hpp"
#include "test_utils.hpp"


class TestPath : public ::testing::Test
{
public :

  TestPath(){}

  virtual void SetUp() override
  {
    std::vector<std::vector<romea::PathWayPoint2D>> wayPoints(3);
    wayPoints[0]=loadWayPoints("/bird_path1.txt");
    wayPoints[1]=loadWayPoints("/bird_path2.txt");
    wayPoints[2]=loadWayPoints("/bird_path3.txt");
    path = std::make_unique<romea::Path2D>(wayPoints,3);
  }

  std::unique_ptr<romea::Path2D> path;
};


//-----------------------------------------------------------------------------
TEST_F(TestPath, isSizeOK)
{
  EXPECT_EQ(path->size(),3);
  EXPECT_EQ(path->getSection(0).size(),259);
  EXPECT_EQ(path->getSection(1).size(),17);
  EXPECT_EQ(path->getSection(2).size(),255);
}

//-----------------------------------------------------------------------------
TEST_F(TestPath, isLengthOK)
{
  EXPECT_DOUBLE_EQ(path->getSection(0).getLength(),25.758932752740488);
  EXPECT_DOUBLE_EQ(path->getSection(1).getLength(),1.5998739194895477);
  EXPECT_DOUBLE_EQ(path->getSection(2).getLength(),25.296885714279526);
  EXPECT_DOUBLE_EQ(path->getLength(),52.655692386509557);
}


//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
