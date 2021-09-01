// gtest
#include <gtest/gtest.h>

//romea
#include "PathCurvilinearAbscissaVector.hpp"


class TestCurvilinearAbscissa: public ::testing::Test
{
public :

  TestCurvilinearAbscissa():
    curvilinearAbscissaVector(1)
  {
  }

  virtual void SetUp() override
  {
   curvilinearAbscissaVector.add(0.1);
   curvilinearAbscissaVector.add(0.13);
   curvilinearAbscissaVector.add(0.06);
   curvilinearAbscissaVector.add(0.3);
   curvilinearAbscissaVector.add(0.08);
   curvilinearAbscissaVector.add(0.4);
   curvilinearAbscissaVector.add(0.23);
   curvilinearAbscissaVector.add(0.27);
   curvilinearAbscissaVector.add(0.43);

//   for(size_t n=0;n<curvilinearAbscissaVector.size();n++)
//     std::cout << curvilinearAbscissaVector[n] << std::endl;
  }

  romea::PathCurvilinearAbscissaVector curvilinearAbscissaVector;
};


//-----------------------------------------------------------------------------
TEST_F(TestCurvilinearAbscissa, isSizeOK)
{
  EXPECT_EQ(curvilinearAbscissaVector.size(),10);
}

//-----------------------------------------------------------------------------
TEST_F(TestCurvilinearAbscissa, findIndexOK)
{
  EXPECT_EQ(curvilinearAbscissaVector.findIndex(1.5),4);
}

//-----------------------------------------------------------------------------
TEST_F(TestCurvilinearAbscissa, isIndexEqualToFirstIndex)
{
  EXPECT_EQ(curvilinearAbscissaVector.findIndex(0.5),0);
}

//-----------------------------------------------------------------------------
TEST_F(TestCurvilinearAbscissa, isIndexEqualToLastIndex)
{
  EXPECT_EQ(curvilinearAbscissaVector.findIndex(3.5),10);
}

//-----------------------------------------------------------------------------
TEST_F(TestCurvilinearAbscissa, interval)
{
  size_t intervalCenterIndex = curvilinearAbscissaVector.findIndex(1.9);
  auto intervalIndexes = curvilinearAbscissaVector.findIntervalBoundIndexes(intervalCenterIndex,{1.4,2.4});
  EXPECT_EQ(intervalIndexes.lower(),3);
  EXPECT_EQ(intervalIndexes.upper(),8);
}

//-----------------------------------------------------------------------------
TEST_F(TestCurvilinearAbscissa, interval2)
{
  size_t intervalCenterIndex = curvilinearAbscissaVector.findIndex(2.4);
  auto intervalIndexes = curvilinearAbscissaVector.findIntervalBoundIndexes(intervalCenterIndex,{1.4,3.4});
  EXPECT_EQ(intervalIndexes.lower(),3);
  EXPECT_EQ(intervalIndexes.upper(),9);
}

//-----------------------------------------------------------------------------
TEST_F(TestCurvilinearAbscissa, interval3)
{
  size_t intervalCenterIndex = curvilinearAbscissaVector.findIndex(1.4);
  auto intervalIndexes = curvilinearAbscissaVector.findIntervalBoundIndexes(intervalCenterIndex,{0.4,2.4});
  EXPECT_EQ(intervalIndexes.lower(),0);
  EXPECT_EQ(intervalIndexes.upper(),8);
}

//-----------------------------------------------------------------------------
TEST_F(TestCurvilinearAbscissa, interval4)
{
  size_t intervalCenterIndex = curvilinearAbscissaVector.findIndex(1.9);
  auto intervalIndexes = curvilinearAbscissaVector.findIntervalBoundIndexes(intervalCenterIndex,{0.4,3.4});
  EXPECT_EQ(intervalIndexes.lower(),0);
  EXPECT_EQ(intervalIndexes.upper(),9);
}



//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
