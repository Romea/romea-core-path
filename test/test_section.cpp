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
#include "romea_core_path/PathSection2D.hpp"

// local
#include "../test/test_helper.h"
#include "test_utils.hpp"

class TestSection : public ::testing::Test
{
public:
  TestSection() {}

  void SetUp() override
  {
    section = std::make_unique<romea::core::PathSection2D>(3);
    section->addWayPoints(loadWayPoints("/section.txt"));
  }

  std::unique_ptr<romea::core::PathSection2D> section;
};

//-----------------------------------------------------------------------------
TEST_F(TestSection, isSizeOK)
{
  EXPECT_EQ(section->size(), 2425);
}

//-----------------------------------------------------------------------------
TEST_F(TestSection, isLengthOK)
{
  EXPECT_DOUBLE_EQ(section->getLength(), 246.80741112805143);
}

//-----------------------------------------------------------------------------
TEST_F(TestSection, testNumberOfCurvesOK)
{
  size_t numberOfCurves = 2425;
  EXPECT_EQ(section->size(), 2425);
  EXPECT_EQ(section->getX().size(), numberOfCurves);
  EXPECT_EQ(section->getY().size(), numberOfCurves);
  EXPECT_EQ(section->getCurvilinearAbscissa().size(), numberOfCurves);
}

//-----------------------------------------------------------------------------
TEST_F(TestSection, testFindMinMaxIndexNearPathBegin)
{
  auto centerIndex = section->findIndex(1.);
  auto range = section->findIntervalBoundIndexes(centerIndex, 2.);

  EXPECT_EQ(range.lower(), 0);
  EXPECT_EQ(range.upper(), 22);
}

//-----------------------------------------------------------------------------
TEST_F(TestSection, testFindMinMaxIndexInCenterOfPahtPath)
{
  auto centerIndex = section->findIndex(section->getLength() / 2.);
  auto range = section->findIntervalBoundIndexes(centerIndex, 2.);

  EXPECT_EQ(range.lower(), 1224);
  EXPECT_EQ(range.upper(), 1244);
}

//-----------------------------------------------------------------------------
TEST_F(TestSection, testFindMinMaxIndexNearPathEnd)
{
  auto centerIndex = section->findIndex(section->getLength() - 1);
  auto range = section->findIntervalBoundIndexes(centerIndex, 2.);
  EXPECT_EQ(range.lower(), 2404);
  EXPECT_EQ(range.upper(), 2424);
}

//-----------------------------------------------------------------------------
TEST_F(TestSection, testFindNearestIndex)
{
  EXPECT_EQ(section->findIndex(42.36, 0), 430);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
