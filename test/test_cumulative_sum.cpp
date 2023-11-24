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


// Eigen
#include <Eigen/Core>

// std
#include <algorithm>

// gtest
#include "gtest/gtest.h"

// romea
#include "romea_core_path/CumulativeSum.hpp"

class TestCumulativeSum : public ::testing::Test
{
public:
  TestCumulativeSum()
  : cumsum(1)
  {
  }

  void SetUp() override
  {
    cumsum.increment(0.1);
    cumsum.increment(0.13);
    cumsum.increment(0.06);
    cumsum.increment(0.3);
    cumsum.increment(0.08);
    cumsum.increment(0.4);
    cumsum.increment(0.23);
    cumsum.increment(0.27);
    cumsum.increment(0.43);
  }

  romea::core::CumulativeSum<double, Eigen::aligned_allocator<double>> cumsum;
};


//-----------------------------------------------------------------------------
TEST_F(TestCumulativeSum, isSizeOK)
{
  EXPECT_EQ(cumsum.size(), 10);
}

//-----------------------------------------------------------------------------
TEST_F(TestCumulativeSum, isInitialValueOk)
{
  EXPECT_DOUBLE_EQ(cumsum.initialValue(), 1);
}

//-----------------------------------------------------------------------------
TEST_F(TestCumulativeSum, isFinalValueOk)
{
  EXPECT_DOUBLE_EQ(cumsum.finalValue(), 3);
}

//-----------------------------------------------------------------------------
TEST_F(TestCumulativeSum, checkClear)
{
  cumsum.clear();

  EXPECT_EQ(cumsum.size(), 1);
  EXPECT_DOUBLE_EQ(cumsum.initialValue(), 0);
  EXPECT_DOUBLE_EQ(cumsum.finalValue(), 0);
}

//-----------------------------------------------------------------------------
TEST_F(TestCumulativeSum, checkGetData)
{
  EXPECT_EQ(cumsum[0], 1);
  EXPECT_EQ(cumsum[2], 1.23);
}

//-----------------------------------------------------------------------------
TEST_F(TestCumulativeSum, checkLowerBound)
{
  auto it = std::lower_bound(std::cbegin(cumsum), std::cend(cumsum), 2.);
  EXPECT_EQ(std::distance(std::cbegin(cumsum), it), 6);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
