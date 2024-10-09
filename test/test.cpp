#include <gtest/gtest.h>
#include "pid.hpp"

TEST(pid_dummy_test_1, this_will_fail){
  EXPECT_DOUBLE_EQ(PIDController(1.0, 1.0, 1.0).compute(1.0, 10.0), 10.0);
  }

TEST(pid_dummy_test_2, this_will_fail_too){
  EXPECT_DOUBLE_EQ(PIDController(1.0, 1.0, 1.0).compute(2.0, 20.0), 20.0);
  }

