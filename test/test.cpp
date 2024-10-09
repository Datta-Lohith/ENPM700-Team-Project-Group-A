#include <gtest/gtest.h>
#include "pid.hpp"

class PIDControllerTest : public ::testing::Test {
 protected:
    double Kp = 0.1;
    double Ki = 0.01;
    double Kd = 0.5;
    double dt = 1.0;
    PIDController pid{Kp, Ki, Kd, dt};  // PID controller instance
};

TEST_F(PIDControllerTest, TestInitialConditions) {
    double setpoint = 100.0;
    double actual_velocity = 0.0;

    double output = pid.compute(setpoint, actual_velocity);
    EXPECT_GT(output, 0.0);
}

TEST_F(PIDControllerTest, this_will_fail) {
    double setpoint = 100.0;
    double actual_velocity = 90.0;

    double output = pid.compute(setpoint, actual_velocity);
    EXPECT_DOUBLE_EQ(output, 6.1);  // Proportional only for this case
}
