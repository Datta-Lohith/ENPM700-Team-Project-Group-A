#include <gtest/gtest.h>
#include "pid.hpp"

class PIDControllerTest : public ::testing::Test {
protected:
    // Example PID parameters
    double Kp = 0.1; // Proportional gain
    double Ki = 0.01; // Integral gain
    double Kd = 0.5; // Derivative gain
    double dt = 1.0; // Time step
    PIDController pid{Kp, Ki, Kd, dt}; // PID controller instance
};

TEST_F(PIDControllerTest, TestInitialConditions) {
    double setpoint = 100.0;
    double actual_velocity = 0.0;
    
    double output = pid.compute(setpoint, actual_velocity);
    EXPECT_GT(output, 0.0); // Expect a positive output since the actual is below the setpoint
}

TEST_F(PIDControllerTest, this_will_fail){
    double setpoint = 100.0;
    double actual_velocity = 90.0;

    double output = pid.compute(setpoint, actual_velocity);
    EXPECT_DOUBLE_EQ(output, 6.1); // Proportional only for this case
}