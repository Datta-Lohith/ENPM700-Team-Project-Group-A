#include <gtest/gtest.h>
#include "pid.hpp"

class PIDControllerTest : public ::testing::Test {
 protected:
    // Example PID parameters
    double Kp = 0.1;  // Proportional gain
    double Ki = 0.01;  // Integral gain
    double Kd = 0.5;  // Derivative gain
    double dt = 1.0;  // Time step
    PIDController pid{Kp, Ki, Kd, dt};  // PID controller instance
};

TEST_F(PIDControllerTest, TestInitialConditions) {
    double setpoint = 100.0;
    double actual_velocity = 0.0;

    double output = pid.compute(setpoint, actual_velocity);
    // Expect a positive output since the actual is below the setpoint
    EXPECT_GT(output, 0.0);
}

TEST_F(PIDControllerTest, TestProportionalControl) {
    double setpoint = 100.0;
    double actual_velocity = 90.0;

    double output = pid.compute(setpoint, actual_velocity);
    EXPECT_DOUBLE_EQ(output, 6.1);  // Proportional only for this case
}

TEST_F(PIDControllerTest, TestIntegralControl) {
    double setpoint = 100.0;
    double actual_velocity = 90.0;

    // Call compute multiple times to simulate time
    for (int i = 0; i < 5; ++i) {
        pid.compute(setpoint, actual_velocity);
    }

    double output = pid.compute(setpoint, actual_velocity);

    // Expect output to increase due to integral
    EXPECT_GT(output, Kp * (setpoint - actual_velocity));
}

TEST_F(PIDControllerTest, TestDerivativeControl) {
    double setpoint = 100.0;
    double actual_velocity = 90.0;

    // First call
    double first_output = pid.compute(setpoint, actual_velocity);
    // Change in actual_velocity simulating a response
    actual_velocity = 95.0;
    double second_output = pid.compute(setpoint, actual_velocity);

    EXPECT_LT(second_output, first_output);  // Expect the output to decrease
}

TEST_F(PIDControllerTest, TestNoOutputWhenAtSetpoint) {
    double setpoint = 100.0;
    double actual_velocity = 100.0;

    double output = pid.compute(setpoint, actual_velocity);
    EXPECT_EQ(output, 0.0);  // Expect no output since we're at the setpoint
}

TEST_F(PIDControllerTest, TestNegativeSetpoint) {
    double setpoint = -50.0;  // Negative target
    double actual_velocity = 0.0;

    double output = pid.compute(setpoint, actual_velocity);

    // Expect negative output since actual is above the negative setpoint
    EXPECT_LT(output, 0.0);
}
