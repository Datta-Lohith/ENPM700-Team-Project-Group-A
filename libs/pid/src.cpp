/**
 * @file pid.cpp
 * @brief Implementation of the PID controller class.
 * 
 * @details This file contains the implementation of the PID controller's constructor 
 * and a stub for the compute() method that simply returns a constant value.
 * 
 * @author Datta Lohith Gannavarapu
 * @copyright MIT License
 */

#include "pid.hpp"

/**
 * @brief Constructor for the PID controller.
 * 
 * Initializes the PID controller with the provided Kp, Ki, and Kd values.
 * 
 * @param Kp Proportional gain.
 * @param Ki Integral gain.
 * @param Kd Derivative gain.
 */
PIDController::PIDController(double kp, double ki, double kd)
    : Kp(kp), Ki(ki), Kd(kd) {}

/**
 * @brief Stub for computing the new velocity based on PID control.
 * 
 * This is just a placeholder implementation and returns a constant value.
 * 
 * @param setpoint The target velocity.
 * @param actual_velocity The current velocity.
 * @return double A constant value (for now).
 */

double PIDController::compute(double setpoint, double actual_velocity) {
    // Stub implementation: return a constant value for now.
    return 2.0;  // Replace with the actual PID control logic in Part 2
}
