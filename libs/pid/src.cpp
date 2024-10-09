/**
 * @file pid.cpp
 * @brief Implementation of the PID controller class.
 * 
 * @details This file contains the implementation of the PID controller's constructor 
 * and the compute() method that calculates the output based on PID control logic.
 * 
 * @author Datta Lohith Gannavarapu
 * @copyright MIT License
 */

#include "pid.hpp"

/**
 * @brief Constructor for the PID controller.
 * 
 * Initializes the PID controller with the provided Kp, Ki, Kd values, and time step.
 * 
 * @param Kp Proportional gain.
 * @param Ki Integral gain.
 * @param Kd Derivative gain.
 * @param dt Time step.
 */
PIDController::PIDController(double kp, double ki, double kd, double dt)
    : _Kp(kp), _Ki(ki), _Kd(kd), _dt(dt), _prev_error(0.0), _integral(0.0) {}

/**
 * @brief Computes the new velocity based on PID control.
 * 
 * This method calculates the PID output based on the error between the setpoint and 
 * the actual velocity, and updates the integral and derivative terms.
 * 
 * @param setpoint The target velocity.
 * @param actual_velocity The current velocity.
 * @return double The computed PID output.
 */
double PIDController::compute(double setpoint, double actual_velocity) {
    // Stub implementation
    return -10.0;  // Stub return value
    // Part 2: Implement PID control logic
}
