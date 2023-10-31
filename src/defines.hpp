#pragma once

/// @brief define to use quaterion instead of RPY angles
#define USE_QUATERIONS 1

/// @brief Simulation constants
namespace def {

/// @brief Step time of simulation. Step of ODE solving methods
const double STEP_TIME = 0.001;

/// @brief Gravity constant on Earth in m/s2
const double GRAVITY_CONST = 9.81;

/// @brief minimal friction that is calculated (numerical float eps)
const double FRICTION_EPS = 0.001;

/// @brief artificial force coefficient. Protect again diving objects in horizontal wall
const double GENTLY_PUSH = 0.15;

/// @brief near zero floating point eps
const double DOUBLE_EPS = 1e-5;

/// @brief mixing window used in blending normal coefficients with standard ones, when stall angle was exceeded
const double MIXING_FUNCTION = 0.1;

/// @brief how many times outer force should be used
const int validityOfForce = 5;
} // namespace def