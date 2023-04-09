#include <iostream>
#include <Eigen/Dense>
#include "RK4.hpp"

// Harmonic oscillator
Eigen::Vector2d fun(double t , Eigen::Vector2d y)
{
  Eigen::Vector2d vec;
  vec(0) = y(1);
  vec(1) = -y(0);
  return vec;
}

int main()
{
  Eigen::Vector2d vec;
  vec(0) = 10;
  vec(1) = 0;


  for(double t = 0.0; t < 10.0; t+= 0.1)
  {
    vec = RK4_step<2>(0.0,vec, fun, 0.1);
    std::cout << vec(0) << std::endl;
  }
}