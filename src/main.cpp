#include <iostream>
#include <Eigen/Dense>
#include "RK4.hpp"


Eigen::Vector3d fun(double t ,Eigen::Vector3d y)
{
  return Eigen::Vector3d(1,2,3);
}

int main()
{
  Eigen::Vector3d vec;
  vec.setZero();
  Eigen::Vector3d res = RK4_step<3>(0.0,vec, fun, 1.0);

  std::cout << res;
}