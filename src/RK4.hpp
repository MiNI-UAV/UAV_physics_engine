#pragma once
#include <functional>
#include <Eigen/Dense>

using namespace Eigen;

VectorXd RK4_step(double t,
                          VectorXd y0,
                          std::function<VectorXd(double,VectorXd)> rhs_fun,
                          double h);

#include "RK4.tpp"

