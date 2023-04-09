#pragma once
#include <functional>
#include <Eigen/Dense>

using namespace Eigen;

template <int S>
Vector<double,S> RK4_step(double t,
                          Vector<double,S> y0,
                          std::function<Vector<double,S>(double,Vector<double,S>)> rhs_fun,
                          double h);

#include "RK4.tpp"

