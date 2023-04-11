#pragma once
#include <Eigen/Dense>

using namespace Eigen;

class Constants
{
    public:
        Constants();
        Matrix<double,6,6> massMatrix;
        Matrix<double,6,6> gyroMatrix(Vector<double,6>  x);
};
