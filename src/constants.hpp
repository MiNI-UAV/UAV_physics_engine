#pragma once
#include <Eigen/Dense>

using namespace Eigen;

class Constants
{
    private:
        double g = 9.81;
        double m = 5;
        double Ix = 10;
        double Iy = 11;
        double Iz = 12;
        double Ixy = 1;
        double Ixz = 2;
        double Iyz = 3;

    public:
        Constants();
        Matrix<double,6,6> massMatrix;
        Matrix<double,6,6> invMassMatrix;
        Matrix<double,6,6> gyroMatrix(Vector<double,6>  x);
        Vector<double,6>   gravity_forces(Vector<double,6>  y);
};
