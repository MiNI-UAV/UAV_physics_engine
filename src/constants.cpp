#pragma once
#include <Eigen/Dense>
#include "constants.hpp"

using namespace Eigen;

Constants::Constants()
{
    double m = 5;
    double Ix = 10;
    double Iy = 11;
    double Iz = 12;
    double Ixy = 1;
    double Ixz = 2;
    double Iyz = 3;
    
    //mass matrix
    massMatrix.setZero();
    massMatrix(0,0) = m;
    massMatrix(1,1) = m;
    massMatrix(2,2) = m;
    massMatrix(3,3) = Ix;
    massMatrix(4,4) = Iy;
    massMatrix(5,5) = Iz;
    massMatrix(3,4) = -Ixy;
    massMatrix(4,3) = -Ixy;
    massMatrix(3,5) = -Ixz;
    massMatrix(5,3) = -Ixz;
    massMatrix(4,5) = -Iyz;
    massMatrix(5,4) = -Iyz;
}

Matrix<double,6,6> Constants::gyroMatrix(Vector<double,6> x)
{
    Matrix<double,6,6> gyro;
    gyro.setZero();
    gyro(1,0) =  x(5);
    gyro(2,0) = -x(4);
    gyro(4,0) =  x(2);
    gyro(5,0) = -x(1);
    gyro(0,1) = -x(5);
    gyro(2,1) =  x(3);
    gyro(3,1) = -x(2);
    gyro(5,1) =  x(0);
    gyro(0,2) =  x(4);
    gyro(1,2) = -x(3);
    gyro(3,2) =  x(1);
    gyro(4,2) = -x(0);
    gyro(4,3) =  x(5);
    gyro(5,3) = -x(4);
    gyro(3,4) = -x(5);
    gyro(5,4) =  x(3);
    gyro(3,5) =  x(4);
    gyro(4,5) = -x(3);
    return gyro;
}