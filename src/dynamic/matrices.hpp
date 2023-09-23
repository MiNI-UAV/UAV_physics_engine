#pragma once
#include <Eigen/Dense>
#include "common.hpp"

using namespace Eigen;

class Matrices
{
    public:
        static Matrix<double,6,6> massMatrix();
        static Matrix<double,6,6> gyroMatrix(Vector<double,6>  x);
        static Matrix<double,6,6> TMatrix(Vector<double,6>  y);
        static Matrix<double,3,3> R_nb(const Vector<double,6>&  y);
        static Matrix<double,3,3> R_nb(const Vector<double,7>&  y);
        static Matrix<double,3,3> R_wind_b(double alpha, double beta);
        static Vector<double,6> quaterionsToRPY(Vector<double,7>  y);
        static Vector<double,7> RPYtoQuaterion(Vector<double,6> y);
        static Matrix4d OM_conj(Vector<double,6>  x);
};
