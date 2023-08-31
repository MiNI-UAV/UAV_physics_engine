#pragma once
#include <Eigen/Dense>
#include "uav_params.hpp"

using namespace Eigen;

class Matrices
{
    public:
        Matrices();
        Matrix<double,6,6> massMatrix;
        Matrix<double,6,6> invMassMatrix;


        static Matrix<double,6,6> massMatrix2();
        static Matrix<double,6,6> gyroMatrix(Vector<double,6>  x);
        static Matrix<double,6,6> TMatrix(Vector<double,6>  y);
        static Matrix<double,3,3> R_nb(const Vector<double,6>&  y);
        static Matrix<double,3,3> R_nb(const Vector<double,7>&  y);
        static Vector<double,6> quaterionsToRPY(Vector<double,7>  y);
        static Vector<double,7> RPYtoQuaterion(Vector<double,6> y);
        static Matrix4d OM_conj(Vector<double,6>  x);

        void updateMatrices();
        void reduceMass(double mass_delta);
};
