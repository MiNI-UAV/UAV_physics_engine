#pragma once
#include <Eigen/Dense>
#include "uav_params.hpp"

using namespace Eigen;

class Matrices
{
    public:
        Matrices(UAVparams& params);
        Matrix<double,6,6> massMatrix;
        Matrix<double,6,6> invMassMatrix;
        Matrix<double,6,6> gyroMatrix(Vector<double,6>  x);
        Matrix<double,6,6> TMatrix(Vector<double,6>  y);
        Matrix<double,7,6> TMatrix(Vector<double,7>  y);
        Matrix<double,3,3> R_nb(const Vector<double,6>&  y);
        Matrix<double,3,3> R_nb(const Vector<double,7>&  y);
        Vector<double,6> Ycorrection(Vector<double,6>  y);
        Vector<double,7> Ycorrection(Vector<double,7>  y);
        Vector<double,6> quaterionsToRPY(Vector<double,7>  y);
        void updateMatrices();
        void reduceMass(double mass_delta);

    private:
        UAVparams& params;
};
