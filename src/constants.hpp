#pragma once
#include <Eigen/Dense>
#include "uav_params.hpp"

using namespace Eigen;

class Constants
{
    public:
        Constants(UAVparams& params);
        Matrix<double,6,6> massMatrix;
        Matrix<double,6,6> invMassMatrix;
        Matrix<double,6,6> gyroMatrix(Vector<double,6>  x);
        Matrix<double,6,6> TMatrix(Vector<double,6>  y);

    private:
        UAVparams& params;
};
