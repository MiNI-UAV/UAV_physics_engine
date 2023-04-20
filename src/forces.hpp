#pragma once
#include <Eigen/Dense>
#include "uav_params.hpp"
#include "matrices.hpp"

using namespace Eigen;

class Forces
{
    public:
        Forces(UAVparams& params);
        Vector<double,6> gravity_loads(Vector<double,6>  y);
        Vector<double,6> lift_loads(VectorXd rotorAngularVelocity);
        Vector<double, 6> aerodynamic_loads(Matrices& matricies, const Vector<double, 6> &x, const Vector<double, 6> &y, Vector3d wind_global);
        VectorXd angularAcceleration(VectorXd demandedAngularVelocity, VectorXd rotorAngularVelocity);
    
    private:
        UAVparams& params;
        double dynamic_pressure(double Vtot);
};
