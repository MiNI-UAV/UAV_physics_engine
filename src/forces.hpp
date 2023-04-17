#pragma once
#include <Eigen/Dense>
#include "uav_params.hpp"

using namespace Eigen;

class Forces
{
    public:
        Forces(UAVparams& params);
        Vector<double,6> gravity_forces(Vector<double,6>  y);
        Vector<double,6> lift_forces(VectorXd rotorAngularVelocity);
        VectorXd angularAcceleration(VectorXd demandedAngularVelocity, VectorXd rotorAngularVelocity);
    
    private:
        UAVparams& params;
};