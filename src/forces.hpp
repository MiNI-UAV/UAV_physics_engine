#pragma once
#include <Eigen/Dense>
#include "uav_params.hpp"
#include "matrices.hpp"

using namespace Eigen;

class Forces
{
    public:
        Forces() = default;

        Vector<double,6> gravity_loads(const Matrix3d& r_nb);
        Vector<double,6> lift_loads(VectorXd rotorAngularVelocity);
        Vector<double, 6> aerodynamic_loads(const Matrix3d& r_nb, const Vector<double, 6> &x, Vector3d wind_global);
        VectorXd angularAcceleration(VectorXd demandedAngularVelocity, VectorXd rotorAngularVelocity);
    
    private:
        double dynamic_pressure(double Vtot);
};
