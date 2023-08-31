#pragma once
#include <Eigen/Dense>
#include "uav_params.hpp"
#include "matrices.hpp"

using namespace Eigen;

class Forces
{
    public:
        static Vector<double,6> gravity_loads(const Matrix3d& r_nb);
        static Vector<double,6> lift_loads(VectorXd rotorAngularVelocity);
        static Vector<double, 6> aerodynamic_loads(const Matrix3d& r_nb, const Vector<double, 6> &x, Vector3d wind_global);
        static VectorXd angularAcceleration(VectorXd demandedAngularVelocity, VectorXd rotorAngularVelocity);
    
    private:
        static double dynamic_pressure(double height, double Vtot);
};
