#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include "uav_params.hpp"
#include "forces.hpp"
#include "matrices.hpp"


using namespace Eigen;

Forces::Forces(UAVparams& params): params{params}
{}

Vector<double,6> Forces::gravity_loads(const Matrix3d& r_nb)
{
    Vector<double,6> Fg;
    Fg.setZero();
    Fg.head<3>() = r_nb * Eigen::Vector3d(0.0,0.0,(params.m*params.g));
    return Fg;
}

Vector<double,6> Forces::lift_loads(VectorXd rotorAngularVelocity)
{
    Vector3d Fr = {0.0, 0.0, 0.0};
    Vector3d Mr = {0.0, 0.0, 0.0};
    for (int i = 0; i < params.noOfRotors; i++)
    {
        double om2 = std::pow(rotorAngularVelocity(i),2);
        Vector3d Fi = {0.0, 0.0, -params.ro*params.forceCoff*om2};
        Fr += Fi;
        Mr(2) += params.rotorDir[i]*params.ro*params.torqueCoff*om2;
        Mr += params.rotorPos[i].cross(Fi);
    }
    // std::cout << "Fr:\n" << Fr << std::endl;
    // std::cout << "Mr:\n" << Mr << std::endl << std::endl;
    Vector<double,6> res;
    res << Fr, Mr;
    return res; 
}

double Forces::dynamic_pressure(double Vtot)
{
    return 0.5*params.ro*Vtot*Vtot;
}

Vector<double, 6> Forces::aerodynamic_loads(const Matrix3d& r_nb, const Vector<double, 6> &x, Vector3d wind_global)
{
    Vector<double, 6> Fa(params.Ci);
    Vector3d wind = r_nb*wind_global;
    Vector3d velocity = x.segment(0,3);
    Vector3d diff = velocity-wind;
    double Vtot = diff.norm();
    if(Vtot == 0.0)
    {
        Fa.setZero();
        return Fa;
    }
    double alpha = atan2(diff(2),diff(0));
    double beta = asin(diff(1)/Vtot);
    Fa(0) *= (cos(alpha)*cos(beta));
    Fa(1) *= sin(beta);
    Fa(2) *= (sin(alpha)*cos(beta));
    Fa.segment(3,3) *= params.d;
    Fa *= -(dynamic_pressure(Vtot)*params.S);
    return Fa;
}

VectorXd Forces::angularAcceleration(VectorXd demandedAngularVelocity, VectorXd rotorAngularVelocity)
{
    VectorXd res;
    res = (demandedAngularVelocity - rotorAngularVelocity);
    return res.cwiseQuotient(params.rotorTimeConstant);
}