#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include "forces.hpp"
//#include "uav_params.hpp"

using namespace Eigen;

Forces::Forces(UAVparams& params): params{params}
{}

Vector<double,6> Forces::gravity_forces(Vector<double,6>  y)
{
    Vector<double,6> Fg = {-std::sin(y(4)), std::sin(y(3))*std::cos(y(4)), std::cos(y(3))*std::cos(y(4)), 0.0, 0.0, 0.0};
    Fg = (params.m*params.g)*Fg;
    return Fg;
}

Vector<double,6> Forces::lift_forces(VectorXd rotorAngularVelocity)
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

VectorXd Forces::angularAcceleration(VectorXd demandedAngularVelocity, VectorXd rotorAngularVelocity)
{
    VectorXd res;
    res = (demandedAngularVelocity - rotorAngularVelocity);
    return res.cwiseQuotient(params.rotorTimeConstant);
}