#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include "forces.hpp"

using namespace Eigen;

Forces::Forces()
{

}

Vector<double,6> Forces::gravity_forces(Vector<double,6>  y)
{
    Vector<double,6> Fg = {-std::sin(y(4)), std::sin(y(3))*std::cos(y(4)), std::cos(y(3))*std::cos(y(4)), 0.0, 0.0, 0.0};
    Fg = (m*g)*Fg;
    return Fg;
}

Vector<double,6> Forces::lift_forces(double rotorAngularVelocity[])
{
    Vector3d Fr = {0.0, 0.0, 0.0};
    Vector3d Mr = {0.0, 0.0, 0.0};
    for (int i = 0; i < noOfRotors; i++)
    {
        double om2 = std::pow(rotorAngularVelocity[i],2);
        Vector3d Fi = {0.0, 0.0, -ro*forceCoff*om2};
        Fr += Fi;
        Mr(2) += rotorDir[i]*ro*torqueCoff*om2;
        Mr += rotorPos[i].cross(Fi);
    }
    // std::cout << "Fr:\n" << Fr << std::endl;
    // std::cout << "Mr:\n" << Mr << std::endl << std::endl;
    Vector<double,6> res;
    res << Fr, Mr;
    return res; 
}