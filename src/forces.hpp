#pragma once
#include <Eigen/Dense>

using namespace Eigen;

const int noOfRotors = 4;

class Forces
{
    public:
        Forces();
        Vector<double,6> gravity_forces(Vector<double,6>  y);
        Vector<double,6> lift_forces(Vector<double,noOfRotors> rotorAngularVelocity);
        Vector<double,noOfRotors> angularAcceleration(Vector<double,noOfRotors> rotorAngularVelocity);
    
    private:
        double g = 9.81;
        double m = 5;
        double ro = 1.204; // kg/m3
        double forceCoff = 1.0;
        double torqueCoff = 1.0;

        Vector3d rotorPos[noOfRotors] = {{ 0.1, 0.1, 0.0},
                                 {-0.1, 0.1, 0.0},
                                 {-0.1,-0.1, 0.0},
                                 { 0.1,-0.1, 0.0}};
        int rotorDir[noOfRotors] = {1,-1, 1,-1};
        Vector<double, noOfRotors> demandedAngularVelocity = {10.0, 10.0, 10.0, 10.0};
        Vector<double, noOfRotors> rotorTimeConstant = {0.05, 0.05, 0.05, 0.05};
};