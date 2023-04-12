#pragma once
#include <Eigen/Dense>

using namespace Eigen;

class Forces
{
    private:
        static const int noOfRotors = 4;

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

    public:
        Forces();
        Vector<double,6> gravity_forces(Vector<double,6>  y);
        Vector<double,6> lift_forces(double rotorAngularVelocity[]);
};