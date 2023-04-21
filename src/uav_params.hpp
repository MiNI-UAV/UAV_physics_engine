#pragma once
#include <Eigen/Dense>

struct UAVparams
{
    public:
        UAVparams();
        UAVparams(const char* configFile);
        ~UAVparams();

        const char name[20] = "dron1";

        double g;
        double ro;

        //Mass params
        double m;
        double Ix;
        double Iy;
        double Iz;
        double Ixy;
        double Ixz;
        double Iyz;

        //Rotor params
        int noOfRotors;
        double forceCoff;
        double torqueCoff;
        Eigen::Vector3d* rotorPos;
        int* rotorDir;
        Eigen::VectorXd rotorTimeConstant;

        //Aerodynamic params
        double S, d;
        double Ci[6] = {1.0,-0.1,0.1,-0.1,-0.1,-0.1};

};
