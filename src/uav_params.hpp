#pragma once
#include <Eigen/Dense>

struct UAVparams
{
    public:
        UAVparams();
        UAVparams(std::string configFile);
        ~UAVparams();

        double g;
        double ro;

        double m;
        double Ix;
        double Iy;
        double Iz;
        double Ixy;
        double Ixz;
        double Iyz;

        int noOfRotors;
        double forceCoff;
        double torqueCoff;
        Eigen::Vector3d* rotorPos;
        int* rotorDir;
        Eigen::VectorXd rotorTimeConstant;
};
