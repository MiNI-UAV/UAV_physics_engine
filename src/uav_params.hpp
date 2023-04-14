#pragma once
#include <Eigen/Dense>

struct UAVparams
{
    public:
        UAVparams();
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
};

UAVparams::UAVparams()
{
    g = 9.81;
    ro = 1.204;

    m = 5;
    Ix = 10;
    Iy = 11;
    Iz = 12;
    Ixy = 1;
    Ixz = 2;
    Iyz = 3;

    forceCoff = 1.0;
    torqueCoff = 1.0;
    noOfRotors = 4;
    rotorPos = new Eigen::Vector3d[noOfRotors]{{ 0.1, 0.1, 0.0},
                                               {-0.1, 0.1, 0.0},
                                               {-0.1,-0.1, 0.0},
                                               { 0.1,-0.1, 0.0}};
    rotorDir = new int[noOfRotors] {1,-1, 1,-1};
}

UAVparams::~UAVparams()
{
    delete[] rotorPos;
    delete[] rotorDir;
}