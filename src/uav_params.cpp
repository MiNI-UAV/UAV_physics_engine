#include <Eigen/Dense>
#include "uav_params.hpp"

/// @brief Initialize default data
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
    rotorTimeConstant.setConstant(noOfRotors,0.05);

    S = 0.1;
    d = 0.001;
}

UAVparams::UAVparams(std::string configFile)
{
    UAVparams();
}

UAVparams::~UAVparams()
{
    delete[] rotorPos;
    delete[] rotorDir;
}