#pragma once
#include <Eigen/Dense>
#include "hinge.hpp"

struct Drive
{
    Eigen::Vector3d position;
    Eigen::Vector3d axis;
    int noOfHinges;
    Hinge hinges[2];
};

struct Rotor : Drive
{
    double forceCoff;
    double torqueCoff;
    int direction;
    double timeConstant;
    double maxSpeed;
    
};

class Jet : public Drive
{
public:
    int phases;
    Eigen::VectorXd thrust;
    Eigen::VectorXd time;
    

    bool start(double time);
    double getThrust(double time);
    double getLastThrust() {return lastThrust;};
    
private:
    enum JetState
    {
        READY,
        WORKING,
        BURNT
    } state = JetState::READY; 
    int currentPhase = 0;
    double startTime = -1.0; 
    double lastThrust = 0.0;
};