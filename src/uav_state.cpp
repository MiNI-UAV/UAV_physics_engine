#include "uav_state.hpp"
#include <Eigen/Dense>

UAVstate::UAVstate(int rotors): noOfRotors{rotors}
{
    y.setZero();
    x.setZero();
    rotorAngularVelocities.setZero(noOfRotors);
}

UAVstate::~UAVstate()
{

}

Eigen::Vector<double,6> UAVstate::getY()
{
    return y;
}

Eigen::Vector<double,6> UAVstate::getX()
{
    return x;
}

Eigen::VectorXd UAVstate::getOm()
{
    return rotorAngularVelocities;
}

UAVstate& UAVstate::operator=(Eigen::VectorXd& other)
{
    y = other.segment(0,6);
    x = other.segment(6,6);
    rotorAngularVelocities = other.segment(12,noOfRotors);
    return *this;
}

std::ostream& operator << ( std::ostream& outs, const UAVstate& state)
{
  return outs << "y:\n" << state.y.transpose() << "\nx:\n" << state.x.transpose() << "\nom:\n" <<state.rotorAngularVelocities.transpose() << std::endl;
}
