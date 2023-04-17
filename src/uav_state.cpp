#include "uav_state.hpp"
#include <Eigen/Dense>

UAVstate::UAVstate(int rotors): noOfRotors{rotors}
{
    y.setZero();
    x.setZero();
    rotorAngularVelocities.setZero(noOfRotors);
    demandedAngularVelocity.setZero(noOfRotors);
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

Eigen::VectorXd UAVstate::getDemandedOm()
{
    return demandedAngularVelocity;
}

Eigen::VectorXd UAVstate::getState()
{
    Eigen::VectorXd res;
    res.setZero(12+noOfRotors);
    res.segment(0,6) = y;
    res.segment(6,6) = x;
    res.segment(12,noOfRotors) = rotorAngularVelocities;
    return res;
}

void UAVstate::setDemandedOm(Eigen::VectorXd newDemandedOm)
{
    demandedAngularVelocity = newDemandedOm;
}

UAVstate& UAVstate::operator=(Eigen::VectorXd& other)
{
    y = getY(other);
    x = getX(other);
    rotorAngularVelocities = getOm(other);
    return *this;
}

void UAVstate::setY(Eigen::VectorXd &state, Eigen::Vector<double, 6> Y)
{
    state.segment(0,6) = Y;
}

void UAVstate::setX(Eigen::VectorXd &state, Eigen::Vector<double, 6> X)
{
    state.segment(6,6) = X;
}

void UAVstate::setOm(Eigen::VectorXd &state, Eigen::VectorXd Om)
{
    state.segment(12,state.size()-12) = Om;
}

Eigen::Vector<double, 6> UAVstate::getY(const Eigen::VectorXd &state)
{
    return state.segment(0,6);
}

Eigen::Vector<double, 6> UAVstate::getX(const Eigen::VectorXd &state)
{
    return state.segment(6,6);
}

Eigen::VectorXd UAVstate::getOm(const Eigen::VectorXd &state)
{
    return state.segment(12,state.size()-12);
}

std::ostream& operator << ( std::ostream& outs, const UAVstate& state)
{
  return outs << "y:\n" << state.y.transpose() << "\nx:\n" << state.x.transpose() << "\nom:\n" <<state.rotorAngularVelocities.transpose() << std::endl;
}
