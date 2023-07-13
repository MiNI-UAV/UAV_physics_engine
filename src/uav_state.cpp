#include <Eigen/Dense>
#include "uav_state.hpp"
#include "status.hpp"

UAVstate::UAVstate(int rotors): noOfRotors{rotors}
{
    y.setZero();
    x.setZero();
    rotorAngularVelocities.setZero(noOfRotors);
    acceleration.setZero();


    demandedAngularBuf[0].setZero(noOfRotors);
    demandedAngularBuf[1].setZero(noOfRotors);
    demanded_ptr = demandedAngularBuf + 1;

    windBuf[0].setZero();
    windBuf[1].setZero();
    wind_ptr = windBuf+1;


    forceBuf[0].setZero();
    forceBuf[1].setZero();
    force_ptr = force_ptr+1;
    forceValidityCounter = 0;

    status = Status::idle;
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
    return (*(demanded_ptr.load()));
}

Eigen::Vector3d UAVstate::getWind()
{
    return (*(wind_ptr.load()));
}

Eigen::Vector<double, 6> UAVstate::getOuterForce()
{   
    if(forceValidityCounter.load() > 0)
    {
        forceValidityCounter--;
        return (*(force_ptr.load()));
    }
    return Eigen::Vector<double,6>(0,0,0,0,0,0);

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

void UAVstate::setX(Eigen::Vector<double,6> new_x) {x = new_x;}

void UAVstate::setDemandedOm(Eigen::VectorXd newDemandedOm) {
  demandedAngularBuf[demandedBufSwitch] = newDemandedOm;
  demanded_ptr = demandedAngularBuf + demandedBufSwitch;
  demandedBufSwitch = 1 - demandedBufSwitch;
}

void UAVstate::setWind(Eigen::Vector3d wind)
{
    windBuf[windBufSwitch] = wind;
    wind_ptr = windBuf + windBufSwitch;
    windBufSwitch = 1 - windBufSwitch;
}

void UAVstate::setForce(Eigen::Vector3d force, Eigen::Vector3d torque)
{
    Eigen::Vector<double,6> newForce;
    newForce << force,torque;
    forceBuf[forceBufSwitch] = newForce;
    forceValidityCounter = validityOfForce * 4; //< 4 times bcs RK4 call function 4 times.
    force_ptr = forceBuf + forceBufSwitch;
    forceBufSwitch = 1 - forceBufSwitch;
}

void UAVstate::setAcceleration(Eigen::Vector<double, 6> new_accel)
{
    acceleration = new_accel;
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
