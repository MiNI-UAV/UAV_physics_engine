#include <Eigen/Dense>
#include "uav_state.hpp"
#include "common.hpp"

UAVstate::UAVstate(int rotors): noOfRotors{rotors}
{
    y.setZero();
    #ifdef USE_QUATERIONS
    y(3) = 1.0;
    #endif
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

#ifdef USE_QUATERIONS
Eigen::Vector<double,7> UAVstate::getY()
#else
Eigen::Vector<double,6> UAVstate::getY()
#endif
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
    res.setZero(omOffset+noOfRotors);
#ifdef USE_QUATERIONS
    res.segment(0,7) = y;
    res.segment(7,6) = x;
#else
    res.segment(0,6) = y;
    res.segment(6,6) = x;
#endif
    res.segment(omOffset,noOfRotors) = rotorAngularVelocities;
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

#ifdef USE_QUATERIONS
void UAVstate::setY(Eigen::VectorXd &state, Eigen::Vector<double, 7> Y)
{
    state.segment(0,7) = Y;
}
#else
void UAVstate::setY(Eigen::VectorXd &state, Eigen::Vector<double, 6> Y)
{
    state.segment(0,6) = Y;
}
#endif

void UAVstate::setX(Eigen::VectorXd &state, Eigen::Vector<double, 6> X)
{
#ifdef USE_QUATERIONS
    state.segment<6>(7) = X;
#else
    state.segment<6>(6) = X;
#endif
}

void UAVstate::setOm(Eigen::VectorXd &state, Eigen::VectorXd Om)
{
    state.segment(omOffset,state.size()-omOffset) = Om;
}

#ifdef USE_QUATERIONS
Eigen::Vector<double, 7> UAVstate::getY(const Eigen::VectorXd &state)
{
    return state.segment<7>(0);
}
#else
Eigen::Vector<double, 6> UAVstate::getY(const Eigen::VectorXd &state)
{
    return state.segment<6>(0);
}
#endif

Eigen::Vector<double, 6> UAVstate::getX(const Eigen::VectorXd &state)
{
#ifdef USE_QUATERIONS
    return state.segment<6>(7);
#else
    return state.segment<6>(6);
#endif
}

Eigen::VectorXd UAVstate::getOm(const Eigen::VectorXd &state)
{
    return state.segment(omOffset,state.size()-omOffset);
}

std::ostream& operator << ( std::ostream& outs, const UAVstate& state)
{
  return outs << "y:\n" << state.y.transpose() << "\nx:\n" << state.x.transpose() << "\nom:\n" <<state.rotorAngularVelocities.transpose() << std::endl;
}
