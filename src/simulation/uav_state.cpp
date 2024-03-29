#include "uav_state.hpp"
#include "../dynamic/matrices.hpp"
#include "../params.hpp"

UAVstate::UAVstate()
{
    const UAVparams* params = UAVparams::getSingleton();
    noOfRotors = params->noOfRotors;

    Eigen::Vector<double,6> pos;
    pos.setZero();
    pos.head<3>() = params->initialPosition;
    pos.tail<3>() = params->initialOrientation;
    #ifdef USE_QUATERIONS
    y = Matrices::RPYtoQuaterion(pos);
    #else
    y = pos;
    #endif
    x.setZero();
    x.head<3>() = params->initialVelocity;
    rotorAngularVelocities.setZero(noOfRotors);
    acceleration.setZero();


    demandedAngularBuf[0].setZero(noOfRotors);
    demandedAngularBuf[1].setZero(noOfRotors);
    demanded_ptr = demandedAngularBuf + 1;

    forceBuf[0].setZero();
    forceBuf[1].setZero();
    force_ptr = force_ptr+1;
    forceValidityCounter = 0;

    status = Status::idle;
    if(params->instantRun) setStatus(Status::running);
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
    static auto maxSpeeds = UAVparams::getSingleton()->getRotorMaxSpeeds();
    newDemandedOm = newDemandedOm.cwiseMin(maxSpeeds);
    demandedAngularBuf[demandedBufSwitch] = newDemandedOm;
    demanded_ptr = demandedAngularBuf + demandedBufSwitch;
    demandedBufSwitch = 1 - demandedBufSwitch;
}

void UAVstate::setForce(Eigen::Vector3d force, Eigen::Vector3d torque)
{
    static const int microsteps = ODE::getMicrosteps(ODE::fromString(Params::getSingleton()->ODE_METHOD));

    Eigen::Vector<double,6> newForce;
    newForce << force,torque;
    forceBuf[forceBufSwitch] = newForce;
    forceValidityCounter = def::validityOfForce * microsteps;
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
