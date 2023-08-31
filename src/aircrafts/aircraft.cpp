#include <Eigen/Dense>
#include <zmq.hpp>
#include <iostream>

#include "aircraft.hpp"
#include "../uav_state.hpp"
#include "../uav_params.hpp"
#include "../matrices.hpp"
#include "../forces.hpp"
#include "../defines.hpp"

void clampOrientationIfNessessery([[maybe_unused]] Eigen::VectorXd& state)
{
#ifndef USE_QUATERIONS
    for (size_t i = 3; i < 6; i++)
    {
        double x = fmod(state(i) + std::numbers::pi,2*std::numbers::pi);
        if (x < 0)
            x += 2*std::numbers::pi;
        state(i) =  x - std::numbers::pi;
    }
#endif
}

void Aircraft::update()
{
    VectorXd next = RK4_step(_state.real_time,_state.getState(),std::bind_front(&Aircraft::RHS, this),STEP_TIME);
    clampOrientationIfNessessery(next);
    _state.setAcceleration((UAVstate::getX(next) - _state.getX())/STEP_TIME);
    _state = next;
    _state.real_time+=STEP_TIME;
}

void Aircraft::sendState(zmq::socket_t socket)
{
    static Eigen::IOFormat commaFormat(3, Eigen::DontAlignCols," ",",");
    std::stringstream ss;
    std::string s;
    ss.precision(3);

    ss << "t:"<< std::fixed << _state.real_time.load();
    s = ss.str();
    zmq::message_t message(s.data(), s.size());
    ss.str("");
    socket.send(message,zmq::send_flags::none);

    ss << "pos:" << _state.getY().format(commaFormat);
    s = ss.str();
    //std::cout << s << std::endl;
    message.rebuild(s.data(), s.size());
    ss.str("");
    socket.send(message,zmq::send_flags::none);

    ss << "vb:" << _state.getX().format(commaFormat);
    s = ss.str();
    message.rebuild(s.data(), s.size());
    ss.str("");
    socket.send(message,zmq::send_flags::none);

#ifdef USE_QUATERIONS
    Eigen::Vector<double,6> Y = Matrices::quaterionsToRPY(_state.getY());
#else
    Eigen::Vector<double,6> Y = _state.getY();
#endif

    Eigen::Vector<double,6> vn = Matrices::TMatrix(Y)*_state.getX(); 
    ss << "vn:" << vn.format(commaFormat);
    s = ss.str();
    message.rebuild(s.data(), s.size());
    ss.str("");
    socket.send(message,zmq::send_flags::none);

    ss << "ab:" << _state.getAcceleration().format(commaFormat);
    s = ss.str();
    message.rebuild(s.data(), s.size());
    ss.str("");
    socket.send(message,zmq::send_flags::none);

    ss << "om:" << _state.getOm().format(commaFormat);
    s = ss.str();
    message.rebuild(s.data(), s.size());
    socket.send(message,zmq::send_flags::none);
}

void Aircraft::updateMass() 
{
    massMatrix = Matrices::massMatrix2();
    invMassMatrix = massMatrix.inverse();
}

Eigen::VectorXd Aircraft::RHS(double, Eigen::VectorXd local_state) {
    VectorXd res;
    res.setZero(local_state.size());
    auto Y = UAVstate::getY(local_state);
    auto X = UAVstate::getX(local_state);
    Matrix3d r_nb = Matrices::R_nb(Y);

#ifdef USE_QUATERIONS
    Vector<double, 7> newY = Vector<double, 7>::Zero();
    newY.head<3>() = r_nb.transpose() * X.head<3>();
    Eigen::Vector4d q = Y.tail<4>();
    newY.tail<4>() = Matrices::OM_conj(X) * q + (1.0 - q.squaredNorm()) * q;
    UAVstate::setY(res, newY);
#else
    UAVstate::setY(res, Matrices::TMatrix(Y) * X);
#endif
    Eigen::Vector<double, 6> accel = invMassMatrix *
        (
            Forces::gravity_loads(r_nb) +
            Forces::lift_loads(UAVstate::getOm(local_state)) +
            Forces::aerodynamic_loads(r_nb, X, _state.getWind()) +
            _state.getOuterForce() -
            Matrices::gyroMatrix(X) * massMatrix * UAVstate::getX(local_state)
        );
    UAVstate::setX(res, accel);
    UAVstate::setOm(res,
    Forces::angularAcceleration(this->_state.getDemandedOm(),
        UAVstate::getOm(local_state)));
    return res;
}
