#include "simulation.hpp"
#include <Eigen/Dense>
#include <zmq.hpp>
#include <iostream>
#include <cstdio>

#include "uav_params.hpp"
#include "uav_state.hpp"
#include "forces.hpp"
#include "matrices.hpp"
#include "RK4.hpp"
#include "timed_loop.hpp"


Simulation::Simulation(UAVparams& params, UAVstate& state):
    _params{params},
    _state{state},
    forces(params),
    matrices(params)
{
    sock = zmq::socket_t(_ctx, zmq::socket_type::pub);
    sock.bind("ipc:///tmp/pos");

    RHS = [this] (double, Eigen::VectorXd local_state)
    {
        VectorXd res;
        res.setZero(local_state.size());
        UAVstate::setY(res,matrices.TMatrix(UAVstate::getY(local_state))*UAVstate::getX(local_state));
        //TODO: missing wind!
        UAVstate::setX(res,matrices.invMassMatrix*(forces.gravity_forces(UAVstate::getY(local_state)) 
           + forces.lift_forces(UAVstate::getOm(local_state)) 
           -  matrices.gyroMatrix(UAVstate::getX(local_state)) * matrices.massMatrix * UAVstate::getX(local_state)));
        UAVstate::setOm(res, forces.angularAcceleration(this->_state.getDemandedOm(),UAVstate::getOm(local_state)));
        return res;
    };
}

void Simulation::sendState()
{
    constexpr int msg_size = 20;
    char msg[msg_size];
    int sz = std::snprintf(msg,msg_size,"t: %5.3lf",real_time);
    zmq::message_t message(msg,sz);
    sock.send(message,zmq::send_flags::none);
}

void Simulation::run()
{
    matrices.updateMatrices();
    TimedLoop loop(1, [this](){
        VectorXd next = RK4_step(real_time,_state.getState(),RHS,0.001);
        real_time+=0.001;
        _state = next;
        sendState();
        return true;
    });
    loop.go();
}
