#include "simulation.hpp"
#include <Eigen/Dense>
#include <zmq.hpp>
#include <iostream>
#include <cstdio>
#include <thread>
#include <filesystem>
namespace fs = std::filesystem;

#include "uav_params.hpp"
#include "uav_state.hpp"
#include "forces.hpp"
#include "matrices.hpp"
#include "RK4.hpp"
#include "timed_loop.hpp"


void controlListenerJob(zmq::context_t* ctx, std::string address)
{
    zmq::socket_t controlInSock = zmq::socket_t(*ctx, zmq::socket_type::sub);
    controlInSock.bind(address);
    //controlInSock.bind("tcp://192.168.234.1:6667");
    //Subscribe wind
    controlInSock.set(zmq::sockopt::subscribe, "w:");
    //Subscribe demanded speed
    controlInSock.set(zmq::sockopt::subscribe, "s:");
    //Subscribe control instructions
    controlInSock.set(zmq::sockopt::subscribe, "c:");

    while(1)
    {
        zmq::message_t msg;
        auto res = controlInSock.recv(msg, zmq::recv_flags::none);
        std::cout << std::string(static_cast<char*>(msg.data()), msg.size()) << std::endl;
    }
}


Simulation::Simulation(UAVparams& params, UAVstate& state):
    _params{params},
    _state{state},
    forces(params),
    matrices(params)
{
    stateOutSock = zmq::socket_t(_ctx, zmq::socket_type::pub);

    char address[100];
    std::snprintf(address,100,"/tmp/%s",_params.name);
    fs::remove_all(address);
    if (!fs::create_directory(address))
        std::cerr << "Can not create comunication folder";
    std::snprintf(address,100,"ipc:///tmp/%s/state",_params.name);
    stateOutSock.bind(address);
    //stateOutSock.bind("tcp://192.168.234.1:5556");

    std::snprintf(address,100,"ipc:///tmp/%s/control",_params.name);
    controlListener = std::thread(controlListenerJob,&_ctx, std::string(address));

    RHS = [this] (double, Eigen::VectorXd local_state)
    {
        VectorXd res;
        res.setZero(local_state.size());
        UAVstate::setY(res,matrices.TMatrix(UAVstate::getY(local_state))*UAVstate::getX(local_state));
        UAVstate::setX(res,matrices.invMassMatrix*(forces.gravity_loads(UAVstate::getY(local_state)) 
           + forces.lift_loads(UAVstate::getOm(local_state))
           + forces.aerodynamic_loads(matrices,UAVstate::getX(local_state),UAVstate::getY(local_state),_state.getWind()) 
           - matrices.gyroMatrix(UAVstate::getX(local_state)) * matrices.massMatrix * UAVstate::getX(local_state)));
        UAVstate::setOm(res, forces.angularAcceleration(this->_state.getDemandedOm(),UAVstate::getOm(local_state)));
        return res;
    };
}

void Simulation::sendState()
{
    static Eigen::IOFormat commaFormat(3, Eigen::DontAlignCols," ",",");
    std::stringstream ss;
    ss.precision(3);
    ss << "t:"<< std::fixed << _state.real_time.load();
    zmq::message_t message(ss.str());
    ss.str("");
    stateOutSock.send(message,zmq::send_flags::none);
    ss << "pos:" << _state.getY().format(commaFormat);
    message.rebuild(ss.str());
    ss.str("");
    stateOutSock.send(message,zmq::send_flags::none);
    ss << "vel:" << _state.getX().format(commaFormat);
    message.rebuild(ss.str());
    ss.str("");
    stateOutSock.send(message,zmq::send_flags::none);
    ss << "om:" << _state.getOm().format(commaFormat);
    message.rebuild(ss.str());
    stateOutSock.send(message,zmq::send_flags::none);
}

void Simulation::run()
{
    matrices.updateMatrices();
    TimedLoop loop(std::round(step_time*1000.0), [this](){
        VectorXd next = RK4_step(_state.real_time,_state.getState(),RHS,step_time);
        _state = next;
        _state.real_time+=step_time;
        sendState();
        return true;
    });
    loop.go();
}

Simulation::~Simulation()
{

}
