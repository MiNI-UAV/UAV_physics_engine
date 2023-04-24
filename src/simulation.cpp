#include <Eigen/Dense>
#include <zmq.hpp>
#include <iostream>
#include <cstdio>
#include <thread>
#include <mutex>
#include <filesystem>
namespace fs = std::filesystem;

#include "simulation.hpp"
#include "uav_params.hpp"
#include "uav_state.hpp"
#include "forces.hpp"
#include "matrices.hpp"
#include "RK4.hpp"
#include "timed_loop.hpp"
#include "control.hpp"


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
    std::cout << "Starting state publisher: " << address << std::endl;
    //stateOutSock.bind("tcp://192.168.234.1:5556");

    std::snprintf(address,100,"ipc:///tmp/%s/control",_params.name);
    controlListener = std::thread(controlListenerJob,&_ctx, std::string(address),std::ref(_state));

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
    std::string s;
    ss.precision(3);

    ss << "t:"<< std::fixed << _state.real_time.load();
    s = ss.str();
    zmq::message_t message(s.data(), s.size());
    ss.str("");
    stateOutSock.send(message,zmq::send_flags::none);

    ss << "pos:" << _state.getY().format(commaFormat);
    s = ss.str();
    message.rebuild(s.data(), s.size());
    ss.str("");
    stateOutSock.send(message,zmq::send_flags::none);

    ss << "vel:" << _state.getX().format(commaFormat);
    s = ss.str();
    message.rebuild(s.data(), s.size());
    ss.str("");
    stateOutSock.send(message,zmq::send_flags::none);

    ss << "om:" << _state.getOm().format(commaFormat);
    s = ss.str();
    message.rebuild(s.data(), s.size());
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
        std::cout << _state.real_time << std::endl;
    }, _state.status);

    std::mutex mtx;
    std::unique_lock<std::mutex> lck(mtx);
    lck.unlock();
    bool run = true;
    while(run)
    {
        switch(_state.status)
        {
            case Status::idle:
                lck.lock();
                std::cout << "Idle..." << std::endl;
                _state.status_cv.wait(lck);
                lck.unlock();
            break;
            case Status::running:
                std::cout << "Running..." << std::endl;
                loop.go();
            break;
            case Status::exiting:
                std::cout << "Exiting..." << std::endl;
                run = false;
            break;
        }
    }

}

Simulation::~Simulation()
{
    controlListener.join();
}
