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
#include <chrono>
using namespace std::chrono_literals;


Simulation::Simulation(UAVparams& params, UAVstate& state):
    _params{params},
    _state{state},
    forces(params),
    matrices(params)
{
    stateOutSock = zmq::socket_t(_ctx, zmq::socket_type::pub);

    std::stringstream ss;
    ss << "/tmp/" << _params.name;
    std::string address = ss.str();
    // try{
    //     fs::remove_all(address);
    // } catch (const fs::filesystem_error& ex) {
    //     std::cerr << "Remove error: " << ex.what() << std::endl;
    // }
    if (!fs::create_directory(address))
        std::cerr << "Can not create comunication folder" <<std::endl;
    ss.str("");

    ss << "ipc:///tmp/" << _params.name << "/state";
    address = ss.str();
    stateOutSock.bind(address);
    std::cout << "Starting state publisher: " << address << std::endl;
    //TODO: Remove below temporiary solution
    //stateOutSock.bind("tcp://*:9090");

    ss.str("");
    ss << "ipc:///tmp/" << _params.name << "/control";
    address = ss.str();
    controlListener = std::thread(controlListenerJob,&_ctx, std::string(address),std::ref(_state),std::ref(matrices));

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
    //std::cout << s << std::endl;
    message.rebuild(s.data(), s.size());
    ss.str("");
    stateOutSock.send(message,zmq::send_flags::none);

    ss << "vb:" << _state.getX().format(commaFormat);
    s = ss.str();
    message.rebuild(s.data(), s.size());
    ss.str("");
    stateOutSock.send(message,zmq::send_flags::none);

    Eigen::Vector<double,6> vn = matrices.TMatrix(_state.getY())*_state.getX(); 
    ss << "vn:" << vn.format(commaFormat);
    s = ss.str();
    message.rebuild(s.data(), s.size());
    ss.str("");
    stateOutSock.send(message,zmq::send_flags::none);

    ss << "om:" << _state.getOm().format(commaFormat);
    s = ss.str();
    message.rebuild(s.data(), s.size());
    stateOutSock.send(message,zmq::send_flags::none);
}

void Simulation::sendIdle()
{
    zmq::message_t message("idle",4);
    stateOutSock.send(message,zmq::send_flags::none);
}

void Simulation::countDown()
{
    constexpr int start = 3;
    std::stringstream ss;
    std::string s;
    ss.precision(3);
    for(int i = start; i > 0; i--)
    {
        ss << "t:" << std::fixed << -i;
        s = ss.str();
        std::cout << s << std::endl;
        zmq::message_t message(s.data(), s.size());
        ss.str("");
        stateOutSock.send(message,zmq::send_flags::none);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void clampOrientation(Eigen::VectorXd& state)
{
    for (size_t i = 3; i < 6; i++)
    {
        double x = fmod(state(i) + std::numbers::pi,2*std::numbers::pi);
        if (x < 0)
            x += 2*std::numbers::pi;
        state(i) =  x - std::numbers::pi;
    }
}

void Simulation::run()
{
    matrices.updateMatrices();
    TimedLoop loop(std::round(step_time*1000.0), [this](){
        VectorXd next = RK4_step(_state.real_time,_state.getState(),RHS,step_time);
        clampOrientation(next);
        _state = next;
        _state.real_time+=step_time;
        sendState();
        //std::cout << _state.real_time << std::endl;
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
                sendIdle();
                _state.status_cv.wait_for(lck,1s);
                lck.unlock();
            break;
            case Status::running:
                //countDown();
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
