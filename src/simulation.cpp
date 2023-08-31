#include <Eigen/Dense>
#include <zmq.hpp>
#include <iostream>
#include <cstdio>
#include <thread>
#include <mutex>
#include <filesystem>
namespace fs = std::filesystem;
#include <chrono>
using namespace std::chrono_literals;

#include "simulation.hpp"
#include "uav_params.hpp"
#include "uav_state.hpp"
#include "forces.hpp"
#include "matrices.hpp"
#include "common.hpp"
#include "control.hpp"
#include "defines.hpp"



Simulation::Simulation(UAVstate& state):
    _state{state}
{
    UAVparams* params = UAVparams::getSingleton();
    stateOutSock = zmq::socket_t(_ctx, zmq::socket_type::pub);

    std::stringstream ss;
    ss << "/tmp/" << params->name;
    std::string address = ss.str();
    // try{
    //     fs::remove_all(address);
    // } catch (const fs::filesystem_error& ex) {
    //     std::cerr << "Remove error: " << ex.what() << std::endl;
    // }
    if (!fs::create_directory(address))
        std::cerr << "Can not create comunication folder" <<std::endl;
    ss.str("");

    ss << "ipc:///tmp/" << params->name << "/state";
    address = ss.str();
    stateOutSock.bind(address);
    std::cout << "Starting state publisher: " << address << std::endl;
    //TODO: Remove below temporiary solution
    //stateOutSock.bind("tcp://*:9090");

    ss.str("");
    ss << "ipc:///tmp/" << params->name << "/control";
    address = ss.str();
    controlListener = std::thread(controlListenerJob,&_ctx, std::string(address),std::ref(_state),std::ref(matrices));

    RHS = [this] (double, Eigen::VectorXd local_state)
    {
        VectorXd res;
        res.setZero(local_state.size());
        auto Y = UAVstate::getY(local_state);
        auto X = UAVstate::getX(local_state);
        Matrix3d r_nb = Matrices::R_nb(Y);

        #ifdef USE_QUATERIONS
        Vector<double,7> newY = Vector<double,7>::Zero();
        newY.head<3>() = r_nb.transpose() * X.head<3>();
        Eigen::Vector4d q = Y.tail<4>();
        newY.tail<4>() = Matrices::OM_conj(X)*q  + (1.0 - q.squaredNorm())*q;
        UAVstate::setY(res,newY);
        #else
        UAVstate::setY(res, Matrices::TMatrix(Y)*X);
        #endif
        Eigen::Vector<double,6> accel = matrices.invMassMatrix*(Forces::gravity_loads(r_nb) 
           + Forces::lift_loads(UAVstate::getOm(local_state))
           + Forces::aerodynamic_loads(r_nb,X,_state.getWind())
           + _state.getOuterForce() 
           - Matrices::gyroMatrix(X) * matrices.massMatrix * UAVstate::getX(local_state));
        UAVstate::setX(res,accel);
        UAVstate::setOm(res, Forces::angularAcceleration(this->_state.getDemandedOm(),UAVstate::getOm(local_state)));
        return res;
    };
}

void clampOrientation([[maybe_unused]] Eigen::VectorXd& state)
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

#ifdef USE_QUATERIONS
    Eigen::Vector<double,6> Y = matrices.quaterionsToRPY(_state.getY());
#else
    Eigen::Vector<double,6> Y = _state.getY();
#endif

    Eigen::Vector<double,6> vn = matrices.TMatrix(Y)*_state.getX(); 
    ss << "vn:" << vn.format(commaFormat);
    s = ss.str();
    message.rebuild(s.data(), s.size());
    ss.str("");
    stateOutSock.send(message,zmq::send_flags::none);

    ss << "ab:" << _state.getAcceleration().format(commaFormat);
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

void Simulation::run()
{
    matrices.updateMatrices();
    TimedLoop loop(std::round(step_time*1000.0), [this](){
        const std::lock_guard<std::mutex> lock(_state.state_mtx);
        VectorXd next = RK4_step(_state.real_time,_state.getState(),RHS,step_time);
        clampOrientation(next);
        _state.setAcceleration((UAVstate::getX(next) - _state.getX())/step_time);
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
                std::cout << "Running..." << std::endl;
                loop.go();
            break;
            case Status::exiting:
                std::cout << "Exiting..." << std::endl;
                run = false;
            break;
            case Status::reload:
                run = false;
            break;
        }
    }

}

Simulation::~Simulation()
{
    controlListener.join();
}
