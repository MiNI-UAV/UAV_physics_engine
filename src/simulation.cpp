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



Simulation::Simulation()
{
    aircraft = new Aircraft();

    const UAVparams* params = UAVparams::getSingleton();
    stateOutSock = zmq::socket_t(_ctx, zmq::socket_type::pub);

    std::stringstream ss;
    ss << "/tmp/" << params->name;
    std::string address = ss.str();
    if (!fs::create_directory(address))
        std::cerr << "Can not create comunication folder" <<std::endl;
    ss.str("");

    ss << "ipc:///tmp/" << params->name << "/state";
    address = ss.str();
    stateOutSock.bind(address);
    std::cout << "Starting state publisher: " << address << std::endl;

    ss.str("");
    ss << "ipc:///tmp/" << params->name << "/control";
    address = ss.str();
    controlListener = std::thread(controlListenerJob,&_ctx, std::string(address), aircraft);
}

void Simulation::sendIdle()
{
    zmq::message_t message("idle",4);
    stateOutSock.send(message,zmq::send_flags::none);
}

void Simulation::run()
{
    TimedLoop loop(std::round(STEP_TIME*1000.0), [this](){
        aircraft->update();
        aircraft->sendState(&stateOutSock);
    },  aircraft->state.status);

    std::mutex mtx;
    std::unique_lock<std::mutex> lck(mtx);
    lck.unlock();
    bool run = true;
    while(run)
    {
        switch(aircraft->state.status)
        {
            case Status::idle:
                lck.lock();
                std::cout << "Idle..." << std::endl;
                sendIdle();
                aircraft->state.status_cv.wait_for(lck,1s);
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
    delete aircraft;
}
