#include <Eigen/Dense>
#include <zmq.hpp>
#include <sstream>
#include <iostream>
#include "control.hpp"
#include "uav_state.hpp"
#include "status.hpp"

void setWind(UAVstate& state, std::string& msg_str, zmq::socket_t& sock)
{
    Eigen::Vector3d wind;
    std::istringstream f(msg_str.substr(2));
    std::string s;
    int i; 
    for (i = 0; i < 3; i++)
    {
        if(!getline(f, s, ','))
        {
            std::cerr << "Invalid wind command" << std::endl;
            break;
        }
        wind(i) = std::stod(s);
    }
    zmq::message_t response("error",5);
    if(i == 3)
    {
        //std::cout << "Setting wind to: " << wind.transpose() << std::endl;
        state.setWind(wind);
        response.rebuild("ok",2);
    }
    sock.send(response,zmq::send_flags::none);
}

void setSpeed(UAVstate& state, std::string& msg_str, int n, zmq::socket_t& sock)
{
    Eigen::VectorXd speed;
    speed.setZero(n);
    std::istringstream f(msg_str.substr(2));
    std::string s;
    int i;
    for (i = 0; i < n; i++)
    {
        if(!getline(f, s, ','))
        {
            std::cerr << "Invalid speed command" << std::endl;
            break;
        }
        speed(i) = std::stod(s);
    }
    zmq::message_t response("error",5);
    if(i == n)
    {
        //std::cout << "Setting speed to: " << speed.transpose() << std::endl;
        state.setDemandedOm(speed);
        response.rebuild("ok",2);
    }
    sock.send(response,zmq::send_flags::none);
}

bool control(UAVstate& state, std::string& msg_str, zmq::socket_t& sock)
{
    zmq::message_t response("ok",2);
    if(msg_str.compare("c:ping") == 0)
    {
        response.rebuild("pong",4);
        sock.send(response,zmq::send_flags::none);
        return true;
    }
    if(msg_str.compare("c:start") == 0)
    {
        state.setStatus(Status::running);
        sock.send(response,zmq::send_flags::none);
        return true;
    }
    if(msg_str.compare("c:pause") == 0)
    {
        state.setStatus(Status::idle);
        sock.send(response,zmq::send_flags::none);
        return true;
    }
    if(msg_str.compare("c:stop") != 0)
    {
        response.rebuild("error",5);
        std::cerr << "Unknown msg: " << msg_str << std::endl;
    }
    sock.send(response,zmq::send_flags::none);
    state.setStatus(Status::exiting);
    return false;
}

void controlListenerJob(zmq::context_t* ctx, std::string address,UAVstate& state)
{
    std::cout << "Starting control subscriber: " << address << std::endl;
    zmq::socket_t controlInSock = zmq::socket_t(*ctx, zmq::socket_type::rep);
    controlInSock.bind(address);
    int n = state.getNoOfRotors();
    bool run = true;
    while(run)
    {
        zmq::message_t msg;
        const auto res = controlInSock.recv(msg, zmq::recv_flags::none);
        if(!res)
        {
            std::cerr << "Listener recv error" << std::endl;
            return;
        } 
        std::string msg_str =  std::string(static_cast<char*>(msg.data()), msg.size());
        switch(msg_str[0])
        {
            case 'w':
                setWind(state,msg_str,controlInSock);
            break;
            case 's':
                setSpeed(state,msg_str,n,controlInSock);
            break;
            case 'c':
                run = control(state,msg_str,controlInSock);
            break;
            default:
                zmq::message_t response("error",5);
                controlInSock.send(response,zmq::send_flags::none);
                std::cerr << "Unknown msg: " << msg_str << std::endl;
                state.setStatus(Status::exiting);
                run = false;
        }
    }
    controlInSock.close();
    std::cout << "Ending listener: " << address << std::endl;
}