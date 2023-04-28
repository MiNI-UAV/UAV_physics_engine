#include <Eigen/Dense>
#include <zmq.hpp>
#include <sstream>
#include <iostream>
#include "control.hpp"
#include "uav_state.hpp"
#include "status.hpp"

void setWind(UAVstate& state, std::string& msg_str)
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
    if(i == 3)
    {
        std::cout << "Setting wind to: " << wind.transpose() << std::endl;
        state.setWind(wind);
    }
}

void setSpeed(UAVstate& state, std::string& msg_str, int n)
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
    if(i == n)
    {
        std::cout << "Setting speed to: " << speed.transpose() << std::endl;
        state.setDemandedOm(speed);
    }
}

bool control(UAVstate& state, std::string& msg_str)
{
    if(msg_str.compare("c:ping") == 0)
    {
        return true;
    }
    if(msg_str.compare("c:start") == 0)
    {
        state.setStatus(Status::running);
        return true;
    }
    if(msg_str.compare("c:pause") == 0)
    {
        state.setStatus(Status::idle);
        return true;
    }
    if(msg_str.compare("c:stop") != 0)
    {
        std::cerr << "Unknown msg: " << msg_str << std::endl;
    }
    state.setStatus(Status::exiting);
    return false;
}

void controlListenerJob(zmq::context_t* ctx, std::string address,UAVstate& state)
{
    std::cout << "Starting constrol subscriber: " << address << std::endl;

    zmq::socket_t controlInSock = zmq::socket_t(*ctx, zmq::socket_type::sub);
    controlInSock.bind(address);
    //controlInSock.bind("tcp://192.168.234.1:6667");
    //Subscribe wind
    controlInSock.set(zmq::sockopt::subscribe, "w:");
    //Subscribe demanded speed
    controlInSock.set(zmq::sockopt::subscribe, "s:");
    //Subscribe control instructions
    controlInSock.set(zmq::sockopt::subscribe, "c:");

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
        //std::cout << msg_str << std::endl;

        switch(msg_str[0])
        {
            case 'w':
                setWind(state,msg_str);
            break;
            case 's':
                setSpeed(state,msg_str,n);
            break;
            case 'c':
                run = control(state,msg_str);
            break;
            default:
                std::cerr << "Unknown msg: " << msg_str << std::endl;
                state.setStatus(Status::exiting);
                run = false;
        }
    }
    controlInSock.close();
    std::cout << "Ending listener: " << address << std::endl;
}