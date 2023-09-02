#include <Eigen/Dense>
#include <zmq.hpp>
#include <sstream>
#include <iostream>
#include "control.hpp"
#include "uav_state.hpp"
#include "common.hpp"
#include "matrices.hpp"
#include "aircrafts/aircraft.hpp"

void setWind(Aircraft* aircraft, std::string& msg_str, zmq::socket_t& sock)
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
        aircraft->state.setWind(wind);
        response.rebuild("ok",2);
    }
    sock.send(response,zmq::send_flags::none);
}

void setForce(Aircraft* aircraft, std::string& msg_str, zmq::socket_t& sock)
{
    Eigen::Vector3d force, torque;
    std::istringstream f(msg_str.substr(2));
    std::string s;
    int i; 
    for (i = 0; i < 6; i++)
    {
        if(!getline(f, s, ','))
        {
            std::cerr << "Invalid force command" << std::endl;
            break;
        }
        if(i < 3)
        {
            force(i) = std::stod(s);
        }
        else
        {
            torque(i-3) = std::stod(s);
        }   
    }
    zmq::message_t response("ok",2);
    if(i == 6)
    {
        aircraft->state.setForce(force,torque);
    }
    else if(i == 3)
    {
        aircraft->state.setForce(force);
    }
    else
    {
        response.rebuild("error",5);
    }
    sock.send(response,zmq::send_flags::none);
}

void setSpeed(Aircraft* aircraft, std::string& msg_str, zmq::socket_t& sock)
{
    int n = aircraft->state.getNoOfRotors();
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
        aircraft->state.setDemandedOm(speed);
        response.rebuild("ok",2);
    }
    sock.send(response,zmq::send_flags::none);
}

bool control(Aircraft* aircraft, std::string& msg_str, zmq::socket_t& sock)
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
        aircraft->state.setStatus(Status::running);
        sock.send(response,zmq::send_flags::none);
        return true;
    }
    if(msg_str.compare("c:pause") == 0)
    {
        aircraft->state.setStatus(Status::idle);
        sock.send(response,zmq::send_flags::none);
        return true;
    }
    if(msg_str.compare("c:stop") != 0)
    {
        response.rebuild("error",5);
        std::cerr << "Unknown msg: " << msg_str << std::endl;
    }
    sock.send(response,zmq::send_flags::none);
    aircraft->state.setStatus(Status::exiting);
    return false;
}

void shot(Aircraft* aircraft, std::string& msg_str, zmq::socket_t& sock)
{
    std::istringstream f(msg_str.substr(2));

    double m = 0.03;
    double speed = 150.0;
    Vector3d r(0.0, 0.0, 0.1);

    std::string res;
    for (int i = 0; i < 5; i++)
    {
        if(!getline(f, res, ',')) break;
        switch (i)
        {
        case 0:
            m = std::stod(res);
            break;
        case 1:
            speed = std::stod(res);
            break;
        case 2:
        case 3:
        case 4:
            r(i-2) = std::stod(res);
            break;
        }
    }

    Vector3d linear_speed = aircraft->calcMomentumConservanceConservation(m,speed,r);

    static Eigen::IOFormat commaFormat(3, Eigen::DontAlignCols," ",",");
    std::stringstream ss;
    ss << "ok;" << linear_speed.format(commaFormat);
    std::string s = ss.str();
    zmq::message_t response(s.data(), s.size()); 
    sock.send(response,zmq::send_flags::none);
}

bool isNormal(double factor)
{
    return factor >= 0.0 && factor <= 1.0;
}

void solidSurfColision(Aircraft* aircraft, std::string& msg_str, zmq::socket_t& sock)
{
    std::istringstream f(msg_str.substr(2));
    zmq::message_t response("error",5);
    int i;
    double COR = 0.0, mi_static = 0.0, mi_dynamic = 0.0;
    Eigen::Vector3d collisionPoint(0.0,0.0,0.0), surfaceNormal(0.0,0.0,0.0);
    std::string res;
    for (i = 0; i < 9; i++)
    {
        if(!getline(f, res, ',')) break;
        switch (i)
        {
        case 0:
            COR = std::stod(res);
            break;
        case 1:
            mi_static = std::stod(res);
            break;
        case 2:
            mi_dynamic = std::stod(res);
            break;
        case 3:
        case 4:
        case 5:
            collisionPoint(i-3) = std::stod(res);
            break;
        case 6:
        case 7:
        case 8:
            surfaceNormal(i-6) = std::stod(res);
            break;
        }
    }
    if (i == 9
        && isNormal(COR)
        && isNormal(mi_static)
        && isNormal(mi_dynamic)
        && mi_static >= mi_dynamic)
    {
        aircraft->calcImpulseForce(COR, mi_static, mi_dynamic, collisionPoint, surfaceNormal);
        response.rebuild("ok",2);
    }
    sock.send(response,zmq::send_flags::none);
    return; 
}

void controlListenerJob(zmq::context_t* ctx, std::string address, Aircraft* aircraft)
{
    std::cout << "Starting control subscriber: " << address << std::endl;
    zmq::socket_t controlInSock = zmq::socket_t(*ctx, zmq::socket_type::rep);
    controlInSock.bind(address);
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
                setWind(aircraft,msg_str,controlInSock);
            break;
            case 's':
                setSpeed(aircraft,msg_str, controlInSock);
            break;
            case 'c':
                run = control(aircraft,msg_str,controlInSock);
            break;
            case 'd':
                shot(aircraft,msg_str,controlInSock);
            break;
            case 'f':
                setForce(aircraft,msg_str,controlInSock);
            break;
            case 'j':
                solidSurfColision(aircraft,msg_str,controlInSock);
            break;
            default:
                zmq::message_t response("error",5);
                controlInSock.send(response,zmq::send_flags::none);
                std::cerr << "Unknown msg: " << msg_str << std::endl;
                aircraft->state.setStatus(Status::exiting);
                run = false;
        }
    }
    controlInSock.close();
    std::cout << "Ending listener: " << address << std::endl;
}