#include <Eigen/Dense>
#include <zmq.hpp>
#include <iostream>

#include "aircraft.hpp"
#include "../uav_state.hpp"
#include "../uav_params.hpp"
#include "../matrices.hpp"
#include "../forces.hpp"
#include "../defines.hpp"


void Aircraft::sendState(zmq::socket_t* socket)
{
    static Eigen::IOFormat commaFormat(3, Eigen::DontAlignCols," ",",");
    std::stringstream ss;
    std::string s;
    ss.precision(3);

    ss << "t:"<< std::fixed << state.real_time.load();
    s = ss.str();
    zmq::message_t message(s.data(), s.size());
    ss.str("");
    socket->send(message,zmq::send_flags::none);

    ss << "pos:" << state.getY().format(commaFormat);
    s = ss.str();
    //std::cout << s << std::endl;
    message.rebuild(s.data(), s.size());
    ss.str("");
    socket->send(message,zmq::send_flags::none);

    ss << "vb:" << state.getX().format(commaFormat);
    s = ss.str();
    message.rebuild(s.data(), s.size());
    ss.str("");
    socket->send(message,zmq::send_flags::none);

#ifdef USE_QUATERIONS
    Eigen::Vector<double,6> Y = Matrices::quaterionsToRPY(state.getY());
#else
    Eigen::Vector<double,6> Y = state.getY();
#endif

    Eigen::Vector<double,6> vn = Matrices::TMatrix(Y)*state.getX(); 
    ss << "vn:" << vn.format(commaFormat);
    s = ss.str();
    message.rebuild(s.data(), s.size());
    ss.str("");
    socket->send(message,zmq::send_flags::none);

    ss << "ab:" << state.getAcceleration().format(commaFormat);
    s = ss.str();
    message.rebuild(s.data(), s.size());
    ss.str("");
    socket->send(message,zmq::send_flags::none);

    if(noOfRotors > 0)
    {
        ss << "om:" << state.getOm().format(commaFormat);
        s = ss.str();
        //std::cout << s << std::endl;
        message.rebuild(s.data(), s.size());
        socket->send(message,zmq::send_flags::none);
    }

    if(surfaces.getNoOfSurface() > 0)
    {
        ss << "cs:" << surfaces.getValues().format(commaFormat);
        s = ss.str();
        //std::cout << s << std::endl;
        message.rebuild(s.data(), s.size());
        socket->send(message,zmq::send_flags::none);
    }

    if(noOfJets > 0)
    {
        ss << "jt:";
        for (int i = 0; i < noOfJets; i++)
        {
            ss << jets[i].getLastThrust() << ",";
        }
        
        s = ss.str();
        s.pop_back();
        //std::cout << s << std::endl;
        message.rebuild(s.data(), s.size());
        socket->send(message,zmq::send_flags::none);
    }
}

bool Aircraft::startJet(int index)
{
    if(index < 0 || index >= noOfJets) return false;
    return jets[index].start(state.real_time);
}

void Aircraft::trim() 
{
    surfaces.restoreTrim();
}

bool Aircraft::setSurface(Eigen::VectorXd new_surface) 
{ 
    return surfaces.setValues(new_surface);
}
