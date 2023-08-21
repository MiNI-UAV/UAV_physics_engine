#include <Eigen/Dense>
#include <zmq.hpp>
#include <sstream>
#include <iostream>
#include "control.hpp"
#include "uav_state.hpp"
#include "common.hpp"
#include "matrices.hpp"

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

void setForce(UAVstate& state, std::string& msg_str, zmq::socket_t& sock)
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
        state.setForce(force,torque);
    }
    else if(i == 3)
    {
        state.setForce(force);
    }
    else
    {
        response.rebuild("error",5);
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

Vector3d calcMomentumConservanceConservation(UAVstate& state, Matrices& matrices, double m, double speed, Vector3d r)
{
    const std::lock_guard<std::mutex> lock(state.state_mtx);
    Matrix3d R_nb = matrices.R_nb(state.getY());
    Matrix3d R_bn = R_nb.inverse();
    Matrix<double,6,6> T, T_inv;
    T.setZero();
    T_inv.setZero();
    T.block<3,3>(0,0) = R_bn;
    T.block<3,3>(3,3) = R_bn;
    T_inv.block<3,3>(0,0) = R_nb;
    T_inv.block<3,3>(3,3) = R_nb;
    Vector<double,6> momentum = matrices.massMatrix * (T * state.getX());
    Vector3d obj_vel = state.getX().head<3>();
    obj_vel(0) += speed;
    Vector3d obj_linear_speed = R_bn * obj_vel;
    Vector3d obj_linear_momentum = m * obj_linear_speed;
    Vector<double,6> obj_momentum;
    r = R_bn * r;
    obj_momentum << obj_linear_momentum, r.cross(obj_linear_momentum);
    matrices.reduceMass(m);
    Vector<double,6> newX = T_inv * matrices.invMassMatrix * (momentum - obj_momentum);
    state.setX(newX);
    return obj_linear_speed;
}

void shot(UAVstate& state, Matrices& matrices, std::string& msg_str, zmq::socket_t& sock)
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

    Vector3d linear_speed = calcMomentumConservanceConservation(state,matrices,m,speed,r);

    static Eigen::IOFormat commaFormat(3, Eigen::DontAlignCols," ",",");
    std::stringstream ss;
    ss << "ok;" << linear_speed.format(commaFormat);
    std::string s = ss.str();
    zmq::message_t response(s.data(), s.size()); 
    sock.send(response,zmq::send_flags::none);
}

void calcImpulseForce(UAVstate& state, Matrices& matrices,
    double COR, double mi_static, double mi_dynamic,
    Eigen::Vector3d collisionPoint, Eigen::Vector3d surfaceNormal)
{
    const std::lock_guard<std::mutex> lock(state.state_mtx);
    auto Y = state.getY();
    Eigen::Matrix3d R_nb = matrices.R_nb(Y);
    Eigen::Matrix3d R_bn = R_nb.inverse();
    Eigen::Matrix<double,6,6> T, T_inv;
    T.setZero();
    T_inv.setZero();
    T.block<3,3>(0,0) = R_bn;
    T.block<3,3>(3,3) = R_bn;
    T_inv.block<3,3>(0,0) = R_nb;
    T_inv.block<3,3>(3,3) = R_nb;      
    Eigen::Vector<double,6> X_g = T * state.getX();
    Eigen::Vector3d r = collisionPoint - Y.head<3>();
    Eigen::Vector3d vr = X_g.head<3>() + X_g.tail<3>().cross(r);
    double vn = vr.dot(surfaceNormal);
    if(vn >= 0.0)
    {
        return;
    }
    std::cout << "Energy before collision: " << X_g.transpose()*matrices.massMatrix*X_g << std::endl;
    double den_n = (matrices.invMassMatrix(0,0) 
        + ((matrices.invMassMatrix.block<3,3>(3,3)*r.cross(surfaceNormal)).cross(r)).dot(surfaceNormal));
    if(vn > -GENTLY_PUSH) vn = -GENTLY_PUSH;
    double jr = (-(1+COR)*vn)/den_n;
    Eigen::Vector<double,6> delta_n;
    delta_n << surfaceNormal, r.cross(surfaceNormal);
    X_g = X_g + jr*matrices.invMassMatrix*delta_n;

    Eigen::Vector3d vt = vr - (vr.dot(surfaceNormal))*surfaceNormal;
    if(vt.squaredNorm() > FRICTION_EPS)
    {
        Eigen::Vector3d tangent = vt.normalized();
        double js = mi_static*jr;
        double jd = mi_dynamic*jr;
        double den_t = (matrices.invMassMatrix(0,0) 
        + ((matrices.invMassMatrix.block<3,3>(3,3)*r.cross(tangent)).cross(r)).dot(tangent));
        double jf = -vt.norm()/den_t;
        if(jf > js) jf = jd;
        Eigen::Vector<double,6> delta_t;
        delta_t << tangent, r.cross(tangent);
        X_g = X_g + jf*matrices.invMassMatrix*delta_t;
    }
    std::cout << "Energy after collision: " << X_g.transpose()*matrices.massMatrix*X_g << std::endl;
    Vector<double,6> newX = T_inv * X_g;
    state.setX(newX);
}

bool isNormal(double factor)
{
    return factor >= 0.0 && factor <= 1.0;
}

void solidSurfColision(UAVstate& state, Matrices& matrices, std::string& msg_str, zmq::socket_t& sock)
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
        calcImpulseForce(state,matrices, COR, mi_static, mi_dynamic, collisionPoint, surfaceNormal);
        response.rebuild("ok",2);
    }
    sock.send(response,zmq::send_flags::none);
    return; 
}

void controlListenerJob(zmq::context_t* ctx, std::string address,UAVstate& state, Matrices& matricies)
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
            case 'd':
                shot(state,matricies,msg_str,controlInSock);
            break;
            case 'f':
                setForce(state,msg_str,controlInSock);
            break;
            case 'j':
                solidSurfColision(state,matricies,msg_str,controlInSock);
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