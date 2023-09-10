#include <Eigen/Dense>
#include <zmq.hpp>
#include <iostream>

#include "aircraft.hpp"
#include "../uav_state.hpp"
#include "../uav_params.hpp"
#include "../matrices.hpp"
#include "../forces.hpp"
#include "../defines.hpp"


void Aircraft::calcImpulseForce(double COR, double mi_static, double mi_dynamic,
    Eigen::Vector3d collisionPoint, Eigen::Vector3d surfaceNormal)
{
    const std::lock_guard<std::mutex> lock(state.state_mtx);
    auto Y = state.getY();
    Eigen::Matrix3d R_nb = Matrices::R_nb(Y);
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
    std::cout << "Energy before collision: " << X_g.transpose()*massMatrix*X_g << std::endl;
    double den_n = (invMassMatrix(0,0) 
        + ((invMassMatrix.block<3,3>(3,3)*r.cross(surfaceNormal)).cross(r)).dot(surfaceNormal));
    if(vn > -GENTLY_PUSH) vn = -GENTLY_PUSH;
    double jr = (-(1+COR)*vn)/den_n;
    Eigen::Vector<double,6> delta_n;
    delta_n << surfaceNormal, r.cross(surfaceNormal);
    X_g = X_g + jr*invMassMatrix*delta_n;

    Eigen::Vector3d vt = vr - (vr.dot(surfaceNormal))*surfaceNormal;
    if(vt.squaredNorm() > FRICTION_EPS)
    {
        Eigen::Vector3d tangent = vt.normalized();
        double js = mi_static*jr;
        double jd = mi_dynamic*jr;
        double den_t = (invMassMatrix(0,0) 
        + ((invMassMatrix.block<3,3>(3,3)*r.cross(tangent)).cross(r)).dot(tangent));
        double jf = -vt.norm()/den_t;
        if(jf > js) jf = jd;
        Eigen::Vector<double,6> delta_t;
        delta_t << tangent, r.cross(tangent);
        X_g = X_g + jf*invMassMatrix*delta_t;
    }
    std::cout << "Energy after collision: " << X_g.transpose()*massMatrix*X_g << std::endl;
    Vector<double,6> newX = T_inv * X_g;
    state.setX(newX);
}

Eigen::Vector3d Aircraft::calcMomentumConservanceConservation(double m, double speed, Vector3d r)
{
    const std::lock_guard<std::mutex> lock(state.state_mtx);
    Matrix3d R_nb = Matrices::R_nb(state.getY());
    Matrix3d R_bn = R_nb.inverse();
    Matrix<double,6,6> T, T_inv;
    T.setZero();
    T_inv.setZero();
    T.block<3,3>(0,0) = R_bn;
    T.block<3,3>(3,3) = R_bn;
    T_inv.block<3,3>(0,0) = R_nb;
    T_inv.block<3,3>(3,3) = R_nb;
    Vector<double,6> momentum = massMatrix * (T * state.getX());
    Vector3d obj_vel = state.getX().head<3>();
    obj_vel(0) += speed;
    Vector3d obj_linear_speed = R_bn * obj_vel;
    Vector3d obj_linear_momentum = m * obj_linear_speed;
    Vector<double,6> obj_momentum;
    r = R_bn * r;
    obj_momentum << obj_linear_momentum, r.cross(obj_linear_momentum);
    reduceMass(m);
    Vector<double,6> newX = T_inv * invMassMatrix * (momentum - obj_momentum);
    state.setX(newX);
    return obj_linear_speed;
}