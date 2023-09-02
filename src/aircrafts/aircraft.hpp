#pragma once

#include <Eigen/Dense>
#include <zmq.hpp>
#include "../uav_state.hpp"
#include "../uav_params.hpp"
#include "../matrices.hpp"


class Aircraft
{
public:
    Aircraft();
    virtual ~Aircraft() {}

    void update();
    void sendState(zmq::socket_t* socket);

    void calcImpulseForce(double COR, double mi_static, double mi_dynamic,
        Eigen::Vector3d collisionPoint, Eigen::Vector3d surfaceNormal);
    Eigen::Vector3d calcMomentumConservanceConservation(double m, double speed, Vector3d r);

    UAVstate state;

protected:

    Matrix<double,6,6> massMatrix;
    Matrix<double,6,6> invMassMatrix;

    void reduceMass(double delta_m);
    virtual Eigen::VectorXd RHS(double, Eigen::VectorXd);
};