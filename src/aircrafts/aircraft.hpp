#pragma once

#include <Eigen/Dense>
#include <zmq.hpp>
#include "../uav_state.hpp"
#include "../uav_params.hpp"
#include "../matrices.hpp"


class Aircraft
{
public:
    void update();
    void sendState(zmq::socket_t socket);

protected:
    UAVstate _state;
    Matrix<double,6,6> massMatrix;
    Matrix<double,6,6> invMassMatrix;

    void updateMass();
    virtual Eigen::VectorXd RHS(double, Eigen::VectorXd);
};