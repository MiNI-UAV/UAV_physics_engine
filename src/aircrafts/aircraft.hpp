#pragma once

#include <Eigen/Dense>
#include <zmq.hpp>
#include <memory>
#include <mutex>
#include "../uav_state.hpp"
#include "../uav_params.hpp"
#include "../matrices.hpp"
#include "../components/components.hpp"

class Aircraft
{
public:
    Aircraft();
    virtual ~Aircraft() {}

    void update();
    void sendState(zmq::socket_t* socket);

    bool startJet(int index);

    void trim();
    bool setSurface(Eigen::VectorXd);

    bool setHinge(char type, int index, int hinge_index, double value);

    void calcImpulseForce(double COR, double mi_static, double mi_dynamic,
        Eigen::Vector3d collisionPoint, Eigen::Vector3d surfaceNormal);
    Eigen::Vector3d calcMomentumConservanceConservation(double m, double speed, Vector3d r);

    UAVstate state;

protected:

    Matrix<double,6,6> massMatrix;
    Matrix<double,6,6> invMassMatrix;

    std::mutex mtx;

    int noOfRotors;
    std::unique_ptr<Rotor[]> rotors;
    int noOfJets;
    std::unique_ptr<Jet[]> jets;
    ControlSurfaces surfaces;
    AeroCofficients aero;

    void reduceMass(double delta_m);
    virtual Eigen::VectorXd RHS(double, Eigen::VectorXd);
};