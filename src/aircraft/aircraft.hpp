#pragma once

#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <zmq.hpp>
#include <iostream>
#include "common.hpp"
#include "../defines.hpp"
#include "../dynamic/forces.hpp"
#include "../dynamic/matrices.hpp"
#include "../simulation/atmosphere.hpp"
#include "../simulation/uav_state.hpp"

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

    std::tuple<int, Eigen::Vector3d> dropCargo(int index);
    std::tuple<int, Eigen::Vector3d> shootAmmo(int index);

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

    int noOfAmmo;
    std::unique_ptr<Ammo[]> ammo;
    
    int noOfCargo;
    std::unique_ptr<Cargo[]> cargo;

    void reduceMass(double delta_m);
    Eigen::Vector3d calcMomentumConservanceConservation(double m,
                                                        Eigen::Vector3d speed,
                                                        Eigen::Vector3d r);

    virtual Eigen::VectorXd RHS(double, Eigen::VectorXd);
};