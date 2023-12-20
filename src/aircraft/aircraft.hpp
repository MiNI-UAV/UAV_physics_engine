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

/// @brief central class in simulation
class Aircraft
{
public:
    /// @brief Default constructor
    Aircraft();

    /// @brief Virtual deconstructor in case of future use in devired class
    virtual ~Aircraft() {}


    /// @brief Simulation step
    void update();


    /// @brief Sends simulation state via publisher socket
    /// @param socket zmq socket to send simulation state
    void sendState(zmq::socket_t* socket);

    /// @brief Starts jet engine
    /// @param index index of engine to start
    /// @return return true if jet was started. False if jet is running or burnt out
    bool startJet(int index);

    /// @brief Restore trim values of surface angles
    void trim();

    /// @brief Set surface deflation
    /// @param angles vector of new surface angles
    /// @return true if angles were set. Returns false if length of vector is not equal to numbers of surfaces
    bool setSurface(Eigen::VectorXd angles);

    /// @brief Set angle of specified hinge in specified drive
    /// @param type type of drive: 'r' - rotor, 'j' - jet
    /// @param index index of drive
    /// @param hinge_index index of hinde
    /// @param value new angle value
    /// @return true, if angle was set. Returns false if specified drive or hinge wasn't found
    bool setHinge(char type, int index, int hinge_index, double value);

    /// @brief Calculate impact and result of collision with with solid surface. Results are applied to UAV state
    /// @param COR coefficient of restitution. e = 0 is perfect inelastic collision, e = 1 is perfect elastic collision.
    /// 0 < e < 1 is a real-world inelastic collision, in which some kinetic energy is dissipated.
    /// @param mi_static static friction coefficient
    /// @param mi_dynamic dynamic friction coefficient
    /// @param collisionPoint point of collision
    /// @param surfaceNormal surface normal vector 
    void calcImpulseForce(double COR, double mi_static, double mi_dynamic,
        Eigen::Vector3d collisionPoint, Eigen::Vector3d surfaceNormal);

    /// @brief Release cargo of specified index 
    /// @param index index of cargo
    /// @return Returns pair of result and velocity of released cargo in world frame. Result is number of cargo of given type that left on board.
    /// Result also informs about fails:
    /// - -1 - cooldown, next drop is not ready
    /// - -2 - out of cargos
    /// - -10 - index not found
    std::tuple<int, Eigen::Vector3d> dropCargo(int index);

    /// @brief Shoot ammo of specified index 
    /// @param index index of ammo
    /// @return Returns pair of result and velocity of released cargo in world frame. Result is number of ammo of given type that left on board.
    /// Result also informs about fails:
    /// - -1 - cooldown, next shoot is not ready
    /// - -2 - out of ammo
    /// - -10 - index not found
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

    AeroCoefficients aero;

    int noOfAmmo;
    std::unique_ptr<Ammo[]> ammo;
    
    int noOfCargo;
    std::unique_ptr<Cargo[]> cargo;

    /// @brief Reduces mass of aircraft of given value. Mass matrix is reduced proportionally - moments of inertia is scaled as well.
    /// @param delta_m mass reduction
    /// @param r lost mass position in local frame
    void reduceMass(double delta_m, Eigen::Vector3d r);

    /// @brief Calculateds result of releasing/launching object from aircraft
    /// @param m object mass
    /// @param speed object's initial velocity vector in body frame
    /// @param r offset of object
    /// @return initial linear velocity of object in world frame
    Eigen::Vector3d calcMomentumConservanceConservation(double m,
                                                        Eigen::Vector3d speed,
                                                        Eigen::Vector3d r);

    /// @brief Right hand side of main differential equation
    /// @param time time of simulation
    /// @param state state vector
    /// @return derivative of state
    virtual Eigen::VectorXd RHS(double, Eigen::VectorXd);

    std::unique_ptr<ODE> ode;
};