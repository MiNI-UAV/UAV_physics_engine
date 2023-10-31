#pragma once
#include <Eigen/Dense>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include "common.hpp"
#include "../defines.hpp"

struct UAVstate
{
public:
    /// @brief Default constructor
    UAVstate();
    /// @brief Deconstructor
    ~UAVstate();

    /// @brief simulation time
    std::atomic<double> real_time;

#ifdef USE_QUATERIONS
    /// @brief Returns position vector Y from state (quaterions)
    /// @return position vector Y 
    Eigen::Vector<double,7> getY();

    /// @brief Sets position vector Y in given state (quaterions)
    /// @param state state that should be updated
    /// @param Y new position vector
    static void setY(Eigen::VectorXd& state, Eigen::Vector<double,7> Y);

    /// @brief Returns position vector Y from given state (quaterions)
    /// @param state source state
    /// @return position vector Y 
    static Eigen::Vector<double,7> getY(const Eigen::VectorXd& state);
#else
    /// @brief Returns position vector Y from state (RPY)
    /// @return position vector Y 
    Eigen::Vector<double,6> getY();

    /// @brief Sets position vector Y in given state (RPY)
    /// @param state state that should be updated
    /// @param Y new position vector
    static void setY(Eigen::VectorXd& state, Eigen::Vector<double,6> Y);

    /// @brief Returns position vector Y from given state (RPY)
    /// @param state source state
    /// @return position vector Y 
    static Eigen::Vector<double,6> getY(const Eigen::VectorXd& state);
#endif


    /// @brief Returns velocity vector X
    /// @return velocity vector X
    Eigen::Vector<double,6> getX();

    /// @brief Returns rotor's angular velocities vector
    /// @return rotor's angular velocities vector
    Eigen::VectorXd getOm();

    /// @brief Returns rotor's demanded angular velocities vector
    /// @return rotor's demanded angular velocities vector rad/s
    Eigen::VectorXd getDemandedOm();

    /// @brief Returns outer force applied to aircraft
    /// @return outer force N
    Eigen::Vector<double,6> getOuterForce();

    /// @brief Returns raw state vector
    /// @return state vector
    Eigen::VectorXd getState();

    /// @brief Returns number of rotors
    /// @return number of rotors
    inline int getNoOfRotors(){return noOfRotors;}

    /// @brief Returns aircraft acceleration
    /// @return acceleraton vector
    inline Eigen::Vector<double,6> getAcceleration() {return acceleration;}

    /// @brief state mutex
    std::mutex state_mtx;


    /// @brief Set velocity vector X
    /// @param newX new velocity vector X
    void setX(Eigen::Vector<double,6> newX);

    /// @brief Set demanded angular velocity vector
    /// @param newOm new demanded angular velocity vector
    void setDemandedOm(Eigen::VectorXd newOm);
    
    /// @brief Set outer load
    /// @param force applied force
    /// @param torque applied torque
    void setForce(Eigen::Vector3d force, Eigen::Vector3d torque = Eigen::Vector3d(0.0,0.0,0.0));


    /// @brief Set acceleration vector
    /// @param newAccel new acceleration vector
    void setAcceleration(Eigen::Vector<double,6> newAccel);

    /// @brief Assigns given raw state vector to state
    UAVstate& operator=(Eigen::VectorXd& other);

    /// @brief Serializes state to stream
    friend std::ostream& operator << ( std::ostream& outs, const UAVstate& state);


    /// @brief Set velocity vector X in given state
    /// @param state state that should be updated
    /// @param X new velocity vector X
    static void setX(Eigen::VectorXd& state, Eigen::Vector<double,6> X);

    /// @brief Set angular velocity vector in given state
    /// @param state state that should be updated
    /// @param Om new angular velocity vector
    static void setOm(Eigen::VectorXd& state, Eigen::VectorXd Om);

    /// @brief Returns velocity vector X from given state
    /// @param state source state
    /// @return velocity vector X
    static Eigen::Vector<double,6> getX(const Eigen::VectorXd& state);

    /// @brief Return angular velocity vector from given state
    /// @param state source state
    /// @return angular velocity vector
    static Eigen::VectorXd getOm(const Eigen::VectorXd& state);
    
    /// @brief Timed loop status
    Status status;
    std::condition_variable status_cv;

    /// @brief Sets new timed loop status
    /// @param newStatus new status
    inline void setStatus(Status newStatus) {status = newStatus; status_cv.notify_all();}

private:
    #ifdef USE_QUATERIONS
    constexpr static int omOffset = 13;
    #else
    constexpr static int omOffset = 12;
    #endif
    
    /* @brief Position and attitude of the quadrotor 
    in inerial coordinate system */
    #ifdef USE_QUATERIONS
        Eigen::Vector<double,7> y;
    #else
        Eigen::Vector<double,6> y;
    #endif

    /* @brief Linear and angular velocity in local coordinate system */
    Eigen::Vector<double,6> x;

    /* @brief Linear and angular acceleration in local coordinate system */
    Eigen::Vector<double,6> acceleration;

    /* @brief Number of rotors */
    int noOfRotors;

    /* @brief Angular velocities of uav motors */
    Eigen::VectorXd rotorAngularVelocities;

    int demandedBufSwitch = 0;
    Eigen::VectorXd demandedAngularBuf[2];
    std::atomic<Eigen::VectorXd*> demanded_ptr;

    int forceBufSwitch = 0;
    Eigen::Vector<double,6> forceBuf[2];
    std::atomic<Eigen::Vector<double,6>*> force_ptr;
    std::atomic_int forceValidityCounter;
};
