#pragma once
#include <Eigen/Dense>

struct UAVstate
{
    private:
        /* @brief Position and attitude of the quadrotor 
        in inerial coordinate system */
        Eigen::Vector<double,6> y;

        /* @brief Linear and angular velocity in local coordinate system */
        Eigen::Vector<double,6> x;

        /* @brief Number of rotors */
        int noOfRotors;

        /* @brief Angular velocities of uav motors */
        Eigen::VectorXd rotorAngularVelocities;


    public:
        UAVstate(int rotors);
        ~UAVstate();
        UAVstate& operator=(Eigen::VectorXd& other);
        friend std::ostream& operator << ( std::ostream& outs, const UAVstate& state);
};

UAVstate::UAVstate(int rotors): noOfRotors{rotors}
{
    y.setZero();
    x.setZero();
    rotorAngularVelocities.setZero(noOfRotors);
}

UAVstate::~UAVstate()
{

}

UAVstate& UAVstate::operator=(Eigen::VectorXd& other)
{
    y = other.segment(0,6);
    x = other.segment(6,6);
    rotorAngularVelocities = other.segment(12,noOfRotors);
    return *this;
}

std::ostream& operator << ( std::ostream& outs, const UAVstate& state)
{
  return outs << "y:\n" << state.y.transpose() << "\nx:\n" << state.x.transpose() << "\nom:\n" <<state.rotorAngularVelocities.transpose() << std::endl;
}
