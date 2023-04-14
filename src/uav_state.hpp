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

        Eigen::Vector<double,6> getY();
        Eigen::Vector<double,6> getX();
        Eigen::VectorXd getOm();

        UAVstate& operator=(Eigen::VectorXd& other);
        friend std::ostream& operator << ( std::ostream& outs, const UAVstate& state);
};
