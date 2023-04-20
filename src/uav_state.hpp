#pragma once
#include <Eigen/Dense>
#include <atomic>

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

        /* @brief Demanded angular velocities of uav motors */
        Eigen::VectorXd demandedAngularVelocity;


        int windBufSwitch = 0;
        Eigen::Vector3d windBuf[2];
        std::atomic<Eigen::Vector3d*> wind_ptr;


    public:
        UAVstate(int rotors);
        ~UAVstate();

        std::atomic<double> real_time;

        Eigen::Vector<double,6> getY();
        Eigen::Vector<double,6> getX();
        Eigen::VectorXd getOm();
        Eigen::VectorXd getDemandedOm();
        Eigen::Vector3d getWind();
        Eigen::VectorXd getState();

        void setDemandedOm(Eigen::VectorXd);
        void setWind(Eigen::Vector3d);

        UAVstate& operator=(Eigen::VectorXd& other);
        friend std::ostream& operator << ( std::ostream& outs, const UAVstate& state);

        static void setY(Eigen::VectorXd& state, Eigen::Vector<double,6> Y);
        static void setX(Eigen::VectorXd& state, Eigen::Vector<double,6> X);
        static void setOm(Eigen::VectorXd& state, Eigen::VectorXd Om);

        static Eigen::Vector<double,6> getY(const Eigen::VectorXd& state);
        static Eigen::Vector<double,6> getX(const Eigen::VectorXd& state);
        static Eigen::VectorXd getOm(const Eigen::VectorXd& state);
};


