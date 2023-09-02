#pragma once
#include <Eigen/Dense>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include "common.hpp"
#include "defines.hpp"

struct UAVstate
{
    private:
        constexpr static int validityOfForce = 10;
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

        int windBufSwitch = 0;
        Eigen::Vector3d windBuf[2];
        std::atomic<Eigen::Vector3d*> wind_ptr;

        int forceBufSwitch = 0;
        Eigen::Vector<double,6> forceBuf[2];
        std::atomic<Eigen::Vector<double,6>*> force_ptr;
        std::atomic_int forceValidityCounter;

    public:
        UAVstate();
        ~UAVstate();

        std::atomic<double> real_time;

        #ifdef USE_QUATERIONS
            Eigen::Vector<double,7> getY();
            static void setY(Eigen::VectorXd& state, Eigen::Vector<double,7> Y);
            static Eigen::Vector<double,7> getY(const Eigen::VectorXd& state);
        #else
            Eigen::Vector<double,6> getY();
            static void setY(Eigen::VectorXd& state, Eigen::Vector<double,6> Y);
            static Eigen::Vector<double,6> getY(const Eigen::VectorXd& state);
        #endif


        Eigen::Vector<double,6> getX();
        Eigen::VectorXd getOm();
        Eigen::VectorXd getDemandedOm();
        Eigen::Vector3d getWind();
        Eigen::Vector<double,6> getOuterForce();
        Eigen::VectorXd getState();
        inline int getNoOfRotors(){return noOfRotors;}
        inline Eigen::Vector<double,6> getAcceleration() {return acceleration;}
        std::mutex state_mtx;

        void setX(Eigen::Vector<double,6>);
        void setDemandedOm(Eigen::VectorXd);
        void setWind(Eigen::Vector3d);
        void setForce(Eigen::Vector3d force, Eigen::Vector3d torque = Eigen::Vector3d(0.0,0.0,0.0));
        void setAcceleration(Eigen::Vector<double,6>);

        UAVstate& operator=(Eigen::VectorXd& other);
        friend std::ostream& operator << ( std::ostream& outs, const UAVstate& state);

 
        static void setX(Eigen::VectorXd& state, Eigen::Vector<double,6> X);
        static void setOm(Eigen::VectorXd& state, Eigen::VectorXd Om);

        static Eigen::Vector<double,6> getX(const Eigen::VectorXd& state);
        static Eigen::VectorXd getOm(const Eigen::VectorXd& state);
        
        Status status;
        std::condition_variable status_cv;
        inline void setStatus(Status newStatus) {status = newStatus; status_cv.notify_all();}
};


