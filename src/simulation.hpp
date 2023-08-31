#pragma once

#include <zmq.hpp>
#include "uav_params.hpp"
#include "uav_state.hpp"
#include "forces.hpp"
#include "matrices.hpp"
#include <thread>

class Simulation
{
    public:
        Simulation(UAVstate& state);
        ~Simulation();
        void run();

    private:
        UAVstate& _state;
        zmq::context_t _ctx;
        zmq::socket_t stateOutSock;
        const double step_time = 0.001;
        std::thread controlListener;

        Matrices matrices;
        std::function<VectorXd(double,VectorXd)> RHS;

        void sendState();
        void sendIdle();
};
