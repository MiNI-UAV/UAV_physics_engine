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
        Simulation(UAVparams& params, UAVstate& state);
        ~Simulation();
        void run();

    private:
        UAVparams& _params;
        UAVstate& _state;
        zmq::context_t _ctx;
        zmq::socket_t stateOutSock;
        const double step_time = 0.001;
        std::thread controlListener;

        Forces forces;
        Matrices matrices;
        std::function<VectorXd(double,VectorXd)> RHS;

        void sendState();
};
