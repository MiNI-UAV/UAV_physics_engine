#pragma once

#include <zmq.hpp>
#include <memory>
#include "uav_params.hpp"
#include "uav_state.hpp"
#include "forces.hpp"
#include "matrices.hpp"
#include "aircrafts/aircraft.hpp"
#include "atmosphere.hpp"

class Simulation
{
    public:
        Simulation();
        ~Simulation();
        void run();

    private:
        zmq::context_t _ctx;
        zmq::socket_t stateOutSock;
        std::thread controlListener;

        Aircraft* aircraft;

        Atmosphere atmosphere;

        void sendIdle();
};
