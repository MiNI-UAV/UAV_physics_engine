#pragma once

#include <zmq.hpp>
#include <memory>
#include "common.hpp"
#include "uav_state.hpp"
#include "../dynamic/forces.hpp"
#include "../dynamic/matrices.hpp"
#include "../aircraft/aircraft.hpp"
#include "atmosphere.hpp"

class Simulation
{
    public:
        /// @brief Default constructor
        Simulation();

        /// @brief Deconstructor
        ~Simulation();

        /// @brief Run simulation
        void run();

    private:
        zmq::context_t _ctx;
        zmq::socket_t stateOutSock;
        std::thread controlListener;

        Aircraft* aircraft;

        Atmosphere atmosphere;

        void sendIdle();
};
