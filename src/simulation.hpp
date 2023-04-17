#pragma once

#include "uav_params.hpp"
#include "uav_state.hpp"
#include "forces.hpp"
#include "matrices.hpp"

class Simulation
{
    public:
        Simulation(UAVparams& params, UAVstate& state);
        void run();

    private:
        UAVparams& _params;
        UAVstate& _state;

        Forces forces;
        Matrices matrices;
        double real_time = 0.0;
        std::function<VectorXd(double,VectorXd)> RHS;
};
