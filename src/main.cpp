#include <iostream>
#include "simulation.hpp"

#include "uav_params.hpp"
#include "uav_state.hpp"

int main()
{
    UAVparams params;
    UAVstate state(params.noOfRotors);
    VectorXd dem;
    dem.setOnes(4);
    dem = dem*3.22;
    state.setDemandedOm(dem);

    Simulation sim(params,state);
    sim.run();
}
