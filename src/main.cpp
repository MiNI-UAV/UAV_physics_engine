#include <iostream>
#include <zmq.h>
#include "simulation.hpp"

#include "uav_params.hpp"
#include "uav_state.hpp"

int main()
{
    UAVparams params;
    UAVstate state(params.noOfRotors);
    zmq::context_t ctx;
    
    
    VectorXd dem;
    dem.setOnes(4);
    dem = dem*3.22;
    //dem = dem*0.0;
    state.setDemandedOm(dem);

    Simulation sim(params,state);
    sim.run();
}
