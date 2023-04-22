#include <iostream>
#include <zmq.h>
#include "simulation.hpp"

#include "uav_params.hpp"
#include "uav_state.hpp"

int main()
{
    //const char* path = "test.xml";
    //UAVparams params(path);
    UAVparams params;
    UAVstate state(params.noOfRotors); 
    VectorXd dem;
    dem.setOnes(4);
    dem = dem*3.22;
    // dem(0) = 3.0;
    // dem(1) = 3.5;
    // dem(2) = 3.0;
    // dem(3) = 3.5;
    //dem = dem*0.0;
    state.setDemandedOm(dem);

    Vector3d wind = {0.0,0.0,0.0};
    state.setWind(wind);
    
    std::cout << "Starting simulation!" <<std::endl;
    Simulation sim(params,state);
    sim.run();
}
