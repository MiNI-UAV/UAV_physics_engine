#include <iostream>
#include <cxxopts.hpp>
#include "simulation.hpp"
#include "uav_params.hpp"
#include "uav_state.hpp"


UAVparams parseArgs(int argc, char** argv)
{
    cxxopts::Options options("uav", "Process representing movement of one UAV with rigid frame and constant propellers");
    options.add_options()
        ("c,config", "Path of config file", cxxopts::value<std::string>()->default_value("config.xml"))
        ("h,help", "Print usage");
    auto result = options.parse(argc, argv);
    if(result.count("help"))
    {
        std::cout << options.help() << std::endl;
        exit(0);
    }
    
    if(result.count("config"))
    {
        return UAVparams(result["config"].as<std::string>().c_str());
    }
    return UAVparams();
}

int main(int argc, char** argv)
{
    UAVparams params = parseArgs(argc,argv);
    UAVstate state(params.noOfRotors);
    std::cout << "Starting simulation!" <<std::endl;
    Simulation sim(params,state);
    sim.run();
}
