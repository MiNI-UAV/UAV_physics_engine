#include <iostream>
#include <cxxopts.hpp>
#include "simulation/simulation.hpp"
#include "simulation/uav_state.hpp"
#include "dynamic/forces.hpp"
#include "common.hpp"
#include "params.hpp"

/// @brief Parse CL arguments
/// @param argc number of argument
/// @param argv argument array
/// @param params pointer to UAVparams instant that should be filled
/// @param p internal params reference
void parseArgs(int argc, char** argv, UAVparams* params, Params& p)
{
    cxxopts::Options options("uav", "Process representing movement of one UAV with rigid frame and constant propellers");
    options.add_options()
        ("c,config", "Path of config file", cxxopts::value<std::string>()->default_value("config.xml"))
        ("i,instant-run", "Instant run. Simulation starts immediately.", cxxopts::value<bool>()->default_value("false"))
        ("n,name", "Override name from config", cxxopts::value<std::string>())
        ("dt", "Step time of simulation in ms. Default: 1 ms", cxxopts::value<int>())
        ("o,ode", "ODE solver. Defaulf: RK4", cxxopts::value<std::string>())
        ("h,help", "Print usage");
    auto result = options.parse(argc, argv);
    if(result.count("help"))
    {
        std::cout << options.help() << std::endl;
        exit(0);
    }
    params->instantRun = result["instant-run"].as<bool>();
    if(result.count("config"))
    {
        params->loadConfig(result["config"].as<std::string>());
    }
    if(result.count("name"))
    {
        params->name = result["name"].as<std::string>();
    }
    std::cout << "Name: " << params->name <<std::endl;
    if(result.count("dt"))
    {
        p.STEP_TIME = result["dt"].as<int>()/1000.0;
        std::cout << "Step time changed to " << p.STEP_TIME << "s" << std::endl;
    }
    if(result.count("ode"))
    {
        p.ODE_METHOD = result["ode"].as<std::string>();
        std::cout << "ODE method changed to " << p.ODE_METHOD  << std::endl;
    }
}

int main(int argc, char** argv)
{
    UAVparams params;
    Params p{};
    parseArgs(argc,argv,&params,p);
    Logger::setLogDirectory(params.name);
    Forces::generateCharacteristics(params.surfaces, params.aero_coffs);
    std::cout << "Starting simulation!" <<std::endl;
    Simulation simulation;
    simulation.run();
}
