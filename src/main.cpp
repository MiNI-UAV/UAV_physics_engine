#include <iostream>
#include <cxxopts.hpp>
#include "simulation/simulation.hpp"
#include "simulation/uav_state.hpp"
#include "common.hpp"


void parseArgs(int argc, char** argv, UAVparams* params)
{
    cxxopts::Options options("uav", "Process representing movement of one UAV with rigid frame and constant propellers");
    options.add_options()
        ("c,config", "Path of config file", cxxopts::value<std::string>()->default_value("config.xml"))
        ("i,instant-run", "Instant run. Simulation starts immediately.", cxxopts::value<bool>()->default_value("false"))
        ("n,name", "Override name from config", cxxopts::value<std::string>())
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
}

int main(int argc, char** argv)
{
    UAVparams params;
    parseArgs(argc,argv,&params);
    std::cout << "Starting simulation!" <<std::endl;
    Simulation simulation;
    simulation.run();
}
