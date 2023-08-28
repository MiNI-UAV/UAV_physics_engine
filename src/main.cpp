#include <iostream>
#include <cxxopts.hpp>
#include "simulation.hpp"
#include "uav_params.hpp"
#include "uav_state.hpp"
#include "common.hpp"


void parseArgs(int argc, char** argv, bool& instantRunFlag)
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
    instantRunFlag = result["instant-run"].as<bool>();
    if(result.count("config"))
    {
        UAVparams::getSingleton()->loadConfig(result["config"].as<std::string>());
    }
    if(result.count("name"))
    {
        UAVparams::getSingleton()->name = result["name"].as<std::string>();
    }
    std::cout << "Name: " << UAVparams::getSingleton()->name <<std::endl;
}

int main(int argc, char** argv)
{
    UAVparams params;
    bool instantRun = false;
    parseArgs(argc,argv,instantRun);
    UAVstate state(params.noOfRotors);
    if(instantRun) state.setStatus(Status::running);
    std::cout << "Starting simulation!" <<std::endl;
    Simulation sim(state);
    sim.run();
}
