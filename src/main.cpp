#include <iostream>
#include <cxxopts.hpp>
#include "simulation.hpp"
#include "uav_params.hpp"
#include "uav_state.hpp"
#include "status.hpp"


UAVparams parseArgs(int argc, char** argv, bool& instantRunFlag)
{
    UAVparams params;
    cxxopts::Options options("uav", "Process representing movement of one UAV with rigid frame and constant propellers");
    options.add_options()
        ("c,config", "Path of config file", cxxopts::value<std::string>()->default_value("config.xml"))
        ("i,instant-run", "Instant run. Simulation starts immediately.", cxxopts::value<bool>()->default_value("false"))
        ("n,name", "Override name from config", cxxopts::value<std::string>()->default_value(""))
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
        params = UAVparams(result["config"].as<std::string>().c_str());
    }
    if(result.count("name"))
    {
        std::string newName = result["name"].as<std::string>();
        params.setName(newName.c_str(),newName.length());
    }
    std::cout << "Name: " << params.name <<std::endl;
    return params;
}

int main(int argc, char** argv)
{
    bool instantRun = false;
    UAVparams params = parseArgs(argc,argv,instantRun);
    UAVstate state(params.noOfRotors);
    if(instantRun) state.setStatus(Status::running);
    std::cout << "Starting simulation!" <<std::endl;
    Simulation sim(params,state);
    sim.run();
}
