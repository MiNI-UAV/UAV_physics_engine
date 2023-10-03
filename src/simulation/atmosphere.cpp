#include "atmosphere.hpp"
#include <iostream>

Atmosphere* Atmosphere::singleton = nullptr;

Atmosphere::Atmosphere(): logger("atmosphere.csv","time,temperature,pressure,density")
{
    atmosphere_ptr = atmosphereBuf + 1;
    if(singleton != nullptr)
    {
        std::cerr << "Only one instance of Atmosphere should exist";
        return;
    }
    singleton = this;
}

Atmosphere::~Atmosphere() 
{
    singleton = nullptr;
}

Eigen::Vector3d Atmosphere::getWind() { return atmosphere_ptr.load()->wind; }

double Atmosphere::getAirTemperature()
{
    return atmosphere_ptr.load()->air_temperature;
}

double Atmosphere::getAirPressure()
{
    return atmosphere_ptr.load()->air_pressure;
}

double Atmosphere::getAirDensity()
{
    return atmosphere_ptr.load()->air_density;
}

void Atmosphere::update(AtmosphereInfo info)
{
    atmosphereBuf[atmosphereBufSwitch] = info;
    atmosphere_ptr = atmosphereBuf + atmosphereBufSwitch;
    atmosphereBufSwitch = 1 - atmosphereBufSwitch;
    logger.log(0.0,{info.air_temperature,info.air_pressure, info.air_density});
}

Atmosphere *Atmosphere::getSingleton()
{
    return singleton;
}
