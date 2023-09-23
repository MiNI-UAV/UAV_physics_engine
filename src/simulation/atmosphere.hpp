#pragma once
#include <Eigen/Dense>
#include <atomic>

struct AtmosphereInfo
{
    Eigen::Vector3d wind = Eigen::Vector3d(0.0,0.0,0.0);
    double air_temperature = 288.15;
    double air_pressure = 101300.0;
    double air_density = 1.224;
};

class Atmosphere
{
public:
    Atmosphere();
    ~Atmosphere();

    Eigen::Vector3d getWind();
    double getAirTemperature();
    double getAirPressure();
    double getAirDensity();
    void update(AtmosphereInfo info);

    static Atmosphere* getSingleton();

private:
    int atmosphereBufSwitch = 0;
    AtmosphereInfo atmosphereBuf[2];
    std::atomic<AtmosphereInfo*> atmosphere_ptr;

    static Atmosphere* singleton;
};
