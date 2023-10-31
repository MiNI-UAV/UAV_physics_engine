#pragma once
#include <Eigen/Dense>
#include <atomic>
#include "common.hpp"

/// @brief DTO containing atmosphere information
struct AtmosphereInfo
{
    Eigen::Vector3d wind = Eigen::Vector3d(0.0,0.0,0.0);
    double air_temperature = 288.15;
    double air_pressure = 101300.0;
    double air_density = 1.224;
};

/// @brief Representation of atmosphere where aircrafts fly
class Atmosphere
{
public:
    /// @brief Default constructor
    Atmosphere();
    /// @brief Default deconstructor
    ~Atmosphere();

    /// @brief Returns wind speed vector in world frame
    /// @return wind speed vector m/s
    Eigen::Vector3d getWind();

    /// @brief Returns air temperature
    /// @return air temperature K
    double getAirTemperature();

    /// @brief Returns air pressure
    /// @return air pressure Pa
    double getAirPressure();

    /// @brief Returns air density
    /// @return air density kg/m3
    double getAirDensity();

    /// @brief Update atmosphere status
    /// @param info dto with new atmosphere info
    void update(AtmosphereInfo info);

    /// @brief Returns pointer to singleton of atmosphere
    /// @return pointer to Atmosphere instance. Nullptr if singleton not exists
    static Atmosphere* getSingleton();

private:
    int atmosphereBufSwitch = 0;
    AtmosphereInfo atmosphereBuf[2];
    std::atomic<AtmosphereInfo*> atmosphere_ptr;
    Logger logger;

    static Atmosphere* singleton;
};
