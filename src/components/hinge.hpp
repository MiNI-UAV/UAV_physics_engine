#pragma once
#include <Eigen/Dense>
#include <mutex>
#include <memory>

class Hinge
{
public:
    Hinge() = default;
    Hinge(Eigen::Vector3d axis, double max, double min, double trim);
    Hinge(const Hinge& old);
    Hinge& operator=(const Hinge& old);

    void updateValue(double newValue);
    const Eigen::Matrix3d getRot();

private:
    Eigen::Vector3d axis;
    double max;
    double min;

    std::mutex mtx;
    double value;
    Eigen::Matrix3d rot;
};