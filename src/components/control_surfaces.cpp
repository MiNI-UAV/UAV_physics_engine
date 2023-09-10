#include "control_surfaces.hpp"

ControlSurfaces::ControlSurfaces():
    noOfSurfaces{0}, matrix{Eigen::Matrix<double,6,-1>()}, min{Eigen::VectorXd(0)}, max{Eigen::VectorXd(0)}, trim{Eigen::VectorXd(0)}
{
}

ControlSurfaces::ControlSurfaces(int noOfSurfaces, Eigen::Matrix<double, 6, -1> matrix,
    Eigen::VectorXd min, Eigen::VectorXd max, Eigen::VectorXd trim) :
        noOfSurfaces{noOfSurfaces}, matrix{matrix}, min{min}, max{max}, trim{trim}
{
    values = trim;
}

Eigen::Vector<double, 6> ControlSurfaces::getCofficients() const
{
    return matrix * values;
}

bool ControlSurfaces::setValues(Eigen::VectorXd new_values) 
{
    if(new_values.size() != noOfSurfaces)
    {
        return false;
    }
    values = max.cwiseMin(min.cwiseMax(new_values));
    return true;
}

void ControlSurfaces::restoreTrim() 
{
    values = trim;
}
