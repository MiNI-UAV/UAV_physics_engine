#pragma once

#include <Eigen/Dense>

class Aircraft;

class ControlSurfaces
{
public:
    ControlSurfaces();
    ControlSurfaces(int noOfSurfaces, Eigen::Matrix<double,6,-1> matrix,
        Eigen::VectorXd min, Eigen::VectorXd max, Eigen::VectorXd trim);

    Eigen::Vector<double,6> getCofficients() const;
    bool setValues(Eigen::VectorXd new_values);
    void restoreTrim();
    int getNoOfSurface() const {return noOfSurfaces;}

private:
    int noOfSurfaces;
    Eigen::Matrix<double,6,-1> matrix;
    Eigen::VectorXd min;
    Eigen::VectorXd max;
    Eigen::VectorXd trim;
    Eigen::VectorXd values;
};