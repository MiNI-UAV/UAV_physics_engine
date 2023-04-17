#include <iostream>
#include <Eigen/Dense>
#include "constants.hpp"
#include "forces.hpp"
#include "RK4.hpp"
//#include "uav_params.hpp"

Eigen::Vector4d fun(double t , Eigen::Vector4d om)
{
  Forces f;
  return f.angularAcceleration(om);
}

int main()
{
    Matrices c;
    Forces f;
    std::cout << "Start!" << std::endl;
    Vector4d om = {0,0,0,0};

    for (int i = 0; i < 1000; i++)
    {
        om = RK4_step<4>(i,om,fun,0.001);
        std::cout << om(1) << "\n";
    }
    

    return 0;
}
