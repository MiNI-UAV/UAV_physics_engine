#include <iostream>
#include <Eigen/Dense>
#include "uav_params.hpp"
#include "uav_state.hpp"


int main()
{
    UAVparams params;
    UAVstate state(4);
    std::cout << state << std::endl;
    Eigen::Vector<double,16> test;
    test.setRandom();
    state = test;
    std::cout << state << std::endl;
    return 0;
}
