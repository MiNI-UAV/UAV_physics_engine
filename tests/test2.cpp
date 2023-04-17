#include <iostream>
#include <Eigen/Dense>
#include "constants.hpp"


int main()
{
  Matrices c;

  std::cout << c.massMatrix << std::endl << "==================" << std::endl;

  Eigen::Vector<double,6> vec;
  vec << 1,2,3,4,5,6;

  std::cout << c.gyroMatrix(vec) << std::endl;

}