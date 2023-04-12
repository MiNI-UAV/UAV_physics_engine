#include <iostream>
#include "constants.hpp"
#include "forces.hpp"

int main()
{
    Constants c;
    Forces f;
    std::cout << "Start!" << std::endl;
    double om[] = {11,11,10,10};
    std::cout << f.lift_forces(om) << std::endl << std::endl;

    Vector<double,6> y;
    y.setZero();
    std::cout << c.TMatrix(y) << std::endl;
    return 0;
}
