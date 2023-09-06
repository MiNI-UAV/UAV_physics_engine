#include "hinge.hpp"

Hinge::Hinge(Eigen::Vector3d axis, double max, double min, double trim):
    axis{axis}, max{max}, min{min}, value{trim}
{
    updateValue(value);
}

Hinge::Hinge(const Hinge &old)
{
    this->axis = old.axis;
    this->min = old.min;
    this->max = old.max;
    updateValue(old.value);
}

Hinge &Hinge::operator=(const Hinge &old)
{
    this->axis = old.axis;
    this->min = old.min;
    this->max = old.max;
    updateValue(old.value);
    return *this;
}

void Hinge::updateValue(double newValue) 
{
    static const Eigen::Matrix3d crossProductMatrix = axis.asSkewSymmetric().toDenseMatrix();
    static const Eigen::Matrix3d outerProduct = axis * axis.transpose();
    std::scoped_lock lck(mtx);
    value = newValue;
    rot = cos(value) * Eigen::Matrix3d::Identity() 
        + sin(value) * crossProductMatrix
        + (1 - cos(value)) * outerProduct;
}

const Eigen::Matrix3d Hinge::getRot()
{
   std::scoped_lock lck(mtx);
   return rot;
}