#include <Eigen/Dense>
#include <cmath>
#include "matrices.hpp"
#include "uav_params.hpp"

using namespace Eigen;
using namespace std;

Matrices::Matrices(UAVparams& params): params{params}
{   
    updateMatrices();
}


void Matrices::updateMatrices()
{
    //mass matrix
    massMatrix.setZero();
    massMatrix(0,0) = params.m;
    massMatrix(1,1) = params.m;
    massMatrix(2,2) = params.m;
    massMatrix(3,3) = params.Ix;
    massMatrix(4,4) = params.Iy;
    massMatrix(5,5) = params.Iz;
    massMatrix(3,4) = -params.Ixy;
    massMatrix(4,3) = -params.Ixy;
    massMatrix(3,5) = -params.Ixz;
    massMatrix(5,3) = -params.Ixz;
    massMatrix(4,5) = -params.Iyz;
    massMatrix(5,4) = -params.Iyz;

    invMassMatrix = massMatrix.inverse();
}

void Matrices::reduceMass(double mass_delta) {
    params.m -= mass_delta;
    updateMatrices();
}

Matrix<double,6,6> Matrices::gyroMatrix(Vector<double,6> x)
{
    Matrix<double,6,6> gyro;
    gyro.setZero();
    gyro(1,0) =  x(5);
    gyro(2,0) = -x(4);
    gyro(4,0) =  x(2);
    gyro(5,0) = -x(1);
    gyro(0,1) = -x(5);
    gyro(2,1) =  x(3);
    gyro(3,1) = -x(2);
    gyro(5,1) =  x(0);
    gyro(0,2) =  x(4);
    gyro(1,2) = -x(3);
    gyro(3,2) =  x(1);
    gyro(4,2) = -x(0);
    gyro(4,3) =  x(5);
    gyro(5,3) = -x(4);
    gyro(3,4) = -x(5);
    gyro(5,4) =  x(3);
    gyro(3,5) =  x(4);
    gyro(4,5) = -x(3);
    return gyro;
}

Matrix<double,6,6> Matrices::TMatrix(Vector<double,6>  y)
{
    Matrix3d Tv, Tom;
    Matrix<double,6,6> res;
    res.setZero();
    double fi = y(3);
    double theta = y(4);
    double psi = y(5);

    Tv  << cos(theta)*cos(psi), sin(fi)*sin(theta)*cos(psi) - cos(fi)*sin(psi), cos(fi)*sin(theta)*cos(psi) + sin(fi)*sin(psi),
           cos(theta)*sin(psi), sin(fi)*sin(theta)*sin(psi) + cos(fi)*cos(psi), cos(fi)*sin(theta)*sin(psi) - sin(fi)*cos(psi),
           -sin(theta)        , sin(fi)*cos(theta)                            , cos(fi)*cos(theta);
    
    Tom << 1, sin(fi)*tan(theta), cos(fi)*tan(theta),
           0, cos(fi)           , -sin(fi),
           0, sin(fi)/cos(theta), cos(fi)/cos(theta);
    res.block<3,3>(0,0) = Tv;
    res.block<3,3>(3,3) = Tom;
    return res;
}

Matrix<double, 3, 3> Matrices::R_nb(const Vector<double,6>&  y)
{
    double fi = y(3);
    double theta = y(4);
    double psi = y(5);
    Matrix<double, 3, 3> r_nb;
    r_nb << cos(theta)*cos(psi),                            cos(theta)*sin(psi),                           -sin(theta),
            sin(fi)*sin(theta)*cos(psi) - cos(fi)*sin(psi), sin(fi)*sin(theta)*sin(psi) + cos(fi)*cos(psi), sin(fi)*cos(theta),
            cos(fi)*sin(theta)*cos(psi) + sin(fi)*sin(psi), cos(fi)*sin(theta)*sin(psi) - sin(fi)*cos(psi), cos(fi)*cos(theta);
    return r_nb;
}
