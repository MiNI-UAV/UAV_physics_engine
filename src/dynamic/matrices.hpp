#pragma once
#include <Eigen/Dense>
#include "common.hpp"

using namespace Eigen;

class Matrices
{
    public:
        /// @brief Constucts initial mass matrix, based on parameters
        /// @return mass matrix
        static Matrix<double,6,6> massMatrix();

        /// @brief Calculates gyroscopic matrix
        /// @param x velocity vector
        /// @return gyroscopic matrix
        static Matrix<double,6,6> gyroMatrix(Vector<double,6>  x);

        /// @brief Calculates transformation matrix from body to world frame for velocities
        /// @param y actual position vector
        /// @return transformation matrix
        static Matrix<double,6,6> TMatrix(Vector<double,6>  y);

        /// @brief Calculates rotation matrix from world to body frame
        /// @param y actual position & orietation vector (RPY)
        /// @return rotation matrix
        static Matrix<double,3,3> R_nb(const Vector<double,6>&  y);

        /// @brief Calculates rotation matrix from world to body frame
        /// @param y actual position & orietation vector (quaterions)
        /// @return rotation matrix
        static Matrix<double,3,3> R_nb(const Vector<double,7>&  y);

        /// @brief Calculates rotation matrix from wind to body frame
        /// @param alpha angle of attack
        /// @param beta angle of slide
        /// @return rotation matrix
        static Matrix<double,3,3> R_wind_b(double alpha, double beta);

        /// @brief Convert position and orientation vector from quaterions to RPY Euler angles
        /// @param y position & orientation vector
        /// @return position & orientation vector. Orientation is given in RPY
        static Vector<double,6> quaterionsToRPY(Vector<double,7>  y);

        /// @brief Convert position and orientation vector from RPY Euler angles to quaterions
        /// @param y position & orientation vector
        /// @return position & orientation vector. Orientation is given in quaterions
        static Vector<double,7> RPYtoQuaterion(Vector<double,6> y);

        /// @brief Calculates conjugation matrix for angular velocity
        /// @param x vector of velocites
        /// @return conjugation matrix
        static Matrix4d OM_conj(Vector<double,6>  x);
};
