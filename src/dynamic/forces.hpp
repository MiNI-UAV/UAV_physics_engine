#pragma once
#include <Eigen/Dense>
#include "common.hpp"
#include "matrices.hpp"

using namespace Eigen;

class Forces {
public:
  static Vector<double, 6> gravity_loads(const Matrix3d &r_nb);
  static Vector<double, 6> rotor_lift_loads(int noOfRotors, Rotor *rotors,
                                            VectorXd rotorAngularVelocity);
  static Vector<double, 6> jet_lift_loads(int noOfJets, Jet *jets, double time);
  static Vector<double, 6> aerodynamic_loads(const Vector<double, 6> &x,
                                             Vector3d wind_body,
                                             const ControlSurfaces &surface,
                                             const AeroCofficients &aero,
                                             double height);
  static VectorXd angularAcceleration(VectorXd demandedAngularVelocity,
                                      VectorXd rotorAngularVelocity);

private:
  static double dynamic_pressure(double height, double Vtot);
  static double getRho();
};
