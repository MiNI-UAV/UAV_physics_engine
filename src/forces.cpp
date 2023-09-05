#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include "uav_params.hpp"
#include "forces.hpp"
#include "matrices.hpp"
#include "defines.hpp"
#include "drives/drive.hpp"


using namespace Eigen;

Vector<double,6> Forces::gravity_loads(const Matrix3d& r_nb)
{
    UAVparams* params = UAVparams::getSingleton();
    Vector<double,6> Fg;
    Fg.setZero();
    Fg.head<3>() = r_nb * Eigen::Vector3d(0.0,0.0,(params->m*GRAVITY_CONST));
    return Fg;
}

Vector<double,6> Forces::rotor_lift_loads(int noOfRotors, Rotor* rotors, VectorXd rotorAngularVelocity)
{
    double rho = getRho();
    Vector3d Fr = {0.0, 0.0, 0.0};
    Vector3d Mr = {0.0, 0.0, 0.0};

    for (int i = 0; i < noOfRotors; i++)
    {
        Rotor& rotor = rotors[i];
        double om2 = std::pow(rotorAngularVelocity(i),2);
        auto axis = rotor.axis;
        for (int j = 0; j < rotor.noOfHinges; j++)
        {
            axis = rotor.hinges[j].getRot()*axis;
        }
        auto Fi = axis*(rho*rotor.forceCoff*om2);
        Fr += Fi;
        Mr += axis*(-(rotor.direction*rho*rotor.torqueCoff*om2));
        Mr += rotor.position.cross(Fi);
    }
    Vector<double,6> res;
    res << Fr, Mr;
    return res; 
}

Vector<double, 6> Forces::jet_lift_loads(int noOfJets, Jet *jets, double time)
{
    Vector3d Fr = {0.0, 0.0, 0.0};
    Vector3d Mr = {0.0, 0.0, 0.0};

    for (int i = 0; i < noOfJets; i++)
    {
        Jet& jet = jets[i];
        double thrust = jet.getThrust(time);
        if(thrust == 0.0) continue;
        auto axis = jet.axis;
        for (int j = 0; j < jet.noOfHinges; j++)
        {
            axis = jet.hinges[j].getRot()*axis;
        }
        auto Fi = axis*thrust;
        Fr += Fi;
        Mr += jet.position.cross(Fi);
    }
    Vector<double,6> res;
    res << Fr, Mr;
    return res;
}

double Forces::dynamic_pressure([[maybe_unused]]double height, double Vtot)
{
    //TODO: More advanced model
    return 0.5*getRho()*Vtot*Vtot;
}

double Forces::getRho()
{
    //TODO: More advanced model
    return DEFAULT_RHO;
}

Vector<double, 6> Forces::aerodynamic_loads(const Matrix3d& r_nb, const Vector<double, 6> &x, Vector3d wind_global)
{
    static const double Ci[6] = {UAVparams::getSingleton()->Ci[0],UAVparams::getSingleton()->Ci[1],UAVparams::getSingleton()->Ci[2],
        UAVparams::getSingleton()->Ci[3],UAVparams::getSingleton()->Ci[4],UAVparams::getSingleton()->Ci[5]}; //TODO: fix it...
    static const double S = UAVparams::getSingleton()->S;
    static const double d = UAVparams::getSingleton()->d;

    Vector<double, 6> Fa(Ci);
    Vector3d wind = r_nb*wind_global;
    Vector3d velocity = x.segment(0,3);
    Vector3d diff = velocity-wind;
    double Vtot = diff.norm();
    if(Vtot == 0.0)
    {
        Fa.setZero();
        return Fa;
    }
    double alpha = atan2(diff(2),diff(0));
    double beta = asin(diff(1)/Vtot);
    Fa(0) *= (cos(alpha)*cos(beta));
    Fa(1) *= sin(beta);
    Fa(2) *= (sin(alpha)*cos(beta));
    Fa.segment(3,3) *= d;
    Fa *= -(dynamic_pressure(-x(2),Vtot)*S);
    return Fa;
}

VectorXd Forces::angularAcceleration(VectorXd demandedAngularVelocity, VectorXd rotorAngularVelocity)
{
    static const VectorXd rotorTimeConstants = UAVparams::getSingleton()->getRotorTimeContants();
    VectorXd res;
    res = (demandedAngularVelocity - rotorAngularVelocity);
    return res.cwiseQuotient(rotorTimeConstants);
}