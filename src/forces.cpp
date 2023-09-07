#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <numbers>
#include "uav_params.hpp"
#include "forces.hpp"
#include "matrices.hpp"
#include "defines.hpp"
#include "components/components.hpp"


using namespace Eigen;

Vector<double,6> Forces::gravity_loads(const Matrix3d& r_nb)
{
    const UAVparams* params = UAVparams::getSingleton();
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

Vector<double, 6> Forces::aerodynamic_loads(const Vector<double, 6> &x, Vector3d wind_body,
    const ControlSurfaces &surface, const AeroCofficients &aero, double height)
{   
    //C0
    Vector<double, 6> C = aero.C0;
    Vector3d velocity = x.segment(0,3);
    Vector3d diff = velocity-wind_body;
    double Vtot = diff.norm();
    if(Vtot <= DOUBLE_EPS)
    {
        return Vector<double, 6>::Zero();
    }
    double alpha = atan2(diff(2),diff(0));
    double beta = asin(diff(1)/Vtot);
    double pd = dynamic_pressure(height,Vtot);
    auto r_wb = Matrices::R_wind_b(alpha,beta);

    //Cpqr
    C += (1.0/(2.0*Vtot))*(aero.Cpqr*x.tail<3>());

    //Cab
    C += aero.Cab * Vector4d(alpha,beta,alpha*alpha,beta*beta);

    //Cab
    C += aero.Cab * Vector4d(alpha,beta,alpha*alpha,beta*beta);

    //Control surface
    if(surface.getNoOfSurface() > 0)
    {
        C += surface.getCofficients();
    }

    //eAR
    if(aero.eAR > DOUBLE_EPS)
    {
        C(0) += (C(2)*C(2))/(std::numbers::pi * aero.eAR);
    }

    Vector<double, 6> Fa = Vector<double, 6>::Zero();
    Fa.head<3>() = pd*aero.S*(r_wb*C.head<3>());
    Fa.tail<3>() = pd*aero.S*aero.d*(r_wb*C.tail<3>());
    return Fa;
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

VectorXd Forces::angularAcceleration(VectorXd demandedAngularVelocity, VectorXd rotorAngularVelocity)
{
    static const VectorXd rotorTimeConstants = UAVparams::getSingleton()->getRotorTimeContants();
    VectorXd res;
    res = (demandedAngularVelocity - rotorAngularVelocity);
    return res.cwiseQuotient(rotorTimeConstants);
}