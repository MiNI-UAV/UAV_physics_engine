#include "aircraft.hpp"
#include "../params.hpp"

Aircraft::Aircraft()
{
    const UAVparams* params = UAVparams::getSingleton();

    noOfRotors = params->noOfRotors;
    noOfJets = params->noOfJets;
    noOfAmmo = params->noOfAmmo;
    noOfCargo = params->noOfCargo;

    rotors = std::make_unique<Rotor[]>(noOfRotors); 
    jets = std::make_unique<Jet[]>(noOfJets);
    ammo = std::make_unique<Ammo[]>(noOfAmmo); 
    cargo = std::make_unique<Cargo[]>(noOfCargo);

    std::copy(params->rotors.get(), params->rotors.get() + noOfRotors, rotors.get());
    std::copy(params->jets.get(), params->jets.get() + noOfJets, jets.get());
    std::copy(params->ammo.get(), params->ammo.get() + noOfAmmo, ammo.get());
    std::copy(params->cargo.get(), params->cargo.get() + noOfCargo, cargo.get());

    surfaces = params->surfaces;
    aero = params->aero_coffs;

    massMatrix = Matrices::massMatrix();
    invMassMatrix = massMatrix.inverse();

    ode = ODE::factory(ODE::fromString(Params::getSingleton()->ODE_METHOD));
}

void clampOrientationIfNessessery([[maybe_unused]] Eigen::VectorXd& state)
{
#ifndef USE_QUATERIONS
    for (size_t i = 3; i < 6; i++)
    {
        double x = fmod(state(i) + std::numbers::pi,2*std::numbers::pi);
        if (x < 0)
            x += 2*std::numbers::pi;
        state(i) =  x - std::numbers::pi;
    }
#endif
}

void Aircraft::update() {
    static double step_time = Params::getSingleton()->STEP_TIME; 
    static Logger rotor_logger("rotors.csv", "time,rotors_om");

    std::scoped_lock lck(mtx);
    VectorXd next = ode->step(state.real_time, state.getState(),
                            std::bind_front(&Aircraft::RHS, this), step_time);
    clampOrientationIfNessessery(next);
    state.setAcceleration((Matrices::TMatrix(UAVstate::getY(next)) *UAVstate::getX(next) -
        Matrices::TMatrix(state.getY()) * state.getX()) / step_time);
    state = next;
    state.real_time += step_time;
    rotor_logger.log(state.real_time, {state.getOm()});
}

void Aircraft::reduceMass(double delta_m, Eigen::Vector3d r) 
{
    double m = massMatrix(0,0);
    if(delta_m > m)
    {
        std::cerr << "Mass can not be negative!" << std::endl;
        return;
    }
    massMatrix(0,0) -= delta_m;
    massMatrix(1,1) -= delta_m;
    massMatrix(2,2) -= delta_m;
    Eigen::Matrix3d r_tilde = Matrices::asSkewSymmeticMatrix(r);
    Eigen::Matrix3d neg_lost_inertia =  delta_m * r_tilde * r_tilde;
    massMatrix.block<3,3>(3,3) += neg_lost_inertia;

    invMassMatrix = massMatrix.inverse();
}

Eigen::VectorXd Aircraft::RHS(double time, Eigen::VectorXd local_state) {
    static auto atmosphere = Atmosphere::getSingleton();

    VectorXd res;
    res.setZero(local_state.size());
    auto Y = UAVstate::getY(local_state);
    auto X = UAVstate::getX(local_state);
    Matrix3d r_nb = Matrices::R_nb(Y);

#ifdef USE_QUATERIONS
    Vector<double, 7> newY = Vector<double, 7>::Zero();
    newY.head<3>() = r_nb.transpose() * X.head<3>();
    Eigen::Vector4d q = Y.tail<4>();
    newY.tail<4>() = Matrices::OM_conj(X) * q + (1.0 - q.squaredNorm()) * q;
    UAVstate::setY(res, newY);
#else
    UAVstate::setY(res, Matrices::TMatrix(Y) * X);
#endif
    Eigen::Vector<double, 6> accel = invMassMatrix *
        (
            Forces::gravity_loads(massMatrix(0,0),r_nb) +
            Forces::rotor_lift_loads(noOfRotors, rotors.get(),UAVstate::getOm(local_state)) +
            Forces::jet_lift_loads(noOfJets, jets.get(), time) +
            Forces::aerodynamic_loads(X,r_nb*atmosphere->getWind(),surfaces, aero, -Y.z()) +
            state.getOuterForce() -
            Matrices::gyroMatrix(X) * massMatrix * UAVstate::getX(local_state)
        );
    UAVstate::setX(res, accel);
    UAVstate::setOm(res,
    Forces::angularAcceleration(this->state.getDemandedOm(),
        UAVstate::getOm(local_state)));
    return res;
}
