#include "aircraft.hpp"

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

    ode = ODE::factory(ODE::ODEMethod::RK4);
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
    static Logger rotor_logger("rotors.csv", "time,rotors_om");

    std::scoped_lock lck(mtx);
    VectorXd next = ode->step(state.real_time, state.getState(),
                            std::bind_front(&Aircraft::RHS, this), def::STEP_TIME);
    clampOrientationIfNessessery(next);
    state.setAcceleration((UAVstate::getX(next) - state.getX()) / def::STEP_TIME);
    state = next;
    state.real_time += def::STEP_TIME;
    rotor_logger.log(state.real_time, {state.getOm()});
}

void Aircraft::reduceMass(double delta_m) 
{
    double m = massMatrix(0,0);
    if(delta_m > m)
    {
        std::cerr << "Mass can not be negative!" << std::endl;
        return;
    }
    massMatrix = ((m-delta_m)/m) * massMatrix;
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
            Forces::gravity_loads(r_nb) +
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