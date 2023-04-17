#include "simulation.hpp"
#include <Eigen/Dense>
#include "iostream"

#include "uav_params.hpp"
#include "uav_state.hpp"
#include "forces.hpp"
#include "matrices.hpp"
#include "RK4.hpp"
#include "timed_loop.hpp"

Simulation::Simulation(UAVparams& params, UAVstate& state):
    _params{params},
    _state{state},
    forces(params),
    matrices(params)
{
    RHS = [this] (double, Eigen::VectorXd local_state)
    {
        VectorXd res;
        res.setZero(local_state.size());
        UAVstate::setY(res,matrices.TMatrix(UAVstate::getY(local_state))*UAVstate::getX(local_state));
        //TODO: missing wind!
        UAVstate::setX(res,matrices.invMassMatrix*(forces.gravity_forces(UAVstate::getY(local_state)) 
           + forces.lift_forces(UAVstate::getOm(local_state)) 
           -  matrices.gyroMatrix(UAVstate::getX(local_state)) * matrices.massMatrix * UAVstate::getX(local_state)));
        UAVstate::setOm(res, forces.angularAcceleration(this->_state.getDemandedOm(),UAVstate::getOm(local_state)));
        return res;
    };
}

void Simulation::run()
{
    matrices.updateMatrices();
    TimedLoop loop(2, [this](){
        VectorXd next = RK4_step(real_time,_state.getState(),RHS,0.002);
        real_time+=0.001;
        _state = next;
        std::cout << _state << std::endl;
        return true;
    });
    loop.go();
}
