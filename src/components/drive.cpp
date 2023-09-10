#include "drive.hpp"

bool Jet::start(double time)
{
    if(state == JetState::READY)
    {
        startTime = time;
        state = JetState::WORKING;
        return true;
    }
    return false;
}

double Jet::getThrust(double sim_time)
{
    if(state == JetState::READY) return 0.0;
    double local_time = sim_time - startTime;
    if(local_time >= time(phases-1)) state = JetState::BURNT;
    if(state == JetState::BURNT) return thrust(phases-1);
    while(currentPhase <= phases - 2 && local_time >= time(currentPhase+1)) currentPhase++;
    double thrust_val = thrust(currentPhase) +
        (thrust(currentPhase+1)-thrust(currentPhase)) *
        (local_time-time(currentPhase)) /
        (time(currentPhase+1)-time(currentPhase));
    lastThrust = thrust_val;
    return thrust_val;
}
