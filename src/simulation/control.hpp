#pragma once
#include <zmq.hpp>
#include "uav_state.hpp"
#include "../dynamic/matrices.hpp"
#include "../defines.hpp"
#include "../aircraft/aircraft.hpp"

void controlListenerJob(zmq::context_t* ctx, std::string address, Aircraft* aircraft);