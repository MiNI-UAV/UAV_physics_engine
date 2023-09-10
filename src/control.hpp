#pragma once
#include <zmq.hpp>
#include "uav_state.hpp"
#include "matrices.hpp"
#include "defines.hpp"
#include "aircrafts/aircraft.hpp"

void controlListenerJob(zmq::context_t* ctx, std::string address, Aircraft* aircraft);