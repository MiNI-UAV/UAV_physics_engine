#pragma once
#include <zmq.hpp>
#include "uav_state.hpp"

void controlListenerJob(zmq::context_t* ctx, std::string address,UAVstate& state);