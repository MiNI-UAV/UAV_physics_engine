#pragma once
#include <zmq.hpp>
#include "uav_state.hpp"
#include "matrices.hpp"

void controlListenerJob(zmq::context_t* ctx, std::string address,UAVstate& state, Matrices& matricies);