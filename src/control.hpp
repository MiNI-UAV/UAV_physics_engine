#pragma once
#include <zmq.hpp>
#include "uav_state.hpp"
#include "matrices.hpp"

#define FRICTION_EPS 0.001

void controlListenerJob(zmq::context_t* ctx, std::string address,UAVstate& state, Matrices& matricies);