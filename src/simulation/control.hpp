#pragma once
#include <zmq.hpp>
#include "uav_state.hpp"
#include "../dynamic/matrices.hpp"
#include "../defines.hpp"
#include "../aircraft/aircraft.hpp"

/// @brief Job of control listener thread. Listen for new control command and handle them
/// @param ctx zero mq context
/// @param address address of listener socket
/// @param aircraft pointer to aircraft
void controlListenerJob(zmq::context_t* ctx, std::string address, Aircraft* aircraft);