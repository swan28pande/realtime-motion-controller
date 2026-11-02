// TCP bridge for the operator UI (line protocol, one client at a time):
//   client -> server:  "SET <axis> <deg>"
//   server -> client:  "STATUS <cycles> <max_late_us> <axis>:<sp_deg>:<pos_deg>:<vel_rad_s>:<effort_nm> ..."  (20 Hz)
#pragma once
#include <atomic>
#include <string>

#include "controller.hpp"

void run_operator_server(Controller& ctl, const std::string& listen_addr, int port, const std::atomic<bool>& running);
