#pragma once
#include <array>
#include <vector>

namespace flexiv_rt {

struct JointState {
    std::vector<double>  q;        // joint positions [rad]
    std::vector<double>  dq;       // joint velocities [rad/s]
    std::vector<double>  tau;      // joint torques [Nm]
    std::vector<double>  tau_ext;  // external torques [Nm]
    std::array<double,7> tcp_pose; // TCP pose (for convenience)
};

} // namespace flexiv_rt
