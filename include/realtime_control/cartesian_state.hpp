#pragma once
#include <array>
#include <vector>

namespace flexiv_bindings {

struct CartesianState {
    std::array<double,7> tcp_pose;
    std::array<double,6> tcp_vel;
    std::array<double,6> ext_wrench_in_tcp;
    std::array<double,6> ext_wrench_in_world;
    std::array<double,6> ft_sensor_raw;
    std::vector<double>  q;
    std::vector<double>  tau_ext;
};

} // namespace flexiv_bindings
