#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <vector>

namespace flexiv_bindings {

// Safety thresholds — defined as maximum velocities.
// The RT thread scales these by the actual Python command interval to get
// per-step thresholds.  This makes jump detection adaptive to any Python
// frequency (30 Hz, 100 Hz, etc.) without manual tuning.
//
//   threshold = max_velocity * cmd_interval_sec
//
// Example at 100 Hz (10 ms interval):
//   position:  10 m/s   * 0.01 s = 0.10 m   per step
//   rotation:  50 rad/s * 0.01 s = 0.50 rad  per step
//   joint:     10 rad/s * 0.01 s = 0.10 rad  per step
//
// Example at 30 Hz (33 ms interval):
//   position:  10 m/s   * 0.033 s = 0.33 m   per step
//   rotation:  50 rad/s * 0.033 s = 1.65 rad  per step
//   joint:     10 rad/s * 0.033 s = 0.33 rad  per step
constexpr double kMaxPositionVelocity = 10.0;   // m/s
constexpr double kMaxRotationVelocity = 50.0;   // rad/s
constexpr double kMaxJointVelocity    = 10.0;   // rad/s

// Clamp range for the measured command interval to avoid extreme thresholds.
constexpr double kMinCommandInterval = 0.001;   // 1 ms   (1 kHz)
constexpr double kMaxCommandInterval = 0.1;     // 100 ms (10 Hz)
// Default interval used before the second command arrives.
constexpr double kDefaultCommandInterval = 0.01; // 10 ms (100 Hz)

constexpr auto kCommandTimeout = std::chrono::milliseconds(500);

// ---------- Safety helper functions ----------

/// Check that all values in a container are finite (not NaN or Inf).
template <typename Container>
inline bool CheckFinite(const Container& values) {
    for (const auto& v : values)
        if (!std::isfinite(v)) return false;
    return true;
}

/// Check for joint-space jump. Returns true if any joint exceeds threshold.
inline bool CheckJointJump(const std::vector<double>& cmd,
                           const std::vector<double>& last,
                           double threshold) {
    if (cmd.size() != last.size()) return true;
    for (size_t i = 0; i < cmd.size(); ++i)
        if (std::abs(cmd[i] - last[i]) > threshold) return true;
    return false;
}

/// Quaternion angular distance [rad]. Pose layout: [x, y, z, qw, qx, qy, qz].
inline double QuatAngularDist(const std::array<double, 7>& p1,
                              const std::array<double, 7>& p2) {
    double dot = p1[3]*p2[3] + p1[4]*p2[4] + p1[5]*p2[5] + p1[6]*p2[6];
    dot = std::max(-1.0, std::min(1.0, std::abs(dot)));
    return 2.0 * std::acos(dot);
}

/// Check for Cartesian jump. Returns true if position or rotation exceeds threshold.
inline bool CheckCartesianJump(const std::array<double, 7>& cmd,
                               const std::array<double, 7>& last,
                               double pos_thresh, double rot_thresh) {
    double d2 = 0;
    for (int i = 0; i < 3; ++i) { double d = cmd[i] - last[i]; d2 += d * d; }
    if (std::sqrt(d2) > pos_thresh) return true;
    if (QuatAngularDist(cmd, last) > rot_thresh) return true;
    return false;
}

} // namespace flexiv_bindings
