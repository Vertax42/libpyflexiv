#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <vector>

namespace flexiv_rt {

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

// ---- Per-cycle velocity/acceleration limits (enforced every 1ms in RT) ----
// Values from Flexiv RDK SendCartesianMotionForce safe defaults (robot.hpp)
// and RDK examples (intermediate1/2).

constexpr double kRtDt = 0.001;  // 1 kHz cycle time [s]

// Cartesian limits
constexpr double kCartMaxLinearVel  = 0.5;   // m/s
constexpr double kCartMaxAngularVel = 1.0;   // rad/s
constexpr double kCartMaxLinearAcc  = 2.0;   // m/s²
constexpr double kCartMaxAngularAcc = 5.0;   // rad/s²

// Joint limits (conservative defaults; runtime limits from RobotInfo::dq_max)
constexpr double kJointMaxVel = 2.0;   // rad/s
constexpr double kJointMaxAcc = 3.0;   // rad/s²

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

// ---------- Per-cycle clamping functions ----------
// Applied every 1kHz cycle BEFORE sending to the robot SDK.
// All functions: inline, noexcept, zero heap alloc, RT-safe.
// Return true if any value was clamped (for optional logging).

/// Clamp Cartesian pose change per cycle (velocity limiting).
/// Position: Euclidean distance clamped to max_linear_vel * dt.
/// Rotation: angular distance clamped via SLERP to max_angular_vel * dt.
inline bool ClampCartesianPose(
    std::array<double, 7>& pose,
    const std::array<double, 7>& last_pose,
    double max_linear_vel  = kCartMaxLinearVel,
    double max_angular_vel = kCartMaxAngularVel,
    double dt = kRtDt) noexcept
{
    bool clamped = false;

    // --- Position: Euclidean distance ---
    double max_pos_delta = max_linear_vel * dt;
    double d2 = 0;
    for (int i = 0; i < 3; ++i) {
        double d = pose[i] - last_pose[i];
        d2 += d * d;
    }
    double dist = std::sqrt(d2);
    if (dist > max_pos_delta && dist > 1e-12) {
        double scale = max_pos_delta / dist;
        for (int i = 0; i < 3; ++i) {
            pose[i] = last_pose[i] + (pose[i] - last_pose[i]) * scale;
        }
        clamped = true;
    }

    // --- Rotation: quaternion angular distance ---
    double max_rot_delta = max_angular_vel * dt;
    double dot = last_pose[3]*pose[3] + last_pose[4]*pose[4]
               + last_pose[5]*pose[5] + last_pose[6]*pose[6];
    double sign = (dot < 0.0) ? -1.0 : 1.0;
    double adot = std::abs(dot);
    if (adot > 1.0) adot = 1.0;
    double angle = 2.0 * std::acos(adot);

    if (angle > max_rot_delta && angle > 1e-10) {
        // SLERP from last_pose toward pose, clamped to max_rot_delta
        double t = max_rot_delta / angle;
        double half_angle = std::acos(adot);  // quaternion half-angle
        double sin_half = std::sin(half_angle);

        if (sin_half > 1e-12) {
            double w0 = std::sin((1.0 - t) * half_angle) / sin_half;
            double w1 = std::sin(t * half_angle) / sin_half * sign;
            for (int i = 3; i < 7; ++i) {
                pose[i] = w0 * last_pose[i] + w1 * pose[i];
            }
        } else {
            // Nearly identical — hold last rotation
            for (int i = 3; i < 7; ++i) {
                pose[i] = last_pose[i];
            }
        }
        clamped = true;
    }

    return clamped;
}

/// Clamp Cartesian velocity feedforward magnitude and acceleration.
/// Velocity: linear/angular magnitude clamped.
/// Acceleration: per-axis velocity change clamped.
inline bool ClampCartesianVelocity(
    std::array<double, 6>& vel,
    const std::array<double, 6>& last_vel,
    double max_linear_vel  = kCartMaxLinearVel,
    double max_angular_vel = kCartMaxAngularVel,
    double max_linear_acc  = kCartMaxLinearAcc,
    double max_angular_acc = kCartMaxAngularAcc,
    double dt = kRtDt) noexcept
{
    bool clamped = false;

    // --- Linear velocity magnitude ---
    double v2 = vel[0]*vel[0] + vel[1]*vel[1] + vel[2]*vel[2];
    if (v2 > max_linear_vel * max_linear_vel) {
        double scale = max_linear_vel / std::sqrt(v2);
        vel[0] *= scale; vel[1] *= scale; vel[2] *= scale;
        clamped = true;
    }

    // --- Angular velocity magnitude ---
    double w2 = vel[3]*vel[3] + vel[4]*vel[4] + vel[5]*vel[5];
    if (w2 > max_angular_vel * max_angular_vel) {
        double scale = max_angular_vel / std::sqrt(w2);
        vel[3] *= scale; vel[4] *= scale; vel[5] *= scale;
        clamped = true;
    }

    // --- Linear acceleration (per-axis) ---
    double max_dv_lin = max_linear_acc * dt;
    for (int i = 0; i < 3; ++i) {
        double dv = vel[i] - last_vel[i];
        if (std::abs(dv) > max_dv_lin) {
            vel[i] = last_vel[i] + std::copysign(max_dv_lin, dv);
            clamped = true;
        }
    }

    // --- Angular acceleration (per-axis) ---
    double max_dv_ang = max_angular_acc * dt;
    for (int i = 3; i < 6; ++i) {
        double dv = vel[i] - last_vel[i];
        if (std::abs(dv) > max_dv_ang) {
            vel[i] = last_vel[i] + std::copysign(max_dv_ang, dv);
            clamped = true;
        }
    }

    return clamped;
}

/// Clamp joint position change per cycle (per-joint velocity limiting).
inline bool ClampJointPosition(
    std::vector<double>& pos,
    const std::vector<double>& last_pos,
    double max_vel = kJointMaxVel,
    double dt = kRtDt) noexcept
{
    bool clamped = false;
    double max_delta = max_vel * dt;
    for (size_t i = 0; i < pos.size() && i < last_pos.size(); ++i) {
        double delta = pos[i] - last_pos[i];
        if (std::abs(delta) > max_delta) {
            pos[i] = last_pos[i] + std::copysign(max_delta, delta);
            clamped = true;
        }
    }
    return clamped;
}

/// Clamp joint velocity change per cycle (per-joint acceleration limiting).
inline bool ClampJointVelocity(
    std::vector<double>& vel,
    const std::vector<double>& last_vel,
    double max_acc = kJointMaxAcc,
    double dt = kRtDt) noexcept
{
    bool clamped = false;
    double max_dv = max_acc * dt;
    for (size_t i = 0; i < vel.size() && i < last_vel.size(); ++i) {
        double dv = vel[i] - last_vel[i];
        if (std::abs(dv) > max_dv) {
            vel[i] = last_vel[i] + std::copysign(max_dv, dv);
            clamped = true;
        }
    }
    return clamped;
}

} // namespace flexiv_rt
