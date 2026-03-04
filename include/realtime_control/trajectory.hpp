#pragma once
#include <array>
#include <cmath>
#include <cstdint>

namespace flexiv_rt {

/// RT-safe minimum-jerk trajectory generator for 7D poses [x,y,z, qw,qx,qy,qz].
///
/// All operations use std::array, are noexcept, and perform zero heap allocation.
/// Suitable for calling from a 1 kHz SCHED_FIFO RT thread.
///
/// Position interpolation:  s(t) = 10*tau^3 - 15*tau^4 + 6*tau^5
/// Rotation interpolation:  SLERP(q_start, q_end, s(t))
/// Velocity feed-forward:   linear from ds/dt * pos_delta,
///                           angular from ds/dt * rot_angle * rot_axis
class MinJerkTrajectory {
public:
    MinJerkTrajectory() noexcept = default;

    // --- Velocity limits for auto-duration ---
    // Min-jerk peak velocity = 1.875 * distance / duration,
    // so duration = 1.875 * distance / max_velocity.
    static constexpr double kMaxLinearVel  = 0.5;   // m/s   (peak Cartesian speed)
    static constexpr double kMaxAngularVel = 1.5;   // rad/s (peak rotation speed)
    static constexpr double kMinDuration   = 1.0;   // s     (minimum trajectory time)

    /// Compute appropriate duration given start/end pose and velocity limits.
    /// Uses min-jerk peak velocity formula: v_peak = 1.875 * distance / duration
    /// => duration = 1.875 * distance / v_max
    static double computeDuration(
        const std::array<double,7>& start_pose,
        const std::array<double,7>& end_pose,
        double max_linear_vel  = kMaxLinearVel,
        double max_angular_vel = kMaxAngularVel,
        double min_duration    = kMinDuration) noexcept
    {
        // Position distance
        double d2 = 0;
        for (int i = 0; i < 3; ++i) {
            double d = end_pose[i] - start_pose[i];
            d2 += d * d;
        }
        double pos_dist = std::sqrt(d2);

        // Rotation distance (quaternion angular distance)
        double dot = start_pose[3]*end_pose[3] + start_pose[4]*end_pose[4]
                   + start_pose[5]*end_pose[5] + start_pose[6]*end_pose[6];
        dot = std::max(-1.0, std::min(1.0, std::abs(dot)));
        double rot_dist = 2.0 * std::acos(dot);  // radians

        // Min-jerk peak factor: v_peak = 1.875 * dist / T
        // => T = 1.875 * dist / v_max
        constexpr double kPeakFactor = 1.875;
        double t_pos = (max_linear_vel > 0)
            ? kPeakFactor * pos_dist / max_linear_vel : 0.0;
        double t_rot = (max_angular_vel > 0)
            ? kPeakFactor * rot_dist / max_angular_vel : 0.0;

        double t = std::max(t_pos, t_rot);
        return std::max(t, min_duration);
    }

    /// Initialize a new trajectory.
    /// @param start_pose  Starting pose [x,y,z, qw,qx,qy,qz]
    /// @param end_pose    Target pose   [x,y,z, qw,qx,qy,qz]
    /// @param duration_sec  Total trajectory duration in seconds.
    ///                      If <= 0, auto-computed from distance and velocity limits.
    void init(const std::array<double,7>& start_pose,
              const std::array<double,7>& end_pose,
              double duration_sec) noexcept
    {
        start_pose_ = start_pose;
        end_pose_   = end_pose;

        // Normalize quaternions
        normalizeQuat(start_pose_);
        normalizeQuat(end_pose_);

        // Ensure SLERP takes the short path (dot product >= 0)
        double dot = quatDot(start_pose_, end_pose_);
        if (dot < 0.0) {
            // Negate end quaternion to flip to short arc
            end_pose_[3] = -end_pose_[3];
            end_pose_[4] = -end_pose_[4];
            end_pose_[5] = -end_pose_[5];
            end_pose_[6] = -end_pose_[6];
            dot = -dot;
        }

        // Precompute position delta
        for (int i = 0; i < 3; ++i) {
            pos_delta_[i] = end_pose_[i] - start_pose_[i];
        }

        // Precompute rotation axis and angle for angular velocity feedforward
        dot = std::min(1.0, dot);  // clamp for acos safety
        rot_angle_ = 2.0 * std::acos(dot);

        if (rot_angle_ > 1e-6) {
            // Compute rotation axis from q_end * q_start^{-1}
            // q_rel = q_end * conj(q_start), axis = normalize(q_rel.xyz)
            double rx = end_pose_[4]*start_pose_[3] - end_pose_[3]*start_pose_[4]
                      + end_pose_[6]*start_pose_[5] - end_pose_[5]*start_pose_[6];
            double ry = end_pose_[5]*start_pose_[3] - end_pose_[3]*start_pose_[5]
                      + end_pose_[4]*start_pose_[6] - end_pose_[6]*start_pose_[4];
            double rz = end_pose_[6]*start_pose_[3] - end_pose_[3]*start_pose_[6]
                      + end_pose_[5]*start_pose_[4] - end_pose_[4]*start_pose_[5];
            // axis = normalize(rx, ry, rz)
            double anorm = std::sqrt(rx*rx + ry*ry + rz*rz);
            if (anorm > 1e-12) {
                double inv = 1.0 / anorm;
                rot_axis_[0] = rx * inv;
                rot_axis_[1] = ry * inv;
                rot_axis_[2] = rz * inv;
            } else {
                rot_axis_ = {0, 0, 0};
                rot_angle_ = 0;
            }
        } else {
            rot_axis_ = {0, 0, 0};
            rot_angle_ = 0;
        }

        // Auto-compute duration if not specified
        if (duration_sec <= 0.0) {
            duration_sec = computeDuration(start_pose_, end_pose_);
        }

        total_steps_ = static_cast<uint32_t>(duration_sec * 1000.0);  // 1 kHz
        if (total_steps_ < 1) total_steps_ = 1;
        current_step_ = 0;
        active_ = true;
        cancelled_ = false;
    }

    /// Advance the trajectory by one timestep (1 ms).
    /// @param[out] out_pose     Interpolated pose [x,y,z, qw,qx,qy,qz]
    /// @param[out] out_velocity Feed-forward velocity [vx,vy,vz, wx,wy,wz]
    /// @return true if trajectory is still in progress, false if finished
    bool step(std::array<double,7>& out_pose,
              std::array<double,6>& out_velocity) noexcept
    {
        if (!active_ || cancelled_) {
            return false;
        }

        current_step_++;

        // Clamp to end
        if (current_step_ >= total_steps_) {
            out_pose = end_pose_;
            out_velocity = {};
            active_ = false;
            return false;  // trajectory complete
        }

        double tau = static_cast<double>(current_step_) / static_cast<double>(total_steps_);

        // Min-jerk basis: s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5
        double tau2 = tau * tau;
        double tau3 = tau2 * tau;
        double tau4 = tau3 * tau;
        double tau5 = tau4 * tau;
        double s = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;

        // ds/dt for velocity feed-forward
        // ds/d_tau = 30*tau^2 - 60*tau^3 + 30*tau^4
        // dt = duration_sec => ds/dt = ds/d_tau / duration_sec
        double duration_sec = static_cast<double>(total_steps_) / 1000.0;
        double ds_dtau = 30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4;
        double ds_dt = ds_dtau / duration_sec;

        // Position: linear interpolation with min-jerk profile
        for (int i = 0; i < 3; ++i) {
            out_pose[i] = start_pose_[i] + s * pos_delta_[i];
        }

        // Rotation: SLERP(q_start, q_end, s)
        slerp(start_pose_, end_pose_, s, out_pose);

        // Linear velocity feed-forward: v = ds/dt * pos_delta
        for (int i = 0; i < 3; ++i) {
            out_velocity[i] = ds_dt * pos_delta_[i];
        }

        // Angular velocity feed-forward: ω = ds/dt * rot_angle * rot_axis
        // SLERP angular speed = theta * ds/dt, direction = rotation axis
        for (int i = 0; i < 3; ++i) {
            out_velocity[3 + i] = ds_dt * rot_angle_ * rot_axis_[i];
        }

        return true;  // still in progress
    }

    /// Cancel the trajectory. The RT thread will detect this on the next cycle.
    void cancel() noexcept {
        cancelled_ = true;
        active_ = false;
    }

    /// Whether the trajectory is currently active (initialized and not yet complete/cancelled).
    bool isActive() const noexcept {
        return active_ && !cancelled_;
    }

private:
    // --- Quaternion helpers (RT-safe, no heap, noexcept) ---

    static void normalizeQuat(std::array<double,7>& pose) noexcept {
        double norm = std::sqrt(
            pose[3]*pose[3] + pose[4]*pose[4] +
            pose[5]*pose[5] + pose[6]*pose[6]);
        if (norm < 1e-12) {
            pose[3] = 1.0; pose[4] = 0.0; pose[5] = 0.0; pose[6] = 0.0;
            return;
        }
        double inv = 1.0 / norm;
        pose[3] *= inv; pose[4] *= inv; pose[5] *= inv; pose[6] *= inv;
    }

    static double quatDot(const std::array<double,7>& a,
                           const std::array<double,7>& b) noexcept {
        return a[3]*b[3] + a[4]*b[4] + a[5]*b[5] + a[6]*b[6];
    }

    /// SLERP between quaternion parts of two poses.
    /// Writes result into out_pose[3..6]. Assumes dot >= 0 (short path).
    static void slerp(const std::array<double,7>& p0,
                       const std::array<double,7>& p1,
                       double t,
                       std::array<double,7>& out) noexcept
    {
        double dot = quatDot(p0, p1);
        // Clamp for numerical safety
        if (dot > 1.0)  dot = 1.0;
        if (dot < -1.0) dot = -1.0;

        double theta = std::acos(dot);

        if (theta < 1e-6) {
            // Nearly identical quaternions — use linear interpolation
            for (int i = 3; i < 7; ++i) {
                out[i] = p0[i] + t * (p1[i] - p0[i]);
            }
            // Normalize result
            double norm = std::sqrt(out[3]*out[3] + out[4]*out[4] +
                                     out[5]*out[5] + out[6]*out[6]);
            if (norm > 1e-12) {
                double inv = 1.0 / norm;
                out[3] *= inv; out[4] *= inv; out[5] *= inv; out[6] *= inv;
            }
        } else {
            double sin_theta = std::sin(theta);
            double w0 = std::sin((1.0 - t) * theta) / sin_theta;
            double w1 = std::sin(t * theta) / sin_theta;
            for (int i = 3; i < 7; ++i) {
                out[i] = w0 * p0[i] + w1 * p1[i];
            }
        }
    }

    // --- State ---
    std::array<double,7> start_pose_ = {0,0,0, 1,0,0,0};
    std::array<double,7> end_pose_   = {0,0,0, 1,0,0,0};
    std::array<double,3> pos_delta_  = {};  // precomputed position delta
    std::array<double,3> rot_axis_   = {};  // precomputed rotation axis (unit vector)
    double               rot_angle_  = 0;   // precomputed rotation angle [rad]

    uint32_t total_steps_   = 0;
    uint32_t current_step_  = 0;
    bool     active_        = false;
    bool     cancelled_     = false;
};

} // namespace flexiv_rt
