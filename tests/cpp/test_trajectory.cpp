#include "realtime_control/trajectory.hpp"
#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <vector>

using namespace flexiv_rt;

// ============================================================================
// Helper: quaternion from axis-angle
// ============================================================================
static std::array<double, 7> MakePose(double x, double y, double z,
                                       double axis_x, double axis_y, double axis_z,
                                       double angle_deg) {
    double half = angle_deg * M_PI / 180.0 / 2.0;
    double s = std::sin(half);
    double c = std::cos(half);
    // Normalize axis
    double norm = std::sqrt(axis_x * axis_x + axis_y * axis_y + axis_z * axis_z);
    if (norm > 1e-12) {
        axis_x /= norm;
        axis_y /= norm;
        axis_z /= norm;
    }
    return {x, y, z, c, axis_x * s, axis_y * s, axis_z * s};
}

static double QuatNorm(const std::array<double, 7>& p) {
    return std::sqrt(p[3]*p[3] + p[4]*p[4] + p[5]*p[5] + p[6]*p[6]);
}

static double PosDist(const std::array<double, 7>& a, const std::array<double, 7>& b) {
    double d2 = 0;
    for (int i = 0; i < 3; ++i) { double d = a[i] - b[i]; d2 += d * d; }
    return std::sqrt(d2);
}

static double QuatAngDist(const std::array<double, 7>& a, const std::array<double, 7>& b) {
    double dot = a[3]*b[3] + a[4]*b[4] + a[5]*b[5] + a[6]*b[6];
    dot = std::max(-1.0, std::min(1.0, std::abs(dot)));
    return 2.0 * std::acos(dot);
}

// ============================================================================
// LinearTrajectory — Pure Translation
// ============================================================================

TEST(LinearTrajectory, PureTranslation_ConstantVelocity) {
    LinearTrajectory traj;
    // Move 0.1m in X over 100ms (100 steps at 1kHz)
    auto start = MakePose(0.0, 0.0, 0.0, 0, 0, 1, 0);
    auto end   = MakePose(0.1, 0.0, 0.0, 0, 0, 1, 0);
    traj.init(start, end, 0.1);

    std::array<double, 7> pose;
    std::array<double, 6> vel;

    // Collect all intermediate velocities
    std::vector<double> vx_samples;
    int steps = 0;
    while (traj.step(pose, vel)) {
        vx_samples.push_back(vel[0]);
        steps++;
    }

    // Should take 99 in-progress steps (step 100 returns false)
    EXPECT_EQ(steps, 99);

    // Final pose should match end
    EXPECT_NEAR(pose[0], 0.1, 1e-10);
    EXPECT_NEAR(pose[1], 0.0, 1e-10);
    EXPECT_NEAR(pose[2], 0.0, 1e-10);

    // All velocities should be constant = 0.1m / 0.1s = 1.0 m/s
    for (double vx : vx_samples) {
        EXPECT_NEAR(vx, 1.0, 1e-10);
    }

    // Y and Z velocity should be zero
    // (check last sample — we verified all are constant)
    // Re-run to check one more
    traj.init(start, end, 0.1);
    traj.step(pose, vel);
    EXPECT_NEAR(vel[1], 0.0, 1e-10);
    EXPECT_NEAR(vel[2], 0.0, 1e-10);
}

TEST(LinearTrajectory, PureTranslation_LinearPosition) {
    LinearTrajectory traj;
    auto start = MakePose(0.0, 0.0, 0.0, 0, 0, 1, 0);
    auto end   = MakePose(0.3, 0.0, 0.0, 0, 0, 1, 0);
    traj.init(start, end, 0.1);  // 100 steps

    std::array<double, 7> pose;
    std::array<double, 6> vel;

    // Step to halfway (50 steps)
    for (int i = 0; i < 50; ++i) {
        ASSERT_TRUE(traj.step(pose, vel));
    }
    // At step 50/100: t = 0.5, pos = 0.0 + 0.5 * 0.3 = 0.15
    EXPECT_NEAR(pose[0], 0.15, 1e-10);
}

// ============================================================================
// LinearTrajectory — Pure Rotation
// ============================================================================

TEST(LinearTrajectory, PureRotation_ConstantAngularVelocity) {
    LinearTrajectory traj;
    // 90 degree rotation around Z over 1s (1000 steps)
    auto start = MakePose(0.5, 0.0, 0.3, 0, 0, 1, 0);
    auto end   = MakePose(0.5, 0.0, 0.3, 0, 0, 1, 90);
    traj.init(start, end, 1.0);

    std::array<double, 7> pose;
    std::array<double, 6> vel;

    // Expected angular velocity = 90deg/s = pi/2 rad/s around Z
    double expected_wz = M_PI / 2.0;

    std::vector<double> wz_samples;
    int steps = 0;
    while (traj.step(pose, vel)) {
        wz_samples.push_back(vel[5]);  // wz
        steps++;

        // Position should not change
        EXPECT_NEAR(pose[0], 0.5, 1e-10);
        EXPECT_NEAR(pose[1], 0.0, 1e-10);
        EXPECT_NEAR(pose[2], 0.3, 1e-10);

        // Quaternion should remain normalized
        EXPECT_NEAR(QuatNorm(pose), 1.0, 1e-10);
    }

    EXPECT_EQ(steps, 999);

    // Angular velocity should be constant
    for (double wz : wz_samples) {
        EXPECT_NEAR(wz, expected_wz, 1e-6);
    }

    // Linear velocity should be zero
    traj.init(start, end, 1.0);
    traj.step(pose, vel);
    EXPECT_NEAR(vel[0], 0.0, 1e-10);
    EXPECT_NEAR(vel[1], 0.0, 1e-10);
    EXPECT_NEAR(vel[2], 0.0, 1e-10);
}

TEST(LinearTrajectory, PureRotation_HalfwayAngle) {
    LinearTrajectory traj;
    // 90 degree rotation around Z
    auto start = MakePose(0.0, 0.0, 0.0, 0, 0, 1, 0);
    auto end   = MakePose(0.0, 0.0, 0.0, 0, 0, 1, 90);
    traj.init(start, end, 1.0);

    std::array<double, 7> pose;
    std::array<double, 6> vel;

    // Step to halfway (500 steps)
    for (int i = 0; i < 500; ++i) {
        ASSERT_TRUE(traj.step(pose, vel));
    }

    // At t=0.5, should be at 45 degrees
    double expected_angle = M_PI / 4.0;
    double actual_angle = QuatAngDist(start, pose);
    EXPECT_NEAR(actual_angle, expected_angle, 1e-4);
}

// ============================================================================
// LinearTrajectory — Combined Translation + Rotation
// ============================================================================

TEST(LinearTrajectory, Combined_EndPoseExact) {
    LinearTrajectory traj;
    auto start = MakePose(0.0, 0.0, 0.0, 0, 0, 1, 0);
    auto end   = MakePose(0.1, 0.2, 0.3, 1, 0, 0, 45);
    traj.init(start, end, 0.5);

    std::array<double, 7> pose;
    std::array<double, 6> vel;

    // Run to completion
    while (traj.step(pose, vel)) {}

    // End pose should be exact (returned directly as end_pose_)
    EXPECT_NEAR(pose[0], end[0], 1e-10);
    EXPECT_NEAR(pose[1], end[1], 1e-10);
    EXPECT_NEAR(pose[2], end[2], 1e-10);

    // Quaternion: check angular distance is ~0
    EXPECT_NEAR(QuatAngDist(pose, end), 0.0, 1e-10);
}

TEST(LinearTrajectory, Combined_MonotonicProgress) {
    LinearTrajectory traj;
    auto start = MakePose(0.0, 0.0, 0.0, 0, 0, 1, 0);
    auto end   = MakePose(0.1, 0.0, 0.0, 0, 0, 1, 60);
    traj.init(start, end, 0.2);  // 200 steps

    std::array<double, 7> pose;
    std::array<double, 6> vel;

    double prev_pos_dist = 0;
    double prev_rot_dist = 0;
    while (traj.step(pose, vel)) {
        double pd = PosDist(start, pose);
        double rd = QuatAngDist(start, pose);
        // Distance from start should monotonically increase
        EXPECT_GE(pd, prev_pos_dist - 1e-12);
        EXPECT_GE(rd, prev_rot_dist - 1e-6);
        prev_pos_dist = pd;
        prev_rot_dist = rd;
    }
}

// ============================================================================
// LinearTrajectory — Edge Cases
// ============================================================================

TEST(LinearTrajectory, ZeroMotion) {
    LinearTrajectory traj;
    auto pose_both = MakePose(0.5, 0.1, 0.3, 0, 0, 1, 30);
    traj.init(pose_both, pose_both, 0.05);

    std::array<double, 7> pose;
    std::array<double, 6> vel;

    while (traj.step(pose, vel)) {
        // Position should stay the same
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(pose[i], pose_both[i], 1e-10);
        }
        // Velocity should be zero
        for (int i = 0; i < 6; ++i) {
            EXPECT_NEAR(vel[i], 0.0, 1e-10);
        }
    }
}

TEST(LinearTrajectory, Cancel) {
    LinearTrajectory traj;
    auto start = MakePose(0, 0, 0, 0, 0, 1, 0);
    auto end   = MakePose(1, 0, 0, 0, 0, 1, 0);
    traj.init(start, end, 1.0);

    std::array<double, 7> pose;
    std::array<double, 6> vel;

    // Step a few times
    EXPECT_TRUE(traj.step(pose, vel));
    EXPECT_TRUE(traj.isActive());

    traj.cancel();
    EXPECT_FALSE(traj.isActive());
    EXPECT_FALSE(traj.step(pose, vel));
}

TEST(LinearTrajectory, MinDuration) {
    // Duration <= 0 should be clamped to 1ms (1 step)
    LinearTrajectory traj;
    auto start = MakePose(0, 0, 0, 0, 0, 1, 0);
    auto end   = MakePose(0.01, 0, 0, 0, 0, 1, 0);
    traj.init(start, end, -1.0);  // negative duration

    std::array<double, 7> pose;
    std::array<double, 6> vel;

    // Should complete in 1 step (returns false immediately with end pose)
    EXPECT_FALSE(traj.step(pose, vel));
    EXPECT_NEAR(pose[0], 0.01, 1e-10);
}

TEST(LinearTrajectory, NotActiveBeforeInit) {
    LinearTrajectory traj;
    EXPECT_FALSE(traj.isActive());

    std::array<double, 7> pose;
    std::array<double, 6> vel;
    EXPECT_FALSE(traj.step(pose, vel));
}

// ============================================================================
// LinearTrajectory vs MinJerkTrajectory — Velocity Profile Comparison
// ============================================================================

TEST(TrajectoryComparison, LinearHasConstantVelocity) {
    LinearTrajectory lin;
    MinJerkTrajectory mjk;

    auto start = MakePose(0, 0, 0, 0, 0, 1, 0);
    auto end   = MakePose(0.1, 0, 0, 0, 0, 1, 0);
    double dur = 0.1;  // 100 steps

    lin.init(start, end, dur);
    mjk.init(start, end, dur);

    std::array<double, 7> pose;
    std::array<double, 6> vel_lin, vel_mjk;

    double max_lin_vel_dev = 0;  // deviation from mean
    double expected_vel = 0.1 / 0.1;  // 1.0 m/s

    double prev_mjk_vx = 0;
    bool mjk_velocity_varies = false;

    int steps = 0;
    while (lin.step(pose, vel_lin)) {
        mjk.step(pose, vel_mjk);
        steps++;

        // Linear: velocity should be constant
        max_lin_vel_dev = std::max(max_lin_vel_dev,
                                    std::abs(vel_lin[0] - expected_vel));

        // MinJerk: velocity should vary (starts at 0, peaks, returns to 0)
        if (steps > 1 && std::abs(vel_mjk[0] - prev_mjk_vx) > 1e-6) {
            mjk_velocity_varies = true;
        }
        prev_mjk_vx = vel_mjk[0];
    }

    EXPECT_NEAR(max_lin_vel_dev, 0.0, 1e-10)
        << "Linear trajectory velocity should be constant";
    EXPECT_TRUE(mjk_velocity_varies)
        << "MinJerk trajectory velocity should vary over time";
}

TEST(TrajectoryComparison, BothReachSameEndPose) {
    LinearTrajectory lin;
    MinJerkTrajectory mjk;

    auto start = MakePose(0, 0, 0, 0, 0, 1, 0);
    auto end   = MakePose(0.1, 0.2, 0.3, 1, 0, 0, 45);
    double dur = 0.5;

    lin.init(start, end, dur);
    mjk.init(start, end, dur);

    std::array<double, 7> pose_lin, pose_mjk;
    std::array<double, 6> vel;

    while (lin.step(pose_lin, vel)) {}
    while (mjk.step(pose_mjk, vel)) {}

    // Both should reach the same end position
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(pose_lin[i], pose_mjk[i], 1e-10);
    }

    // Both should reach the same end orientation
    EXPECT_NEAR(QuatAngDist(pose_lin, pose_mjk), 0.0, 1e-10);
}

// ============================================================================
// LinearTrajectory — Streaming VLA Simulation
// ============================================================================

// Simulate 30Hz policy sending commands, 1kHz RT consuming with decimation
// Verify: no zero-velocity dips at command boundaries
TEST(LinearTrajectory, StreamingVLA_NoZeroVelocityDips) {
    const int policy_hz = 30;
    const int decimation = static_cast<int>(std::round(1000.0 / policy_hz));
    const double period_sec = decimation * 0.001;
    const int num_commands = 10;

    // Simulate a straight-line trajectory: each action moves +0.01m in X
    LinearTrajectory stream_interp;
    std::array<double, 7> last_sent = MakePose(0, 0, 0, 0, 0, 1, 0);

    std::vector<double> all_velocities;
    int cycle = 0;

    for (int cmd = 0; cmd < num_commands; ++cmd) {
        // New command arrives
        auto target = last_sent;
        target[0] += 0.01;  // +10mm per action

        stream_interp.init(last_sent, target, period_sec);

        // Simulate decimation cycles
        for (int sub = 0; sub < decimation; ++sub) {
            std::array<double, 7> pose;
            std::array<double, 6> vel;

            bool in_progress = stream_interp.step(pose, vel);
            if (in_progress) {
                all_velocities.push_back(vel[0]);
                last_sent = pose;
            } else {
                // Interpolation done, hold at target
                last_sent = target;
                all_velocities.push_back(0.0);  // end of segment
            }
            cycle++;
        }
    }

    // Check: the vast majority of velocity samples should be non-zero and constant
    // Only the very last step of each segment might be zero (when interpolation completes)
    int zero_count = 0;
    double expected_vel = 0.01 / period_sec;  // m/s

    for (size_t i = 0; i < all_velocities.size(); ++i) {
        if (std::abs(all_velocities[i]) < 1e-10) {
            zero_count++;
        } else {
            // Non-zero velocities should be approximately constant
            EXPECT_NEAR(all_velocities[i], expected_vel, expected_vel * 0.01)
                << "at cycle " << i;
        }
    }

    // At most 1 zero per command segment (the final step)
    EXPECT_LE(zero_count, num_commands)
        << "Too many zero-velocity samples — suggests stop-and-go behavior";

    // Final position should be close to expected
    double expected_final_x = num_commands * 0.01;
    EXPECT_NEAR(last_sent[0], expected_final_x, 1e-6);
}

// Contrast: MinJerk streaming would have zero velocity at EVERY boundary
TEST(MinJerkTrajectory, StreamingVLA_HasZeroVelocityAtBoundaries) {
    const int policy_hz = 30;
    const int decimation = static_cast<int>(std::round(1000.0 / policy_hz));
    const double period_sec = decimation * 0.001;
    const int num_commands = 10;

    MinJerkTrajectory stream_interp;
    std::array<double, 7> last_sent = MakePose(0, 0, 0, 0, 0, 1, 0);

    // Track velocity at first step of each command (should be ~0 for min-jerk)
    std::vector<double> first_step_velocities;

    for (int cmd = 0; cmd < num_commands; ++cmd) {
        auto target = last_sent;
        target[0] += 0.01;

        stream_interp.init(last_sent, target, period_sec);

        for (int sub = 0; sub < decimation; ++sub) {
            std::array<double, 7> pose;
            std::array<double, 6> vel;

            bool in_progress = stream_interp.step(pose, vel);
            if (sub == 0) {
                first_step_velocities.push_back(vel[0]);
            }
            if (in_progress) {
                last_sent = pose;
            } else {
                last_sent = target;
            }
        }
    }

    // For min-jerk, the velocity at the start of each segment should be near zero
    // (because it starts from zero velocity). This is the stop-and-go problem.
    double expected_peak = 1.875 * 0.01 / period_sec;  // min-jerk peak velocity
    for (double v : first_step_velocities) {
        // First step velocity should be much less than peak (near zero start)
        EXPECT_LT(std::abs(v), expected_peak * 0.1)
            << "Min-jerk first-step velocity should be near zero (stop-and-go)";
    }
}
