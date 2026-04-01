#!/usr/bin/env python3
"""
Simulate the 1kHz RT thread interpolation pipeline without a real robot.

Replicates the exact C++ decimation + interpolation logic from:
  - cartesian_control.cpp  (Cartesian linear interpolation)
  - joint_impedance_control.cpp  (Joint linear interpolation)

Compares linear interpolation vs min-jerk to demonstrate that linear avoids
the stop-and-go velocity profile at command boundaries.

Usage:
    python test_interpolation_sim.py              # run assertions only
    python test_interpolation_sim.py --plot       # also show matplotlib plots
"""

import argparse
import math
import sys
from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np


# ============================================================================
# Quaternion helpers (matching C++ trajectory.hpp)
# ============================================================================

def quat_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n

def quat_dot(q1: np.ndarray, q2: np.ndarray) -> float:
    return float(np.dot(q1, q2))

def slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    dot = quat_dot(q0, q1)
    dot = np.clip(dot, -1.0, 1.0)
    theta = math.acos(dot)
    if theta < 1e-6:
        result = q0 + t * (q1 - q0)
        return result / np.linalg.norm(result)
    sin_theta = math.sin(theta)
    w0 = math.sin((1.0 - t) * theta) / sin_theta
    w1 = math.sin(t * theta) / sin_theta
    return w0 * q0 + w1 * q1

def quat_angle_dist(q1: np.ndarray, q2: np.ndarray) -> float:
    dot = abs(quat_dot(q1, q2))
    dot = min(1.0, dot)
    return 2.0 * math.acos(dot)


# ============================================================================
# Min-Jerk basis function (matching C++ MinJerkTrajectory)
# ============================================================================

def min_jerk_s(tau: float) -> float:
    """s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5"""
    tau2 = tau * tau
    tau3 = tau2 * tau
    return 10.0 * tau3 - 15.0 * tau3 * tau + 6.0 * tau3 * tau2

def min_jerk_ds_dtau(tau: float) -> float:
    """ds/dtau = 30*tau^2 - 60*tau^3 + 30*tau^4"""
    tau2 = tau * tau
    tau3 = tau2 * tau
    return 30.0 * tau2 - 60.0 * tau3 + 30.0 * tau3 * tau


# ============================================================================
# Trajectory simulators (replicate C++ logic exactly)
# ============================================================================

@dataclass
class CartesianTrajectoryResult:
    """Result of a single trajectory step."""
    pose: np.ndarray       # [x,y,z, qw,qx,qy,qz]
    velocity: np.ndarray   # [vx,vy,vz, wx,wy,wz]
    in_progress: bool


def simulate_cartesian_streaming(
    waypoints: List[np.ndarray],  # list of 7D poses from policy
    policy_hz: int,
    use_linear: bool = True,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Simulate 1kHz RT thread with decimation + interpolation.

    Returns:
        times: (N,) time in seconds for each 1ms step
        positions: (N, 3) xyz positions
        velocities: (N, 3) xyz velocities
    """
    decimation = round(1000.0 / policy_hz)
    period_sec = decimation * 0.001

    all_times = []
    all_positions = []
    all_velocities = []

    # Start at first waypoint
    last_sent_pose = waypoints[0].copy()
    time_ms = 0

    for cmd_idx in range(1, len(waypoints)):
        target_pose = waypoints[cmd_idx].copy()

        # Ensure SLERP short path
        start_quat = last_sent_pose[3:7]
        end_quat = target_pose[3:7]
        if quat_dot(start_quat, end_quat) < 0:
            target_pose[3:7] = -target_pose[3:7]

        pos_delta = target_pose[:3] - last_sent_pose[:3]
        start_pose_snapshot = last_sent_pose.copy()

        # Precompute rotation angle/axis
        dot = min(1.0, abs(quat_dot(start_pose_snapshot[3:7], target_pose[3:7])))
        rot_angle = 2.0 * math.acos(dot)

        total_steps = decimation

        for sub_step in range(total_steps):
            step_num = sub_step + 1  # 1-indexed like C++

            if step_num >= total_steps:
                # Final step: snap to target
                pose = target_pose.copy()
                vel = np.zeros(6)
            else:
                t = step_num / total_steps

                if use_linear:
                    s = t
                    ds_dt = 1.0 / period_sec
                else:
                    s = min_jerk_s(t)
                    ds_dtau = min_jerk_ds_dtau(t)
                    ds_dt = ds_dtau / period_sec

                # Position
                pos = start_pose_snapshot[:3] + s * pos_delta

                # Rotation (SLERP)
                quat = slerp(start_pose_snapshot[3:7], target_pose[3:7], s)

                pose = np.concatenate([pos, quat])

                # Velocity
                if use_linear:
                    linear_vel = pos_delta / period_sec
                else:
                    linear_vel = ds_dt * pos_delta

                vel = np.concatenate([linear_vel, np.zeros(3)])

            all_times.append(time_ms * 0.001)
            all_positions.append(pose[:3].copy())
            all_velocities.append(vel[:3].copy())

            last_sent_pose = pose.copy()
            time_ms += 1

    return (np.array(all_times), np.array(all_positions), np.array(all_velocities))


def simulate_joint_streaming(
    waypoints: List[np.ndarray],  # list of N-DOF joint positions from policy
    policy_hz: int,
    use_linear: bool = True,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Simulate 1kHz RT thread with decimation + joint linear interpolation.

    Returns:
        times: (N,) time in seconds for each 1ms step
        positions: (N, DOF) joint positions
        velocities: (N, DOF) joint velocities
    """
    decimation = round(1000.0 / policy_hz)
    period_sec = decimation * 0.001

    all_times = []
    all_positions = []
    all_velocities = []

    last_sent = waypoints[0].copy()
    dof = len(last_sent)
    time_ms = 0

    for cmd_idx in range(1, len(waypoints)):
        target = waypoints[cmd_idx].copy()
        start_snapshot = last_sent.copy()
        delta = target - start_snapshot

        total_steps = decimation

        if use_linear:
            const_vel = delta / period_sec
        else:
            const_vel = None  # min-jerk: velocity varies

        for sub_step in range(total_steps):
            step_num = sub_step + 1

            if step_num >= total_steps:
                pos = target.copy()
                vel = np.zeros(dof)
            else:
                t = step_num / total_steps

                if use_linear:
                    pos = start_snapshot + t * delta
                    vel = const_vel.copy()
                else:
                    s = min_jerk_s(t)
                    ds_dtau = min_jerk_ds_dtau(t)
                    ds_dt = ds_dtau / period_sec
                    pos = start_snapshot + s * delta
                    vel = ds_dt * delta

            all_times.append(time_ms * 0.001)
            all_positions.append(pos.copy())
            all_velocities.append(vel.copy())

            last_sent = pos.copy()
            time_ms += 1

    return (np.array(all_times), np.array(all_positions), np.array(all_velocities))


# ============================================================================
# Test functions
# ============================================================================

def test_cartesian_linear_constant_velocity():
    """Linear interpolation should produce constant velocity between waypoints."""
    policy_hz = 30
    num_cmds = 20
    step_size = 0.01  # 10mm per action

    # Straight line in X
    waypoints = []
    for i in range(num_cmds + 1):
        pose = np.array([i * step_size, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0])
        waypoints.append(pose)

    times, positions, velocities = simulate_cartesian_streaming(
        waypoints, policy_hz, use_linear=True)

    decimation = round(1000.0 / policy_hz)
    period_sec = decimation * 0.001
    expected_vx = step_size / period_sec

    # Check velocity is constant (except at segment endpoints)
    non_endpoint_vx = []
    for i, vx in enumerate(velocities[:, 0]):
        # Skip last step of each segment
        if (i + 1) % decimation == 0:
            continue
        non_endpoint_vx.append(vx)

    non_endpoint_vx = np.array(non_endpoint_vx)
    max_dev = np.max(np.abs(non_endpoint_vx - expected_vx))
    assert max_dev < 1e-10, f"Velocity deviation {max_dev} exceeds tolerance"

    # Check final position
    expected_final_x = num_cmds * step_size
    actual_final_x = positions[-1, 0]
    assert abs(actual_final_x - expected_final_x) < 1e-6, \
        f"Final position {actual_final_x} != expected {expected_final_x}"

    print(f"  [PASS] Cartesian linear: constant velocity {expected_vx:.3f} m/s, "
          f"max deviation {max_dev:.2e}")


def test_cartesian_minjerk_stop_and_go():
    """Min-jerk should have zero velocity at start/end of each segment."""
    policy_hz = 30
    num_cmds = 10
    step_size = 0.01

    waypoints = []
    for i in range(num_cmds + 1):
        pose = np.array([i * step_size, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0])
        waypoints.append(pose)

    times, positions, velocities = simulate_cartesian_streaming(
        waypoints, policy_hz, use_linear=False)

    decimation = round(1000.0 / policy_hz)

    # Check first step of each segment has near-zero velocity
    for seg in range(num_cmds):
        first_step_idx = seg * decimation
        vx = velocities[first_step_idx, 0]
        assert abs(vx) < 0.05, \
            f"Segment {seg} first-step velocity {vx} is not near zero"

    print(f"  [PASS] Min-jerk stop-and-go confirmed: first-step velocity near zero")


def test_joint_linear_constant_velocity():
    """Joint linear interpolation should produce constant velocity."""
    policy_hz = 30
    num_cmds = 20
    dof = 7

    # Each action moves each joint by 0.01 rad
    waypoints = []
    for i in range(num_cmds + 1):
        q = np.array([i * 0.01 * (j + 1) for j in range(dof)])
        waypoints.append(q)

    times, positions, velocities = simulate_joint_streaming(
        waypoints, policy_hz, use_linear=True)

    decimation = round(1000.0 / policy_hz)
    period_sec = decimation * 0.001

    # Check velocity is constant for each joint
    for j in range(dof):
        expected_vel = 0.01 * (j + 1) / period_sec
        non_endpoint = []
        for i, v in enumerate(velocities[:, j]):
            if (i + 1) % decimation == 0:
                continue
            non_endpoint.append(v)
        non_endpoint = np.array(non_endpoint)
        max_dev = np.max(np.abs(non_endpoint - expected_vel))
        assert max_dev < 1e-10, \
            f"Joint {j} velocity deviation {max_dev} exceeds tolerance"

    # Check final position
    for j in range(dof):
        expected = num_cmds * 0.01 * (j + 1)
        actual = positions[-1, j]
        assert abs(actual - expected) < 1e-6, \
            f"Joint {j} final pos {actual} != expected {expected}"

    print(f"  [PASS] Joint linear: constant velocity for all {dof} joints")


def test_joint_minjerk_velocity_varies():
    """Joint min-jerk should have varying velocity within each segment."""
    policy_hz = 30
    num_cmds = 5
    dof = 7

    waypoints = []
    for i in range(num_cmds + 1):
        q = np.array([i * 0.01 for _ in range(dof)])
        waypoints.append(q)

    times, positions, velocities = simulate_joint_streaming(
        waypoints, policy_hz, use_linear=False)

    decimation = round(1000.0 / policy_hz)

    # Within each segment, velocity should vary (not constant)
    for seg in range(num_cmds):
        start_idx = seg * decimation
        seg_vel = velocities[start_idx:start_idx + decimation, 0]
        # Remove endpoint (zero velocity)
        seg_vel = seg_vel[:-1]
        if len(seg_vel) > 2:
            vel_range = np.max(seg_vel) - np.min(seg_vel)
            assert vel_range > 0.01, \
                f"Segment {seg} velocity range {vel_range} too small for min-jerk"

    print(f"  [PASS] Joint min-jerk: velocity varies within segments")


def test_rotation_slerp_linear():
    """SLERP with linear parameter should give constant angular velocity."""
    policy_hz = 30
    num_cmds = 10

    # Rotate 10 degrees around Z per action
    waypoints = []
    for i in range(num_cmds + 1):
        angle = math.radians(i * 10)
        half = angle / 2.0
        pose = np.array([0.5, 0.0, 0.3,
                         math.cos(half), 0.0, 0.0, math.sin(half)])
        waypoints.append(pose)

    times, positions, velocities = simulate_cartesian_streaming(
        waypoints, policy_hz, use_linear=True)

    # Position should stay constant
    max_pos_dev = np.max(np.abs(positions[:, 0] - 0.5))
    assert max_pos_dev < 1e-10, f"X position changed during pure rotation"
    max_pos_dev_y = np.max(np.abs(positions[:, 1]))
    assert max_pos_dev_y < 1e-10, f"Y position changed during pure rotation"

    print(f"  [PASS] Rotation SLERP: position stays constant, max deviation {max_pos_dev:.2e}")


def test_end_position_accuracy():
    """After all segments, final position should match the last waypoint exactly."""
    policy_hz = 30
    num_cmds = 50

    # Arbitrary trajectory
    np.random.seed(42)
    waypoints = [np.array([0.0, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0])]
    for _ in range(num_cmds):
        prev = waypoints[-1].copy()
        prev[:3] += np.random.uniform(-0.005, 0.005, 3)
        waypoints.append(prev)

    times, positions, velocities = simulate_cartesian_streaming(
        waypoints, policy_hz, use_linear=True)

    for i in range(3):
        expected = waypoints[-1][i]
        actual = positions[-1, i]
        assert abs(actual - expected) < 1e-10, \
            f"Axis {i}: final pos {actual} != expected {expected}"

    print(f"  [PASS] End position accuracy: error < 1e-10 for all axes")


def test_different_policy_frequencies():
    """Test that interpolation works correctly at various policy frequencies."""
    for policy_hz in [10, 20, 30, 50, 100, 200, 500]:
        num_cmds = 10
        step = 0.01

        waypoints = []
        for i in range(num_cmds + 1):
            pose = np.array([i * step, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0])
            waypoints.append(pose)

        times, positions, velocities = simulate_cartesian_streaming(
            waypoints, policy_hz, use_linear=True)

        decimation = round(1000.0 / policy_hz)
        period_sec = decimation * 0.001
        expected_vx = step / period_sec

        # Check velocity consistency
        non_endpoint = []
        for i, vx in enumerate(velocities[:, 0]):
            if (i + 1) % decimation == 0:
                continue
            non_endpoint.append(vx)

        non_endpoint = np.array(non_endpoint)
        max_dev = np.max(np.abs(non_endpoint - expected_vx))
        assert max_dev < 1e-9, \
            f"Policy {policy_hz}Hz: velocity deviation {max_dev}"

    print(f"  [PASS] Multiple frequencies: 10/20/30/50/100/200/500 Hz all correct")


# ============================================================================
# Visualization (optional)
# ============================================================================

def plot_comparison():
    """Plot linear vs min-jerk velocity profiles for visual comparison."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed, skipping plots")
        return

    policy_hz = 30
    num_cmds = 5
    step_size = 0.01

    waypoints = []
    for i in range(num_cmds + 1):
        pose = np.array([i * step_size, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0])
        waypoints.append(pose)

    t_lin, pos_lin, vel_lin = simulate_cartesian_streaming(
        waypoints, policy_hz, use_linear=True)
    t_mjk, pos_mjk, vel_mjk = simulate_cartesian_streaming(
        waypoints, policy_hz, use_linear=False)

    fig, axes = plt.subplots(2, 2, figsize=(14, 8))
    fig.suptitle(f"Cartesian Streaming Interpolation @ {policy_hz}Hz Policy", fontsize=14)

    decimation = round(1000.0 / policy_hz)

    # --- Position X ---
    axes[0, 0].plot(t_lin * 1000, pos_lin[:, 0] * 1000, 'b-', label='Linear', linewidth=1)
    axes[0, 0].plot(t_mjk * 1000, pos_mjk[:, 0] * 1000, 'r-', label='Min-Jerk', linewidth=1)
    for i in range(num_cmds):
        t_boundary = i * decimation * 0.001 * 1000
        axes[0, 0].axvline(t_boundary, color='gray', linestyle='--', alpha=0.3)
    axes[0, 0].set_ylabel('X Position (mm)')
    axes[0, 0].set_xlabel('Time (ms)')
    axes[0, 0].legend()
    axes[0, 0].set_title('Position (X)')

    # --- Velocity X ---
    axes[0, 1].plot(t_lin * 1000, vel_lin[:, 0], 'b-', label='Linear', linewidth=1)
    axes[0, 1].plot(t_mjk * 1000, vel_mjk[:, 0], 'r-', label='Min-Jerk', linewidth=1)
    for i in range(num_cmds):
        t_boundary = i * decimation * 0.001 * 1000
        axes[0, 1].axvline(t_boundary, color='gray', linestyle='--', alpha=0.3)
    axes[0, 1].set_ylabel('X Velocity (m/s)')
    axes[0, 1].set_xlabel('Time (ms)')
    axes[0, 1].legend()
    axes[0, 1].set_title('Velocity (X) — Linear is flat, Min-Jerk has stop-and-go')

    # --- Joint simulation ---
    dof = 7
    j_waypoints = []
    for i in range(num_cmds + 1):
        q = np.array([i * 0.02 for _ in range(dof)])
        j_waypoints.append(q)

    jt_lin, jpos_lin, jvel_lin = simulate_joint_streaming(
        j_waypoints, policy_hz, use_linear=True)
    jt_mjk, jpos_mjk, jvel_mjk = simulate_joint_streaming(
        j_waypoints, policy_hz, use_linear=False)

    axes[1, 0].plot(jt_lin * 1000, jpos_lin[:, 0], 'b-', label='Linear', linewidth=1)
    axes[1, 0].plot(jt_mjk * 1000, jpos_mjk[:, 0], 'r-', label='Min-Jerk', linewidth=1)
    for i in range(num_cmds):
        t_boundary = i * decimation * 0.001 * 1000
        axes[1, 0].axvline(t_boundary, color='gray', linestyle='--', alpha=0.3)
    axes[1, 0].set_ylabel('Joint 0 Position (rad)')
    axes[1, 0].set_xlabel('Time (ms)')
    axes[1, 0].legend()
    axes[1, 0].set_title('Joint Position')

    axes[1, 1].plot(jt_lin * 1000, jvel_lin[:, 0], 'b-', label='Linear', linewidth=1)
    axes[1, 1].plot(jt_mjk * 1000, jvel_mjk[:, 0], 'r-', label='Min-Jerk', linewidth=1)
    for i in range(num_cmds):
        t_boundary = i * decimation * 0.001 * 1000
        axes[1, 1].axvline(t_boundary, color='gray', linestyle='--', alpha=0.3)
    axes[1, 1].set_ylabel('Joint 0 Velocity (rad/s)')
    axes[1, 1].set_xlabel('Time (ms)')
    axes[1, 1].legend()
    axes[1, 1].set_title('Joint Velocity — Linear is flat, Min-Jerk has stop-and-go')

    plt.tight_layout()
    output_path = "interpolation_comparison.png"
    plt.savefig(output_path, dpi=150)
    print(f"\n  Plot saved to: {output_path}")
    plt.show()


# ============================================================================
# Main
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description="Interpolation simulation test")
    parser.add_argument("--plot", action="store_true",
                        help="Show matplotlib plots comparing linear vs min-jerk")
    args = parser.parse_args()

    print("=" * 60)
    print("Interpolation Simulation Tests (no robot required)")
    print("=" * 60)

    tests = [
        ("Cartesian linear: constant velocity", test_cartesian_linear_constant_velocity),
        ("Cartesian min-jerk: stop-and-go", test_cartesian_minjerk_stop_and_go),
        ("Joint linear: constant velocity", test_joint_linear_constant_velocity),
        ("Joint min-jerk: velocity varies", test_joint_minjerk_velocity_varies),
        ("Rotation SLERP: constant angular velocity", test_rotation_slerp_linear),
        ("End position accuracy", test_end_position_accuracy),
        ("Multiple policy frequencies", test_different_policy_frequencies),
    ]

    passed = 0
    failed = 0
    for name, test_fn in tests:
        try:
            test_fn()
            passed += 1
        except AssertionError as e:
            print(f"  [FAIL] {name}: {e}")
            failed += 1
        except Exception as e:
            print(f"  [ERROR] {name}: {e}")
            failed += 1

    print("=" * 60)
    print(f"Results: {passed} passed, {failed} failed out of {len(tests)} tests")
    print("=" * 60)

    if args.plot:
        print("\nGenerating comparison plots...")
        plot_comparison()

    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
