"""
Flexiv RT integration tests — requires a real robot.

Run with:
    sudo -E pytest tests/python/ --robot-sn Rizon4s-XXXXXX -v -s --timeout=120
"""

import math
import time

import pytest

import flexiv_bindings as fb


# ===========================================================================
# Read-only tests (no motion)
# ===========================================================================


class TestConnection:
    def test_connection(self, robot):
        assert robot.connected()
        assert robot.operational()
        assert not robot.fault()

    def test_robot_info(self, robot):
        info = robot.info()
        assert info.DoF == 7
        assert len(info.K_q_nom) == 7
        assert len(info.serial_num) > 0

    def test_states_readable(self, robot):
        s = robot.states()
        assert len(s.q) == 7
        assert len(s.tcp_pose) == 7
        # All values should be finite
        for v in s.q:
            assert math.isfinite(v)
        for v in s.tcp_pose:
            assert math.isfinite(v)


# ===========================================================================
# Mode switching (no motion)
# ===========================================================================


class TestModeSwitching:
    def test_mode_switch_joint(self, ensure_no_fault):
        robot = ensure_no_fault
        robot.SwitchMode(fb.Mode.RT_JOINT_IMPEDANCE)
        time.sleep(0.5)
        assert robot.mode() == fb.Mode.RT_JOINT_IMPEDANCE

    def test_mode_switch_cart(self, ensure_no_fault):
        robot = ensure_no_fault
        robot.SwitchMode(fb.Mode.RT_CARTESIAN_MOTION_FORCE)
        time.sleep(0.5)
        assert robot.mode() == fb.Mode.RT_CARTESIAN_MOTION_FORCE


# ===========================================================================
# Joint impedance RT control
# ===========================================================================


class TestJointImpedance:
    def test_joint_hold(self, ensure_no_fault):
        """Start RT joint control and hold initial position for 2s."""
        robot = ensure_no_fault
        robot.SwitchMode(fb.Mode.RT_JOINT_IMPEDANCE)
        time.sleep(0.5)

        with robot.start_joint_impedance_control() as jc:
            init_state = jc.get_state()
            init_q = list(init_state.q)

            # Hold for 2 seconds
            for _ in range(200):
                jc.set_target_joints(init_q)
                time.sleep(0.01)

            final_state = jc.get_state()
            for i in range(7):
                assert abs(final_state.q[i] - init_q[i]) < 0.001, (
                    f"Joint {i} drifted: {final_state.q[i]:.4f} vs {init_q[i]:.4f}"
                )

        assert not robot.fault()

    def test_joint_small_motion(self, ensure_no_fault):
        """Move joint[3] with a small sinusoidal motion."""
        robot = ensure_no_fault
        robot.SwitchMode(fb.Mode.RT_JOINT_IMPEDANCE)
        time.sleep(0.5)

        with robot.start_joint_impedance_control() as jc:
            init_state = jc.get_state()
            init_q = list(init_state.q)
            amplitude = 0.02  # rad

            # 2 seconds at 100Hz
            for step in range(200):
                t = step * 0.01
                target_q = init_q[:]
                target_q[3] = init_q[3] + amplitude * math.sin(2 * math.pi * 0.5 * t)
                jc.set_target_joints(target_q)
                time.sleep(0.01)

            # Verify tracking on the last step
            state = jc.get_state()
            expected = init_q[3] + amplitude * math.sin(2 * math.pi * 0.5 * 2.0)
            assert abs(state.q[3] - expected) < 0.005, (
                f"Joint 3 tracking error: {state.q[3]:.4f} vs {expected:.4f}"
            )

        assert not robot.fault()

    def test_joint_state_fields(self, ensure_no_fault):
        """Verify JointState fields are populated correctly."""
        robot = ensure_no_fault
        robot.SwitchMode(fb.Mode.RT_JOINT_IMPEDANCE)
        time.sleep(0.5)

        with robot.start_joint_impedance_control() as jc:
            init_q = list(jc.get_state().q)
            # Send a few commands to let the RT loop stabilize
            for _ in range(50):
                jc.set_target_joints(init_q)
                time.sleep(0.01)

            state = jc.get_state()
            assert len(state.q) == 7
            assert len(state.dq) == 7
            assert len(state.tau) == 7
            assert len(state.tau_ext) == 7
            assert len(state.tcp_pose) == 7

            # Joints should be within limits
            info = robot.info()
            for i in range(7):
                assert info.q_min[i] <= state.q[i] <= info.q_max[i], (
                    f"Joint {i} out of limits: {state.q[i]}"
                )

        assert not robot.fault()


# ===========================================================================
# Cartesian RT control
# ===========================================================================


class TestCartesian:
    def test_cartesian_hold(self, ensure_no_fault):
        """Start RT Cartesian control and hold initial pose for 2s."""
        robot = ensure_no_fault
        robot.SwitchMode(fb.Mode.RT_CARTESIAN_MOTION_FORCE)
        time.sleep(0.5)
        robot.SetForceControlAxis([False] * 6)

        with robot.start_cartesian_control() as cc:
            init_state = cc.get_state()
            init_pose = list(init_state.tcp_pose)

            for _ in range(200):
                cc.set_target_pose(init_pose)
                time.sleep(0.01)

            final_state = cc.get_state()
            # Position error < 0.5mm
            pos_err = math.sqrt(sum(
                (final_state.tcp_pose[i] - init_pose[i]) ** 2 for i in range(3)
            ))
            assert pos_err < 0.0005, f"TCP position drift: {pos_err * 1000:.2f} mm"

        assert not robot.fault()

    def test_cartesian_small(self, ensure_no_fault):
        """Move TCP along Y axis with a small sinusoidal motion."""
        robot = ensure_no_fault
        robot.SwitchMode(fb.Mode.RT_CARTESIAN_MOTION_FORCE)
        time.sleep(0.5)
        robot.SetForceControlAxis([False] * 6)

        with robot.start_cartesian_control() as cc:
            init_state = cc.get_state()
            init_pose = list(init_state.tcp_pose)
            amplitude = 0.005  # 5mm

            # 3 seconds at 100Hz, 0.2Hz sine
            for step in range(300):
                t = step * 0.01
                target_pose = init_pose[:]
                target_pose[1] = init_pose[1] + amplitude * math.sin(
                    2 * math.pi * 0.2 * t
                )
                cc.set_target_pose(target_pose)
                time.sleep(0.01)

            # Verify tracking
            state = cc.get_state()
            expected_y = init_pose[1] + amplitude * math.sin(
                2 * math.pi * 0.2 * 3.0
            )
            assert abs(state.tcp_pose[1] - expected_y) < 0.002, (
                f"Y tracking error: {(state.tcp_pose[1] - expected_y) * 1000:.2f} mm"
            )

        assert not robot.fault()

    def test_cartesian_state(self, ensure_no_fault):
        """Verify CartesianState fields are populated correctly."""
        robot = ensure_no_fault
        robot.SwitchMode(fb.Mode.RT_CARTESIAN_MOTION_FORCE)
        time.sleep(0.5)
        robot.SetForceControlAxis([False] * 6)

        with robot.start_cartesian_control() as cc:
            init_pose = list(cc.get_state().tcp_pose)
            for _ in range(50):
                cc.set_target_pose(init_pose)
                time.sleep(0.01)

            state = cc.get_state()
            assert len(state.tcp_pose) == 7
            assert len(state.tcp_vel) == 6
            assert len(state.ext_wrench_in_tcp) == 6

            # Quaternion norm should be ~1.0
            quat = state.tcp_pose[3:7]
            quat_norm = math.sqrt(sum(q * q for q in quat))
            assert abs(quat_norm - 1.0) < 0.001, f"Quat norm: {quat_norm}"

        assert not robot.fault()


# ===========================================================================
# Safety features
# ===========================================================================


class TestSafety:
    def test_emergency_stop(self, ensure_no_fault):
        """Trigger e-stop and verify RT loop stops."""
        robot = ensure_no_fault
        robot.SwitchMode(fb.Mode.RT_JOINT_IMPEDANCE)
        time.sleep(0.5)

        with robot.start_joint_impedance_control() as jc:
            init_q = list(jc.get_state().q)
            for _ in range(50):
                jc.set_target_joints(init_q)
                time.sleep(0.01)

            assert jc.is_running()
            jc.trigger_estop()
            time.sleep(0.1)
            assert not jc.is_running()

    def test_context_manager(self, ensure_no_fault):
        """Verify context manager cleans up without fault."""
        robot = ensure_no_fault
        robot.SwitchMode(fb.Mode.RT_JOINT_IMPEDANCE)
        time.sleep(0.5)

        with robot.start_joint_impedance_control() as jc:
            init_q = list(jc.get_state().q)
            for _ in range(50):
                jc.set_target_joints(init_q)
                time.sleep(0.01)

        # After exiting context, robot should have no fault
        time.sleep(0.5)
        assert not robot.fault()

    def test_command_timeout(self, ensure_no_fault):
        """Send one command then sleep — robot should hold, no fault."""
        robot = ensure_no_fault
        robot.SwitchMode(fb.Mode.RT_JOINT_IMPEDANCE)
        time.sleep(0.5)

        with robot.start_joint_impedance_control() as jc:
            init_q = list(jc.get_state().q)
            jc.set_target_joints(init_q)
            # Sleep past timeout (500ms) but not too long
            time.sleep(0.8)

            # Robot should still be holding, no fault
            assert not robot.fault()
            state = jc.get_state()
            for i in range(7):
                assert abs(state.q[i] - init_q[i]) < 0.005, (
                    f"Joint {i} drifted during timeout hold"
                )


# ===========================================================================
# lerobot compatibility workflows
# ===========================================================================


class TestLeRobotCompatibility:
    def test_lerobot_joint_workflow(self, ensure_no_fault):
        """Simulate lerobot 100-step joint control loop."""
        robot = ensure_no_fault
        robot.SwitchMode(fb.Mode.RT_JOINT_IMPEDANCE)
        time.sleep(0.5)

        init_q = list(robot.states().q)

        with robot.start_joint_impedance_control() as jc:
            for _ in range(100):
                state = jc.get_state()
                obs_q = list(state.q)       # get_observation
                target_q = obs_q[:]         # send_action (hold)
                jc.set_target_joints(target_q)
                time.sleep(0.01)            # 100Hz policy rate

        assert not robot.fault()
        # Verify no drift
        final_q = list(robot.states().q)
        for i in range(7):
            assert abs(final_q[i] - init_q[i]) < 0.005, (
                f"Joint {i} drifted in lerobot workflow"
            )

    def test_lerobot_cartesian_workflow(self, ensure_no_fault):
        """Simulate lerobot 100-step Cartesian control loop."""
        robot = ensure_no_fault
        robot.SwitchMode(fb.Mode.RT_CARTESIAN_MOTION_FORCE)
        time.sleep(0.5)
        robot.SetForceControlAxis([False] * 6)

        init_pose = list(robot.states().tcp_pose)

        with robot.start_cartesian_control() as cc:
            for _ in range(100):
                state = cc.get_state()
                obs_pose = list(state.tcp_pose)   # get_observation
                cc.set_target_pose(obs_pose)      # send_action (hold)
                time.sleep(0.01)                  # 100Hz policy rate

        assert not robot.fault()
        # Verify no drift
        final_pose = list(robot.states().tcp_pose)
        pos_err = math.sqrt(sum(
            (final_pose[i] - init_pose[i]) ** 2 for i in range(3)
        ))
        assert pos_err < 0.001, f"TCP drifted {pos_err * 1000:.2f} mm in lerobot workflow"
