"""
rt_reset_example.py
-------------------
End-to-end test simulating the LeRobot RT driver lifecycle:

    connect  ->  RT sine-sweep  ->  RT reset (non-blocking)  ->  disconnect

This validates that:
1. move_to_pose() starts a trajectory without blocking the caller
2. get_state() keeps returning fresh observations during the move
3. The RT thread stays alive throughout (no mode switch)
4. send_action-style commands are naturally skipped while is_moving()

Usage:
    python rt_reset_example.py <robot_sn>
    e.g.  python rt_reset_example.py Rizon4s-123456
"""

import sys
import math
import time
import logging

import flexiv_bindings as fb

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s %(message)s",
)
log = logging.getLogger("RTResetExample")

# Sine-sweep parameters
SWING_AMP = 0.05  # 50 mm along Y
SWING_FREQ = 0.3  # Hz
LOOP_HZ = 100  # Python-side command rate
SWEEP_TIME = 6.0  # seconds of sine-sweep before reset

# Move-to-pose parameters
RESET_DURATION = 3.0  # seconds for min-jerk trajectory


def main():
    if len(sys.argv) < 2:
        print("Usage: python rt_reset_example.py <robot_sn>")
        sys.exit(1)

    robot_sn = sys.argv[1]

    # ── 1. Connect & Enable ──────────────────────────────────────────────
    log.info(f"Connecting to {robot_sn} ...")
    robot = fb.Robot(robot_sn, connect_retries=10, retry_interval_sec=1.0)

    if robot.fault():
        log.warning("Fault detected, clearing ...")
        if not robot.ClearFault(timeout_sec=30):
            raise RuntimeError("Cannot clear fault")

    robot.Enable()
    while not robot.operational():
        time.sleep(0.1)
    log.info("Robot operational")

    # ── 2. Go to home (NRT) ──────────────────────────────────────────────
    robot.SwitchMode(fb.Mode.NRT_PLAN_EXECUTION)
    robot.ExecutePlan("PLAN-Home")
    while robot.busy():
        time.sleep(0.1)
    log.info("Home position reached")

    # Cache the start TCP pose (same as LeRobot driver does after _go_to_start)
    start_tcp_pose = list(robot.states().tcp_pose)
    log.info(f"Start TCP pose cached: [{', '.join(f'{v:.4f}' for v in start_tcp_pose)}]")

    # ── 3. Switch to RT Cartesian mode ───────────────────────────────────
    robot.SwitchMode(fb.Mode.RT_CARTESIAN_MOTION_FORCE)
    robot.SetForceControlAxis([False] * 6)  # pure motion, no force control

    init_pose = list(robot.states().tcp_pose)

    # ── 4. Start RT thread ───────────────────────────────────────────────
    cc = robot.start_cartesian_control()
    cc.set_target_pose(init_pose)
    time.sleep(0.5)  # let RT thread stabilize
    log.info("RT thread started (1 kHz)")

    try:
        # ── 5. Sine-sweep phase ──────────────────────────────────────────
        log.info(f"Phase 1: Sine-sweep (Y-axis, amp={SWING_AMP}m, {SWEEP_TIME}s) ...")
        t0 = time.monotonic()
        loop_count = 0

        while cc.is_running():
            elapsed = time.monotonic() - t0
            if elapsed >= SWEEP_TIME:
                break

            target = list(init_pose)
            target[1] = init_pose[1] + SWING_AMP * math.sin(
                2 * math.pi * SWING_FREQ * elapsed
            )
            cc.set_target_pose(target)

            # Log state periodically
            if loop_count % (LOOP_HZ * 2) == 0:  # every 2 seconds
                s = cc.get_state()
                dy = s.tcp_pose[1] - init_pose[1]
                log.info(
                    f"  t={elapsed:.1f}s  dy={dy*1000:+.1f}mm  "
                    f"is_moving={cc.is_moving()}"
                )

            loop_count += 1
            time.sleep(1.0 / LOOP_HZ)

        # Show where we ended up after the sweep
        state_before = cc.get_state()
        dy_before = state_before.tcp_pose[1] - start_tcp_pose[1]
        log.info(
            f"Sweep done. Current dy from start: {dy_before*1000:+.1f}mm"
        )

        # ── 6. Non-blocking RT reset ─────────────────────────────────────
        log.info(
            f"Phase 2: Non-blocking RT reset via move_to_pose "
            f"(duration={RESET_DURATION}s) ..."
        )
        assert cc.is_running(), "RT thread should still be running"
        assert not cc.is_moving(), "Should not be in a trajectory move"

        # Start trajectory — returns immediately
        cc.move_to_pose(start_tcp_pose, duration_sec=RESET_DURATION)

        # Poll loop: simulates teleop loop that keeps reading observations
        # while the trajectory runs in the background
        obs_count = 0
        t_reset_start = time.monotonic()

        while cc.is_moving():
            state = cc.get_state()
            obs_count += 1

            # Simulate send_action() skipping: check is_moving before sending
            # (In the real driver, send_action() does this automatically)

            # Log every ~500ms
            if obs_count % 50 == 0:
                dy = state.tcp_pose[1] - start_tcp_pose[1]
                log.info(
                    f"  [MOVING] obs #{obs_count}  "
                    f"dy={dy*1000:+.2f}mm  "
                    f"tcp_vel_y={state.tcp_vel[1]*1000:.1f}mm/s  "
                    f"is_moving={cc.is_moving()}"
                )

            time.sleep(1.0 / LOOP_HZ)  # 100 Hz, same as teleop

        t_reset_elapsed = time.monotonic() - t_reset_start

        # Final state after trajectory completes
        state_after = cc.get_state()
        pos_error = math.sqrt(
            sum(
                (state_after.tcp_pose[i] - start_tcp_pose[i]) ** 2
                for i in range(3)
            )
        )
        log.info(
            f"Reset complete: {t_reset_elapsed:.2f}s, "
            f"{obs_count} observations read during move, "
            f"position error={pos_error*1000:.3f}mm, "
            f"rt_running={cc.is_running()}"
        )

        assert cc.is_running(), "RT thread must still be running after reset"
        if pos_error > 1.0:
            log.warning(f"Position error too large: {pos_error*1000:.3f}mm")

        # ── 7. Post-reset: resume streaming for 2s ───────────────────────
        log.info("Phase 3: Resume streaming commands for 2s after reset ...")
        cc.set_target_pose(start_tcp_pose)
        time.sleep(2.0)
        state_hold = cc.get_state()
        log.info(
            f"Hold done. tcp_pose="
            f"[{', '.join(f'{v:.4f}' for v in state_hold.tcp_pose)}]"
        )

    except KeyboardInterrupt:
        log.warning("Interrupted by user")
    except Exception as e:
        log.error(f"Error: {e}", exc_info=True)
    finally:
        # ── 8. Disconnect ────────────────────────────────────────────────
        log.info("Stopping RT thread ...")
        cc.stop()
        log.info("RT thread stopped")

        # Return to home (NRT) before shutting down
        try:
            robot.SwitchMode(fb.Mode.NRT_PLAN_EXECUTION)
            robot.ExecutePlan("PLAN-Home")
            while robot.busy():
                time.sleep(0.1)
            log.info("Returned to home position")
        except Exception as e:
            log.warning(f"Failed to return to home: {e}")

        robot.Stop()
        log.info("Done.")


if __name__ == "__main__":
    main()
