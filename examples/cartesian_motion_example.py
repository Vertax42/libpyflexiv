"""
cartesian_motion_example.py
---------------------------
Demonstrates real-time Cartesian pure motion control with flexiv_rt.
The robot holds its current TCP pose, then performs a small linear
sine-sweep along the Y axis.

Usage:
    python cartesian_motion_example.py <robot_sn>
    e.g.  python cartesian_motion_example.py Rizon4s-123456
"""

import sys
import math
import time
import logging
import flexiv_rt as frt

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(message)s",
)
log = logging.getLogger("CartesianExample")

# Sine-sweep parameters
SWING_AMP  = 0.05   # m
SWING_FREQ = 0.2    # Hz
LOOP_DT    = 0.001  # s
RUN_TIME   = 15.0   # s


def main():
    if len(sys.argv) < 2:
        print("Usage: python cartesian_motion_example.py <robot_sn>")
        sys.exit(1)

    robot_sn = sys.argv[1]

    # Connect and enable robot
    with frt.Robot(robot_sn, connect_retries=10, retry_interval_sec=1.0) as robot:
        if robot.fault():
            print("Fault detected, clearing ...")
            if not robot.ClearFault(timeout_sec=30):
                raise RuntimeError("Cannot clear fault")

        robot.Enable()
        while not robot.operational():
            time.sleep(1.0)
        log.info("Robot operational")

        # Move to home pose
        robot.SwitchMode(frt.Mode.NRT_PLAN_EXECUTION)
        robot.ExecutePlan("PLAN-Home")
        while robot.busy():
            time.sleep(1.0)
        log.info("Home pose reached")

        # Zero the F/T sensor before Cartesian control
        robot.SwitchMode(frt.Mode.NRT_PRIMITIVE_EXECUTION)
        robot.ExecutePrimitive("ZeroFTSensor", {})
        log.info("Zeroing F/T sensor – keep robot free of contact ...")
        time.sleep(3.0)  # Allow zeroing to complete

        # Switch to RT Cartesian motion-force mode
        robot.SwitchMode(frt.Mode.RT_CARTESIAN_MOTION_FORCE)

        # Pure motion control – all axes motion-controlled, none force-controlled
        robot.SetForceControlAxis([False, False, False, False, False, False])

        init_pose = list(robot.states().tcp_pose)
        log.info(f"Initial TCP pose: {init_pose}")

        # Start RT control
        with robot.start_cartesian_control() as cc:
            # Hold initial pose briefly to let RT thread stabilize
            cc.set_target_pose(init_pose)
            time.sleep(0.5)

            t0 = time.monotonic()
            loop_count = 0

            while cc.is_running():
                elapsed = time.monotonic() - t0
                if elapsed >= RUN_TIME:
                    break

                # Sine-sweep along Y axis
                target_pose = list(init_pose)
                target_pose[1] = init_pose[1] + SWING_AMP * math.sin(
                    2 * math.pi * SWING_FREQ * elapsed)
                cc.set_target_pose(target_pose)

                # Read state periodically
                if loop_count % 500 == 0:
                    state = cc.get_state()
                    log.info(f"t={elapsed:.1f}s  tcp_y={state.tcp_pose[1]:.4f} m  "
                             f"ext_fz={state.ext_wrench_in_tcp[2]:.2f} N")

                loop_count += 1
                time.sleep(LOOP_DT)

            # Return to initial pose
            cc.set_target_pose(init_pose)
            time.sleep(2.0)

    log.info("Done.")


if __name__ == "__main__":
    main()
