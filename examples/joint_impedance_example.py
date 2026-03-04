"""
joint_impedance_example.py
--------------------------
Demonstrates real-time joint impedance control with flexiv_rt.
The robot holds its current joint positions, then performs a small
sine-sweep on all joints.

Usage:
    python joint_impedance_example.py <robot_sn>
    e.g.  python joint_impedance_example.py Rizon4s-123456
"""

import sys
import math
import time
import flexiv_rt as frt

# Sine-sweep parameters
SINE_AMP  = 0.05    # rad
SINE_FREQ = 0.3     # Hz
LOOP_DT   = 0.001   # s  (matches 1 kHz RT loop)
RUN_TIME  = 20.0    # s


def main():
    if len(sys.argv) < 2:
        print("Usage: python joint_impedance_example.py <robot_sn>")
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
        print("Robot operational")

        # Move to home pose (NRT plan execution)
        robot.SwitchMode(frt.Mode.NRT_PLAN_EXECUTION)
        robot.ExecutePlan("PLAN-Home")
        while robot.busy():
            time.sleep(1.0)
        print("Home pose reached")

        # Switch to RT joint impedance mode
        robot.SwitchMode(frt.Mode.RT_JOINT_IMPEDANCE)
        robot.SetJointImpedance(robot.info().K_q_nom)

        init_q = list(robot.states().q)
        print(f"Initial joint positions: {init_q}")

        # Start RT control
        with robot.start_joint_impedance_control() as jc:
            # Hold current position first
            jc.set_target_joints(init_q)
            time.sleep(1.0)

            t0 = time.monotonic()
            loop_count = 0

            while jc.is_running():
                elapsed = time.monotonic() - t0
                if elapsed >= RUN_TIME:
                    break

                # Sine-sweep all joints
                target_q = [
                    q0 + SINE_AMP * math.sin(2 * math.pi * SINE_FREQ * elapsed)
                    for q0 in init_q
                ]
                jc.set_target_joints(target_q)

                # Read state periodically
                if loop_count % 500 == 0:
                    state = jc.get_state()
                    q_str = ", ".join(f"{q:.4f}" for q in state.q)
                    print(f"  t={elapsed:.1f}s  q=[{q_str}] rad")

                loop_count += 1
                time.sleep(LOOP_DT)

            # Return to initial position
            jc.set_target_joints(init_q)
            time.sleep(1.0)

    print("Done.")


if __name__ == "__main__":
    main()
