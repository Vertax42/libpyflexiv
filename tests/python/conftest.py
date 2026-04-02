"""
Shared fixtures for Flexiv RT integration tests.

Usage:
    sudo -E pytest tests/python/ --robot-sn Rizon4s-XXXXXX -v -s --timeout=120
    sudo -E pytest tests/python/ --lerobot-bi-mount-type side --arm-side left -v -s --timeout=120
"""

from dataclasses import dataclass
from pathlib import Path
import signal
import sys
import time

import pytest

import flexiv_rt as frt


def pytest_addoption(parser):
    parser.addoption(
        "--robot-sn",
        action="store",
        default=None,
        help="Flexiv robot serial number (e.g. Rizon4s-XXXXXX)",
    )
    parser.addoption(
        "--lerobot-bi-mount-type",
        action="store",
        choices=["forward", "side"],
        default=None,
        help="Resolve robot SN and safe start joints from lerobot bi_flexiv_rizon4_rt presets",
    )
    parser.addoption(
        "--arm-side",
        action="store",
        choices=["left", "right"],
        default=None,
        help="Arm side to test when using --lerobot-bi-mount-type",
    )
    parser.addoption(
        "--start-joints-deg",
        action="store",
        default=None,
        help="Comma-separated 7 joint angles in degrees used for initial MoveJ instead of Home",
    )
    parser.addoption(
        "--movej-vel-scale",
        action="store",
        type=int,
        default=None,
        help="Joint velocity scale for initial MoveJ (1-100)",
    )


@pytest.fixture(scope="session")
def robot_setup(request):
    return _resolve_robot_setup(request.config)


@dataclass(frozen=True)
class RobotTestSetup:
    robot_sn: str
    start_joints_deg: list[float] | None
    movej_vel_scale: int | None
    description: str


def _parse_joint_list(value: str) -> list[float]:
    joints = [float(x.strip()) for x in value.split(",") if x.strip()]
    if len(joints) != 7:
        raise pytest.UsageError(
            f"--start-joints-deg must contain exactly 7 comma-separated values, got {len(joints)}"
        )
    return joints


def _load_lerobot_bi_preset(mount_type: str, arm_side: str) -> RobotTestSetup:
    repo_root = Path(__file__).resolve().parents[4]
    src_dir = repo_root / "src"
    if src_dir.exists():
        sys.path.insert(0, str(src_dir))

    from lerobot.robots.bi_flexiv_rizon4_rt.config_bi_flexiv_rizon4_rt import (
        BiFlexivRizon4RTConfig,
    )

    cfg = BiFlexivRizon4RTConfig(bi_mount_type=mount_type)

    if arm_side == "left":
        robot_sn = cfg.left_robot_sn
        start_joints = list(cfg.left_start_position_degree)
    else:
        robot_sn = cfg.right_robot_sn
        start_joints = list(cfg.right_start_position_degree)

    return RobotTestSetup(
        robot_sn=robot_sn,
        start_joints_deg=start_joints,
        movej_vel_scale=cfg.start_vel_scale,
        description=f"lerobot preset bi_mount_type={mount_type}, arm_side={arm_side}",
    )


def _resolve_robot_setup(config) -> RobotTestSetup:
    robot_sn = config.getoption("--robot-sn")
    mount_type = config.getoption("--lerobot-bi-mount-type")
    arm_side = config.getoption("--arm-side")
    start_joints_arg = config.getoption("--start-joints-deg")
    movej_vel_scale = config.getoption("--movej-vel-scale")

    if movej_vel_scale is not None and not 1 <= movej_vel_scale <= 100:
        raise pytest.UsageError(
            f"--movej-vel-scale must be between 1 and 100, got {movej_vel_scale}"
        )

    if mount_type or arm_side:
        if not (mount_type and arm_side):
            raise pytest.UsageError(
                "--lerobot-bi-mount-type and --arm-side must be provided together"
            )
        if start_joints_arg is not None:
            raise pytest.UsageError(
                "--start-joints-deg cannot be combined with --lerobot-bi-mount-type/--arm-side"
            )
        if movej_vel_scale is not None:
            raise pytest.UsageError(
                "--movej-vel-scale cannot be combined with --lerobot-bi-mount-type/--arm-side"
            )
        setup = _load_lerobot_bi_preset(mount_type, arm_side)
        if robot_sn is not None and robot_sn != setup.robot_sn:
            raise pytest.UsageError(
                f"--robot-sn={robot_sn} does not match preset serial number {setup.robot_sn}"
            )
        return setup

    start_joints_deg = None
    if start_joints_arg is not None:
        start_joints_deg = _parse_joint_list(start_joints_arg)
        if movej_vel_scale is None:
            movej_vel_scale = 30

    if robot_sn is None:
        raise pytest.UsageError(
            "Either --robot-sn or (--lerobot-bi-mount-type and --arm-side) must be provided"
        )

    return RobotTestSetup(
        robot_sn=robot_sn,
        start_joints_deg=start_joints_deg,
        movej_vel_scale=movej_vel_scale,
        description="manual CLI arguments",
    )


def _movej_to_start(robot: frt.Robot, joints_deg: list[float], vel_scale: int) -> None:
    robot.Stop()
    robot.SwitchMode(frt.Mode.NRT_PRIMITIVE_EXECUTION)
    time.sleep(0.5)
    robot.ExecutePrimitive(
        "MoveJ",
        {
            "target": joints_deg,
            "jntVelScale": vel_scale,
        },
    )

    timeout = 30.0
    start_time = time.time()
    while True:
        if time.time() - start_time > timeout:
            raise RuntimeError(
                f"MoveJ did not complete within {timeout}s, fault={robot.fault()}, mode={robot.mode()}"
            )
        try:
            primitive_states = robot.primitive_states()
            if primitive_states.get("reachedTarget", 0) == 1:
                return
        except Exception:
            pass
        time.sleep(0.1)


@pytest.fixture(scope="session")
def robot(robot_setup):
    """Session-scoped robot: connect → ClearFault → Enable → safe start pose → yield → Stop."""
    r = frt.Robot(robot_setup.robot_sn)

    # Wait for connection
    for _ in range(50):
        if r.connected():
            break
        time.sleep(0.1)
    assert r.connected(), "Failed to connect to robot"

    # Clear any existing fault
    if r.fault():
        r.ClearFault(30)
        time.sleep(1.0)
    assert not r.fault(), "Could not clear robot fault"

    # Enable
    r.Enable()
    for _ in range(100):
        if r.operational():
            break
        time.sleep(0.1)
    assert r.operational(), "Robot did not become operational"

    original_sigint_handler = signal.getsignal(signal.SIGINT)

    def _handle_sigint(sig, frame):
        print("\nSIGINT received, calling robot.Stop() before pytest teardown...")
        try:
            r.Stop()
        except Exception as e:
            print(f"Warning: robot.Stop() failed during SIGINT: {e}", file=sys.stderr)
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _handle_sigint)

    if robot_setup.start_joints_deg is not None:
        assert robot_setup.movej_vel_scale is not None
        print(
            f"Moving robot to safe start pose from {robot_setup.description}: "
            f"SN={robot_setup.robot_sn}, joints_deg={robot_setup.start_joints_deg}"
        )
        _movej_to_start(r, robot_setup.start_joints_deg, robot_setup.movej_vel_scale)
    else:
        # Default behavior for generic single-arm tests when no preset/start pose is given.
        r.SwitchMode(frt.Mode.NRT_PRIMITIVE_EXECUTION)
        time.sleep(0.5)
        r.ExecutePrimitive("Home", {})
        for _ in range(300):
            if not r.busy():
                break
            time.sleep(0.1)

    yield r

    # Teardown: stop the robot
    try:
        r.Stop()
    except Exception:
        pass
    try:
        r.close()
    except Exception:
        pass
    signal.signal(signal.SIGINT, original_sigint_handler)


@pytest.fixture()
def ensure_no_fault(robot):
    """Per-test fixture: ensure robot has no fault before and after each test."""
    if robot.fault():
        robot.ClearFault(10)
        time.sleep(0.5)
    assert not robot.fault(), "Robot has fault before test"

    yield robot

    # After test: check fault
    if robot.fault():
        robot.ClearFault(10)
        time.sleep(0.5)
