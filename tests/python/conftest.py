"""
Shared fixtures for Flexiv RT integration tests.

Usage:
    sudo -E pytest tests/python/ --robot-sn Rizon4s-XXXXXX -v -s --timeout=120
"""

import time
import pytest

import flexiv_bindings as fb


def pytest_addoption(parser):
    parser.addoption(
        "--robot-sn",
        action="store",
        required=True,
        help="Flexiv robot serial number (e.g. Rizon4s-XXXXXX)",
    )


@pytest.fixture(scope="session")
def robot_sn(request):
    return request.config.getoption("--robot-sn")


@pytest.fixture(scope="session")
def robot(robot_sn):
    """Session-scoped robot: connect → ClearFault → Enable → Home → yield → Stop."""
    r = fb.Robot(robot_sn)

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

    # Home via primitive
    r.SwitchMode(fb.Mode.NRT_PRIMITIVE_EXECUTION)
    time.sleep(0.5)
    r.ExecutePrimitive("Home", {})
    # Wait for homing to complete (robot becomes idle / not busy)
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
