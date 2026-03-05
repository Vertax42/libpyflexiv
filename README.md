# libpyflexiv

Python real-time control bindings for Flexiv robots (Rizon 4/4s/10/10s), built on [Flexiv RDK v1.9](https://github.com/flexivrobotics/flexiv_rdk) and [pybind11](https://github.com/pybind/pybind11).

## Why This Project Exists

Flexiv RDK requires **1 kHz SCHED_FIFO callbacks** for real-time streaming control (`StreamJointPosition`, `StreamCartesianMotionForce`). Python cannot meet this deadline due to the GIL and non-RT scheduling.

**libpyflexiv** solves this by running the 1 kHz RT loop entirely in C++ on a dedicated SCHED_FIFO thread, while Python sets targets and reads state through mutex-protected shared memory at any rate (typically 100 Hz).

```
Python (non-RT, ~100 Hz)              C++ RT thread (SCHED_FIFO, 1 kHz)
┌──────────────────────┐              ┌──────────────────────────────┐
│ set_target_pose()    │──── mutex ──►│ Read command from SHM        │
│ set_target_joints()  │              │ Safety checks (NaN, jump)    │
│ state = get_state()  │◄─── mutex ──│ Timeout detection (500ms)    │
│ move_to_pose()       │              │ Trajectory interpolation     │
└──────────────────────┘              │ StreamCartesianMotionForce() │
                                      │ Write state back to SHM      │
                                      └──────────────────────────────┘
```

## Key Design Decisions

| Decision | Rationale |
|---|---|
| Mutex-protected shared memory | Minimal contention at 1 kHz vs 100 Hz; simpler than lock-free for variable-size vectors |
| Pre-allocated RT buffers | Zero heap allocation in RT callback — uses `std::copy`, never `vector::operator=` |
| Split-lock RT callback | `robot_.states()` (slow IPC) called **outside** the mutex; lock held only for fast memory copies. Prevents Python contention from causing timeliness failures |
| Adaptive jump detection | Velocity-based thresholds scale with Python command frequency: `threshold = max_velocity × cmd_interval`. No manual tuning needed |
| `mlockall()` interception | Flexiv Scheduler calls `mlockall()` which pins all process memory; in a Python+PyTorch process this can be 10+ GB and triggers OOM. We intercept via linker `--wrap` and no-op it |
| Min-jerk trajectory in RT thread | Non-blocking `move_to_pose()` generates smooth 1 kHz commands without mode switching |
| 500ms command timeout | Holds last position if Python crashes or hangs |
| GIL release on blocking ops | `stop()`, `start_*_control()` release the GIL to avoid Python deadlock |
| Context managers | Ensures RT threads are stopped even on unhandled exceptions |

## Build & Install

Complete instructions from creating a fresh mamba environment to a working `import flexiv_rt`.

### Prerequisites

- Ubuntu 22.04 x86_64
- [Mamba](https://mamba.readthedocs.io/) (or conda)
- `sudo` access (runtime SCHED_FIFO only)
- Standard build tools: `sudo apt install build-essential cmake git`

### Step 0: System Dependencies

```bash
sudo apt install build-essential cmake git
```

No ROS 2 required — all C++ dependencies (Eigen, spdlog, Fast-DDS, Fast-CDR, RBDyn, etc.) are built from source by the provided script in Step 1.

### Step 1: Install Flexiv RDK and All Dependencies

The `flexiv_rdk` SDK is already included as a submodule. First, build and install all its C++ dependencies to `~/rdk_install`, then build and install the RDK itself.

```bash
cd /path/to/libpyflexiv

# Step 1a: Build and install all C++ dependencies from source
# (Eigen, spdlog, tinyxml2, yaml-cpp, foonathan_memory,
#  Fast-CDR, Fast-DDS, Boost, SpaceVecAlg, RBDyn)
cd flexiv_rdk/thirdparty
bash build_and_install_dependencies.sh ~/rdk_install $(nproc)

# Step 1b: Build and install flexiv_rdk itself
cd /path/to/libpyflexiv/flexiv_rdk
mkdir -p build && cd build
cmake .. \
  -DCMAKE_INSTALL_PREFIX=~/rdk_install \
  -DCMAKE_PREFIX_PATH=~/rdk_install \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build . --target install --config Release -j$(nproc)
```

> **Note**: No `sudo` needed — everything installs to your home directory.
> The RDK static library (`libflexiv_rdk.x86_64-linux-gnu.a`) is automatically downloaded
> from GitHub during the cmake configure step.

Verify:
- `ls ~/rdk_install/include/flexiv/rdk/robot.hpp` should exist
- `ls ~/rdk_install/lib/cmake/flexiv_rdk/flexiv_rdk-config.cmake` should exist

### Step 2: Create Mamba Environment

Create a Python 3.10 environment with the packages needed for building and running:

```bash
mamba create -n lerobot-xense python=3.10 -y
mamba activate lerobot-xense

# Build dependency
pip install pybind11

# (Optional) runtime dependencies for lerobot
# pip install -e /path/to/lerobot-xense
```

### Step 3: Build C++ Bindings

Point CMake at the Flexiv RDK install prefix and at your conda Python interpreter.

**Important**: Only pass `~/rdk_install` as `CMAKE_PREFIX_PATH` — do NOT include the conda env prefix. The conda env contains a different spdlog version that will cause header conflicts.

```bash
mamba activate lerobot-xense

cd /path/to/libpyflexiv
mkdir -p build && cd build

cmake .. \
  -DCMAKE_PREFIX_PATH=~/rdk_install \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DPython3_EXECUTABLE=$(which python)

make -j$(nproc)
```

This produces `flexiv_rt/_flexiv_rt.cpython-310-x86_64-linux-gnu.so` in the project root.

> **Troubleshooting**: If you see spdlog errors like `is_convertible_to_basic_format_string` or `basic_runtime is not a member of fmt`, it means cmake found a wrong spdlog. Ensure `CMAKE_PREFIX_PATH` is set to `~/rdk_install` and does NOT include the conda env prefix.

### Step 4: Install Python Package

Editable install — the compiled `.so` is already in place:

```bash
mamba run -n lerobot-xense pip install -e /path/to/libpyflexiv
```

Verify:

```bash
mamba run -n lerobot-xense python -c "import flexiv_rt; print('OK')"
```

### Running (requires `sudo` for SCHED_FIFO)

The Flexiv RDK Scheduler creates SCHED_FIFO threads at maximum priority, which requires root. Use `sudo -E` to preserve your conda environment:

```bash
sudo -E mamba run -n lerobot-xense python your_script.py

# Or with explicit PATH preservation:
sudo -E env PATH=$PATH \
  LD_LIBRARY_PATH=$LD_LIBRARY_PATH \
  python your_script.py
```

## Quick Start

### Joint Impedance Control

```python
from flexiv_rt import Robot, Mode

robot = Robot("Rizon4s-XXXXXX")
robot.Enable()

# Home the robot
robot.SwitchMode(Mode.NRT_PLAN_EXECUTION)
robot.ExecutePlan("PLAN-Home")
import time; time.sleep(5)
robot.Stop()

# Switch to RT joint impedance
robot.SwitchMode(Mode.RT_JOINT_IMPEDANCE)
robot.SetJointImpedance(
    K_q=[3000, 3000, 800, 800, 200, 200, 200],  # stiffness
    Z_q=[0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7],   # damping ratio
)

# Start RT control — spawns C++ SCHED_FIFO thread at 1 kHz
with robot.start_joint_impedance_control() as ctrl:
    for i in range(5000):  # 5 seconds at 1 kHz Python-side
        state = ctrl.get_state()          # read current joint state
        target = list(state.q)            # copy current positions
        target[3] += 0.05 * math.sin(i * 0.001 * 2 * math.pi * 0.3)
        ctrl.set_target_joints(target)    # write to shared memory
        time.sleep(0.001)
    # Context manager auto-calls ctrl.stop()
```

### Cartesian Motion+Force Control

```python
robot.SwitchMode(Mode.RT_CARTESIAN_MOTION_FORCE)
robot.SetForceControlAxis([False]*6)  # pure motion, no force control

with robot.start_cartesian_control() as ctrl:
    state = ctrl.get_state()
    init_pose = list(state.tcp_pose)  # [x, y, z, qw, qx, qy, qz]

    for i in range(10000):  # 10 seconds
        pose = init_pose.copy()
        pose[1] += 0.05 * math.sin(i * 0.001 * 2 * math.pi * 0.2)  # Y sine
        ctrl.set_target_pose(pose)
        time.sleep(0.001)
```

### Non-Blocking Trajectory (Reset to Home)

```python
with robot.start_cartesian_control() as ctrl:
    # ... teleop streaming ...

    # User presses reset button:
    ctrl.move_to_pose(home_pose, duration_sec=3.0)  # returns immediately

    # Keep reading observations while robot moves
    while ctrl.is_moving():
        state = ctrl.get_state()  # observations still flowing
        time.sleep(0.01)

    # Trajectory complete — resume streaming
    ctrl.set_target_pose(home_pose)
    # ... continue teleop ...
```

## API Reference

### `Robot`

The main entry point. Wraps `flexiv::rdk::Robot` with connection retry logic, context manager support, and safe shutdown ordering.

```python
Robot(serial_number: str,
      network_whitelist: list[str] = [],
      connect_retries: int = 3,
      retry_interval_sec: float = 1.0,
      verbose: bool = True,
      lite: bool = False)
```

**Connection & Lifecycle**

| Method | Description |
|---|---|
| `Enable()` | Enable the robot (must be called after connection) |
| `Stop()` | Stop all motion. Stops active RT controls first, then sends robot stop. Releases GIL |
| `close()` | Full cleanup: stop all RT controls, stop robot, mark as closed. Idempotent |
| `ClearFault(timeout: int = 5)` | Clear robot fault with timeout in seconds |
| `SwitchMode(mode: Mode)` | Switch robot operating mode |

**State Accessors**

| Method | Returns | Description |
|---|---|---|
| `connected()` | `bool` | Whether robot is connected |
| `operational()` | `bool` | Whether robot is in operational state |
| `fault()` | `bool` | Whether robot has a fault |
| `busy()` | `bool` | Whether robot is busy executing a plan/primitive |
| `stopped()` | `bool` | Whether robot is stopped |
| `estop_released()` | `bool` | Whether e-stop is released |
| `reduced()` | `bool` | Whether robot is in reduced mode |
| `recovery()` | `bool` | Whether robot is in recovery mode |
| `mode()` | `Mode` | Current operating mode |
| `states()` | `RobotStates` | Full robot state snapshot (returns by value) |
| `info()` | `RobotInfo` | Static robot information (DoF, limits, serial number) |

**Plan & Primitive Execution**

| Method | Description |
|---|---|
| `ExecutePlan(name: str)` | Execute a named plan (e.g. `"PLAN-Home"`) |
| `ExecutePrimitive(name: str, params: dict)` | Execute a primitive with parameters. Special: `"MoveJ"` auto-converts `list[7]` target to `JPos` |

**Impedance Configuration** (call before starting RT control)

| Method | Parameters | Description |
|---|---|---|
| `SetJointImpedance(K_q, Z_q)` | `list[7]`, `list[7]` | Joint stiffness and damping ratio |
| `SetCartesianImpedance(K_x, Z_x)` | `list[6]`, `list[6]` | Cartesian stiffness `[Kx,Ky,Kz,Krx,Kry,Krz]` and damping |
| `SetMaxContactWrench(wrench)` | `list[6]` | Max contact wrench before protective stop |
| `SetMaxContactTorque(torques)` | `list[7]` | Max contact torque per joint |
| `SetForceControlAxis(axes, max_vel)` | `list[6] (bool)`, `list[3]` | Enable/disable force control per axis. `max_vel` default `[1,1,1]` |
| `SetForceControlFrame(coord, T)` | `CoordType`, `list[7]` | Set force control reference frame. `T` is `[x,y,z,qw,qx,qy,qz]` |
| `SetNullSpacePosture(ref_pos)` | `list[7]` | Preferred null-space joint positions |

**RT Control Entry Points**

| Method | Returns | Description |
|---|---|---|
| `start_joint_impedance_control()` | `JointImpedanceControl` | Spawns RT thread for joint streaming. Releases GIL during thread creation |
| `start_cartesian_control()` | `CartesianMotionForceControl` | Spawns RT thread for Cartesian streaming. Releases GIL during thread creation |

Both return context managers and maintain `keep_alive` references to prevent premature GC of the `Robot` object.

---

### `JointImpedanceControl`

Context manager for real-time joint impedance streaming. Created by `robot.start_joint_impedance_control()`.

**Usage**: Always use as a context manager or explicitly call `stop()`.

| Method | Description |
|---|---|
| `set_target_joints(positions, velocities=[], accelerations=[])` | Set joint target. All `list[7]` of `float`. Velocity/acceleration default to zero. Writes to shared memory (mutex-protected) |
| `get_state()` → `JointState` | Read current joint state from shared memory |
| `trigger_estop()` | Trigger emergency stop (lockless `atomic<bool>`, checked at top of every RT cycle) |
| `stop()` | Stop the RT thread. Releases GIL. Idempotent |
| `is_running()` → `bool` | Whether the RT thread is active |

---

### `CartesianMotionForceControl`

Context manager for real-time Cartesian streaming with integrated trajectory support. Created by `robot.start_cartesian_control()`.

**Streaming API** (RT state: `STREAMING`)

| Method | Description |
|---|---|
| `set_target_pose(pose, wrench=[], velocity=[], acceleration=[])` | Set Cartesian target. `pose`: `list[7]` `[x,y,z,qw,qx,qy,qz]`. `wrench`: `list[6]`. Writes to shared memory |
| `get_state()` → `CartesianState` | Read current Cartesian state from shared memory |

**Trajectory API** (RT state transitions: `STREAMING` → `MOVING` → `STREAMING`)

| Method | Description |
|---|---|
| `move_to_pose(pose, duration_sec=3.0)` | Start a min-jerk trajectory to `pose` (7D). **Non-blocking**: returns immediately, RT thread interpolates at 1 kHz. Idempotent if already moving to same target |
| `move_to_pose_sync(pose, duration_sec=3.0, timeout_sec=0.0)` | **Blocking** version. Releases GIL, polls at 10ms. Returns `True` on completion, `False` on timeout. `timeout_sec=0` means no timeout |
| `is_moving()` → `bool` | Whether a trajectory is in progress |
| `cancel_move()` | Cancel active trajectory. RT thread holds at current interpolated position |

**Lifecycle**

| Method | Description |
|---|---|
| `trigger_estop()` | Emergency stop (lockless atomic) |
| `stop()` | Stop RT thread. Releases GIL. Idempotent |
| `is_running()` → `bool` | Whether RT thread is active |

**State Machine**:
- During `STREAMING`: `set_target_pose()` commands are executed each RT cycle
- Calling `move_to_pose()` transitions to `MOVING`: RT thread follows the min-jerk trajectory, `set_target_pose()` calls are ignored
- When trajectory completes (or `cancel_move()`): transitions back to `STREAMING` with end pose as the hold target

---

### `JointState`

Snapshot of joint-space state, returned by `JointImpedanceControl.get_state()`.

| Field | Type | Description |
|---|---|---|
| `q` | `list[7]` | Joint positions (rad) |
| `dq` | `list[7]` | Joint velocities (rad/s) |
| `tau` | `list[7]` | Joint torques (Nm) |
| `tau_ext` | `list[7]` | External torques (Nm) |
| `tcp_pose` | `list[7]` | TCP pose `[x,y,z,qw,qx,qy,qz]` (convenience, always available) |

---

### `CartesianState`

Snapshot of Cartesian-space state, returned by `CartesianMotionForceControl.get_state()`.

| Field | Type | Description |
|---|---|---|
| `tcp_pose` | `list[7]` | TCP pose `[x,y,z,qw,qx,qy,qz]` (m, quaternion) |
| `tcp_vel` | `list[6]` | TCP velocity `[vx,vy,vz,wx,wy,wz]` (m/s, rad/s) |
| `ext_wrench_in_tcp` | `list[6]` | External wrench in TCP frame `[fx,fy,fz,mx,my,mz]` (N, Nm) |
| `ext_wrench_in_world` | `list[6]` | External wrench in world frame |
| `ft_sensor_raw` | `list[6]` | Raw F/T sensor readings |
| `q` | `list[7]` | Joint positions (rad) |
| `tau_ext` | `list[7]` | External joint torques (Nm) |

---

### `Mode` (Enum)

| Value | Description |
|---|---|
| `IDLE` | Robot idle |
| `RT_JOINT_TORQUE` | Real-time joint torque streaming |
| `RT_JOINT_IMPEDANCE` | Real-time joint impedance streaming |
| `RT_CARTESIAN_MOTION_FORCE` | Real-time Cartesian motion+force streaming |
| `NRT_JOINT_IMPEDANCE` | Non-real-time joint impedance |
| `NRT_CARTESIAN_MOTION_FORCE` | Non-real-time Cartesian motion+force |
| `NRT_PLAN_EXECUTION` | Plan execution mode |
| `NRT_PRIMITIVE_EXECUTION` | Primitive execution mode |
| `UNKNOWN` | Unknown mode |

### `CoordType` (Enum)

| Value | Description |
|---|---|
| `WORLD` | World (base) coordinate frame |
| `TCP` | Tool center point frame |

### `RobotInfo`

| Field | Type | Description |
|---|---|---|
| `serial_num` | `str` | Robot serial number |
| `model_name` | `str` | Robot model (e.g. "Rizon4s") |
| `DoF` | `int` | Degrees of freedom (7) |
| `K_q_nom` | `list[7]` | Nominal joint stiffness |
| `K_x_nom` | `list[6]` | Nominal Cartesian stiffness |
| `q_min` / `q_max` | `list[7]` | Joint position limits (rad) |
| `dq_max` | `list[7]` | Joint velocity limits (rad/s) |
| `tau_max` | `list[7]` | Joint torque limits (Nm) |

### `RobotStates`

Full robot state snapshot returned by `robot.states()`.

| Field | Type | Description |
|---|---|---|
| `q` | `list[7]` | Current joint positions (rad) |
| `theta` | `list[7]` | Motor-side joint positions (rad) |
| `dq` | `list[7]` | Joint velocities (rad/s) |
| `dtheta` | `list[7]` | Motor-side joint velocities (rad/s) |
| `tau` | `list[7]` | Joint torques (Nm) |
| `tau_des` | `list[7]` | Desired joint torques (Nm) |
| `tau_dot` | `list[7]` | Joint torque derivatives (Nm/s) |
| `tau_ext` | `list[7]` | External joint torques (Nm) |
| `tcp_pose` | `list[7]` | TCP pose `[x,y,z,qw,qx,qy,qz]` |
| `tcp_vel` | `list[6]` | TCP velocity `[vx,vy,vz,wx,wy,wz]` |
| `flange_pose` | `list[7]` | Flange pose |
| `ft_sensor_raw` | `list[6]` | Raw F/T sensor readings |
| `ext_wrench_in_tcp` | `list[6]` | External wrench in TCP frame |
| `ext_wrench_in_world` | `list[6]` | External wrench in world frame |

## Safety Features

The RT callback implements multiple safety layers, evaluated **every 1ms cycle**:

1. **Emergency Stop** — `atomic<bool>`, lockless check at the top of each cycle. Holds last sent command. Triggered by `trigger_estop()` or automatically on robot fault.

2. **NaN/Inf Detection** — Every command field is checked with `std::isfinite()`. Any NaN or Inf triggers an immediate e-stop.

3. **Adaptive Jump Detection** — Velocity-based thresholds that automatically adapt to the Python command frequency. The Python-side `set_target_pose()`/`set_target_joints()` measures the time interval between consecutive calls and stores it in shared memory. The RT thread computes per-step thresholds as:

   ```
   threshold = max_velocity × cmd_interval
   ```

   | Domain | Max Velocity | @100 Hz (10ms) | @30 Hz (33ms) | @10 Hz (100ms) |
   |---|---|---|---|---|
   | Joint position | 10 rad/s | 0.10 rad | 0.33 rad | 1.00 rad |
   | Cartesian position | 10 m/s | 100 mm | 330 mm | 1000 mm |
   | Cartesian rotation | 50 rad/s | 0.50 rad | 1.65 rad | 5.00 rad |

   The command interval is clamped to [1ms, 100ms] to prevent extreme thresholds. Before the second command arrives, the default interval (10ms / 100 Hz) is used. If a jump is detected, the RT thread clamps to the last sent position with zero velocity/acceleration.

4. **Command Timeout** — If no new command arrives within **500ms**, the RT thread holds the last sent position with zero velocity/acceleration. Prevents runaway if Python crashes.

5. **Context Manager Cleanup** — `with robot.start_*_control() as ctrl:` guarantees `ctrl.stop()` is called even on exceptions, preventing orphaned RT threads.

6. **Idempotent Stop** — `stop()` uses `atomic<bool>` to ensure it's safe to call multiple times from multiple threads.

## Min-Jerk Trajectory

The `move_to_pose()` API uses a minimum-jerk (5th-order polynomial) trajectory generator that runs entirely in the RT thread:

**Position**: `s(t) = 10t^3 - 15t^4 + 6t^5` (normalized time `t = elapsed / duration`)

**Rotation**: Quaternion SLERP with short-path selection (dot product sign correction)

**Velocity feed-forward**: Analytical derivative `ds/dt = (30t^2 - 60t^3 + 30t^4) / duration`

Properties:
- Zero velocity and acceleration at start and end
- Smooth, continuous commands at 1 kHz
- Zero heap allocation (all buffers pre-allocated)
- Safe to call from any thread (mutex-protected request handoff to RT thread)

## Project Structure

```
libpyflexiv/
├── CMakeLists.txt                  # Top-level build (pybind11 module)
├── pyproject.toml                  # Python package metadata
├── setup.py                        # pip install support
├── flexiv_rdk/                     # Flexiv RDK v1.8 SDK (headers + prebuilt lib)
├── include/realtime_control/       # C++ RT control headers
│   ├── rt_common.hpp               #   Safety constants & check functions
│   ├── shared_memory.hpp           #   SPSC ring buffer (alternative design)
│   ├── joint_state.hpp             #   JointState struct
│   ├── cartesian_state.hpp         #   CartesianState struct
│   ├── joint_impedance_control.hpp #   Joint RT controller declaration
│   ├── cartesian_control.hpp       #   Cartesian RT controller declaration
│   └── trajectory.hpp              #   Min-jerk trajectory generator
├── src/                            # C++ implementation
│   ├── flexiv_rt.cpp               #   Pybind11 module (Robot, controls, enums)
│   ├── joint_impedance_control.cpp #   Joint RT callback implementation
│   └── cartesian_control.cpp       #   Cartesian RT callback + mlockall wrap
├── flexiv_rt/                      # Python package
│   ├── __init__.py                 #   Re-exports from compiled .so
│   └── _flexiv_rt.*.so             #   Compiled pybind11 module
├── examples/
│   ├── joint_impedance_example.py  #   Joint sine-sweep demo
│   ├── cartesian_motion_example.py #   Cartesian Y-axis sine demo
│   └── rt_reset_example.py         #   Non-blocking trajectory reset demo
└── tests/
    ├── cpp/test_rt_safety.cpp      #   C++ unit tests (safety functions, buffer)
    └── python/
        ├── conftest.py             #   Pytest fixtures (robot connection)
        └── test_rt_integration.py  #   Integration tests (requires real robot)
```

## License

See `flexiv_rdk/LICENSE` for the Flexiv RDK license.
