#include "realtime_control/cartesian_control.hpp"
#include "realtime_control/logging.hpp"
#include "realtime_control/rt_common.hpp"
#include <cerrno>
#include <cmath>
#include <semaphore.h>
#include <sys/mman.h>
#include <stdexcept>

// ---------------------------------------------------------------------------
// Intercept mlockall() called by flexiv::rdk::Scheduler.
// Linked via -Wl,--wrap=mlockall so all calls to mlockall() in the Flexiv
// static library are redirected here.  This prevents OOM in Python processes
// with large virtual memory (PyTorch, numpy, etc.).
// Note: Flexiv Scheduler calls mlockall() twice — once in its constructor
// and once in Start().  Both are intercepted and no-op'd here.
// ---------------------------------------------------------------------------
extern "C" int __wrap_mlockall(int flags) {
    flexiv_rt::logger()->info("mlockall(flags={}) intercepted — skipped (OOM prevention)", flags);
    return 0;  // pretend success
}

// ---------------------------------------------------------------------------
// Intercept sem_unlink() called by flexiv::rdk::Scheduler::Stop() and
// ~Scheduler().  The SDK calls sem_unlink() in both paths, so the second
// call always fails with ENOENT ("No such file or directory").  This is
// harmless but prints a noisy [error] log line.  We suppress ENOENT and
// forward all other errors to the real sem_unlink.
// Linked via -Wl,--wrap=sem_unlink.
// ---------------------------------------------------------------------------
extern "C" int __real_sem_unlink(const char* name);
extern "C" int __wrap_sem_unlink(const char* name) {
    int ret = __real_sem_unlink(name);
    if (ret == -1 && errno == ENOENT) {
        errno = 0;
        return 0;
    }
    return ret;
}

namespace flexiv_rt {

// ---------------------------------------------------------------------------
// Common shared-memory initialisation (called by both constructors)
// ---------------------------------------------------------------------------
void CartesianMotionForceControl::initSharedMemory()
{
    auto s = robot_.states();
    last_sent_pose_ = s.tcp_pose;

    shm_->state_tcp_pose          = s.tcp_pose;
    shm_->state_tcp_vel           = s.tcp_vel;
    shm_->state_ext_wrench_in_tcp = s.ext_wrench_in_tcp;
    shm_->state_ext_wrench_in_world = s.ext_wrench_in_world;
    shm_->state_ft_sensor_raw     = s.ft_sensor_raw;
    shm_->state_q                 = s.q;
    shm_->state_tau_ext           = s.tau_ext;

    // Pre-populate command to hold current pose
    shm_->command.pose = s.tcp_pose;
}

// ---------------------------------------------------------------------------
// Helper: validate and store inner_control_hz / interpolate_cmds
// ---------------------------------------------------------------------------
void CartesianMotionForceControl::initControlParams(int inner_control_hz, bool interpolate_cmds)
{
    if (inner_control_hz < 1 || inner_control_hz > 1000) {
        throw std::invalid_argument(
            "inner_control_hz must be in [1, 1000], got "
            + std::to_string(inner_control_hz));
    }
    // 1000 must be divisible by inner_control_hz for exact timing.
    // We round to the nearest integer — the resulting jitter is < 1 ms.
    cmd_decimation_   = static_cast<int>(std::round(1000.0 / inner_control_hz));
    interpolate_cmds_ = interpolate_cmds;
    logger()->info(
        "CartesianMotionForceControl: inner_control_hz={} → decimation={} interpolate={}",
        inner_control_hz, cmd_decimation_, interpolate_cmds_);
}

// ---------------------------------------------------------------------------
// Legacy constructor: Scheduler() + AddTask + Start() — blocks ~2-4s
// ---------------------------------------------------------------------------
CartesianMotionForceControl::CartesianMotionForceControl(
    flexiv::rdk::Robot& robot,
    std::unique_ptr<flexiv::rdk::Scheduler> pre_scheduler,
    std::string task_name,
    int  inner_control_hz,
    bool interpolate_cmds)
    : robot_(robot)
    , task_name_(std::move(task_name))
    , shm_(std::make_shared<CartesianSharedMemory>())
{
    initControlParams(inner_control_hz, interpolate_cmds);
    initSharedMemory();

    if (pre_scheduler) {
        scheduler_ = std::move(pre_scheduler);
        logger()->info("{}", "CartesianMotionForceControl: using pre-created Scheduler");
    } else {
        scheduler_ = std::make_unique<flexiv::rdk::Scheduler>();
    }

    scheduler_->AddTask(
        [this]() { PeriodicCallback(); },
        task_name_, 1, scheduler_->max_priority());
    logger()->info("{}", "CartesianMotionForceControl: AddTask done, calling Start()...");
    scheduler_->Start();
    logger()->info("{}", "CartesianMotionForceControl: Start() returned, RT thread running");
}

// ---------------------------------------------------------------------------
// Pre-started constructor: Scheduler already running idle proxy — near-instant
// ---------------------------------------------------------------------------
CartesianMotionForceControl::CartesianMotionForceControl(
    flexiv::rdk::Robot& robot,
    PrestartedScheduler prestarted,
    int  inner_control_hz,
    bool interpolate_cmds)
    : robot_(robot)
    , scheduler_(std::move(prestarted.scheduler))
    , shm_(std::make_shared<CartesianSharedMemory>())
    , proxy_(std::move(prestarted.proxy))
{
    initControlParams(inner_control_hz, interpolate_cmds);
    initSharedMemory();

    proxy_->activate([this]() { PeriodicCallback(); });
    logger()->info("{}", "CartesianMotionForceControl: activated pre-started Scheduler, RT thread running");
}

CartesianMotionForceControl::~CartesianMotionForceControl()
{
    stop();
}

void CartesianMotionForceControl::stop()
{
    // Idempotent: only the first caller executes the body.
    if (stopped_.exchange(true)) return;
    shm_->emergency_stop.store(true);
    is_running_.store(false);
    // Release the Scheduler via reset() rather than calling Stop() explicitly.
    // Flexiv RDK's Scheduler calls sem_unlink() in both Stop() and ~Scheduler(),
    // so calling Stop() before the destructor produces a harmless but noisy
    // "sem_unlink() failed" error.  By letting ~Scheduler() be the sole caller
    // of Stop() (which also does the sem_unlink), we get exactly one unlink.
    // ~Scheduler() blocks until the RT thread has joined, providing the same
    // ordering guarantee as an explicit Stop() call.
    try {
        scheduler_.reset();   // ~Scheduler(): Stop() + sem_unlink (once)
    } catch (...) {}

    // Immediately tell the robot to stop expecting RT commands.
    // Without this, there is a gap between the RT thread joining (above) and
    // the Python-level robot.Stop() call, during which the robot detects
    // missed 1 kHz commands and logs "Timeliness failure".
    try {
        robot_.Stop();
    } catch (...) {}
}

void CartesianMotionForceControl::setTargetPose(
    const std::array<double,7>& pose,
    const std::array<double,6>& wrench,
    const std::array<double,6>& velocity,
    const std::array<double,6>& acceleration)
{
    std::lock_guard<std::mutex> lock(shm_mutex_);
    auto now = std::chrono::steady_clock::now();

    // Compute command interval for adaptive jump detection
    if (shm_->command_received.load()) {
        double dt = std::chrono::duration<double>(now - shm_->last_cmd_time).count();
        shm_->cmd_interval_sec = std::clamp(dt, kMinCommandInterval, kMaxCommandInterval);
    }

    shm_->command.pose         = pose;
    shm_->command.wrench       = wrench;
    shm_->command.velocity     = velocity;
    shm_->command.acceleration = acceleration;
    shm_->last_cmd_time        = now;
    shm_->command_received.store(true);
}

CartesianState CartesianMotionForceControl::getState()
{
    std::lock_guard<std::mutex> lock(shm_mutex_);
    CartesianState s;
    s.tcp_pose           = shm_->state_tcp_pose;
    s.tcp_vel            = shm_->state_tcp_vel;
    s.ext_wrench_in_tcp  = shm_->state_ext_wrench_in_tcp;
    s.ext_wrench_in_world = shm_->state_ext_wrench_in_world;
    s.ft_sensor_raw      = shm_->state_ft_sensor_raw;
    s.q                  = shm_->state_q;
    s.tau_ext            = shm_->state_tau_ext;
    return s;
}

void CartesianMotionForceControl::triggerEstop()
{
    shm_->emergency_stop.store(true);
}

bool CartesianMotionForceControl::isRunning() const
{
    return is_running_.load();
}

// ---------------------------------------------------------------------------
// Move-to-pose API (called from Python thread)
// ---------------------------------------------------------------------------
void CartesianMotionForceControl::moveToPose(
    const std::array<double,7>& target_pose, double duration_sec)
{
    // Validate input
    for (const auto& v : target_pose) {
        if (!std::isfinite(v)) {
            throw std::invalid_argument("moveToPose: target_pose contains NaN/Inf");
        }
    }
    if (duration_sec > 0.0 && !std::isfinite(duration_sec)) {
        throw std::invalid_argument("moveToPose: duration_sec must be finite");
    }
    // duration_sec <= 0 means auto-compute (handled by MinJerkTrajectory::init)

    std::lock_guard<std::mutex> lock(shm_mutex_);
    shm_->move_request.target_pose = target_pose;
    shm_->move_request.duration_sec = duration_sec;
    shm_->move_request.pending = true;
}

bool CartesianMotionForceControl::isMoving() const
{
    return shm_->is_moving.load();
}

void CartesianMotionForceControl::cancelMove()
{
    // Cancel is detected by the RT thread on the next cycle.
    // We clear the pending flag (under mutex) and mark is_moving false.
    {
        std::lock_guard<std::mutex> lock(shm_mutex_);
        shm_->move_request.pending = false;
    }
    // The atomic is_moving flag lets the RT thread know to cancel.
    // We set it to false — the RT thread checks trajectory_.isActive()
    // and will see the cancel via trajectory_.cancel() next cycle.
    // Actually: just set a cancel flag on the trajectory from here is not RT-safe
    // because trajectory_ is owned by RT thread. Instead, we clear pending and
    // rely on the RT thread to detect !is_moving.
    // Simplest approach: write a cancel into the move_request and let RT handle it.
    // Let's use a different approach — trajectory_.cancel() is noexcept and just
    // sets a bool, which is safe from any thread (single writer pattern on cancel).
    trajectory_.cancel();
}

// ---------------------------------------------------------------------------
// RT periodic callback (runs on SCHED_FIFO thread at 1 kHz)
// ---------------------------------------------------------------------------
void CartesianMotionForceControl::PeriodicCallback()
{
    // 1. Emergency stop – hold at last sent pose
    if (shm_->emergency_stop.load()) {
        try {
            robot_.StreamCartesianMotionForce(last_sent_pose_);
        } catch (const std::exception&) {
            // Intentionally swallow SDK errors during emergency stop.
            // Do NOT use catch(...) here — it would catch abi::__forced_unwind
            // from pthread_cancel (used by Scheduler::Stop()) and cause
            // "FATAL: exception not rethrown" / std::terminate().
        }
        is_running_.store(false);
        shm_->is_moving.store(false);
        return;
    }

    // 2. Robot fault
    if (robot_.fault()) {
        logger()->error("{}", "CartesianMotionForceControl: robot fault detected");
        shm_->emergency_stop.store(true);
        is_running_.store(false);
        shm_->is_moving.store(false);
        return;
    }

    // 3. Frequency decimation:
    //    Always increment the cycle counter. Only poll SHM for a new Python
    //    command every cmd_decimation_ cycles (e.g. every 2nd cycle = 500 Hz).
    //    moveToPose requests are always consumed immediately (safety critical).
    ++cycle_counter_;
    const bool consume_cmd = (cycle_counter_ >= cmd_decimation_);
    if (consume_cmd) cycle_counter_ = 0;

    MoveRequest move_req;
    CartesianCommand cmd;
    bool cmd_received = false;
    bool timed_out = false;
    double cmd_interval = kDefaultCommandInterval;

    {
        std::lock_guard<std::mutex> lock(shm_mutex_);

        // Always consume move requests (safety: don't delay reset trajectories)
        move_req = shm_->move_request;
        if (move_req.pending) {
            shm_->move_request.pending = false;
        }

        // Only read streaming commands on decimation boundary
        if (consume_cmd) {
            cmd          = shm_->command;
            cmd_received = shm_->command_received.load();
            cmd_interval = shm_->cmd_interval_sec;

            if (cmd_received) {
                auto elapsed = std::chrono::steady_clock::now() - shm_->last_cmd_time;
                if (elapsed > kCommandTimeout) {
                    timed_out = true;
                }
            }
        }
    }

    // 4. Handle state transitions
    //    STREAMING → MOVING: when move_request.pending is consumed
    //    MOVING → MOVING:    new moveToPose restarts the trajectory
    //    MOVING → STREAMING: trajectory complete or cancelled
    if (move_req.pending) {
        // Start (or restart) trajectory from current actual pose
        std::array<double,7> current_pose = last_sent_pose_;
        trajectory_.init(current_pose, move_req.target_pose, move_req.duration_sec);
        rt_state_ = RTState::MOVING;
        shm_->is_moving.store(true);
    }

    // 5. Execute based on current state — send command ASAP, read states AFTER.
    //    All paths set send_error=true on failure so we skip the state refresh.
    bool send_error = false;

    if (rt_state_ == RTState::MOVING) {
        // --- MOVING state ---
        std::array<double,7> interp_pose;
        std::array<double,6> interp_velocity;

        bool in_progress = trajectory_.step(interp_pose, interp_velocity);

        if (!in_progress) {
            // Trajectory complete (or cancelled) — switch back to STREAMING
            rt_state_ = RTState::STREAMING;
            shm_->is_moving.store(false);

            // Write end pose into SHM command so streaming holds here
            {
                std::lock_guard<std::mutex> lock(shm_mutex_);
                shm_->command.pose = last_sent_pose_;
                shm_->command.wrench = {};
                shm_->command.velocity = {};
                shm_->command.acceleration = {};
                shm_->last_cmd_time = std::chrono::steady_clock::now();
                shm_->command_received.store(true);
            }

            // Send the last pose to hold position
            try {
                robot_.StreamCartesianMotionForce(last_sent_pose_);
            } catch (const std::exception& e) {
                logger()->error("CartesianMotionForceControl: StreamCartesianMotionForce error: {}", e.what());
                shm_->emergency_stop.store(true);
                is_running_.store(false);
                send_error = true;
            }
            if (!send_error) last_sent_vel_ = {};
        } else if (!CheckFinite(interp_pose)) {
            // NaN/Inf check
            logger()->error("{}", "CartesianMotionForceControl: NaN/Inf in trajectory pose");
            shm_->emergency_stop.store(true);
            is_running_.store(false);
            shm_->is_moving.store(false);
            send_error = true;
        } else {
            // Send interpolated command — clamp velocity/acceleration first
            ClampCartesianPose(interp_pose, last_sent_pose_);
            ClampCartesianVelocity(interp_velocity, last_sent_vel_);
            try {
                robot_.StreamCartesianMotionForce(
                    interp_pose, {}, interp_velocity, {});
            } catch (const std::exception& e) {
                logger()->error("CartesianMotionForceControl: StreamCartesianMotionForce error: {}", e.what());
                shm_->emergency_stop.store(true);
                is_running_.store(false);
                shm_->is_moving.store(false);
                send_error = true;
            }
            if (!send_error) {
                last_sent_pose_ = interp_pose;
                last_sent_vel_  = interp_velocity;
            }
        }
    } else {
        // --- STREAMING state ---

        if (!consume_cmd) {
            // Non-decimation cycle: step the streaming interpolation (if active)
            // or just hold the last pose.
            std::array<double,7> interp_pose = last_sent_pose_;
            std::array<double,6> interp_vel  = {};
            if (interpolate_cmds_ && stream_interp_.isActive()) {
                bool in_progress = stream_interp_.step(interp_pose, interp_vel);
                if (!in_progress) {
                    // Interpolation done, hold at target
                    interp_pose = last_sent_pose_;
                    interp_vel  = {};
                }
            }
            // Clamp before sending
            ClampCartesianPose(interp_pose, last_sent_pose_);
            ClampCartesianVelocity(interp_vel, last_sent_vel_);
            try {
                robot_.StreamCartesianMotionForce(interp_pose, {}, interp_vel, {});
            } catch (const std::exception& e) {
                logger()->error("CartesianMotionForceControl: StreamCartesianMotionForce error: {}", e.what());
                shm_->emergency_stop.store(true);
                is_running_.store(false);
                send_error = true;
            }
            if (!send_error) {
                last_sent_pose_ = interp_pose;
                last_sent_vel_  = interp_vel;
            }

        } else {
            // Decimation boundary cycle: process new Python command.

            // NaN/Inf check
            if (!CheckFinite(cmd.pose)) {
                logger()->error("{}", "CartesianMotionForceControl: NaN/Inf in command pose");
                shm_->emergency_stop.store(true);
                is_running_.store(false);
                send_error = true;
            }

            if (!send_error) {
                // Jump check (threshold adapts to effective command interval)
                const double effective_interval = cmd_interval * cmd_decimation_;
                const double pos_jump_thresh = kMaxPositionVelocity * effective_interval;
                const double rot_jump_thresh = kMaxRotationVelocity * effective_interval;
                if (CheckCartesianJump(cmd.pose, last_sent_pose_,
                                       pos_jump_thresh, rot_jump_thresh)) {
                    cmd.pose         = last_sent_pose_;
                    cmd.wrench       = {};
                    cmd.velocity     = {};
                    cmd.acceleration = {};
                }

                // Timeout – hold last pose
                if (timed_out) {
                    cmd.pose         = last_sent_pose_;
                    cmd.wrench       = {};
                    cmd.velocity     = {};
                    cmd.acceleration = {};
                }

                // Start streaming interpolation toward the new command target
                if (interpolate_cmds_ && cmd_decimation_ > 1) {
                    // Duration = one command period (1/inner_control_hz seconds)
                    double period_sec = cmd_decimation_ * 0.001;
                    stream_interp_.init(last_sent_pose_, cmd.pose, period_sec);
                    // Step once immediately for this cycle
                    std::array<double,7> interp_pose;
                    std::array<double,6> interp_vel;
                    stream_interp_.step(interp_pose, interp_vel);
                    // Clamp before sending
                    ClampCartesianPose(interp_pose, last_sent_pose_);
                    ClampCartesianVelocity(interp_vel, last_sent_vel_);
                    try {
                        robot_.StreamCartesianMotionForce(interp_pose, cmd.wrench, interp_vel, {});
                    } catch (const std::exception& e) {
                        logger()->error("CartesianMotionForceControl: StreamCartesianMotionForce error: {}", e.what());
                        shm_->emergency_stop.store(true);
                        is_running_.store(false);
                        send_error = true;
                    }
                    if (!send_error) {
                        last_sent_pose_ = interp_pose;
                        last_sent_vel_  = interp_vel;
                    }
                } else {
                    // No interpolation: snap to command — clamp first
                    ClampCartesianPose(cmd.pose, last_sent_pose_);
                    ClampCartesianVelocity(cmd.velocity, last_sent_vel_);
                    try {
                        robot_.StreamCartesianMotionForce(
                            cmd.pose, cmd.wrench, cmd.velocity, cmd.acceleration);
                    } catch (const std::exception& e) {
                        logger()->error("CartesianMotionForceControl: StreamCartesianMotionForce error: {}", e.what());
                        shm_->emergency_stop.store(true);
                        is_running_.store(false);
                        send_error = true;
                    }
                    if (!send_error) {
                        last_sent_pose_ = cmd.pose;
                        last_sent_vel_  = cmd.velocity;
                    }
                }
            }
        }
    }

    // 6. Refresh robot states — ONLY on decimation boundary cycles.
    //
    //    robot_.states() is a heavy IPC/DDS call that can take 0.2–0.8 ms
    //    with occasional spikes > 1 ms (especially under system load / JAX JIT).
    //    Calling it every 1 ms cycle pushes the NEXT cycle's StreamCartesianMotionForce
    //    past the 1 ms deadline, which increments the Flexiv timeliness counter.
    //
    //    Since Python reads observations at << 1 kHz (typically 25–100 Hz), staleness
    //    of up to cmd_decimation_ ms (e.g. 2 ms at 500 Hz, 10 ms at 100 Hz) is
    //    invisible to the control loop.  On non-decimation cycles we skip states()
    //    entirely, keeping the RT callback as short as possible.
    if (!send_error && consume_cmd) {
        try {
            auto rs = robot_.states();
            std::lock_guard<std::mutex> lock(shm_mutex_);
            shm_->state_tcp_pose            = rs.tcp_pose;
            shm_->state_tcp_vel             = rs.tcp_vel;
            shm_->state_ext_wrench_in_tcp   = rs.ext_wrench_in_tcp;
            shm_->state_ext_wrench_in_world = rs.ext_wrench_in_world;
            shm_->state_ft_sensor_raw       = rs.ft_sensor_raw;
            shm_->state_q                   = rs.q;
            shm_->state_tau_ext             = rs.tau_ext;
            shm_->state_sequence.fetch_add(1, std::memory_order_relaxed);
        } catch (const std::exception& e) {
            logger()->warn("CartesianMotionForceControl: states() read failed: {}", e.what());
        }
    }
}

} // namespace flexiv_rt
