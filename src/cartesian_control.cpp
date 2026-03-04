#include "realtime_control/cartesian_control.hpp"
#include "realtime_control/logging.hpp"
#include "realtime_control/rt_common.hpp"
#include <cerrno>
#include <cmath>
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
    flexiv_bindings::logger()->info("mlockall(flags={}) intercepted — skipped (OOM prevention)", flags);
    return 0;  // pretend success
}

namespace flexiv_bindings {

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
// Legacy constructor: Scheduler() + AddTask + Start() — blocks ~2-4s
// ---------------------------------------------------------------------------
CartesianMotionForceControl::CartesianMotionForceControl(
    flexiv::rdk::Robot& robot,
    std::unique_ptr<flexiv::rdk::Scheduler> pre_scheduler)
    : robot_(robot)
    , shm_(std::make_shared<CartesianSharedMemory>())
{
    initSharedMemory();

    // Use pre-created Scheduler or construct a new one.
    // mlockall() inside the constructor is intercepted by __wrap_mlockall
    // (no-op), so no OOM risk.
    if (pre_scheduler) {
        scheduler_ = std::move(pre_scheduler);
        logger()->info("{}", "CartesianMotionForceControl: using pre-created Scheduler");
    } else {
        scheduler_ = std::make_unique<flexiv::rdk::Scheduler>();
    }

    scheduler_->AddTask(
        [this]() { PeriodicCallback(); },
        "CartesianRT", 1, scheduler_->max_priority());
    logger()->info("{}", "CartesianMotionForceControl: AddTask done, calling Start()...");
    scheduler_->Start();
    logger()->info("{}", "CartesianMotionForceControl: Start() returned, RT thread running");
}

// ---------------------------------------------------------------------------
// Pre-started constructor: Scheduler already running idle proxy — near-instant
// ---------------------------------------------------------------------------
CartesianMotionForceControl::CartesianMotionForceControl(
    flexiv::rdk::Robot& robot,
    PrestartedScheduler prestarted)
    : robot_(robot)
    , scheduler_(std::move(prestarted.scheduler))
    , shm_(std::make_shared<CartesianSharedMemory>())
    , proxy_(std::move(prestarted.proxy))
{
    initSharedMemory();

    // Activate the proxy: RT thread starts calling PeriodicCallback()
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
    try {
        if (scheduler_) scheduler_->Stop();   // blocks until the RT thread has joined
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
    if (duration_sec <= 0.0 || !std::isfinite(duration_sec)) {
        throw std::invalid_argument("moveToPose: duration_sec must be > 0 and finite");
    }

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

    // 3. Lock and read commands from SHM (short lock — just memory copies)
    MoveRequest move_req;
    CartesianCommand cmd;
    bool cmd_received = false;
    bool timed_out = false;
    double cmd_interval = kDefaultCommandInterval;

    {
        std::lock_guard<std::mutex> lock(shm_mutex_);

        // Read move request
        move_req = shm_->move_request;
        if (move_req.pending) {
            shm_->move_request.pending = false;  // consume the request
        }

        // Read streaming command (needed for STREAMING state)
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

    // 3b. Refresh robot states into SHM (outside lock to minimize contention).
    //     robot_.states() is a potentially slow IPC call — holding the mutex
    //     during it blocks Python's setTargetPose()/getState() and can cause
    //     the RT thread to miss the 1 ms deadline.
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

    // 5. Execute based on current state
    if (rt_state_ == RTState::MOVING) {
        // --- MOVING state ---
        std::array<double,7> interp_pose;
        std::array<double,6> interp_velocity;

        bool in_progress = trajectory_.step(interp_pose, interp_velocity);

        if (!in_progress) {
            // Trajectory complete (or cancelled) — switch back to STREAMING
            rt_state_ = RTState::STREAMING;
            shm_->is_moving.store(false);

            // Use the final trajectory pose as the streaming command
            // so the robot holds position smoothly
            interp_pose = trajectory_.isActive() ? interp_pose : last_sent_pose_;
            // If trajectory just finished (not cancelled), interp_pose is end_pose
            // which was set by the last step() call.

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
                return;
            }
            return;
        }

        // NaN/Inf check on interpolated pose (should never happen with min-jerk, but safety first)
        if (!CheckFinite(interp_pose)) {
            logger()->error("{}", "CartesianMotionForceControl: NaN/Inf in trajectory pose");
            shm_->emergency_stop.store(true);
            is_running_.store(false);
            shm_->is_moving.store(false);
            return;
        }

        // Send interpolated command (skip jump check — min-jerk guarantees continuity)
        try {
            robot_.StreamCartesianMotionForce(
                interp_pose, {}, interp_velocity, {});
        } catch (const std::exception& e) {
            logger()->error("CartesianMotionForceControl: StreamCartesianMotionForce error: {}", e.what());
            shm_->emergency_stop.store(true);
            is_running_.store(false);
            shm_->is_moving.store(false);
            return;
        }

        last_sent_pose_ = interp_pose;
        return;
    }

    // --- STREAMING state (existing logic) ---

    // 4s. NaN/Inf check on pose
    if (!CheckFinite(cmd.pose)) {
        logger()->error("{}", "CartesianMotionForceControl: NaN/Inf in command pose");
        shm_->emergency_stop.store(true);
        is_running_.store(false);
        return;
    }

    // 5s. Jump check – position and rotation
    //     Thresholds = max_velocity * cmd_interval → adapts to Python frequency
    const double pos_jump_thresh = kMaxPositionVelocity * cmd_interval;
    const double rot_jump_thresh = kMaxRotationVelocity * cmd_interval;
    if (CheckCartesianJump(cmd.pose, last_sent_pose_,
                           pos_jump_thresh, rot_jump_thresh)) {
        cmd.pose         = last_sent_pose_;
        cmd.wrench       = {};
        cmd.velocity     = {};
        cmd.acceleration = {};
    }

    // 6s. Timeout – hold last pose, zero dynamics
    if (timed_out) {
        cmd.pose         = last_sent_pose_;
        cmd.wrench       = {};
        cmd.velocity     = {};
        cmd.acceleration = {};
    }

    // 7s. Send command
    try {
        robot_.StreamCartesianMotionForce(
            cmd.pose, cmd.wrench, cmd.velocity, cmd.acceleration);
    } catch (const std::exception& e) {
        logger()->error("CartesianMotionForceControl: StreamCartesianMotionForce error: {}", e.what());
        shm_->emergency_stop.store(true);
        is_running_.store(false);
        return;
    }

    // 8s. Update last sent
    last_sent_pose_ = cmd.pose;
}

} // namespace flexiv_bindings
