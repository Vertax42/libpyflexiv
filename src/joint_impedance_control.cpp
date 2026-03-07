#include "realtime_control/joint_impedance_control.hpp"
#include "realtime_control/logging.hpp"
#include "realtime_control/rt_common.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace flexiv_rt {

JointImpedanceControl::JointImpedanceControl(
    flexiv::rdk::Robot& robot,
    std::unique_ptr<flexiv::rdk::Scheduler> pre_scheduler)
    : robot_(robot)
    , shm_(std::make_shared<JointSharedMemory>())
{
    // Capture initial joint positions as the first hold target
    auto s = robot_.states();
    const size_t n = s.q.size();
    last_sent_pos_ = s.q;

    // Pre-allocate RT-local buffers (these will NEVER reallocate in the callback)
    rt_pos_ = s.q;
    rt_vel_.assign(n, 0.0);
    rt_acc_.assign(n, 0.0);

    shm_->state_q       = s.q;
    shm_->state_dq      = s.dq;
    shm_->state_tau     = s.tau;
    shm_->state_tau_ext = s.tau_ext;
    shm_->state_tcp_pose = s.tcp_pose;

    // Pre-populate command with current position so the RT loop can hold
    // immediately before the Python side sends the first command.
    shm_->command.positions     = s.q;
    shm_->command.velocities.assign(n, 0.0);
    shm_->command.accelerations.assign(n, 0.0);

    // Use pre-created Scheduler or construct a new one.
    // mlockall() inside the constructor is intercepted by __wrap_mlockall
    // (no-op), so no OOM risk.
    if (pre_scheduler) {
        scheduler_ = std::move(pre_scheduler);
        logger()->info("{}", "JointImpedanceControl: using pre-created Scheduler");
    } else {
        scheduler_ = std::make_unique<flexiv::rdk::Scheduler>();
    }

    scheduler_->AddTask(
        [this]() { PeriodicCallback(); },
        "JointImpedanceRT", 1, scheduler_->max_priority());
    scheduler_->Start();
}

JointImpedanceControl::~JointImpedanceControl()
{
    stop();
}

void JointImpedanceControl::stop()
{
    // Idempotent: only the first caller executes the body.
    if (stopped_.exchange(true)) return;
    shm_->emergency_stop.store(true);
    is_running_.store(false);
    try {
        scheduler_.reset();   // ~Scheduler(): Stop() + sem_unlink (once, no double)
    } catch (...) {}
}

void JointImpedanceControl::setTargetJoints(
    const std::vector<double>& positions,
    const std::vector<double>& velocities,
    const std::vector<double>& accelerations)
{
    std::lock_guard<std::mutex> lock(shm_mutex_);
    auto now = std::chrono::steady_clock::now();

    // Compute command interval for adaptive jump detection
    if (shm_->command_received.load()) {
        double dt = std::chrono::duration<double>(now - shm_->last_cmd_time).count();
        shm_->cmd_interval_sec = std::clamp(dt, kMinCommandInterval, kMaxCommandInterval);
    }

    const size_t n = positions.size();
    shm_->command.positions     = positions;
    shm_->command.velocities    = velocities.empty()
                                    ? std::vector<double>(n, 0.0) : velocities;
    shm_->command.accelerations = accelerations.empty()
                                    ? std::vector<double>(n, 0.0) : accelerations;
    shm_->last_cmd_time         = now;
    shm_->command_received.store(true);
}

JointState JointImpedanceControl::getState()
{
    std::lock_guard<std::mutex> lock(shm_mutex_);
    JointState s;
    s.q        = shm_->state_q;
    s.dq       = shm_->state_dq;
    s.tau      = shm_->state_tau;
    s.tau_ext  = shm_->state_tau_ext;
    s.tcp_pose = shm_->state_tcp_pose;
    return s;
}

void JointImpedanceControl::triggerEstop()
{
    shm_->emergency_stop.store(true);
}

bool JointImpedanceControl::isRunning() const
{
    return is_running_.load();
}

// ---------------------------------------------------------------------------
// RT periodic callback (runs on SCHED_FIFO thread)
// ** ZERO heap allocation — all buffers pre-allocated in constructor **
// ---------------------------------------------------------------------------
void JointImpedanceControl::PeriodicCallback()
{
    // 1. Emergency stop – hold at last sent position
    if (shm_->emergency_stop.load()) {
        try {
            // rt_vel_/rt_acc_ are already zero from init; just send last_sent_pos_
            robot_.StreamJointPosition(last_sent_pos_, rt_vel_, rt_acc_);
        } catch (const std::exception&) {
            // Intentionally swallow SDK errors during emergency stop.
            // Do NOT use catch(...) here — it would catch abi::__forced_unwind
            // from pthread_cancel (used by Scheduler::Stop()) and cause
            // "FATAL: exception not rethrown" / std::terminate().
        }
        is_running_.store(false);
        return;
    }

    // 2. Robot fault
    if (robot_.fault()) {
        logger()->error("{}", "JointImpedanceControl: robot fault detected");
        shm_->emergency_stop.store(true);
        is_running_.store(false);
        return;
    }

    // 3. Lock and read/write shared memory — element-level copy, NO vector alloc
    bool cmd_received;
    bool timed_out = false;
    double cmd_interval = kDefaultCommandInterval;

    {
        std::lock_guard<std::mutex> lock(shm_mutex_);

        // Copy command into pre-allocated RT buffers (same size → no realloc)
        const auto& cmd = shm_->command;
        std::copy(cmd.positions.begin(), cmd.positions.end(), rt_pos_.begin());
        std::copy(cmd.velocities.begin(), cmd.velocities.end(), rt_vel_.begin());
        std::copy(cmd.accelerations.begin(), cmd.accelerations.end(), rt_acc_.begin());

        cmd_received = shm_->command_received.load();
        cmd_interval = shm_->cmd_interval_sec;

        // Timeout check – only after the first command has been received
        if (cmd_received) {
            auto elapsed = std::chrono::steady_clock::now() - shm_->last_cmd_time;
            if (elapsed > kCommandTimeout) {
                timed_out = true;
            }
        }
    }

    // 4. NaN/Inf check
    if (!CheckFinite(rt_pos_)) {
        logger()->error("{}", "JointImpedanceControl: NaN/Inf in command positions");
        shm_->emergency_stop.store(true);
        is_running_.store(false);
        return;
    }

    // 5. Jump check – clamp to last sent if delta too large
    //    Threshold = max_velocity * cmd_interval → adapts to Python frequency
    const double joint_jump_thresh = kMaxJointVelocity * cmd_interval;
    if (!last_sent_pos_.empty()
        && CheckJointJump(rt_pos_, last_sent_pos_, joint_jump_thresh)) {
        std::copy(last_sent_pos_.begin(), last_sent_pos_.end(), rt_pos_.begin());
        std::fill(rt_vel_.begin(), rt_vel_.end(), 0.0);
        std::fill(rt_acc_.begin(), rt_acc_.end(), 0.0);
    }

    // 6. Timeout – resend last position with zero vel/acc
    if (timed_out) {
        std::copy(last_sent_pos_.begin(), last_sent_pos_.end(), rt_pos_.begin());
        std::fill(rt_vel_.begin(), rt_vel_.end(), 0.0);
        std::fill(rt_acc_.begin(), rt_acc_.end(), 0.0);
    }

    // 7. Send command
    try {
        robot_.StreamJointPosition(rt_pos_, rt_vel_, rt_acc_);
    } catch (const std::exception& e) {
        logger()->error("JointImpedanceControl: StreamJointPosition error: {}", e.what());
        shm_->emergency_stop.store(true);
        is_running_.store(false);
        return;
    }

    // 8. Refresh robot states for Python-side monitoring.
    try {
        auto rs = robot_.states();
        std::lock_guard<std::mutex> lock(shm_mutex_);
        shm_->state_q        = rs.q;
        shm_->state_dq       = rs.dq;
        shm_->state_tau      = rs.tau;
        shm_->state_tau_ext  = rs.tau_ext;
        shm_->state_tcp_pose = rs.tcp_pose;
        shm_->state_sequence.fetch_add(1, std::memory_order_relaxed);
    } catch (const std::exception& e) {
        logger()->warn("JointImpedanceControl: states() read failed: {}", e.what());
    }

    // 9. Update last sent (element copy, no alloc — same size)
    std::copy(rt_pos_.begin(), rt_pos_.end(), last_sent_pos_.begin());
}

} // namespace flexiv_rt
