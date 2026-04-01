#include "realtime_control/joint_impedance_control.hpp"
#include "realtime_control/logging.hpp"
#include "realtime_control/rt_common.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace flexiv_rt {

// ---------------------------------------------------------------------------
// Helper: validate and store inner_control_hz / interpolate_cmds
// ---------------------------------------------------------------------------
void JointImpedanceControl::initControlParams(int inner_control_hz, bool interpolate_cmds)
{
    if (inner_control_hz < 1 || inner_control_hz > 1000) {
        throw std::invalid_argument(
            "inner_control_hz must be in [1, 1000], got "
            + std::to_string(inner_control_hz));
    }
    cmd_decimation_   = static_cast<int>(std::round(1000.0 / inner_control_hz));
    interpolate_cmds_ = interpolate_cmds;
    logger()->info(
        "JointImpedanceControl: inner_control_hz={} → decimation={} interpolate={}",
        inner_control_hz, cmd_decimation_, interpolate_cmds_);
}

JointImpedanceControl::JointImpedanceControl(
    flexiv::rdk::Robot& robot,
    std::unique_ptr<flexiv::rdk::Scheduler> pre_scheduler,
    std::string task_name,
    int  inner_control_hz,
    bool interpolate_cmds)
    : robot_(robot)
    , task_name_(std::move(task_name))
    , shm_(std::make_shared<JointSharedMemory>())
{
    initControlParams(inner_control_hz, interpolate_cmds);

    // Capture initial joint positions as the first hold target
    auto s = robot_.states();
    const size_t n = s.q.size();
    last_sent_pos_ = s.q;

    // Pre-allocate RT-local buffers (these will NEVER reallocate in the callback)
    rt_pos_ = s.q;
    rt_vel_.assign(n, 0.0);
    rt_acc_.assign(n, 0.0);
    last_sent_vel_.assign(n, 0.0);

    // Pre-allocate interpolation buffers
    interp_start_pos_.assign(n, 0.0);
    interp_target_pos_.assign(n, 0.0);
    interp_target_vel_.assign(n, 0.0);

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
        task_name_, 1, scheduler_->max_priority());
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
// RT periodic callback (runs on SCHED_FIFO thread at 1 kHz)
// ** ZERO heap allocation — all buffers pre-allocated in constructor **
// ---------------------------------------------------------------------------
void JointImpedanceControl::PeriodicCallback()
{
    // 1. Emergency stop – hold at last sent position
    if (shm_->emergency_stop.load()) {
        try {
            robot_.StreamJointPosition(last_sent_pos_, rt_vel_, rt_acc_);
        } catch (const std::exception&) {
            // Intentionally swallow SDK errors during emergency stop.
            // Do NOT use catch(...) — it would catch abi::__forced_unwind
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

    // 3. Frequency decimation
    ++cycle_counter_;
    const bool consume_cmd = (cycle_counter_ >= cmd_decimation_);
    if (consume_cmd) cycle_counter_ = 0;

    bool cmd_received = false;
    bool timed_out = false;
    double cmd_interval = kDefaultCommandInterval;
    bool send_error = false;

    if (consume_cmd) {
        // --- Decimation boundary: read new command from SHM ---
        {
            std::lock_guard<std::mutex> lock(shm_mutex_);

            const auto& cmd = shm_->command;
            std::copy(cmd.positions.begin(), cmd.positions.end(), rt_pos_.begin());
            std::copy(cmd.velocities.begin(), cmd.velocities.end(), rt_vel_.begin());
            std::copy(cmd.accelerations.begin(), cmd.accelerations.end(), rt_acc_.begin());

            cmd_received = shm_->command_received.load();
            cmd_interval = shm_->cmd_interval_sec;

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

        // 5. Jump check
        const double joint_jump_thresh = kMaxJointVelocity * cmd_interval;
        if (!last_sent_pos_.empty()
            && CheckJointJump(rt_pos_, last_sent_pos_, joint_jump_thresh)) {
            std::copy(last_sent_pos_.begin(), last_sent_pos_.end(), rt_pos_.begin());
            std::fill(rt_vel_.begin(), rt_vel_.end(), 0.0);
            std::fill(rt_acc_.begin(), rt_acc_.end(), 0.0);
        }

        // 6. Timeout – hold last position
        if (timed_out) {
            std::copy(last_sent_pos_.begin(), last_sent_pos_.end(), rt_pos_.begin());
            std::fill(rt_vel_.begin(), rt_vel_.end(), 0.0);
            std::fill(rt_acc_.begin(), rt_acc_.end(), 0.0);
        }

        // 7. Start linear interpolation or send immediately
        if (interpolate_cmds_ && cmd_decimation_ > 1) {
            // Snapshot current position as interpolation start
            std::copy(last_sent_pos_.begin(), last_sent_pos_.end(), interp_start_pos_.begin());
            std::copy(rt_pos_.begin(), rt_pos_.end(), interp_target_pos_.begin());

            double period_sec = cmd_decimation_ * 0.001;
            for (size_t i = 0; i < interp_target_vel_.size(); ++i) {
                interp_target_vel_[i] = (interp_target_pos_[i] - interp_start_pos_[i]) / period_sec;
            }

            interp_total_steps_   = static_cast<uint32_t>(cmd_decimation_);
            interp_current_step_  = 1;  // step once immediately
            interp_active_        = true;

            // Compute first interpolated position
            double t = 1.0 / static_cast<double>(interp_total_steps_);
            for (size_t i = 0; i < rt_pos_.size(); ++i) {
                rt_pos_[i] = interp_start_pos_[i] + t * (interp_target_pos_[i] - interp_start_pos_[i]);
                rt_vel_[i] = interp_target_vel_[i];
            }
            std::fill(rt_acc_.begin(), rt_acc_.end(), 0.0);

            // Clamp velocity and acceleration before sending
            ClampJointPosition(rt_pos_, last_sent_pos_);
            ClampJointVelocity(rt_vel_, last_sent_vel_);

            try {
                robot_.StreamJointPosition(rt_pos_, rt_vel_, rt_acc_);
            } catch (const std::exception& e) {
                logger()->error("JointImpedanceControl: StreamJointPosition error: {}", e.what());
                shm_->emergency_stop.store(true);
                is_running_.store(false);
                send_error = true;
            }
            if (!send_error) {
                std::copy(rt_pos_.begin(), rt_pos_.end(), last_sent_pos_.begin());
                std::copy(rt_vel_.begin(), rt_vel_.end(), last_sent_vel_.begin());
            }
        } else {
            // No interpolation: send command directly — clamp first
            ClampJointPosition(rt_pos_, last_sent_pos_);
            ClampJointVelocity(rt_vel_, last_sent_vel_);

            try {
                robot_.StreamJointPosition(rt_pos_, rt_vel_, rt_acc_);
            } catch (const std::exception& e) {
                logger()->error("JointImpedanceControl: StreamJointPosition error: {}", e.what());
                shm_->emergency_stop.store(true);
                is_running_.store(false);
                send_error = true;
            }
            if (!send_error) {
                std::copy(rt_pos_.begin(), rt_pos_.end(), last_sent_pos_.begin());
                std::copy(rt_vel_.begin(), rt_vel_.end(), last_sent_vel_.begin());
            }
        }
    } else {
        // --- Non-decimation cycle ---
        if (interpolate_cmds_ && interp_active_) {
            interp_current_step_++;

            if (interp_current_step_ >= interp_total_steps_) {
                // Interpolation done — snap to target
                std::copy(interp_target_pos_.begin(), interp_target_pos_.end(), rt_pos_.begin());
                std::fill(rt_vel_.begin(), rt_vel_.end(), 0.0);
                interp_active_ = false;
            } else {
                double t = static_cast<double>(interp_current_step_)
                         / static_cast<double>(interp_total_steps_);
                for (size_t i = 0; i < rt_pos_.size(); ++i) {
                    rt_pos_[i] = interp_start_pos_[i]
                               + t * (interp_target_pos_[i] - interp_start_pos_[i]);
                    rt_vel_[i] = interp_target_vel_[i];
                }
            }
            std::fill(rt_acc_.begin(), rt_acc_.end(), 0.0);

            // Clamp before sending
            ClampJointPosition(rt_pos_, last_sent_pos_);
            ClampJointVelocity(rt_vel_, last_sent_vel_);

            try {
                robot_.StreamJointPosition(rt_pos_, rt_vel_, rt_acc_);
            } catch (const std::exception& e) {
                logger()->error("JointImpedanceControl: StreamJointPosition error: {}", e.what());
                shm_->emergency_stop.store(true);
                is_running_.store(false);
                send_error = true;
            }
            if (!send_error) {
                std::copy(rt_pos_.begin(), rt_pos_.end(), last_sent_pos_.begin());
                std::copy(rt_vel_.begin(), rt_vel_.end(), last_sent_vel_.begin());
            }
        } else {
            // Hold last sent position
            try {
                robot_.StreamJointPosition(last_sent_pos_, rt_vel_, rt_acc_);
            } catch (const std::exception& e) {
                logger()->error("JointImpedanceControl: StreamJointPosition error: {}", e.what());
                shm_->emergency_stop.store(true);
                is_running_.store(false);
                send_error = true;
            }
        }
    }

    // 8. Refresh robot states — ONLY on decimation boundary cycles.
    //    robot_.states() is expensive IPC/DDS (0.2-0.8 ms); skip on
    //    non-decimation cycles to keep the RT callback short.
    if (!send_error && consume_cmd) {
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
    }
}

} // namespace flexiv_rt
