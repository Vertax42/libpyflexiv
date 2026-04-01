#pragma once
#include "rt_common.hpp"
#include "shared_memory.hpp"
#include "joint_state.hpp"
#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <vector>

namespace flexiv_rt {

struct JointCommand {
    std::vector<double> positions;     // [rad]
    std::vector<double> velocities;    // [rad/s]
    std::vector<double> accelerations; // [rad/s²]
};

struct JointSharedMemory {
    JointCommand command;
    std::chrono::steady_clock::time_point last_cmd_time;
    double cmd_interval_sec = kDefaultCommandInterval;  // Python command interval

    // RT -> Python (protected by shm_mutex)
    std::vector<double>  state_q;
    std::vector<double>  state_dq;
    std::vector<double>  state_tau;
    std::vector<double>  state_tau_ext;
    std::array<double,7> state_tcp_pose;

    std::atomic<bool>     emergency_stop{false};
    std::atomic<bool>     command_received{false};
    std::atomic<uint64_t> state_sequence{0};
};

class JointImpedanceControl {
public:
    /// @param inner_control_hz  How often the RT callback consumes a new Python
    ///   command (1-1000 Hz). Default=1000 (consume every 1 ms cycle).
    /// @param interpolate_cmds  When true, linearly interpolate between commands
    ///   for smooth streaming at low command rates (e.g. 30 Hz VLA policy).
    explicit JointImpedanceControl(
        flexiv::rdk::Robot& robot,
        std::unique_ptr<flexiv::rdk::Scheduler> pre_scheduler = nullptr,
        std::string task_name = "JointImpedanceRT",
        int  inner_control_hz = 1000,
        bool interpolate_cmds = true);
    ~JointImpedanceControl();

    // Python-thread-safe API
    void setTargetJoints(const std::vector<double>& positions,
                         const std::vector<double>& velocities     = {},
                         const std::vector<double>& accelerations  = {});
    JointState getState();
    void triggerEstop();
    bool isRunning() const;

    /// Trigger e-stop AND block until the RT scheduler thread has fully joined.
    /// Must be called before the Robot object is destroyed.  Idempotent.
    void stop();

private:
    void initControlParams(int inner_control_hz, bool interpolate_cmds);

    flexiv::rdk::Robot&    robot_;
    std::string            task_name_;
    std::unique_ptr<flexiv::rdk::Scheduler> scheduler_;
    std::shared_ptr<JointSharedMemory> shm_;
    std::mutex             shm_mutex_;
    std::vector<double>    last_sent_pos_;
    std::atomic<bool>      is_running_{true};
    std::atomic<bool>      stopped_{false};   // guards against double-stop

    // Pre-allocated RT-local buffers — NO heap allocation in PeriodicCallback
    std::vector<double>    rt_pos_;
    std::vector<double>    rt_vel_;
    std::vector<double>    rt_acc_;
    std::vector<double>    last_sent_vel_;  // for per-cycle acceleration clamping

    // --- Frequency decimation (RT thread only — no mutex needed) ---
    int  cmd_decimation_{1};
    int  cycle_counter_{0};
    bool interpolate_cmds_{false};

    // --- Joint linear interpolation state (RT thread only, pre-allocated) ---
    std::vector<double>    interp_start_pos_;
    std::vector<double>    interp_target_pos_;
    std::vector<double>    interp_target_vel_;
    uint32_t               interp_total_steps_{0};
    uint32_t               interp_current_step_{0};
    bool                   interp_active_{false};

    void PeriodicCallback();
};

} // namespace flexiv_rt
