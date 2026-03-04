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
    explicit JointImpedanceControl(
        flexiv::rdk::Robot& robot,
        std::unique_ptr<flexiv::rdk::Scheduler> pre_scheduler = nullptr);
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
    flexiv::rdk::Robot&    robot_;
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

    void PeriodicCallback();
};

} // namespace flexiv_rt
