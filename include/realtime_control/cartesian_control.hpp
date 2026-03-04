#pragma once
#include "rt_common.hpp"
#include "shared_memory.hpp"
#include "cartesian_state.hpp"
#include "trajectory.hpp"
#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <vector>

namespace flexiv_bindings {

// ---------------------------------------------------------------------------
// RTCallbackProxy — registered with pre-started Scheduler, delegates to
// the actual PeriodicCallback once activated.
//
// Thread safety:  `activate()` is called once from the main thread.
//                 `call()` is invoked by the RT thread at 1 kHz.
//                 Release/acquire on `active_` ensures `fn_` is visible.
// ---------------------------------------------------------------------------
struct RTCallbackProxy {
    /// Called by the Scheduler RT thread every 1 ms.
    void call() {
        if (active_.load(std::memory_order_acquire)) {
            fn_();
        }
    }

    /// Set the real callback and enable it.  Must be called exactly once.
    void activate(std::function<void()> callback) {
        fn_ = std::move(callback);
        active_.store(true, std::memory_order_release);
    }

private:
    std::atomic<bool>      active_{false};
    std::function<void()>  fn_;
};

/// Bundle returned by PyRobot::precreate_scheduler() async task.
/// Holds a Scheduler that is already running an idle callback via the proxy.
struct PrestartedScheduler {
    std::unique_ptr<flexiv::rdk::Scheduler> scheduler;
    std::shared_ptr<RTCallbackProxy>        proxy;
};

struct CartesianCommand {
    std::array<double,7> pose         = {0,0,0,1,0,0,0};
    std::array<double,6> wrench       = {};
    std::array<double,6> velocity     = {};
    std::array<double,6> acceleration = {};
};

/// Request from Python thread to start a trajectory move (protected by shm_mutex).
struct MoveRequest {
    std::array<double,7> target_pose = {0,0,0,1,0,0,0};
    double duration_sec = 3.0;
    bool pending = false;
};

struct CartesianSharedMemory {
    CartesianCommand command;
    std::chrono::steady_clock::time_point last_cmd_time;
    double cmd_interval_sec = kDefaultCommandInterval;  // Python command interval

    // RT -> Python (protected by shm_mutex)
    std::array<double,7> state_tcp_pose;
    std::array<double,6> state_tcp_vel;
    std::array<double,6> state_ext_wrench_in_tcp;
    std::array<double,6> state_ext_wrench_in_world;
    std::array<double,6> state_ft_sensor_raw;
    std::vector<double>  state_q;
    std::vector<double>  state_tau_ext;

    // Move-to-pose request (Python → RT, protected by shm_mutex)
    MoveRequest move_request;

    // Move-to-pose status (RT → Python, lockless)
    std::atomic<bool>     is_moving{false};

    std::atomic<bool>     emergency_stop{false};
    std::atomic<bool>     command_received{false};
    std::atomic<uint64_t> state_sequence{0};
};

class CartesianMotionForceControl {
public:
    /// Legacy constructor: creates Scheduler (if nullptr), AddTask, Start.
    /// Blocks for ~2-4s.
    explicit CartesianMotionForceControl(
        flexiv::rdk::Robot& robot,
        std::unique_ptr<flexiv::rdk::Scheduler> pre_scheduler = nullptr);

    /// Pre-started constructor: Scheduler is already running an idle proxy.
    /// Just activates the callback — near-instant.
    CartesianMotionForceControl(
        flexiv::rdk::Robot& robot,
        PrestartedScheduler prestarted);

    ~CartesianMotionForceControl();

    // Python-thread-safe API
    void setTargetPose(const std::array<double,7>& pose,
                       const std::array<double,6>& wrench       = {},
                       const std::array<double,6>& velocity     = {},
                       const std::array<double,6>& acceleration = {});
    CartesianState getState();
    void triggerEstop();
    bool isRunning() const;

    // Move-to-pose API (trajectory in RT thread)
    void moveToPose(const std::array<double,7>& target_pose, double duration_sec);
    bool isMoving() const;
    void cancelMove();

    /// Trigger e-stop AND block until the RT scheduler thread has fully joined.
    /// Must be called before the Robot object is destroyed.  Idempotent.
    void stop();

private:
    void initSharedMemory();   // common init for both constructors

    flexiv::rdk::Robot&    robot_;
    std::unique_ptr<flexiv::rdk::Scheduler> scheduler_;
    std::shared_ptr<CartesianSharedMemory> shm_;
    std::shared_ptr<RTCallbackProxy>       proxy_;   // kept alive for pre-started path
    std::mutex             shm_mutex_;
    std::array<double,7>   last_sent_pose_;
    std::atomic<bool>      is_running_{true};
    std::atomic<bool>      stopped_{false};   // guards against double-stop

    // Trajectory state (RT thread only — no mutex needed)
    MinJerkTrajectory      trajectory_;
    enum class RTState { STREAMING, MOVING };
    RTState                rt_state_ = RTState::STREAMING;

    void PeriodicCallback();
};

} // namespace flexiv_bindings
