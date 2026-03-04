#include "realtime_control/joint_impedance_control.hpp"
#include "realtime_control/cartesian_control.hpp"
#include "realtime_control/joint_state.hpp"
#include "realtime_control/cartesian_state.hpp"
#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/data.hpp>
#include <flexiv/rdk/mode.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <exception>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <vector>

namespace py = pybind11;
using namespace flexiv_bindings;
using namespace flexiv::rdk;

// ---------------------------------------------------------------------------
// Helper: convert Python list<double> -> std::array<double, N>
// ---------------------------------------------------------------------------
template <size_t N>
std::array<double, N> ListToArray(const py::list& lst)
{
    if (lst.size() != N) {
        throw std::invalid_argument(
            "Expected list of size " + std::to_string(N)
            + ", got " + std::to_string(lst.size()));
    }
    std::array<double, N> arr;
    for (size_t i = 0; i < N; ++i) {
        arr[i] = lst[i].cast<double>();
    }
    return arr;
}

template <size_t N>
py::list ArrayToList(const std::array<double, N>& arr)
{
    py::list lst;
    for (const auto& v : arr) {
        lst.append(v);
    }
    return lst;
}

JPos VectorToJPos(const std::vector<double>& q_m_vec)
{
    if (q_m_vec.size() != kSerialJointDoF) {
        throw std::invalid_argument(
            "Expected joint target of size " + std::to_string(kSerialJointDoF)
            + ", got " + std::to_string(q_m_vec.size()));
    }

    std::array<double, kSerialJointDoF> q_m = {};
    for (size_t i = 0; i < kSerialJointDoF; ++i) {
        q_m[i] = q_m_vec[i];
    }
    return JPos(q_m);
}

// ---------------------------------------------------------------------------
// PyRobot: thin Python wrapper around flexiv::rdk::Robot
// ---------------------------------------------------------------------------
class PyRobot {
public:
    PyRobot(const std::string& robot_sn,
            const std::vector<std::string>& network_interface_whitelist = {},
            bool verbose = true,
            unsigned int connect_retries = 3,
            double retry_interval_sec = 1.0)
    {
        if (connect_retries == 0) {
            throw std::invalid_argument("connect_retries must be >= 1");
        }

        std::exception_ptr last_error;
        for (unsigned int attempt = 1; attempt <= connect_retries; ++attempt) {
            try {
                robot_ = std::make_unique<Robot>(
                    robot_sn, network_interface_whitelist, verbose);
                return;
            } catch (...) {
                last_error = std::current_exception();
                if (attempt < connect_retries && retry_interval_sec > 0.0) {
                    std::this_thread::sleep_for(
                        std::chrono::duration<double>(retry_interval_sec));
                }
            }
        }

        if (last_error) {
            std::rethrow_exception(last_error);
        }
        throw std::runtime_error("Failed to connect robot");
    }

    ~PyRobot()
    {
        close();
    }

    // ── Accessors ──
    bool connected()      const { return robot_->connected(); }
    bool operational()    const { return robot_->operational(); }
    bool fault()          const { return robot_->fault(); }
    bool estop_released() const { return robot_->estop_released(); }
    bool busy()           const { return robot_->busy(); }
    bool stopped()        const { return robot_->stopped(); }
    bool reduced()        const { return robot_->reduced(); }
    bool recovery()       const { return robot_->recovery(); }
    RobotInfo   info()    const { return robot_->info(); }
    RobotStates states()  const { return robot_->states(); }
    Mode        mode()    const { return robot_->mode(); }

    // ── System control ──
    void Enable()                         { robot_->Enable(); }
    void Stop() {
        stop_active_controls();
        stop_robot();
    }
    void close() noexcept {
        if (closed_.exchange(true)) return;
        stop_active_controls();
        try {
            stop_robot();
        } catch (...) {}
    }
    PyRobot* enter() { return this; }
    void exit(py::object, py::object, py::object) {
        py::gil_scoped_release release;
        close();
    }
    bool ClearFault(unsigned int timeout) { return robot_->ClearFault(timeout); }
    void SwitchMode(Mode m)               { robot_->SwitchMode(m); }

    // ── Plan / Primitive execution ──
    void ExecutePlan(const std::string& name) {
        robot_->ExecutePlan(name);
    }
    void ExecutePrimitive(const std::string& name,
                          const std::map<std::string, FlexivDataTypes>& params = {}) {
        auto converted_params = params;

        // For convenience in Python, allow MoveJ "target" to be a plain list[7] in degree.
        if (name == "MoveJ") {
            auto target_it = converted_params.find("target");
            if (target_it != converted_params.end()) {
                if (auto q_m_vec = std::get_if<std::vector<double>>(&target_it->second)) {
                    target_it->second = VectorToJPos(*q_m_vec);
                }
            }
        }

        robot_->ExecutePrimitive(name, converted_params);
    }
    std::map<std::string, FlexivDataTypes> primitive_states() const {
        return robot_->primitive_states();
    }

    // ── Joint impedance config ──
    void SetJointImpedance(const std::vector<double>& K_q,
                           const std::vector<double>& Z_q = {}) {
        robot_->SetJointImpedance(K_q, Z_q);
    }
    void SetMaxContactTorque(const std::vector<double>& max_torques) {
        robot_->SetMaxContactTorque(max_torques);
    }

    // ── Cartesian config ──
    void SetCartesianImpedance(const py::list& K_x_list,
                               const py::list& Z_x_list = py::list()) {
        auto K_x = ListToArray<6>(K_x_list);
        if (Z_x_list.empty()) {
            robot_->SetCartesianImpedance(K_x);
        } else {
            robot_->SetCartesianImpedance(K_x, ListToArray<6>(Z_x_list));
        }
    }
    void SetMaxContactWrench(const py::list& max_wrench_list) {
        robot_->SetMaxContactWrench(ListToArray<6>(max_wrench_list));
    }
    void SetForceControlAxis(const py::list& enabled_axes_list,
                             const py::list& max_linear_vel_list = py::list()) {
        if (enabled_axes_list.size() != 6) {
            throw std::invalid_argument("enabled_axes must have 6 elements");
        }
        std::array<bool, 6> enabled_axes;
        for (size_t i = 0; i < 6; ++i) {
            enabled_axes[i] = enabled_axes_list[i].cast<bool>();
        }
        if (max_linear_vel_list.empty()) {
            robot_->SetForceControlAxis(enabled_axes);
        } else {
            robot_->SetForceControlAxis(enabled_axes, ListToArray<3>(max_linear_vel_list));
        }
    }
    void SetForceControlFrame(CoordType root_coord,
                              const py::list& T_in_root_list = py::list()) {
        if (T_in_root_list.empty()) {
            robot_->SetForceControlFrame(root_coord, {0,0,0,1,0,0,0});
        } else {
            robot_->SetForceControlFrame(root_coord, ListToArray<7>(T_in_root_list));
        }
    }
    void SetNullSpacePosture(const std::vector<double>& ref_positions) {
        robot_->SetNullSpacePosture(ref_positions);
    }

    // ── RT control entry points ──
    std::shared_ptr<JointImpedanceControl> start_joint_impedance_control() {
        if (closed_.load()) {
            throw std::runtime_error("Robot is closed");
        }
        // JointImpedanceControl always creates its own Scheduler (no pre-start).
        auto ctrl = std::make_shared<JointImpedanceControl>(*robot_, nullptr);
        register_joint_control(ctrl);
        return ctrl;
    }
    std::shared_ptr<CartesianMotionForceControl> start_cartesian_control() {
        if (closed_.load()) {
            throw std::runtime_error("Robot is closed");
        }
        auto prestarted = take_prestarted();
        std::shared_ptr<CartesianMotionForceControl> ctrl;
        if (prestarted) {
            // Fast path: Scheduler already running idle proxy
            ctrl = std::make_shared<CartesianMotionForceControl>(
                *robot_, std::move(*prestarted));
        } else {
            // Fallback: legacy path (Scheduler + AddTask + Start)
            ctrl = std::make_shared<CartesianMotionForceControl>(*robot_);
        }
        register_cartesian_control(ctrl);
        return ctrl;
    }

    // ── Scheduler pre-creation + pre-start (overlap with homing/FT-zeroing) ──
    // Launches async thread that: Scheduler() ~2s + AddTask(idle) + Start() ~2s
    // Total ~4s hidden behind homing + FT-zeroing.
    void precreate_scheduler() {
        std::lock_guard<std::mutex> lock(scheduler_mutex_);
        if (prestarted_future_.valid()) return;  // already in progress
        prestarted_future_ = std::async(std::launch::async, []() {
            auto sched = std::make_unique<flexiv::rdk::Scheduler>();
            auto proxy = std::make_shared<RTCallbackProxy>();
            sched->AddTask(
                [proxy]() { proxy->call(); },
                "CartesianRT", 1, sched->max_priority());
            sched->Start();
            return PrestartedScheduler{std::move(sched), std::move(proxy)};
        });
    }

private:
    std::optional<PrestartedScheduler> take_prestarted() {
        std::lock_guard<std::mutex> lock(scheduler_mutex_);
        if (prestarted_future_.valid()) {
            return prestarted_future_.get();  // blocks if not ready yet
        }
        return std::nullopt;
    }

    void stop_robot()
    {
        if (!robot_stop_sent_.exchange(true)) {
            robot_->Stop();
        }
    }

    void stop_active_controls() noexcept
    {
        std::vector<std::shared_ptr<JointImpedanceControl>> joint_controls;
        std::vector<std::shared_ptr<CartesianMotionForceControl>> cart_controls;
        {
            std::lock_guard<std::mutex> lock(active_controls_mutex_);
            joint_controls.reserve(active_joint_controls_.size());
            for (auto& weak_ctrl : active_joint_controls_) {
                if (auto ctrl = weak_ctrl.lock()) {
                    joint_controls.push_back(std::move(ctrl));
                }
            }
            cart_controls.reserve(active_cart_controls_.size());
            for (auto& weak_ctrl : active_cart_controls_) {
                if (auto ctrl = weak_ctrl.lock()) {
                    cart_controls.push_back(std::move(ctrl));
                }
            }
            active_joint_controls_.clear();
            active_cart_controls_.clear();
        }

        for (auto& ctrl : joint_controls) {
            try {
                ctrl->stop();
            } catch (...) {}
        }
        for (auto& ctrl : cart_controls) {
            try {
                ctrl->stop();
            } catch (...) {}
        }
    }

    void register_joint_control(const std::shared_ptr<JointImpedanceControl>& ctrl)
    {
        std::lock_guard<std::mutex> lock(active_controls_mutex_);
        active_joint_controls_.erase(
            std::remove_if(active_joint_controls_.begin(), active_joint_controls_.end(),
                           [](const auto& weak_ctrl) { return weak_ctrl.expired(); }),
            active_joint_controls_.end());
        active_joint_controls_.push_back(ctrl);
    }

    void register_cartesian_control(const std::shared_ptr<CartesianMotionForceControl>& ctrl)
    {
        std::lock_guard<std::mutex> lock(active_controls_mutex_);
        active_cart_controls_.erase(
            std::remove_if(active_cart_controls_.begin(), active_cart_controls_.end(),
                           [](const auto& weak_ctrl) { return weak_ctrl.expired(); }),
            active_cart_controls_.end());
        active_cart_controls_.push_back(ctrl);
    }

    std::unique_ptr<Robot> robot_;
    std::mutex active_controls_mutex_;
    std::vector<std::weak_ptr<JointImpedanceControl>> active_joint_controls_;
    std::vector<std::weak_ptr<CartesianMotionForceControl>> active_cart_controls_;
    std::atomic<bool> robot_stop_sent_{false};
    std::atomic<bool> closed_{false};
    std::future<PrestartedScheduler> prestarted_future_;
    std::mutex scheduler_mutex_;
};

// ---------------------------------------------------------------------------
// Python context manager for JointImpedanceControl
// ---------------------------------------------------------------------------
class PyJointImpedanceControl {
public:
    explicit PyJointImpedanceControl(std::shared_ptr<JointImpedanceControl> ctrl)
        : ctrl_(std::move(ctrl)) {}

    void set_target_joints(const std::vector<double>& positions,
                           const std::vector<double>& velocities = {},
                           const std::vector<double>& accelerations = {}) {
        ctrl_->setTargetJoints(positions, velocities, accelerations);
    }
    JointState get_state()      { return ctrl_->getState(); }
    void trigger_estop()        { ctrl_->triggerEstop(); }
    void stop() {
        py::gil_scoped_release release;
        ctrl_->stop();
    }
    bool is_running() const     { return ctrl_->isRunning(); }

    // Context manager
    PyJointImpedanceControl* enter() { return this; }
    void exit(py::object, py::object, py::object) {
        stop();
    }

private:
    std::shared_ptr<JointImpedanceControl> ctrl_;
};

// ---------------------------------------------------------------------------
// Python context manager for CartesianMotionForceControl
// ---------------------------------------------------------------------------
class PyCartesianMotionForceControl {
public:
    explicit PyCartesianMotionForceControl(std::shared_ptr<CartesianMotionForceControl> ctrl)
        : ctrl_(std::move(ctrl)) {}

    void set_target_pose(const py::list& pose_list,
                         const py::list& wrench_list       = py::list(),
                         const py::list& velocity_list     = py::list(),
                         const py::list& acceleration_list = py::list()) {
        auto pose = ListToArray<7>(pose_list);
        std::array<double,6> wrench       = {};
        std::array<double,6> velocity     = {};
        std::array<double,6> acceleration = {};
        if (!wrench_list.empty())       wrench       = ListToArray<6>(wrench_list);
        if (!velocity_list.empty())     velocity     = ListToArray<6>(velocity_list);
        if (!acceleration_list.empty()) acceleration = ListToArray<6>(acceleration_list);
        ctrl_->setTargetPose(pose, wrench, velocity, acceleration);
    }
    CartesianState get_state()  { return ctrl_->getState(); }
    void trigger_estop()        { ctrl_->triggerEstop(); }
    void stop() {
        py::gil_scoped_release release;
        ctrl_->stop();
    }
    bool is_running() const     { return ctrl_->isRunning(); }

    // --- Move-to-pose API ---
    void move_to_pose(const py::list& pose_list, double duration_sec = 3.0) {
        auto pose = ListToArray<7>(pose_list);
        ctrl_->moveToPose(pose, duration_sec);
    }

    bool move_to_pose_sync(const py::list& pose_list,
                           double duration_sec = 3.0,
                           double timeout_sec = 0.0) {
        auto pose = ListToArray<7>(pose_list);
        ctrl_->moveToPose(pose, duration_sec);

        // Release GIL and poll isMoving() every 10 ms
        py::gil_scoped_release release;

        double effective_timeout = timeout_sec > 0.0
            ? timeout_sec
            : duration_sec + 2.0;  // generous default: duration + 2s margin

        auto deadline = std::chrono::steady_clock::now()
                      + std::chrono::duration<double>(effective_timeout);

        while (ctrl_->isMoving()) {
            if (std::chrono::steady_clock::now() > deadline) {
                ctrl_->cancelMove();
                return false;  // timeout
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return true;  // success
    }

    bool is_moving() const { return ctrl_->isMoving(); }

    void cancel_move() { ctrl_->cancelMove(); }

    // Context manager
    PyCartesianMotionForceControl* enter() { return this; }
    void exit(py::object, py::object, py::object) {
        stop();
    }

private:
    std::shared_ptr<CartesianMotionForceControl> ctrl_;
};

// ---------------------------------------------------------------------------
// Module definition
// ---------------------------------------------------------------------------
PYBIND11_MODULE(_flexiv_bindings, m)
{
    m.doc() = "Python bindings for Flexiv RDK real-time control";

    // ── Mode enum ──
    py::enum_<Mode>(m, "Mode")
        .value("UNKNOWN",                   Mode::UNKNOWN)
        .value("IDLE",                      Mode::IDLE)
        .value("RT_JOINT_TORQUE",           Mode::RT_JOINT_TORQUE)
        .value("RT_JOINT_IMPEDANCE",        Mode::RT_JOINT_IMPEDANCE)
        .value("NRT_JOINT_IMPEDANCE",       Mode::NRT_JOINT_IMPEDANCE)
        .value("RT_JOINT_POSITION",         Mode::RT_JOINT_POSITION)
        .value("NRT_JOINT_POSITION",        Mode::NRT_JOINT_POSITION)
        .value("NRT_PLAN_EXECUTION",        Mode::NRT_PLAN_EXECUTION)
        .value("NRT_PRIMITIVE_EXECUTION",   Mode::NRT_PRIMITIVE_EXECUTION)
        .value("RT_CARTESIAN_MOTION_FORCE", Mode::RT_CARTESIAN_MOTION_FORCE)
        .value("NRT_CARTESIAN_MOTION_FORCE",Mode::NRT_CARTESIAN_MOTION_FORCE)
        .export_values();

    // ── CoordType enum ──
    py::enum_<CoordType>(m, "CoordType")
        .value("WORLD", CoordType::WORLD)
        .value("TCP",   CoordType::TCP)
        .export_values();

    // ── RobotInfo ──
    py::class_<RobotInfo>(m, "RobotInfo")
        .def_readonly("serial_num",   &RobotInfo::serial_num)
        .def_readonly("software_ver", &RobotInfo::software_ver)
        .def_readonly("model_name",   &RobotInfo::model_name)
        .def_readonly("DoF",          &RobotInfo::DoF)
        .def_readonly("DoF_m",        &RobotInfo::DoF_m)
        .def_readonly("K_q_nom",      &RobotInfo::K_q_nom)
        .def_readonly("K_x_nom",      &RobotInfo::K_x_nom)
        .def_readonly("q_min",        &RobotInfo::q_min)
        .def_readonly("q_max",        &RobotInfo::q_max)
        .def_readonly("dq_max",       &RobotInfo::dq_max)
        .def_readonly("tau_max",      &RobotInfo::tau_max)
        .def_readonly("has_FT_sensor",&RobotInfo::has_FT_sensor);

    // ── RobotStates ──
    py::class_<RobotStates>(m, "RobotStates")
        .def_readonly("q",                   &RobotStates::q)
        .def_readonly("dq",                  &RobotStates::dq)
        .def_readonly("theta",               &RobotStates::theta)
        .def_readonly("dtheta",              &RobotStates::dtheta)
        .def_readonly("tau",                 &RobotStates::tau)
        .def_readonly("tau_des",             &RobotStates::tau_des)
        .def_readonly("tau_dot",             &RobotStates::tau_dot)
        .def_readonly("tau_ext",             &RobotStates::tau_ext)
        .def_readonly("tcp_pose",            &RobotStates::tcp_pose)
        .def_readonly("tcp_vel",             &RobotStates::tcp_vel)
        .def_readonly("flange_pose",         &RobotStates::flange_pose)
        .def_readonly("ft_sensor_raw",       &RobotStates::ft_sensor_raw)
        .def_readonly("ext_wrench_in_tcp",   &RobotStates::ext_wrench_in_tcp)
        .def_readonly("ext_wrench_in_world", &RobotStates::ext_wrench_in_world);

    // ── JointState snapshot ──
    py::class_<JointState>(m, "JointState")
        .def_readonly("q",        &JointState::q)
        .def_readonly("dq",       &JointState::dq)
        .def_readonly("tau",      &JointState::tau)
        .def_readonly("tau_ext",  &JointState::tau_ext)
        .def_readonly("tcp_pose", &JointState::tcp_pose);

    // ── CartesianState snapshot ──
    py::class_<CartesianState>(m, "CartesianState")
        .def_readonly("tcp_pose",            &CartesianState::tcp_pose)
        .def_readonly("tcp_vel",             &CartesianState::tcp_vel)
        .def_readonly("ext_wrench_in_tcp",   &CartesianState::ext_wrench_in_tcp)
        .def_readonly("ext_wrench_in_world", &CartesianState::ext_wrench_in_world)
        .def_readonly("ft_sensor_raw",       &CartesianState::ft_sensor_raw)
        .def_readonly("q",                   &CartesianState::q)
        .def_readonly("tau_ext",             &CartesianState::tau_ext);

    // ── JointImpedanceControl ──
    py::class_<PyJointImpedanceControl, std::shared_ptr<PyJointImpedanceControl>>(
        m, "JointImpedanceControl")
        .def("set_target_joints",
             &PyJointImpedanceControl::set_target_joints,
             py::arg("positions"),
             py::arg("velocities")     = std::vector<double>{},
             py::arg("accelerations")  = std::vector<double>{})
        .def("get_state",      &PyJointImpedanceControl::get_state)
        .def("trigger_estop",  &PyJointImpedanceControl::trigger_estop)
        .def("stop",           &PyJointImpedanceControl::stop)
        .def("is_running",     &PyJointImpedanceControl::is_running)
        .def("__enter__",      &PyJointImpedanceControl::enter,
             py::return_value_policy::reference_internal)
        .def("__exit__",       &PyJointImpedanceControl::exit);

    // ── CartesianMotionForceControl ──
    py::class_<PyCartesianMotionForceControl, std::shared_ptr<PyCartesianMotionForceControl>>(
        m, "CartesianMotionForceControl")
        .def("set_target_pose",
             &PyCartesianMotionForceControl::set_target_pose,
             py::arg("pose"),
             py::arg("wrench")       = py::list(),
             py::arg("velocity")     = py::list(),
             py::arg("acceleration") = py::list())
        .def("get_state",     &PyCartesianMotionForceControl::get_state)
        .def("trigger_estop", &PyCartesianMotionForceControl::trigger_estop)
        .def("stop",          &PyCartesianMotionForceControl::stop)
        .def("is_running",    &PyCartesianMotionForceControl::is_running)
        .def("move_to_pose",  &PyCartesianMotionForceControl::move_to_pose,
             py::arg("pose"),
             py::arg("duration_sec") = 3.0,
             "Start a min-jerk trajectory to target pose (non-blocking)")
        .def("move_to_pose_sync", &PyCartesianMotionForceControl::move_to_pose_sync,
             py::arg("pose"),
             py::arg("duration_sec") = 3.0,
             py::arg("timeout_sec")  = 0.0,
             "Move to target pose and block until complete (releases GIL)")
        .def("is_moving",     &PyCartesianMotionForceControl::is_moving,
             "Whether a trajectory move is currently in progress")
        .def("cancel_move",   &PyCartesianMotionForceControl::cancel_move,
             "Cancel the current trajectory move")
        .def("__enter__",     &PyCartesianMotionForceControl::enter,
             py::return_value_policy::reference_internal)
        .def("__exit__",      &PyCartesianMotionForceControl::exit);

    // ── Robot ──
    py::class_<PyRobot>(m, "Robot")
        .def(py::init<const std::string&,
                      const std::vector<std::string>&,
                      bool,
                      unsigned int,
                      double>(),
             py::arg("robot_sn"),
             py::arg("network_interface_whitelist") = std::vector<std::string>{},
             py::arg("verbose") = true,
             py::arg("connect_retries") = 3u,
             py::arg("retry_interval_sec") = 1.0)
        // Accessors
        .def("connected",       &PyRobot::connected)
        .def("operational",     &PyRobot::operational)
        .def("fault",           &PyRobot::fault)
        .def("estop_released",  &PyRobot::estop_released)
        .def("busy",            &PyRobot::busy)
        .def("stopped",         &PyRobot::stopped)
        .def("reduced",         &PyRobot::reduced)
        .def("recovery",        &PyRobot::recovery)
        .def("info",            &PyRobot::info)
        .def("states",          &PyRobot::states)
        .def("mode",            &PyRobot::mode)
        // System control
        .def("Enable",          &PyRobot::Enable)
        .def("Stop",            [](PyRobot& self) {
            py::gil_scoped_release release;
            self.Stop();
        })
        .def("close",           [](PyRobot& self) {
            py::gil_scoped_release release;
            self.close();
        })
        .def("__enter__",       &PyRobot::enter,
             py::return_value_policy::reference_internal)
        .def("__exit__",        &PyRobot::exit)
        .def("ClearFault",      &PyRobot::ClearFault, py::arg("timeout_sec") = 30u)
        .def("SwitchMode",      &PyRobot::SwitchMode)
        // Plan / Primitive
        .def("ExecutePlan",      &PyRobot::ExecutePlan)
        .def("ExecutePrimitive", &PyRobot::ExecutePrimitive,
             py::arg("name"),
             py::arg("params") = std::map<std::string, FlexivDataTypes>{})
        .def("primitive_states", &PyRobot::primitive_states)
        // Joint impedance config
        .def("SetJointImpedance",    &PyRobot::SetJointImpedance,
             py::arg("K_q"), py::arg("Z_q") = std::vector<double>{})
        .def("SetMaxContactTorque",  &PyRobot::SetMaxContactTorque)
        // Cartesian config
        .def("SetCartesianImpedance", &PyRobot::SetCartesianImpedance,
             py::arg("K_x"), py::arg("Z_x") = py::list())
        .def("SetMaxContactWrench",   &PyRobot::SetMaxContactWrench)
        .def("SetForceControlAxis",   &PyRobot::SetForceControlAxis,
             py::arg("enabled_axes"),
             py::arg("max_linear_vel") = py::list())
        .def("SetForceControlFrame",  &PyRobot::SetForceControlFrame,
             py::arg("root_coord"),
             py::arg("T_in_root") = py::list())
        .def("SetNullSpacePosture",   &PyRobot::SetNullSpacePosture)
        // Scheduler pre-creation (overlap with homing to hide latency)
        .def("precreate_scheduler", [](PyRobot& self) {
            py::gil_scoped_release release;
            self.precreate_scheduler();
        })
        // RT control entry points – wrap in Python helper classes
        // GIL must be released while Scheduler creates SCHED_FIFO threads
        .def("start_joint_impedance_control",
             [](PyRobot& self) {
                 std::shared_ptr<JointImpedanceControl> ctrl;
                 {
                     py::gil_scoped_release release;
                     ctrl = self.start_joint_impedance_control();
                 }
                 return std::make_shared<PyJointImpedanceControl>(ctrl);
             },
             py::keep_alive<0, 1>())
        .def("start_cartesian_control",
             [](PyRobot& self) {
                 std::shared_ptr<CartesianMotionForceControl> ctrl;
                 {
                     py::gil_scoped_release release;
                     ctrl = self.start_cartesian_control();
                 }
                 return std::make_shared<PyCartesianMotionForceControl>(ctrl);
             },
             py::keep_alive<0, 1>());
}
