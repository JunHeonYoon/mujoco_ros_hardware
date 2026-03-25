#include "mujoco_ros_hardware/mujoco_gripper_action_server.hpp"
#include "mujoco_ros_hardware/mujoco_world_singleton.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <cmath>
#include <thread>

namespace mujoco_ros_hardware
{

// Franka Hand max opening (m).
static constexpr double kMaxWidth    = 0.08;
// Position tolerance for convergence check (m).
static constexpr double kPositionTol = 0.002;
// Action timeout (s).
static constexpr double kMoveTimeout = 10.0;
// Tendon actuator ctrl range: ctrl=0 → closed, ctrl=255 → fully open.
static constexpr double kCtrlMax     = 255.0;

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

MujocoGripperActionServer::MujocoGripperActionServer(
    const std::string & node_namespace,
    const std::string & joint1_name,
    const std::string & joint2_name,
    const std::string & hand_actuator)
: joint1_name_(joint1_name)
, joint2_name_(joint2_name)
, hand_actuator_name_(hand_actuator)
{
    rclcpp::NodeOptions opts;
    opts.allow_undeclared_parameters(true);

    // Build an absolute namespace prefix, e.g. "/left_franka_gripper".
    // All action/service/topic names are made absolute so they are immune to
    // any namespace remapping imposed by the parent ros2_control_node process.
    const std::string ns_clean = node_namespace.empty() ? "franka_gripper"
        : (node_namespace[0] == '/' ? node_namespace.substr(1) : node_namespace);
    const std::string abs_prefix = "/" + ns_clean;  // e.g. "/left_franka_gripper"

    node_ = rclcpp::Node::make_shared("franka_gripper_node", ns_clean, opts);

    // ----- Action servers (absolute names) -----
    move_server_ = rclcpp_action::create_server<Move>(
        node_, abs_prefix + "/move",
        [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Move::Goal> goal)
            { return handleMoveGoal(uuid, goal); },
        [this](std::shared_ptr<GoalHandleMove> gh)
            { return handleMoveCancel(gh); },
        [this](std::shared_ptr<GoalHandleMove> gh)
            { handleMoveAccepted(gh); });

    grasp_server_ = rclcpp_action::create_server<Grasp>(
        node_, abs_prefix + "/grasp",
        [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Grasp::Goal> goal)
            { return handleGraspGoal(uuid, goal); },
        [this](std::shared_ptr<GoalHandleGrasp> gh)
            { return handleGraspCancel(gh); },
        [this](std::shared_ptr<GoalHandleGrasp> gh)
            { handleGraspAccepted(gh); });

    // ----- Homing action (absolute name) -----
    // In simulation the gripper is already homed; immediately succeeds.
    homing_server_ = rclcpp_action::create_server<Homing>(
        node_, abs_prefix + "/homing",
        [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Homing::Goal> goal)
            { return handleHomingGoal(uuid, goal); },
        [this](std::shared_ptr<GoalHandleHoming> gh)
            { return handleHomingCancel(gh); },
        [this](std::shared_ptr<GoalHandleHoming> gh)
            { handleHomingAccepted(gh); });

    // ----- Stop service (absolute name) -----
    stop_service_ = node_->create_service<std_srvs::srv::Trigger>(
        abs_prefix + "/stop",
        [this](std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res)
            { stopCallback(req, res); });

    // ----- Joint states publisher (absolute name) -----
    js_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
        abs_prefix + "/joint_states", 1);

    // ----- 100 Hz control timer: drives tendon actuator -----
    control_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(10),
        [this]() { controlLoop(); });

    // ----- 30 Hz joint-state publisher -----
    js_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(33),
        [this]() { publishJointStates(); });

    // ----- Spin the node in its own thread -----
    executor_.add_node(node_);
    executor_thread_ = std::thread([this]() { executor_.spin(); });

    RCLCPP_INFO(node_->get_logger(),
        "\033[34mMujocoGripperActionServer started — joints:[%s,%s] actuator:%s\033[0m",
        joint1_name_.c_str(), joint2_name_.c_str(), hand_actuator_name_.c_str());
}

MujocoGripperActionServer::~MujocoGripperActionServer()
{
    executor_.cancel();
    if (executor_thread_.joinable()) executor_thread_.join();
}

// ---------------------------------------------------------------------------
// Scene loaded → map joints / actuator
// ---------------------------------------------------------------------------

void MujocoGripperActionServer::onSceneLoaded()
{
    mapJoints();
}

void MujocoGripperActionServer::mapJoints()
{
    auto & world = MujocoWorldSingleton::get();
    const auto * m = world.model();
    if (!m) return;

    std::lock_guard<std::mutex> lock(world.dataMutex());
    const auto * d = world.data();
    if (!d) return;

    const int j1 = mj_name2id(m, mjOBJ_JOINT,    joint1_name_.c_str());
    const int j2 = mj_name2id(m, mjOBJ_JOINT,    joint2_name_.c_str());
    const int ha = mj_name2id(m, mjOBJ_ACTUATOR,  hand_actuator_name_.c_str());

    if (j1 < 0 || j2 < 0) {
        RCLCPP_WARN(node_->get_logger(),
            "Gripper joints '%s','%s' not found in MuJoCo model",
            joint1_name_.c_str(), joint2_name_.c_str());
        return;
    }

    qpos1_     = m->jnt_qposadr[j1];
    qpos2_     = m->jnt_qposadr[j2];
    hand_ctrl_ = ha;   // may be -1 if no actuator (passive)

    // Initialise target to current position.
    {
        std::lock_guard<std::mutex> tl(target_mtx_);
        target_width_ = d->qpos[qpos1_] + d->qpos[qpos2_];
    }

    joints_mapped_.store(true);

    RCLCPP_INFO(node_->get_logger(),
        "\033[34m[gripper] %s:qpos=%d  %s:qpos=%d  actuator(%s)=%d\033[0m",
        joint1_name_.c_str(), qpos1_,
        joint2_name_.c_str(), qpos2_,
        hand_actuator_name_.c_str(), hand_ctrl_);
}

// ---------------------------------------------------------------------------
// Control loop (100 Hz): writes hand tendon actuator
//
// The Franka Hand MJCF uses a single "general" actuator on the finger tendon:
//   ctrlrange="0 255"  gainprm="0.01568627451 0 0"  biasprm="0 -100 -10"
// At equilibrium:  ctrl = (target_width / kMaxWidth) * kCtrlMax
// ---------------------------------------------------------------------------

void MujocoGripperActionServer::controlLoop()
{
    if (!joints_mapped_.load()) return;
    if (hand_ctrl_ < 0) return;  // no actuator → nothing to drive

    auto & world = MujocoWorldSingleton::get();
    if (!world.isSceneLoaded()) return;

    std::lock_guard<std::mutex> lock(world.dataMutex());
    auto * d = world.data();
    if (!d) return;

    double w;
    {
        std::lock_guard<std::mutex> tl(target_mtx_);
        w = target_width_;
    }

    // ctrl=0 → closed (0 m), ctrl=255 → fully open (0.08 m)
    d->ctrl[hand_ctrl_] = std::clamp(w / kMaxWidth * kCtrlMax, 0.0, kCtrlMax);
}

// ---------------------------------------------------------------------------
// Joint-state publisher (30 Hz)
// ---------------------------------------------------------------------------

void MujocoGripperActionServer::publishJointStates()
{
    if (!joints_mapped_.load()) return;

    auto & world = MujocoWorldSingleton::get();
    if (!world.isSceneLoaded()) return;

    double p1, p2;
    {
        std::lock_guard<std::mutex> lock(world.dataMutex());
        const auto * d = world.data();
        if (!d) return;
        p1 = d->qpos[qpos1_];
        p2 = d->qpos[qpos2_];
    }

    sensor_msgs::msg::JointState js;
    js.header.stamp = node_->now();
    js.name     = {joint1_name_, joint2_name_};
    js.position = {p1, p2};
    js.velocity = {0.0, 0.0};
    js.effort   = {0.0, 0.0};
    js_pub_->publish(js);
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

double MujocoGripperActionServer::getCurrentWidth()
{
    if (!joints_mapped_.load()) return kMaxWidth;

    auto & world = MujocoWorldSingleton::get();
    std::lock_guard<std::mutex> lock(world.dataMutex());
    const auto * d = world.data();
    if (!d) return kMaxWidth;
    return d->qpos[qpos1_] + d->qpos[qpos2_];
}

void MujocoGripperActionServer::setTargetWidth(double w)
{
    w = std::clamp(w, 0.0, kMaxWidth);
    std::lock_guard<std::mutex> tl(target_mtx_);
    target_width_ = w;
}

// ---------------------------------------------------------------------------
// Move action
// ---------------------------------------------------------------------------

rclcpp_action::GoalResponse MujocoGripperActionServer::handleMoveGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Move::Goal> goal)
{
    RCLCPP_INFO(node_->get_logger(),
        "Move goal: width=%.4f m  speed=%.4f m/s", goal->width, goal->speed);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MujocoGripperActionServer::handleMoveCancel(
    std::shared_ptr<GoalHandleMove>)
{
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MujocoGripperActionServer::handleMoveAccepted(std::shared_ptr<GoalHandleMove> gh)
{
    std::thread([this, gh]() { executeMove(gh); }).detach();
}

void MujocoGripperActionServer::executeMove(std::shared_ptr<GoalHandleMove> gh)
{
    const auto goal   = gh->get_goal();
    auto       result = std::make_shared<Move::Result>();

    if (!joints_mapped_.load()) {
        result->success = false;
        result->error   = "Gripper not mapped yet";
        gh->abort(result);
        return;
    }

    setTargetWidth(goal->width);

    const auto deadline = node_->now() + rclcpp::Duration::from_seconds(kMoveTimeout);
    rclcpp::Rate rate(50);

    while (rclcpp::ok()) {
        if (gh->is_canceling()) {
            setTargetWidth(getCurrentWidth());
            result->success = false;
            result->error   = "canceled";
            gh->canceled(result);
            return;
        }

        const double current = getCurrentWidth();
        auto fb = std::make_shared<Move::Feedback>();
        fb->current_width = current;
        gh->publish_feedback(fb);

        if (std::abs(current - goal->width) < kPositionTol) {
            result->success = true;
            gh->succeed(result);
            return;
        }

        if (node_->now() > deadline) {
            result->success = false;
            result->error   = "timeout";
            gh->abort(result);
            return;
        }

        rate.sleep();
    }
}

// ---------------------------------------------------------------------------
// Grasp action
// ---------------------------------------------------------------------------

rclcpp_action::GoalResponse MujocoGripperActionServer::handleGraspGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Grasp::Goal> goal)
{
    RCLCPP_INFO(node_->get_logger(),
        "Grasp goal: width=%.4f  epsilon=[%.4f,%.4f]  force=%.1f N",
        goal->width, goal->epsilon.inner, goal->epsilon.outer, goal->force);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MujocoGripperActionServer::handleGraspCancel(
    std::shared_ptr<GoalHandleGrasp>)
{
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MujocoGripperActionServer::handleGraspAccepted(std::shared_ptr<GoalHandleGrasp> gh)
{
    std::thread([this, gh]() { executeGrasp(gh); }).detach();
}

void MujocoGripperActionServer::executeGrasp(std::shared_ptr<GoalHandleGrasp> gh)
{
    const auto goal   = gh->get_goal();
    auto       result = std::make_shared<Grasp::Result>();

    if (!joints_mapped_.load()) {
        result->success = false;
        result->error   = "Gripper not mapped yet";
        gh->abort(result);
        return;
    }

    setTargetWidth(goal->width);

    const auto deadline = node_->now() + rclcpp::Duration::from_seconds(kMoveTimeout);
    rclcpp::Rate rate(50);

    while (rclcpp::ok()) {
        if (gh->is_canceling()) {
            setTargetWidth(getCurrentWidth());
            result->success = false;
            result->error   = "canceled";
            gh->canceled(result);
            return;
        }

        const double current = getCurrentWidth();
        auto fb = std::make_shared<Grasp::Feedback>();
        fb->current_width = current;
        gh->publish_feedback(fb);

        // Success: arrived at target OR within epsilon band
        const bool at_target  = std::abs(current - goal->width) < kPositionTol;
        const bool in_epsilon = (current >= goal->width - goal->epsilon.inner) &&
                                (current <= goal->width + goal->epsilon.outer);
        if (at_target || in_epsilon) {
            result->success = true;
            gh->succeed(result);
            return;
        }

        if (node_->now() > deadline) {
            // Still do a final epsilon check (object may be blocking the gripper)
            const double w = getCurrentWidth();
            result->success = (w >= goal->width - goal->epsilon.inner) &&
                              (w <= goal->width + goal->epsilon.outer);
            result->error   = result->success ? "" : "timeout";
            if (result->success) gh->succeed(result);
            else                 gh->abort(result);
            return;
        }

        rate.sleep();
    }
}

// ---------------------------------------------------------------------------
// Stop service
// ---------------------------------------------------------------------------

void MujocoGripperActionServer::stopCallback(
    std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    setTargetWidth(getCurrentWidth());
    response->success = true;
    response->message = "Gripper stopped";
    RCLCPP_INFO(node_->get_logger(), "Gripper stopped at current position");
}

// ---------------------------------------------------------------------------
// Homing action — immediately succeeds (no physical homing needed in sim)
// ---------------------------------------------------------------------------

rclcpp_action::GoalResponse MujocoGripperActionServer::handleHomingGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Homing::Goal>)
{
    RCLCPP_INFO(node_->get_logger(), "Homing goal received (sim: instant success)");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MujocoGripperActionServer::handleHomingCancel(
    std::shared_ptr<GoalHandleHoming>)
{
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MujocoGripperActionServer::handleHomingAccepted(std::shared_ptr<GoalHandleHoming> gh)
{
    // Open the gripper to max width as part of homing, then immediately succeed.
    setTargetWidth(kMaxWidth);
    auto result = std::make_shared<Homing::Result>();
    result->success = true;
    gh->succeed(result);
}

}  // namespace mujoco_ros_hardware
