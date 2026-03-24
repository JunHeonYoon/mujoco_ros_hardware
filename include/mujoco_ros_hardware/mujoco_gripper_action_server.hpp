#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <franka_msgs/action/grasp.hpp>
#include <franka_msgs/action/move.hpp>

namespace mujoco_ros_hardware
{

/**
 * In-process simulated Franka gripper action server.
 *
 * Provides the same ROS2 interfaces as franka_gripper/GripperActionServer:
 *  ~/move  (franka_msgs/action/Move)
 *  ~/grasp (franka_msgs/action/Grasp)
 *  ~/stop  (std_srvs/srv/Trigger)
 *  ~/joint_states (sensor_msgs/msg/JointState)
 *
 * Directly drives the finger joint actuators in MujocoWorldSingleton.
 * Must be constructed after rclcpp::init(); onSceneLoaded() must be called
 * after MujocoWorldSingleton has loaded the scene.
 */
class MujocoGripperActionServer
{
public:
    /**
     * @param node_namespace  ROS2 namespace for the gripper node (e.g. "left_franka_gripper")
     * @param joint1_name     MuJoCo joint name for finger 1 (e.g. "left_fr3_finger_joint1")
     * @param joint2_name     MuJoCo joint name for finger 2 (e.g. "left_fr3_finger_joint2")
     * @param hand_actuator   MuJoCo actuator name for the hand tendon (e.g. "left_fr3_hand")
     */
    MujocoGripperActionServer(
        const std::string & node_namespace,
        const std::string & joint1_name,
        const std::string & joint2_name,
        const std::string & hand_actuator);

    ~MujocoGripperActionServer();

    /** Call after MujocoWorldSingleton::loadScene* has succeeded. */
    void onSceneLoaded();

private:
    using Move  = franka_msgs::action::Move;
    using Grasp = franka_msgs::action::Grasp;
    using GoalHandleMove  = rclcpp_action::ServerGoalHandle<Move>;
    using GoalHandleGrasp = rclcpp_action::ServerGoalHandle<Grasp>;

    // ---- Joint / actuator mapping ----
    std::string joint1_name_, joint2_name_;
    std::string hand_actuator_name_;
    int qpos1_ = -1, qpos2_ = -1;
    int hand_ctrl_ = -1;           // single tendon actuator for both fingers
    std::atomic<bool> joints_mapped_ {false};

    // ---- Target width in metres (mutex-protected) ----
    mutable std::mutex target_mtx_;
    double target_width_ = 0.08;   // default: fully open

    // ---- ROS2 node + executor ----
    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread executor_thread_;

    rclcpp_action::Server<Move>::SharedPtr  move_server_;
    rclcpp_action::Server<Grasp>::SharedPtr grasp_server_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr js_timer_;

    // ---- Internal helpers ----
    void mapJoints();
    void controlLoop();        // 100 Hz timer: drives hand tendon actuator
    void publishJointStates(); // 30 Hz timer

    double getCurrentWidth();       // reads qpos from MuJoCo (takes data_mutex_)
    void   setTargetWidth(double w); // clamps and stores target width

    // ---- Move action ----
    rclcpp_action::GoalResponse  handleMoveGoal(const rclcpp_action::GoalUUID &, std::shared_ptr<const Move::Goal>);
    rclcpp_action::CancelResponse handleMoveCancel(std::shared_ptr<GoalHandleMove>);
    void handleMoveAccepted(std::shared_ptr<GoalHandleMove>);
    void executeMove(std::shared_ptr<GoalHandleMove>);

    // ---- Grasp action ----
    rclcpp_action::GoalResponse  handleGraspGoal(const rclcpp_action::GoalUUID &, std::shared_ptr<const Grasp::Goal>);
    rclcpp_action::CancelResponse handleGraspCancel(std::shared_ptr<GoalHandleGrasp>);
    void handleGraspAccepted(std::shared_ptr<GoalHandleGrasp>);
    void executeGrasp(std::shared_ptr<GoalHandleGrasp>);

    // ---- Stop service ----
    void stopCallback(
        std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response>);
};

}  // namespace mujoco_ros_hardware
