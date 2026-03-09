#pragma once

#include <array>
#include <vector>

#include <mujoco/mujoco.h>
#include "franka_hardware/model.hpp"

namespace mujoco_ros_hardware
{

/**
 * MuJoCo-backed implementation of franka_hardware::Model.
 *
 * Provides mass matrix, bias forces (gravity+Coriolis), and stub Jacobians
 * computed from the live MuJoCo simulation.
 *
 * gravity() returns qfrc_bias (gravity + Coriolis combined).
 * coriolis() returns zeros (bias already in gravity()).
 * pose() / bodyJacobian() / zeroJacobian() return zeros.
 */
class MujocoFrankaModel : public franka_hardware::Model
{
public:
    MujocoFrankaModel() = default;

    /** Set the DOF (qvel) indices of the 7 arm joints in the MuJoCo model. */
    void setJointIndices(const std::vector<int> & qvel_indices);

    // ---- franka_hardware::Model virtual overrides ----

    std::array<double, 49> mass(const franka::RobotState &) const override;

    std::array<double, 7> coriolis(const franka::RobotState &) const override;

    std::array<double, 7> gravity(const franka::RobotState &) const override;

    std::array<double, 16> pose(
        franka::Frame frame, const franka::RobotState &) const override;

    std::array<double, 42> bodyJacobian(
        franka::Frame frame, const franka::RobotState &) const override;

    std::array<double, 42> zeroJacobian(
        franka::Frame frame, const franka::RobotState &) const override;

private:
    std::vector<int> qvel_indices_;  // DOF indices of the 7 arm joints
};

}  // namespace mujoco_ros_hardware
