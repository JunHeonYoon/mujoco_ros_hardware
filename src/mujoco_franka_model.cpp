#include "mujoco_ros_hardware/mujoco_franka_model.hpp"
#include "mujoco_ros_hardware/mujoco_world_singleton.hpp"

#include <mutex>

namespace mujoco_ros_hardware
{

void MujocoFrankaModel::setJointIndices(const std::vector<int> & qvel_indices)
{
    qvel_indices_ = qvel_indices;
}

std::array<double, 49> MujocoFrankaModel::mass(const franka::RobotState &) const
{
    auto & world = MujocoWorldSingleton::get();
    std::lock_guard<std::mutex> lock(world.dataMutex());
    const auto * m = world.model();
    const auto * d = world.data();
    if (!m || !d || static_cast<int>(qvel_indices_.size()) < 7) return {};

    // Get full dense mass matrix (nv x nv, row-major)
    std::vector<double> fullM(static_cast<size_t>(m->nv * m->nv));
    mj_fullM(m, fullM.data(), d->qM);

    // Extract 7x7 submatrix for arm joints (column-major output for Franka convention)
    std::array<double, 49> armM{};
    const int nv = m->nv;
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            armM[static_cast<size_t>(j * 7 + i)] = fullM[static_cast<size_t>(qvel_indices_[i] * nv + qvel_indices_[j])];
        }
    }
    return armM;
}

std::array<double, 7> MujocoFrankaModel::coriolis(const franka::RobotState &) const
{
    // Combined bias (gravity + Coriolis) is returned via gravity().
    return {};
}

std::array<double, 7> MujocoFrankaModel::gravity(const franka::RobotState &) const
{
    auto & world = MujocoWorldSingleton::get();
    std::lock_guard<std::mutex> lock(world.dataMutex());
    const auto * d = world.data();
    if (!d || static_cast<int>(qvel_indices_.size()) < 7) return {};

    // qfrc_bias = gravity + Coriolis (combined bias forces)
    std::array<double, 7> g{};
    for (int i = 0; i < 7; i++) {
        g[static_cast<size_t>(i)] = d->qfrc_bias[qvel_indices_[static_cast<size_t>(i)]];
    }
    return g;
}

std::array<double, 16> MujocoFrankaModel::pose(
    franka::Frame, const franka::RobotState &) const
{
    // Return identity matrix (4x4 column-major)
    return {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
}

std::array<double, 42> MujocoFrankaModel::bodyJacobian(
    franka::Frame, const franka::RobotState &) const
{
    return {};
}

std::array<double, 42> MujocoFrankaModel::zeroJacobian(
    franka::Frame, const franka::RobotState &) const
{
    return {};
}

}  // namespace mujoco_ros_hardware
