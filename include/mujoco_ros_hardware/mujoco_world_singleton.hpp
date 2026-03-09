#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unistd.h>

#include <mujoco/mujoco.h>
#include "simulate.h"
#include "glfw_adapter.h"

namespace mujoco_ros_hardware
{

/**
 * Process-level singleton that owns the single MuJoCo world shared across
 * all MujocoHardwareInterface plugin instances.
 *
 * Lifecycle:
 *  1. Each plugin calls registerPlugin() in on_init().
 *  2. The highest-priority plugin calls loadScene() from perform_command_mode_switch().
 *  3. loadScene() starts the viewer and the simulation thread.
 *  4. The simulation thread runs mj_step() in real-time (mj_model->opt.timestep).
 *  5. SubHandlers lock dataMutex() to read/write mjData in their read()/write().
 */
class MujocoWorldSingleton
{
public:
    static MujocoWorldSingleton & get();

    // ---- Registration (called from on_init of each plugin) ----
    void registerPlugin(int priority = 0);
    int  totalPlugins() const { return total_plugins_; }
    int  maxPriority()  const { return max_priority_; }

    // ---- Init: reads mujoco_scene_xacro_path and mujoco_scene_xacro_args
    //      from controller_manager ROS params (like robot_description).
    //      Called once by the first plugin's on_init(). ----
    bool init();
    bool isInitialized() const { return initialized_; }

    const std::string & xmlPath()       const { return xml_path_; }
    const std::string & xacroPath()     const { return xacro_path_; }
    const std::string & xacroBaseArgs() const { return xacro_base_args_; }

    // ---- Scene loading (called by MujocoHardwareInterface after xacro processing) ----
    bool loadSceneFromPath(const std::string & xml_path);
    bool loadSceneFromXML(const std::string & xml_string);
    bool isSceneLoaded() const { return scene_loaded_; }

    // ---- MuJoCo model/data accessors ----
    mjModel * model() const { return model_; }
    mjData  * data()  const { return data_;  }

    // ---- Data mutex (SubHandlers must hold this while accessing model/data) ----
    std::mutex & dataMutex() { return data_mutex_; }

private:
    MujocoWorldSingleton() = default;
    ~MujocoWorldSingleton();

    MujocoWorldSingleton(const MujocoWorldSingleton &)            = delete;
    MujocoWorldSingleton & operator=(const MujocoWorldSingleton &) = delete;

    void startViewer();
    void stopViewer();
    void startSimulation();
    void stopSimulation();

    // ---- Registration state ----
    int total_plugins_ = 0;
    int max_priority_  = 0;

    // ---- Init state (read from controller_manager ROS params) ----
    bool        initialized_     = false;
    std::string xml_path_;
    std::string xacro_path_;
    std::string xacro_base_args_;

    // ---- MuJoCo world ----
    mjModel * model_       = nullptr;
    mjData  * data_        = nullptr;
    bool      scene_loaded_ = false;

    // ---- Simulation thread ----
    std::atomic<bool> sim_running_ {false};
    std::thread       sim_thread_;

    // ---- Viewer (passive) ----
    mjvCamera cam_;
    mjvOption opt_;
    mjvPerturb pert_;
    std::unique_ptr<mujoco::Simulate> sim_;
    std::thread render_thread_;
    std::mutex  sim_ready_mtx_;
    bool        sim_ready_ = false;
    std::condition_variable sim_ready_cv_;

    // ---- Protects model_/data_ during read/write/step ----
    std::mutex data_mutex_;
};

}  // namespace mujoco_ros_hardware
