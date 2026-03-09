#include "mujoco_ros_hardware/mujoco_world_singleton.hpp"

#include <chrono>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"

namespace mujoco_ros_hardware
{

MujocoWorldSingleton & MujocoWorldSingleton::get()
{
    static MujocoWorldSingleton instance;
    return instance;
}

MujocoWorldSingleton::~MujocoWorldSingleton()
{
    stopSimulation();
    stopViewer();
    if (data_)  { mj_deleteData(data_);   data_  = nullptr; }
    if (model_) { mj_deleteModel(model_); model_ = nullptr; }
}

void MujocoWorldSingleton::registerPlugin(int priority)
{
    ++total_plugins_;
    if (priority > max_priority_) max_priority_ = priority;
    RCLCPP_INFO(
        rclcpp::get_logger("MujocoWorldSingleton"),
        "Plugin registered (total: %d, priority: %d, max: %d)",
        total_plugins_, priority, max_priority_);
}

bool MujocoWorldSingleton::init()
{
    if (initialized_) return true;

    // Create a temporary node to read parameters from controller_manager
    rclcpp::NodeOptions opts;
    opts.allow_undeclared_parameters(true);
    auto tmp = rclcpp::Node::make_shared("_mujoco_singleton_init", opts);
    rclcpp::SyncParametersClient cli(tmp, "controller_manager");

    if (!cli.wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("MujocoWorldSingleton"),
            "\033[31mcontroller_manager parameter service not available within 5s\033[0m");
        return false;
    }

    const auto params = cli.get_parameters({"mujoco_scene_path", "mujoco_scene_xacro_path", "mujoco_scene_xacro_args"});
    for (const auto & p : params)
    {
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) continue;
        if      (p.get_name() == "mujoco_scene_path")       xml_path_        = p.as_string();
        else if (p.get_name() == "mujoco_scene_xacro_path") xacro_path_      = p.as_string();
        else if (p.get_name() == "mujoco_scene_xacro_args") xacro_base_args_ = p.as_string();
    }

    if (xml_path_.empty() && xacro_path_.empty())
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("MujocoWorldSingleton"),
            "\033[31mROS param 'mujoco_scene_path' and 'mujoco_scene_xacro_path' not set on controller_manager\033[0m");
        return false;
    }

    if(!xml_path_.empty() && (access(xml_path_.c_str(), F_OK) == -1))
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("MujocoWorldSingleton"),
            "\033[31mXML file (mujoco_scene_path) not found: '%s'\033[0m", xml_path_.c_str());
        return false;
    }

    if(!xacro_path_.empty() && (access(xacro_path_.c_str(), F_OK) == -1))
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("MujocoWorldSingleton"),
            "\033[31mXML.xacro file (mujoco_scene_xacro_path) not found: '%s'\033[0m", xacro_path_.c_str());
        return false;
    }

    initialized_ = true;
    if(!xml_path_.empty())
    {
        RCLCPP_INFO(
            rclcpp::get_logger("MujocoWorldSingleton"),
            "\033[34mInit: xml_path=%s\033[0m",
            xml_path_.c_str());
        return true;
    }
    RCLCPP_INFO(
            rclcpp::get_logger("MujocoWorldSingleton"),
            "\033[34mInit: xacro_path=%s  base_args=[%s]\033[0m",
            xacro_path_.c_str(), xacro_base_args_.c_str());
    return true;
}

bool MujocoWorldSingleton::loadSceneFromPath(const std::string & xml_path)
{
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        if (data_)  { mj_deleteData(data_);   data_  = nullptr; }
        if (model_) { mj_deleteModel(model_); model_ = nullptr; }

        char err[1000] = {};
        model_ = mj_loadXML(xml_path.c_str(), nullptr, err, sizeof(err));
        if (!model_)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("MujocoWorldSingleton"),
                "\033[31mmj_loadXML failed:  %s\033[0m", err);
            return false;
        }

        data_ = mj_makeData(model_);
        mj_forward(model_, data_);
        scene_loaded_ = true;
    }

    RCLCPP_INFO(
        rclcpp::get_logger("MujocoWorldSingleton"),
        "\033[34mScene loaded: nq=%d nv=%d nu=%d njnt=%d\033[0m",
        model_->nq, model_->nv, model_->nu, model_->njnt);

    startViewer();
    startSimulation();
    return true;
}

bool MujocoWorldSingleton::loadSceneFromXML(const std::string & xml_string)
{
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        if (data_)  { mj_deleteData(data_);   data_  = nullptr; }
        if (model_) { mj_deleteModel(model_); model_ = nullptr; }

        char err[1000] = {};
        mjSpec * spec = mj_parseXMLString(xml_string.c_str(), nullptr, err, sizeof(err));
        if (!spec)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("MujocoWorldSingleton"),
                "\033[31mmj_parseXMLString failed: %s\033[0m", err);
            return false;
        }

        model_ = mj_compile(spec, nullptr);
        mj_deleteSpec(spec);
        if (!model_)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("MujocoWorldSingleton"),
                "\033[31mmj_compile failed\033[0m");
            return false;
        }

        data_ = mj_makeData(model_);
        mj_forward(model_, data_);
        scene_loaded_ = true;
    }

    RCLCPP_INFO(
        rclcpp::get_logger("MujocoWorldSingleton"),
        "\033[34mScene loaded: nq=%d nv=%d nu=%d njnt=%d\033[0m",
        model_->nq, model_->nv, model_->nu, model_->njnt);

    startViewer();
    startSimulation();
    return true;
}

void MujocoWorldSingleton::startSimulation()
{
    if (sim_running_.load() || !model_ || !data_) return;
    sim_running_.store(true);

    sim_thread_ = std::thread([this]()
    {
        const double step_sec =
            (model_->opt.timestep > 0.0) ? model_->opt.timestep : 0.001;

        auto next      = std::chrono::steady_clock::now();
        auto last_sync = next;
        const auto sync_period = std::chrono::duration<double>(1.0 / 60.0);

        while (sim_running_.load())
        {
            next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(step_sec));

            if (sim_)
            {
                // Viewer active: mj_step under both mutexes; Sync under sim_->mtx at 60 Hz
                const std::unique_lock<std::recursive_mutex> sim_lk(sim_->mtx);
                {
                    std::lock_guard<std::mutex> data_lk(data_mutex_);
                    mj_step(model_, data_);
                }
                const auto now = std::chrono::steady_clock::now();
                if (now - last_sync >= sync_period)
                {
                    sim_->Sync(false);
                    last_sync = now;
                }
            }
            else
            {
                std::lock_guard<std::mutex> lk(data_mutex_);
                mj_step(model_, data_);
            }

            std::this_thread::sleep_until(next);
        }
    });

    RCLCPP_INFO(rclcpp::get_logger("MujocoWorldSingleton"),
        "\033[34mSimulation thread started (dt=%.4f s)\033[0m", model_->opt.timestep);
}

void MujocoWorldSingleton::stopSimulation()
{
    if (!sim_running_.load()) return;
    sim_running_.store(false);
    if (sim_thread_.joinable()) sim_thread_.join();
}

void MujocoWorldSingleton::startViewer()
{
    mjv_defaultCamera(&cam_);
    mjv_defaultOption(&opt_);
    mjv_defaultPerturb(&pert_);

    {
        std::lock_guard<std::mutex> lk(sim_ready_mtx_);
        sim_ready_ = false;
    }

    render_thread_ = std::thread([this]() {
        auto sim = std::make_unique<mujoco::Simulate>(
            std::make_unique<mujoco::GlfwAdapter>(),
            &cam_, &opt_, &pert_, /*is_passive=*/true);
        {
            std::lock_guard<std::mutex> lk(sim_ready_mtx_);
            sim_ = std::move(sim);
            sim_ready_ = true;
        }
        sim_ready_cv_.notify_one();
        sim_->RenderLoop();
    });

    {
        std::unique_lock<std::mutex> lk(sim_ready_mtx_);
        sim_ready_cv_.wait(lk, [this]() { return sim_ready_; });
    }

    sim_->Load(model_, data_, "mujoco_scene");
    RCLCPP_INFO(rclcpp::get_logger("MujocoWorldSingleton"), "\033[34mViewer started\033[0m");
}

void MujocoWorldSingleton::stopViewer()
{
    if (!sim_) return;
    sim_->exitrequest.store(1);
    if (render_thread_.joinable()) render_thread_.join();
    sim_.reset();
}

}  // namespace mujoco_ros_hardware
