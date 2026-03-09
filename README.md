# mujoco_ros_hardware

A unified `ros2_control` hardware plugin that runs robot simulations inside MuJoCo, using a singleton world and a pluggable sub-handler architecture.

---

## Table of Contents
- [Overview](#overview)
- [Dependencies](#dependencies)
- [Build](#build)
- [Architecture](#architecture)
  - [MujocoHardwareInterface](#mujocohardwareinterface)
  - [MujocoWorldSingleton](#mujocoworldsingleton)
  - [SubHandlerBase & SubHandlerRegistry](#subhandlerbase--subhandlerregistry)
  - [FrankaSubHandler](#frankasubhandler)
  - [FrankaMultiSubHandler](#frankamultisubhandler)
  - [HuskySubHandler](#huskysubhandler)
  - [MujocoFrankaModel](#mujocofrankmodel)
- [URDF Configuration](#urdf-configuration)
- [Launch File Integration](#launch-file-integration)
- [Adding a New Robot Type](#adding-a-new-robot-type)

---

## Overview

`mujoco_ros_hardware` provides a single `ros2_control` hardware plugin — `mujoco_ros_hardware/MujocoHardwareInterface` — that can simulate any robot type inside MuJoCo. Robot-specific logic is delegated to **sub-handlers** selected at runtime via the `robot_type` URDF parameter. The MuJoCo world (`mjModel`/`mjData`) is owned by a process-level singleton, allowing multiple hardware plugins in the same controller_manager to share a single simulation.

---

## Dependencies

- ROS 2 Humble
- [MuJoCo](https://mujoco.org/) (>= 3.x)
- `ros2_control` / `hardware_interface`
- `franka_hardware` (for `FrankaSubHandler` and `FrankaMultiSubHandler`)
- `libfranka` (for `franka::RobotState`)

---

## Build

```bash
colcon build --packages-select mujoco_ros_hardware
```

For typical use with Franka + Husky:

```bash
colcon build --packages-select \
  mujoco_ros_hardware \
  franka_description \
  franka_bringup \
  fr3_husky_description \
  fr3_husky_controller
```

---

## Architecture

### MujocoHardwareInterface

**Plugin type:** `mujoco_ros_hardware/MujocoHardwareInterface`

Single `hardware_interface::SystemInterface` used for **all** robot types. On `on_init()`, it reads the `robot_type` hw_param, creates the matching sub-handler via `SubHandlerRegistry`, and registers itself with `MujocoWorldSingleton`.

Scene loading happens in `perform_command_mode_switch()` (not `on_init()` or `on_activate()`) because the ROS 2 executor must already be running for `MujocoWorldSingleton::init()` to call the `controller_manager` parameter service:

1. `sub_handler->performCommandModeSwitch()` — determines runtime state (e.g. `control_mode`)
2. `sub_handler->isReadyToLoadScene()` — waits until enough runtime state is available
3. `sub_handler->getXacroArgs()` — returns handler-specific xacro arguments
4. `runXacro(xacro_path, base_args + handler_args)` → XML string
5. `MujocoWorldSingleton::loadScene(xml)` — loads the MuJoCo world (once, thread-safe)
6. `sub_handler->onSceneLoaded()` — maps joints against the loaded `mjModel`

---

### MujocoWorldSingleton

Process-level singleton that owns the single `mjModel*`/`mjData*` shared across all plugin instances.

**Key responsibilities:**
- Reads `mujoco_scene_xacro_path` and `mujoco_scene_xacro_args` from the `controller_manager` ROS node parameters (set in the launch file)
- Loads the compiled XML scene via `mj_loadXML()`
- Runs a real-time simulation thread calling `mj_step()` at the model's `opt.timestep`
- Opens a passive MuJoCo viewer window (optional, uses GLFW)
- Exposes `dataMutex()` — sub-handlers **must** hold this mutex while accessing `model()`/`data()`

**Priority system:** Multiple plugins can register with different `scenePriority()` values. Only the plugin whose priority equals `maxPriority()` actually loads the scene. This prevents race conditions in multi-robot setups (e.g. `FrankaSubHandler` at priority 10 always loads the scene before `HuskySubHandler` at priority 0).

**controller_manager parameters required (set from launch file):**

| Parameter | Description |
|-----------|-------------|
| `mujoco_scene_xacro_path` | Path to the MJCF xacro scene file |
| `mujoco_scene_xacro_args` | Base xacro args passed to the scene (e.g. `"as_two_wheels:=false"`) |

---

### SubHandlerBase & SubHandlerRegistry

`SubHandlerBase` is the abstract interface for all robot-specific logic:

| Method | Description |
|--------|-------------|
| `onInit()` | Read hw_params, allocate state/command buffers |
| `exportStateInterfaces()` | Return `hardware_interface::StateInterface` list |
| `exportCommandInterfaces()` | Return `hardware_interface::CommandInterface` list |
| `scenePriority()` | Priority for scene loading (default: 0) |
| `isReadyToLoadScene()` | Return true when xacro args are available (default: true) |
| `getXacroArgs()` | Return xacro argument string for scene parameterisation |
| `onSceneLoaded()` | Map joints against loaded `mjModel` |
| `read()` / `write()` | Read states from / write commands to `mjData` |

`SubHandlerRegistry` maps `robot_type` strings to factory functions. Handlers self-register using the `REGISTER_SUB_HANDLER` macro in their `.cpp` file:

```cpp
// At namespace scope in my_handler.cpp:
REGISTER_SUB_HANDLER("my_robot", mujoco_ros_hardware::MySubHandler)
```

`MujocoHardwareInterface` only depends on `SubHandlerRegistry` — it has no compile-time dependency on any concrete handler.

---

### FrankaSubHandler

**robot_type:** `"franka"`

Sub-handler for a single Franka FR3 arm.

**hw_params:**

| Parameter | Description |
|-----------|-------------|
| `arm_id` | Arm model identifier (e.g. `"fr3"`) |
| `prefix` | Arm prefix for multi-arm setups (e.g. `"left"`) |
| `load_gripper` | `"true"` \| `"false"` |

**State interfaces exported:**

| Interface | Description |
|-----------|-------------|
| `{joint_name}/position` | Joint position (rad) |
| `{joint_name}/velocity` | Joint velocity (rad/s) |
| `{joint_name}/effort` | Joint effort (Nm) |
| `{name_stem}/robot_time` | MuJoCo simulation time |
| `{name_stem}/robot_state` | Bit-cast `franka::RobotState*` as `double` |
| `{name_stem}/robot_model` | Bit-cast `franka_hardware::Model*` as `double` |

where `name_stem = prefix + "_" + arm_id` (or `arm_id` if prefix is empty).

**Scene loading:** `FrankaSubHandler` has `scenePriority() = 10` and defers scene loading until `control_mode_` is set by `performCommandModeSwitch()` (i.e. a command-mode controller activates). This prevents the scene from being built with an empty `control_mode` xacro arg.

**Xacro args generated:** `"arm_id:=<> hand:=<> control_mode:=<>"`

---

### FrankaMultiSubHandler

**robot_type:** `"franka_multi"`

Sub-handler for multiple Franka FR3 arms sharing a single URDF plugin block (mirrors `franka_multi_hardware_interface`).

**hw_params:**

| Parameter | Description |
|-----------|-------------|
| `robot_count` | Number of arms (e.g. `"2"`) |
| `arm_id_<i>` | Arm identifier for robot `i` |
| `prefix_<i>` | Arm prefix for robot `i` |
| `load_gripper_<i>` | `"true"` \| `"false"` for robot `i` |

**Xacro args generated:** `"robot_count:=N arm_id_0:=... hand_0:=... control_mode_0:=... prefix_0:=... ..."`

**Scene loading:** Waits until ALL robots have a `control_mode` set (i.e. all command-mode controllers have activated) before building the scene.

---

### HuskySubHandler

**robot_type:** `"husky"`

Sub-handler for the Clearpath Husky mobile base.

**hw_params:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `wheel_radius` | `0.1651` | Wheel radius (m) |
| `max_speed` | `6.06` | Max wheel speed (rad/s) |

**Interfaces:** Position + velocity state, velocity command for each wheel joint.

**Scene loading:** `scenePriority() = 0` — scene is always loaded by `FrankaSubHandler` first. `HuskySubHandler` maps its joints lazily on the first `read()`/`write()` call after the scene is loaded.

---

### MujocoFrankaModel

`MujocoFrankaModel` implements `franka_hardware::Model` backed by the live MuJoCo simulation, allowing Franka controllers that depend on dynamics (gravity compensation, impedance control) to run in simulation without real hardware.

| Method | Implementation |
|--------|---------------|
| `mass()` | 7×7 submatrix from `mj_fullM()` (column-major) |
| `gravity()` | `qfrc_bias[qvel_idx]` per joint (gravity + Coriolis combined) |
| `coriolis()` | Zeros (bias already included in `gravity()`) |
| `pose()` | Identity (stub) |
| `bodyJacobian()` / `zeroJacobian()` | Zeros (stub) |

---

## URDF Configuration

### Single Franka arm

```xml
<hardware>
  <plugin>mujoco_ros_hardware/MujocoHardwareInterface</plugin>
  <param name="robot_type">franka</param>
  <param name="arm_id">fr3</param>
  <param name="prefix">left</param>
  <param name="load_gripper">true</param>
</hardware>
```

### Franka + Husky (separate plugin blocks in the same URDF)

```xml
<!-- Arm -->
<hardware>
  <plugin>mujoco_ros_hardware/MujocoHardwareInterface</plugin>
  <param name="robot_type">franka</param>
  <param name="arm_id">fr3</param>
  <param name="prefix">left</param>
  <param name="load_gripper">false</param>
</hardware>

<!-- Mobile base -->
<hardware>
  <plugin>mujoco_ros_hardware/MujocoHardwareInterface</plugin>
  <param name="robot_type">husky</param>
</hardware>
```

> **Note:** The Franka plugin block must appear **before** the Husky block in the URDF. The `perform_command_mode_switch` of the first-listed plugin runs first, ensuring `FrankaSubHandler` (priority 10) loads the scene before `HuskySubHandler` (priority 0) tries to map its joints.

---

## Launch File Integration

The MuJoCo scene is specified via `controller_manager` node parameters. Set them in your launch file:

```python
cm_params = [
    controllers_yaml,
    {'robot_description': robot_description},
    {'mujoco_scene_xacro_path': '/path/to/scene.xml.xacro'},
    {'mujoco_scene_xacro_args': 'side:=left hand:=false mobile:=true'},
]

Node(
    package='controller_manager',
    executable='ros2_control_node',
    parameters=cm_params,
    ...
)
```

Typical launch command patterns:

```bash
# FR3 single arm (standalone)
ros2 launch franka_bringup example.launch.py \
  controller_name:=joint_position_example_controller \
  use_mujoco:=true \
  load_gripper:=false

# FR3 + Husky
ros2 launch fr3_husky_controller fr3_husky_controller.launch.py \
  robot_side:=left load_gripper:=true use_mujoco:=true

# FR3 only (no Husky)
ros2 launch fr3_husky_controller fr3_controller.launch.py \
  robot_side:=left load_gripper:=true load_mobile:=false use_mujoco:=true
```

---

## Adding a New Robot Type

1. Create `my_sub_handler.hpp` / `my_sub_handler.cpp` inheriting `SubHandlerBase`.
2. Implement `onInit()`, `exportStateInterfaces()`, `exportCommandInterfaces()`, `read()`, `write()`.
3. Optionally override `scenePriority()`, `isReadyToLoadScene()`, `getXacroArgs()`, `onSceneLoaded()`.
4. Register using the macro **inside the closing namespace brace** of the `.cpp` file:

```cpp
} // namespace mujoco_ros_hardware

REGISTER_SUB_HANDLER("my_robot", mujoco_ros_hardware::MySubHandler)
```

5. Add `robot_type:=my_robot` to the URDF `<hardware>` block.
6. Link the new `.cpp` in `CMakeLists.txt` alongside the existing handlers.
