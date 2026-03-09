# mujoco_ros_hardware

Unified `ros2_control` hardware plugin for MuJoCo simulation. A single plugin (`MujocoHardwareInterface`) handles any robot type via pluggable sub-handlers, sharing one MuJoCo world across all plugin instances.

See [ARCHITECTURE.md](ARCHITECTURE.md) for design details.

---

## Dependencies

- ROS 2 Humble, `ros2_control` / `hardware_interface`, `pluginlib`
- MuJoCo (>= 3.x)
- `franka_hardware`, `libfranka` (for Franka sub-handlers)

---

## Build

```bash
colcon build --packages-select mujoco_ros_hardware
```

---

## Usage

Set `robot_type` in the URDF `<hardware>` block and pass the scene xacro path as `controller_manager` parameters in the launch file.

### URDF

```xml
<hardware>
  <plugin>mujoco_ros_hardware/MujocoHardwareInterface</plugin>
  <param name="robot_type">franka</param>   <!-- or "husky", "franka_multi" -->
  <param name="arm_id">fr3</param>
  <param name="prefix">left</param>
  <param name="load_gripper">true</param>
</hardware>
```

### Launch file

```python
cm_params = [
    controllers_yaml,
    {'robot_description': robot_description},
    {'mujoco_scene_xacro_path': '/path/to/scene.xml.xacro'},
    {'mujoco_scene_xacro_args': 'side:=left hand:=true'},
]
```

### Example launch commands

```bash
# FR3 single arm
ros2 launch franka_bringup example.launch.py \
  controller_name:=joint_position_example_controller use_mujoco:=true

# FR3 + Husky
ros2 launch fr3_husky_controller fr3_husky_controller.launch.py \
  robot_side:=left load_gripper:=true use_mujoco:=true

# FR3 only (no mobile base)
ros2 launch fr3_husky_controller fr3_controller.launch.py \
  robot_side:=left load_gripper:=true load_mobile:=false use_mujoco:=true
```

---

## Supported Robot Types

| `robot_type` | Handler | Description |
|---|---|---|
| `franka` | `FrankaSubHandler` | Single FR3 arm |
| `franka_multi` | `FrankaMultiSubHandler` | Multiple FR3 arms (one plugin block) |
| `husky` | `HuskySubHandler` | Clearpath Husky mobile base |
