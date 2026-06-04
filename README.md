# 3D-SLAM-Perception

基于 ROS2 Humble、FishBot 仿真和 Nav2 的自然语言语义导航项目。

当前仓库的 V1 版本已经完成一个确定性语义导航闭环：

```text
自然语言关键词
  -> 预定义具名地点或静态语义物体
  -> map 坐标系安全停靠位姿
  -> Nav2 NavigateToPose goal
  -> 小车导航到目标附近
```

V1 的重点是先验证机器人系统闭环，而不是让大模型直接控制底盘。自然语言解析采用关键词和别名匹配；目标来自 YAML 中的人工配置地点和静态物体；路径规划、避障和运动控制仍由 Nav2 完成。

## V1 已实现能力

- FishBot Gazebo 仿真基线。
- Nav2 二维导航闭环，包括地图、AMCL、planner、controller 和 RViz。
- 固定目标导航示例：导航到地图坐标 `(1.0, 1.0)`。
- 多路点导航示例。
- 确定性语义导航节点 `semantic_nav_orchestrator`。
- 文本命令到具名地点，例如“去充电区”。
- 文本命令到静态语义物体，例如“去圆柱旁边”。
- 物体附近停靠点采样，避免直接导航到物体中心。
- 地图占用过滤和 Nav2 `/compute_path_to_pose` 路径过滤。
- 通过 Nav2 `/navigate_to_pose` action 执行导航。
- 导航状态发布和任务取消。
- 对未知目标、歧义目标、失效对象和无安全停靠点的保守拒绝。

## 当前边界

V1 是一个确定性 MVP，不包含在线 3D 感知和大模型 grounding。

尚未实现：

- 相机或点云自动识别物体。
- 在线更新语义对象地图。
- 多帧跟踪与稳定对象 ID。
- 3D MLLM / Chat-Scene 语言 grounding。
- 单目多视角三角化。
- 对象位置不确定度和过期时间管理。

后续接入大模型时，推荐保持职责边界：模型只负责把语言解析为对象 ID，机器人系统继续负责对象地图查询、安全停靠点计算、路径可达性验证和 Nav2 导航。

## 目录结构

```text
.
├── README.md
├── PROJECT_OVERVIEW_CN.md
├── BUILD_PROMPTS.md
├── docs/
│   ├── DETERMINISTIC_SEMANTIC_NAV_CN.md
│   └── STACK_INVENTORY.md
└── ros2_ws/
    └── src/
        ├── fishbot_description/
        ├── fishbot_navigation2/
        ├── fishbot_application/
        ├── nav2_custom_controller/
        └── semantic_nav_orchestrator/
```

主要包说明：

| 包 | 作用 |
|---|---|
| `fishbot_description` | Gazebo world、URDF/Xacro、传感器和控制器配置 |
| `fishbot_navigation2` | 地图、Nav2 参数和导航启动文件 |
| `fishbot_application` | Nav2 Python 示例：初始位姿、固定目标、多路点 |
| `nav2_custom_controller` | 当前 Nav2 参数使用的自定义 controller |
| `semantic_nav_orchestrator` | V1 语义导航闭环：文本解析、停靠点规划、Nav2 action 编排 |

## 构建

```bash
cd ~/3d-slam-perception/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

首次构建后，每个新终端都先执行：

```bash
cd ~/3d-slam-perception/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 启动 Nav2 仿真基线

终端 1：启动 Gazebo 仿真。

```bash
ros2 launch fishbot_description gazebo_sim.launch.py
```

终端 2：启动 Nav2 和 RViz。

```bash
ros2 launch fishbot_navigation2 navigation2.launch.py
```

终端 3：注入仿真小车的 `(0, 0)` AMCL 初始位姿。

```bash
ros2 run fishbot_application init_robot_pose
```

终端 4：运行固定目标导航测试，使小车导航到地图坐标 `(1.0, 1.0)`。

```bash
ros2 run fishbot_application nav_to_pose
```

也可以在 RViz 中使用 `2D Pose Estimate` 设置初始位姿，再使用 `Nav2 Goal` 点击任意可达目标点。

多路点导航示例：

```bash
ros2 run fishbot_application waypoint_follower
```

## 启动 V1 语义导航闭环

先按照上一节启动 Gazebo、Nav2 和 `init_robot_pose`，再打开一个新终端启动语义导航节点：

```bash
cd ~/3d-slam-perception/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch semantic_nav_orchestrator semantic_navigation.launch.py
```

观察语义导航状态：

```bash
ros2 topic echo /semantic_nav/status
```

发送具名地点命令：

```bash
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String \
  "{data: '去充电区'}"
```

发送静态物体命令。该命令会选择圆柱附近的安全停靠点，而不是导航到圆柱中心：

```bash
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String \
  "{data: '去圆柱旁边'}"
```

拒绝未知目标：

```bash
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String \
  "{data: '去不存在的桌子'}"
```

取消当前任务：

```bash
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String \
  "{data: '取消'}"
```

## V1 配置

语义导航配置位于 `ros2_ws/src/semantic_nav_orchestrator/config/`：

| 文件 | 作用 |
|---|---|
| `named_locations.yaml` | 具名地点，例如“充电区”“原点” |
| `object_map.yaml` | 静态物体 ID、别名、状态和 `map` 坐标 |
| `semantic_nav_params.yaml` | 停靠距离、障碍余量和候选点数量 |

示例具名地点：

```yaml
locations:
  charging_area:
    aliases: ["充电区", "充电桩", "charging area"]
    pose: {x: 1.0, y: 1.0, yaw: 0.0}
```

示例静态物体：

```yaml
objects:
  - object_id: cylinder_0001
    label: cylinder
    aliases: ["圆柱", "柱子", "cylinder"]
    position: {x: -0.961, y: 0.717, z: 0.5}
    confidence: 1.0
    status: active
```

修改配置后需要重新构建，或确保 launch 使用的是安装空间中更新后的配置。

## 测试

运行纯 Python 单元测试：

```bash
cd ~/3d-slam-perception/ros2_ws
source /opt/ros/humble/setup.bash
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
colcon test --packages-select semantic_nav_orchestrator
colcon test-result --verbose
```

如果直接运行 `pytest` 遇到用户级 Python 插件冲突，可以保留 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`。当前已知 WSL2 用户环境中可能存在与系统 pytest 版本不兼容的第三方插件。

## 文档

- [确定性的语义导航骨架：原理与接入方式](docs/DETERMINISTIC_SEMANTIC_NAV_CN.md)
- [环境与栈清单](docs/STACK_INVENTORY.md)
- [项目蓝图与后续 3D MLLM 接入规划](PROJECT_OVERVIEW_CN.md)
- [分阶段构建提示词](BUILD_PROMPTS.md)

## WSL2 备注

Gazebo Classic 首次启动可能等待远程模型库。当前 Nav2 launch 已显式关闭 composition，以避免低资源场景下 composable 节点加载超时。
