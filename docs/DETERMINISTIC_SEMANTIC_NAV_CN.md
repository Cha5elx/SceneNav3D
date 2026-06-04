# 确定性的语义导航骨架：原理与接入方式

## 1. 为什么先实现规则版本

当前 FishBot 已经具备普通二维导航闭环：

```text
地图 + 2D 激光雷达 + 里程计
  -> AMCL 定位
  -> Nav2 全局规划与局部避障
  -> /cmd_vel
  -> 差速底盘运动
```

这个闭环能够执行一个明确的 `map` 坐标系目标位姿，例如：

```text
x = 1.0 m
y = 1.0 m
yaw = 0.0 rad
```

但它不能直接理解“去圆柱旁边”或“去充电区”。

确定性的语义导航骨架在 Nav2 上方增加一个规则层，将自然语言转换为安全的
`PoseStamped`。它不使用大模型，也不改变 Nav2 的定位、路径规划、避障和底盘
控制。这样可以先验证机器人系统中的语义导航接口，再逐步接入 3D 感知和
3D MLLM。

## 2. 总体架构

新增后的流程为：

```text
用户文本
  -> /semantic_nav/command
  -> semantic_nav_node
      -> 规则解析
      -> 具名地点查询，或静态语义对象查询
      -> 物体周围候选停靠点生成
      -> OccupancyGrid 障碍过滤
      -> Nav2 ComputePathToPose 路径过滤
      -> Nav2 NavigateToPose
  -> 原有 Nav2 二维导航闭环
  -> /cmd_vel
  -> 底盘运动
```

关键边界是：

```text
语义导航层只生成目标位姿
Nav2 仍然负责运动规划和执行
```

`semantic_nav_node` 不发布 `/cmd_vel`。以后接入 3D MLLM 时，也必须保持这个
边界。

## 3. 原本导航流程没有被替换

原有 FishBot 导航基线仍然负责：

1. Gazebo 中发布传感器数据和机器人状态。
2. AMCL 根据 `/scan`、地图和里程计估计机器人位姿。
3. TF 提供 `map -> odom -> base_link` 坐标变换。
4. Nav2 接收 `/navigate_to_pose` action 目标。
5. Nav2 规划路径、局部避障并发布 `/cmd_vel`。

原有示例 `fishbot_application/nav_to_pose.py` 是直接写死 `(1.0, 1.0)`：

```text
固定 PoseStamped -> Nav2 NavigateToPose
```

新增语义导航层只是把“固定 PoseStamped”替换为“由文本和语义地图推导出的
PoseStamped”：

```text
文本 -> 规则推导 PoseStamped -> Nav2 NavigateToPose
```

因此，普通二维导航闭环仍然可以独立使用。语义导航节点是一个可选的上层入口。

## 4. 配置数据

### 4.1 具名地点

`config/named_locations.yaml` 保存可以直接导航到达的位置：

```yaml
locations:
  charging_area:
    aliases: ["充电区", "充电桩", "charging area"]
    pose: {x: 1.0, y: 1.0, yaw: 0.0}
```

当用户输入“去充电区”时，节点直接生成 `(1.0, 1.0, 0.0)`，然后调用
`NavigateToPose`。

适合使用具名地点的目标包括：

- 充电区；
- 房间门口；
- 固定巡检点；
- 人工确认过的安全停靠位置。

### 4.2 静态语义对象

`config/object_map.yaml` 保存物体 ID、别名和物体中心在 `map` 坐标系中的位置：

```yaml
objects:
  - object_id: cylinder_0001
    label: cylinder
    aliases: ["圆柱", "柱子", "cylinder"]
    position: {x: -0.961, y: 0.717, z: 0.5}
    confidence: 1.0
    status: active
```

当前 MVP 中，这些物体坐标是人工配置的。以后接入 3D 感知时，可以由视觉、
点云和 TF 自动更新对象地图，但上层导航接口不需要改变。

## 5. 文本如何被确定性解析

规则解析器位于 `semantic_nav_orchestrator/core.py` 的 `SemanticResolver`。

处理顺序如下：

1. 将文本转为小写并去除空白。
2. 优先匹配具名地点及其别名。
3. 如果没有命中地点，再匹配静态对象的 `label` 和 `aliases`。
4. 如果恰好命中一个活动对象，返回稳定的 `object_id`。
5. 如果没有命中，拒绝执行。
6. 如果命中多个活动对象，返回歧义错误，不自动猜测。
7. 如果对象状态为 `inactive`，拒绝执行。

示例：

| 输入 | 解析结果 |
|---|---|
| `去充电区` | 具名地点 `charging_area` |
| `去圆柱旁边` | 静态对象 `cylinder_0001` |
| `去不存在的桌子` | 拒绝：`unknown target` |
| `去椅子旁边`，但存在两把椅子 | 拒绝：`ambiguous semantic object` |

这个 resolver 是以后接入 3D MLLM 时的替换点。未来模型需要输出稳定的
`object_id`，而不是直接输出速度指令。

## 6. 为什么不能直接导航到物体中心

对象地图中的 `position` 表示物体中心。物体中心通常位于障碍物内部，例如桌子、
柜子或圆柱内部。如果把物体中心直接传给 Nav2，目标点可能不可达，或者机器人会
尝试贴近障碍物。

因此，需要一个安全停靠点规划器：

```text
物体中心
  -> 周围圆环采样
  -> 障碍过滤
  -> 路径可达性过滤
  -> 选择较近候选点
```

## 7. 安全停靠点如何计算

`ApproachPosePlanner` 在物体周围生成一圈候选点。默认参数位于
`config/semantic_nav_params.yaml`：

```yaml
approach_distance: 0.8
clearance: 0.25
candidate_count: 16
```

假设对象中心为：

```text
(object_x, object_y)
```

第 `i` 个候选点为：

```text
angle = 2 * pi * i / candidate_count
x = object_x + approach_distance * cos(angle)
y = object_y + approach_distance * sin(angle)
yaw = atan2(object_y - y, object_x - x)
```

其中 `yaw` 让机器人停靠后朝向目标物体。

每个候选点依次经过两层过滤。

### 7.1 地图占用过滤

节点订阅 Nav2 地图 `/map`，读取 `nav_msgs/msg/OccupancyGrid`。

候选点必须满足：

- 位于地图范围内；
- 所在栅格为空闲区域；
- 以候选点为中心、`clearance` 为半径的邻域内没有障碍或未知区域。

### 7.2 Nav2 路径过滤

通过地图占用过滤后，节点调用 Nav2 的 `/compute_path_to_pose` action。

Nav2 会判断从机器人当前位置到候选点是否存在可规划路径。节点按照候选点到
机器人当前位置的距离由近到远检查，选择第一个可达候选点。

最后，再把选中的位姿发送给 `/navigate_to_pose`。

## 8. ROS2 节点与接口

新增节点：

```text
semantic_nav_orchestrator / semantic_nav_node
```

输入 topic：

```text
/semantic_nav/command
类型：std_msgs/msg/String
```

状态 topic：

```text
/semantic_nav/status
类型：std_msgs/msg/String
```

读取的数据：

```text
/map
类型：nav_msgs/msg/OccupancyGrid

TF: map -> base_link
```

调用的 Nav2 actions：

```text
/compute_path_to_pose
类型：nav2_msgs/action/ComputePathToPose

/navigate_to_pose
类型：nav2_msgs/action/NavigateToPose
```

节点采用异步 action client。路径筛选和导航执行不会阻塞 ROS2 executor。
每个任务都有递增的内部 ID，取消之后迟到的旧回调不会重新触发导航。

## 9. 如何融入原有启动流程

原有三个终端保持不变：

```bash
# 终端 1：启动 Gazebo
ros2 launch fishbot_description gazebo_sim.launch.py

# 终端 2：启动 Nav2 和 RViz
ros2 launch fishbot_navigation2 navigation2.launch.py

# 终端 3：注入 AMCL 初始位姿
ros2 run fishbot_application init_robot_pose
```

新增一个终端启动语义导航节点：

```bash
ros2 launch semantic_nav_orchestrator semantic_navigation.launch.py
```

原本的二维导航流程没有被覆盖。可以继续使用：

```bash
ros2 run fishbot_application nav_to_pose
```

也可以改用语义入口：

```bash
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String \
  "{data: '去圆柱旁边'}"
```

## 10. 当前能力边界

当前实现是一个用于验证系统接口的 MVP，不是完整的 3D 感知系统。

已经具备：

- 文本到具名地点；
- 文本到静态对象 ID；
- 对未知、歧义和失效对象的保守拒绝；
- 物体周围停靠点采样；
- 地图障碍过滤；
- Nav2 路径过滤；
- 导航状态发布；
- 导航取消；
- 与原有二维 Nav2 闭环解耦。

尚未具备：

- 相机或点云自动识别物体；
- 在线更新对象地图；
- 多帧跟踪与稳定对象 ID；
- 3D MLLM 语言 grounding；
- 单目多视角三角化；
- 对象位置不确定度与过期时间管理。

## 11. 后续接入 3D MLLM 的位置

未来接入 3D MLLM 时，推荐保留现有安全链路：

```text
用户文本
  -> 3D MLLM grounding
  -> object_id
  -> semantic_object_map
  -> ApproachPosePlanner
  -> ComputePathToPose
  -> NavigateToPose
  -> 原有 Nav2 二维导航闭环
```

模型只负责回答“用户指的是哪个对象”。物体坐标、停靠点安全检查、路径规划和
底盘控制仍然由确定性的机器人系统负责。

## 12. 对应代码

| 内容 | 文件 |
|---|---|
| 文本解析、栅格过滤、停靠点采样 | `ros2_ws/src/semantic_nav_orchestrator/semantic_nav_orchestrator/core.py` |
| ROS2 topic、TF 和 Nav2 action 编排 | `ros2_ws/src/semantic_nav_orchestrator/semantic_nav_orchestrator/node.py` |
| 具名地点 | `ros2_ws/src/semantic_nav_orchestrator/config/named_locations.yaml` |
| 静态对象地图 | `ros2_ws/src/semantic_nav_orchestrator/config/object_map.yaml` |
| 停靠参数 | `ros2_ws/src/semantic_nav_orchestrator/config/semantic_nav_params.yaml` |
| 启动文件 | `ros2_ws/src/semantic_nav_orchestrator/launch/semantic_navigation.launch.py` |
| 单元测试 | `ros2_ws/src/semantic_nav_orchestrator/test/test_core.py` |

