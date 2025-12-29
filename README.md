# TEB局部路径规划器 - ROS 2实现

一个基于**时间弹性带（Timed Elastic Band, TEB）**算法的ROS 2移动机器人局部路径规划插件。通过图优化技术实现平滑、动态可行的轨迹生成和实时避障。

## 主要特性

- **平滑轨迹生成** - 基于弹性带理论生成时间最优路径
- **实时动态避障** - 对静态和动态障碍物进行实时路径调整
- **运动学约束** - 严格遵守速度、加速度和Jerk限制
- **精确目标到达** - 精准到达目标位置和姿态
- **图优化框架** - 基于g2o的高效非线性优化
- **Nav2集成** - 完全兼容ROS 2导航栈

## 系统架构

```
┌──────────────────────────────────────────────────────────┐
│                    ROS 2 Nav2接口层                       │
│                  (teb_controller.cpp)                    │
│  • 接收全局路径和目标点                                    │
│  • 发布速度控制命令                                       │
│  • 可视化局部轨迹                                         │
└───────────────────┬──────────────────────────────────────┘
                    │
┌───────────────────▼──────────────────────────────────────┐
│                TEB算法核心 (planner_manager)              │
│                                                          │
│  图构建模块                优化求解模块                    │
│  ├─ 顶点 (Vertices)       ├─ g2o优化器                   │
│  │  ├─ VertexPoint2D     ├─ Levenberg-Marquardt         │
│  │  └─ VertexTimeDiff    └─ 非线性最小二乘                │
│  │                                                       │
│  └─ 约束边 (Edges)                                       │
│     ├─ EdgeVelocity         速度约束                     │
│     ├─ EdgeAcceleration     加速度约束                    │
│     ├─ EdgeJerk             Jerk约束                     │
│     ├─ EdgeObstacle         障碍物约束                    │
│     ├─ EdgeViaPoint         路径点约束                    │
│     └─ EdgeGoalPose         目标点约束                 │
└──────────────────────────────────────────────────────────┘
```

## 核心技术

### 1. TEB算法原理

TEB将轨迹规划问题建模为**时间弹性带优化问题**：

- **弹性带** - 由一系列位姿点和时间间隔组成的序列
- **时间优化** - 动态调整时间间隔以满足速度约束
- **图优化** - 将约束表示为图的边，通过最小化误差函数优化轨迹

**优化目标函数：**

```
min Σ w_i · ||error_i||²

包括：
• 平滑度误差（速度/加速度/Jerk连续性）
• 障碍物距离误差
• 目标点到达误差
• 运动学约束误差
```

### 2. 图优化实现

使用**g2o (General Graph Optimization)** 框架：

```cpp
// 顶点：表示机器人的位姿和时间
VertexPoint2D     -> (x, y, θ)
VertexTimeDiff    -> Δt

// 边：表示各种约束
EdgeVelocity      -> ||v|| ≤ v_max
EdgeAcceleration  -> ||a|| ≤ a_max  
EdgeJerk          -> ||j|| ≤ j_max
EdgeObstacle      -> dist(robot, obstacle) ≥ d_safe
EdgeGoalPose      -> ||pose_final - pose_goal||² → 0
```

### 3. 约束边详解

#### EdgeGoalPose - 目标点约束

确保轨迹终点精确到达目标位置和姿态：

```cpp
void EdgeGoalPoseConstraint::computeError() {
    // 位置误差
    double dx = current_x - goal_x;
    double dy = current_y - goal_y;
    _error[0] = sqrt(dx*dx + dy*dy);
    
    // 方向误差
    double dtheta = normalize_theta(current_theta - goal_theta);
    _error[1] = dtheta;
}
```

#### EdgeJerk - Jerk约束

限制加加速度，确保运动平滑舒适：

```cpp
// Jerk定义：加速度的变化率
jerk_linear = (a_k1 - a_k) / Δt
jerk_angular = (α_k1 - α_k) / Δt
```

#### EdgeObstacle - 障碍物约束

计算机器人到障碍物的最小距离，施加排斥力：

```cpp
double dist = min_distance(robot_footprint, obstacle);
if (dist < inflation_radius) {
    error = inflation_radius - dist;  // 越近误差越大
}
```

## 🛠️ 技术栈

| 组件 | 技术 | 用途 |
|-----|------|------|
| **框架** | ROS 2 Humble/Iron | 机器人操作系统 |
| **优化器** | g2o | 图优化/非线性最小二乘 |
| **构建** | Colcon | ROS 2构建工具 |
| **数学库** | Eigen3 | 线性代数运算 |
| **TF库** | tf2_ros | 坐标变换 |

## 安装

### 依赖项

```bash
sudo apt install ros-humble-nav2-core \
                 ros-humble-nav2-costmap-2d \
                 ros-humble-tf2-geometry-msgs \
                 libeigen3-dev \
                 libg2o-dev
```

### 编译

```bash
# 克隆仓库
cd ~/ros2_ws/src
git clone git@github.com:Lv-Jiahao/TEB_controller_plugin.git

# 编译
cd ~/ros2_ws
colcon build --packages-select teb_controller_plugin

# 加载环境
source install/setup.bash
```

## 🚀 使用方法

### 1. 配置参数

创建配置文件 `teb_params.yaml`：

```yaml
teb_controller:
  ros__parameters:
    # 轨迹配置
    teb_autosize: true
    dt_ref: 0.3
    dt_hysteresis: 0.1
    min_samples: 3
    max_samples: 500
    
    # 目标点权重
    weight_goal_position: 500.0
    weight_goal_orientation: 200.0
    
    # 运动学约束
    max_vel_x: 0.5
    max_vel_theta: 1.0
    acc_lim_x: 0.5
    acc_lim_theta: 1.0
    
    # 障碍物
    min_obstacle_dist: 0.3
    inflation_dist: 0.6
    
    # 优化参数
    no_inner_iterations: 5
    no_outer_iterations: 4
    optimization_activate: true
```

### 2. 启动导航

```bash
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=/path/to/teb_params.yaml
```

### 3. 发送目标点

```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped \
"{header: {frame_id: 'map'}, 
  pose: {position: {x: 5.0, y: 3.0}, 
         orientation: {w: 1.0}}}"
```


## 算法流程

1. **接收全局路径** - 从全局规划器获取参考路径
2. **局部窗口截取** - 提取当前位置周围的路径段
3. **初始化TEB** - 在局部路径上均匀采样初始轨迹点
4. **构建优化图** 
   - 添加位姿顶点和时间顶点
   - 添加各类约束边（速度、加速度、障碍物等）
5. **图优化求解** - 使用LM算法迭代优化
6. **提取速度** - 从优化后的轨迹提取当前速度命令
7. **发布控制** - 发布速度到机器人底盘

## 性能特点

- **实时性** - 单次优化耗时：10-30ms（普通PC）
- **精度** - 目标到达误差：< 5cm（位置），< 5°（方向）
- **平滑度** - 速度曲线连续，加速度有界
- **鲁棒性** - 自适应调整轨迹点数量，处理复杂环境

## 项目结构

```
teb_controller_plugin/
├── teb_algorithm/              # TEB算法核心
│   ├── include/
│   │   ├── planner_manager.h      # 规划器管理类
│   │   ├── vertexPoint.h          # 位姿顶点
│   │   ├── vertexTimeDiff.h       # 时间顶点
│   │   ├── base_teb_edges.h       # 边基类
│   │   ├── edge_velocity.h        # 速度约束边
│   │   ├── edge_acceleration.h    # 加速度约束边
│   │   ├── jerk_edge.h            # Jerk约束边
│   │   ├── edge_obstacle.h        # 障碍物约束边
│   │   ├── edge_via_point.h       # 路径点约束边
│   │   └── edge_goal_pose.h       # 目标点约束边
│   └── src/
│       └── planner_manager.cpp    # 算法实现
├── teb_controller/             # ROS 2控制器接口
│   ├── include/
│   │   └── teb_controller.h
│   └── src/
│       └── teb_controller.cpp
├── config/
│   └── teb_params.yaml         # 默认参数配置
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 核心类说明

### PlannerManager

规划器管理类，负责TEB图的构建和优化：

```cpp
class plannerManager {
public:
    void initOptimizer();           // 初始化g2o优化器
    void AddVertices();             // 添加顶点到图
    void AddVelocityEdges();        // 添加速度约束
    void AddAccelerationEdges();    // 添加加速度约束
    void AddJerkEdges();            // 添加Jerk约束
    void AddObstacleEdges();        // 添加障碍物约束
    void AddGoalPoseEdge();         // 添加目标约束
    bool optimizeGraph();           // 执行优化
    geometry_msgs::msg::Twist getVelocityCommand();
};
```

### VertexPoint2D

表示机器人位姿的顶点：

```cpp
class VertexPoint2D : public g2o::BaseVertex<3, Eigen::Vector3d> {
    // 状态向量: [x, y, theta]
    virtual void setToOriginImpl() override;
    virtual void oplusImpl(const double* update) override;
};
```

### EdgeGoalPoseConstraint

目标点约束边：

```cpp
class EdgeGoalPoseConstraint : public BaseTebUnaryEdge<2, PoseStamped*, VertexPoint2D> {
    void computeError() override;
    // 误差向量: [position_error, orientation_error]
};
```

##  参数说明

### 轨迹参数

| 参数 | 类型 | 说明 | 默认值 |
|-----|------|------|--------|
| `teb_autosize` | bool | 自动调整轨迹点数 | true |
| `dt_ref` | double | 参考时间间隔(s) | 0.3 |
| `min_samples` | int | 最少轨迹点数 | 3 |
| `max_samples` | int | 最多轨迹点数 | 500 |

### 权重参数

| 参数 | 说明 | 推荐值 |
|-----|------|--------|
| `weight_goal_position` | 目标位置误差权重 | 500.0 |
| `weight_goal_orientation` | 目标方向误差权重 | 200.0 |
| `weight_obstacle` | 障碍物误差权重 | 50.0 |
| `weight_viapoint` | 路径点误差权重 | 10.0 |
| `weight_velocity` | 速度平滑权重 | 2.0 |
| `weight_acceleration` | 加速度平滑权重 | 1.0 |
| `weight_jerk` | Jerk平滑权重 | 0.5 |

### 运动学参数

| 参数 | 说明 | 单位 |
|-----|------|------|
| `max_vel_x` | 最大线速度 | m/s |
| `max_vel_theta` | 最大角速度 | rad/s |
| `acc_lim_x` | 最大线加速度 | m/s² |
| `acc_lim_theta` | 最大角加速度 | rad/s² |
| `max_jerk_x` | 最大线Jerk | m/s³ |
| `max_jerk_theta` | 最大角Jerk | rad/s³ |

##  调试与可视化

### 发布的Topic

```bash
# 局部规划路径
/local_plan (nav_msgs/Path)

# 速度命令
/cmd_vel (geometry_msgs/Twist)

# TEB轨迹点（用于可视化）
/teb_poses (geometry_msgs/PoseArray)

# 障碍物信息
/teb_obstacles (visualization_msgs/MarkerArray)
```

### 使用RViz可视化

```bash
ros2 run rviz2 rviz2
# 添加显示：
# - /local_plan (Path)
# - /teb_poses (PoseArray)  
# - /teb_obstacles (MarkerArray)
```

## 🙏 致谢

- ROS 2社区
- g2o开发团队
- Nav2项目

---

⭐ 如果这个项目对你有帮助，请给个Star支持一下！
