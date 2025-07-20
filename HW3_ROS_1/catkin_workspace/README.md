# Turtle Control Package

## 项目描述

这是一个标准的ROS工作空间，按照catkin工作空间结构组织。包含一个完整的ROS包用于控制turtlesim中的乌龟运动。

## 🏗️ 工作空间结构

```
catkin_workspace/
├── CMakeLists.txt                           # 工作空间顶层CMakeLists
├── src/                                     # 源代码文件夹
│   └── turtle_control_pkg/                  # ROS包
│       ├── CMakeLists.txt                   # 包的CMakeLists
│       ├── package.xml                      # 包描述文件
│       ├── src/                             # C++源代码
│       │   ├── turtle_controller.cpp        # C++乌龟控制器
│       │   └── simple_publisher.cpp         # 简单发布器示例
│       ├── scripts/                         # Python脚本
│       │   └── turtle_velocity_publisher.py # Python速度发布器
│       ├── msg/                             # 自定义消息定义
│       │   └── TurtleVelocity.msg          # 乌龟速度消息
│       ├── launch/                          # Launch文件
│       │   └── turtle_control.launch        # 主启动文件
│       ├── config/                          # 配置文件
│       │   └── turtle_params.yaml          # 参数配置
│       └── include/                         # C++头文件目录
├── build/                                   # 编译生成文件 (编译后生成)
└── devel/                                   # 开发环境文件 (编译后生成)
```

## 🎯 功能特性

- **标准ROS工作空间结构**: 完全遵循catkin工作空间规范
- **自定义消息类型**: `TurtleVelocity.msg` 包含完整的速度和轨迹信息
- **多语言节点**: Python发布器 + C++控制器
- **参数服务器集成**: YAML配置文件统一管理参数
- **安全机制**: 边界检查、速度限制、紧急停止
- **Launch系统**: 一键启动所有必要节点

## 🚀 编译和运行

### 1. 初始化和编译工作空间

```bash
# 进入工作空间
cd /home/simon/Desktop/Homeworks/HW3_ROS_1/catkin_workspace

# 编译工作空间
catkin_make

# 设置环境变量
source devel/setup.bash
```

### 2. 运行完整系统

```bash
# 启动完整系统
roslaunch turtle_control_pkg turtle_control.launch

# 带调试模式启动
roslaunch turtle_control_pkg turtle_control.launch debug_mode:=true

# 启动并记录数据
roslaunch turtle_control_pkg turtle_control.launch record_bag:=true
```

### 3. 单独运行节点

```bash
# 确保roscore在运行
roscore

# 在新终端中运行各个节点
rosrun turtlesim turtlesim_node
rosparam load src/turtle_control_pkg/config/turtle_params.yaml
rosrun turtle_control_pkg turtle_velocity_publisher.py
rosrun turtle_control_pkg turtle_controller
```

## 📋 自定义消息

### TurtleVelocity.msg
```
std_msgs/Header header          # 时间戳和坐标系信息
float32 linear_x               # X方向线速度
float32 linear_y               # Y方向线速度
float32 angular_z              # Z轴角速度
float32 velocity_magnitude     # 速度大小
bool is_emergency_stop         # 紧急停止标志
string trajectory_type         # 轨迹类型
float32 trajectory_progress    # 轨迹完成进度
```

## 🔧 节点说明

### 1. turtle_velocity_publisher.py (Python节点)
- **功能**: 生成并发布自定义速度命令
- **发布话题**: 
  - `/turtle_velocity_cmd` (turtle_control_pkg/TurtleVelocity)
  - `/turtle_linear_velocity` (std_msgs/Float32)
- **支持轨迹**: 圆形、正弦波、8字形、直线

### 2. turtle_controller (C++节点)
- **功能**: 接收速度命令，控制turtlesim乌龟
- **订阅话题**:
  - `/turtle_velocity_cmd` (turtle_control_pkg/TurtleVelocity)
  - `/turtle_linear_velocity` (std_msgs/Float32)
  - `/turtle1/pose` (turtlesim/Pose)
- **发布话题**:
  - `/turtle1/cmd_vel` (geometry_msgs/Twist)
  - `/turtle_controller_status` (std_msgs/Float32)

### 3. simple_publisher (C++示例节点)
- **功能**: 演示基本的ROS发布器模式
- **用途**: 学习ROS发布器的标准写法

## 📊 系统架构

```
参数服务器 (turtle_params.yaml)
    ↓
Python发布器 (turtle_velocity_publisher.py)
    ↓ TurtleVelocity消息
C++控制器 (turtle_controller)
    ↓ Twist消息
turtlesim节点
```

## 🛠️ 开发工具

### 话题监控
```bash
# 查看所有话题
rostopic list

# 监控速度命令
rostopic echo /turtle_velocity_cmd

# 查看消息类型
rosmsg show turtle_control_pkg/TurtleVelocity
```

### 参数操作
```bash
# 查看参数
rosparam list
rosparam get /turtle_params/motion/max_linear_velocity

# 修改参数
rosparam set /turtle_params/trajectory/type "sinusoidal"
```

### 节点信息
```bash
# 查看节点
rosnode list
rosnode info /turtle_controller

# 查看计算图
rosrun rqt_graph rqt_graph
```

## 🎓 学习要点

1. **工作空间结构**: 理解catkin工作空间的标准组织方式
2. **包管理**: package.xml和CMakeLists.txt的配置
3. **消息系统**: 自定义消息的定义和使用
4. **多语言开发**: Python和C++节点的协作
5. **参数服务器**: 集中化配置管理
6. **Launch系统**: 复杂系统的启动管理

## 🔍 故障排除

### 编译问题
```bash
# 清理编译文件
cd catkin_workspace
rm -rf build devel
catkin_make clean
catkin_make
```

### 环境变量问题
```bash
# 确保正确设置环境
source /opt/ros/noetic/setup.bash  # ROS系统环境
source devel/setup.bash            # 工作空间环境
```

### 权限问题
```bash
# 设置Python脚本权限
chmod +x src/turtle_control_pkg/scripts/turtle_velocity_publisher.py
```

## 📝 作者信息

- **作者**: Student
- **日期**: 2025-07-20
- **版本**: 1.0.0
- **工作空间**: Catkin Workspace Standard Structure

这个项目完全遵循ROS官方推荐的工作空间结构，是学习ROS开发的标准范例。
