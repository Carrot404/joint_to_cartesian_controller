# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述
这是一个ROS 2 (Humble)控制器包，实现了关节空间到笛卡尔空间的坐标转换。该控制器使用KDL库进行正向运动学计算，并发布末端执行器的位姿和速度信息。

## 项目结构
```
joint_to_cartesian_controller/
├── CMakeLists.txt              # 构建配置
├── package.xml                 # ROS包配置
├── controller_plugins.xml       # 插件描述
├── README.md                   # 项目说明
├── include/
│   └── joint_to_cartesian_controller/
│       └── joint_to_cartesian_controller.hpp
├── src/
│   ├── joint_to_cartesian_controller.cpp
│   └── joint_to_cartesian_controller_parameters.yaml
└── .gitignore
```

## 构建和测试

### 构建项目
```bash
cd /Users/carrot/ws_docker/src/joint_to_cartesian_controller
colcon build --packages-select joint_to_cartesian_controller
```

### 运行测试
```bash
colcon test --packages-select joint_to_cartesian_controller
```

### 源码安装
```bash
source /Users/carrot/ws_docker/install/setup.bash
```

## 开发工作流

### 1. 代码编译
- 使用ament_cmake构建系统
- 代码遵循ROS 2控制器接口标准
- 使用generate_parameter_library进行参数管理

### 2. 参数配置
编辑`src/joint_to_cartesian_controller_parameters.yaml`文件来调整控制器参数：
- 修改`robot_base_link`和`end_effector_link`以匹配您的机器人
- 设置正确的`joints`列表
- 调整发布话题名称以符合您的ROS系统约定

### 3. 插件使用
控制器通过pluginlib注册，可在ROS 2控制器管理器中使用：
```bash
ros2 run controller_manager spawner
```

## 代码规范

### 1. 编码标准
- 使用C++17标准
- 遵循ROS 2命名规范
- 使用rclcpp进行ROS 2节点开发
- 使用KDL库进行运动学计算

### 2. 错误处理
- 使用ROS 2标准的日志系统
- 在关键函数中进行错误检查
- 遵循控制器接口的生命周期管理

### 3. 性能考虑
- 使用realtime_tools进行实时发布
- 避免在update循环中进行内存分配
- 使用引用传递减少拷贝开销

## 依赖项

### 核心依赖
- rclcpp: ROS 2 C++客户端库
- controller_interface: ROS 2控制器接口
- hardware_interface: 硬件接口抽象层
- kdl_parser: KDL解析器
- geometry_msgs: 几何消息类型
- tf2_kdl: KDL与TF2转换
- urdf: 机器人描述格式

### 构建依赖
- ament_cmake: ROS 2构建系统
- pluginlib: 插件系统
- generate_parameter_library: 参数库生成

## 调试和测试

### 1. 运行时调试
```bash
# 启动控制器管理器
ros2 run controller_manager controller_manager

# 查看发布的位姿信息
ros2 topic echo /current_pose

# 查看发布的速度信息
ros2 topic echo /current_twist
```

### 2. 常见问题
- 确保URDF文件正确配置
- 检查关节名称是否与硬件接口匹配
- 验证TF前缀设置是否正确

## 作者信息
- 作者: Songjie Xiao
- 邮箱: songjiexiao@zju.edu.cn
- 许可证: MIT

## 相关链接
- [ROS 2控制器接口文档](https://control.ros.org/master/doc/)
- [KDL文档](https://www.orocos.org/kdl)
- [pluginlib文档](https://wiki.ros.org/pluginlib)