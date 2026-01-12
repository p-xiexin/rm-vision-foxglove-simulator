# foxglove_cpp_example

![](./doc/ssr.gif)


## 项目简介

**foxglove_websocket_cpp** 是一个面向 RoboMaster 视觉自瞄系统开发的轻量级实验平台。使用 Foxglove WebSocket 协议，帮助开发者方便地把自己的自瞄算法和各种仿真工具连起来，快速搭建和验证视觉相关功能。目前还是实验阶段，欢迎大家体验、吐槽和参与改进。

## 动机

RoboMaster 视觉自瞄环节算法迭代频繁、硬件调试成本高。单靠真机调试很容易踩坑、耗时，还可能有安全风险。我们希望有一个灵活的仿真平台，既能和自己写的视觉算法对接，也能配合 Foxglove Studio 这样可视化好用的工具，帮大家更快摸索和测试自瞄思路，不用频繁地搬砖和修硬件。

## 平台亮点

- **彻底脱离硬件**：在纯仿真环境下测试算法，完全不用担心机械、底盘、电控等硬件因素对算法表现产生干扰，算法效果一目了然。
- **通用通信协议**：用 Foxglove WebSocket，让视觉数据在不同工具和平台间“无障碍流通”，比如快速接入 Foxglove Studio 或 ROS 2 环境。
- **C++实现，性能靠谱**：用起来不拖后腿，适合需要高实时性的场景。
- **结构简单、易扩展**：随时可以魔改（实验嘛），不同视觉算法或仿真器都能比较容易地接进来。
- **灵活可视化**：和 Foxglove Studio、RViz 等配合，调试更直观，有问题能迅速定位。
- **低门槛起步**：即插即用的思路，减少配置和折腾时间，只需关注你自己的视觉算法和逻辑。
- **高质量坐标建模**：基于 Eigen 库实现相机内参、外参建模，以及装甲板、机体、世界、相机等多个主要坐标系之间的转换，适合做真“工程量级”的联调和算法验证。

## 典型用法

- 搭建你的自瞄算法在仿真环境下的测试闭环
- 联合调试目标检测+预测+仿真策略
- 用 Foxglove Studio 直观可视化算法输出效果
- 做为视觉通信层的实验“沙箱”，尽情试错
- 完全剥离硬件干扰，单独验证视觉算法、跟踪与预测模型

## EKF 说明

`src/rm_robot_sim.cpp` 内置了一个简化的 EKF，用于从 PnP 测量中平滑估计机器人中心与姿态。定义如下。

- 输入量 (观测向量 z，世界坐标系)：  
  `z = [x_armor, y_armor, z_armor, yaw_armor]`  
  其中 `x_armor,y_armor,z_armor` 为 PnP 得到的装甲板中心位置（由相机坐标转换到世界坐标），`yaw_armor` 为装甲板姿态的世界坐标 yaw。
- 状态量 (9 维状态向量 x)：  
  `x = [x_c, v_x, y_c, v_y, z_c, v_z, yaw, v_yaw, r]`  
  其中 `x_c,y_c,z_c` 是机器人中心位置，`v_x,v_y,v_z` 是中心速度，`yaw` 是机器人朝向，`v_yaw` 是角速度，`r` 是装甲板环绕半径。
- 观测模型：  
  根据中心位置与半径，得到装甲板中心：  
  `x_armor = x_c - r * cos(yaw)`  
  `y_armor = y_c - r * sin(yaw)`  
  `z_armor = z_c`  
  `yaw_armor = yaw`

## 下一步计划

* [x] 我们计划提供将 chenjun 的 [rm-vision](https://github.com/chenjunnn/rm_auto_aim) 对接本平台的示例工程，支持大家参考和体验目前优秀算法在纯仿真环境下的效果。

## 如何体验
依赖于[foxglove/ws-protocol/cpp/foxglove-websocket](https://github.com/foxglove/ws-protocol/tree/main/cpp/foxglove-websocket)

1. 克隆仓库：
   ```bash
   git clone https://github.com/p-xiexin/foxglove_websocket_cpp.git
   ```
2. 按说明或参考代码编译运行，对接你的自瞄模块或适配仿真数据。
  ```bash
  sudo apt update
  sudo apt install -y nlohmann-json3-dev libwebsocketpp-dev libprotobuf-dev
  ./compile_proto.sh
  cmake . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=1
  cmake --build build -j
  ./build/rm_robot_sim
  ```

  打开 https://app.foxglove.dev/ 可视化查看结果

## 关于本项目

- 实验平台，文档和功能持续补充中，踩坑欢迎提 issue。
- 结构简单，喜欢折腾可以 fork 改造。
- 拥抱交流，欢迎讨论、建议和 PR！

> 这个平台的初衷：让 RoboMaster 视觉自瞄开发更自由、少挨打、多点乐趣～

