# motion-core

[English](README.md) | [简体中文](README.zh-CN.md)

一个无第三方运行时依赖的 C++17 运动模型与控制算法库，适用于机器人和车辆算法原型、软件在环实验及数值回归测试。

## 核心能力

- 差速机器人与运动学自行车模型的确定性积分；
- 带限幅、抗积分饱和和饱和遥测的 PID；
- 离散双积分器有限时域 LQR；
- Pure Pursuit 与 Stanley 路径跟踪；
- 受加速度和加加速度约束的纵向预测控制；
- 可复现闭环 CSV 轨迹、库接口、CLI 和 CMake 安装包。

## 构建与运行

```powershell
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
.\build\motion-core.exe demo --steps 200 --dt 0.05 --output trace.csv
```

项目需要 CMake、Ninja 和 C++17 编译器，不需要 ROS、GPU、网络、容器或实车硬件。安装后的 `find_package` 接口有独立消费测试。

## 工程边界

模型是确定性的紧凑参考实现，省略轮胎、执行器、延迟和状态估计等高保真因素。预测控制器返回满足配置约束的固定迭代结果，但不声称具备量产 MPC 求解器的全部收敛证明和功能。

## 协作

史浩轩负责总体设计与主要实现；刘泽康参与安全边界和集成接口核验。职责说明见 [CONTRIBUTORS.md](CONTRIBUTORS.md)。

采用 [MIT License](LICENSE)。
