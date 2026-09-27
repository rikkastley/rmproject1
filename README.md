# RoboMaster GM6020 正弦波跟随控制系统

## 📌 项目简介
本项目是基于 STM32F407IGHx 微控制器的 GM6020 直流无刷电机控制工程。通过 CAN 总线与电机进行通信，实现了**速度与位置的正弦波曲线跟随**，并内置了**独立看门狗（IWDG）**保护机制，确保系统运行的安全性。

## ✨ 功能特性
- **速度正弦波跟随**：电机转速跟随自定义的正弦波曲线（`sin`函数生成）。
- **位置正弦波跟随**：电机累计角度跟随自定义的正弦波曲线。
- **控制模式切换**：通过全局变量 `control_mode` 一键切换（`0` 为速度环，`1` 为位置环）。
- **看门狗保护**：启用 STM32 内部 IWDG，超时时间约 1.25 秒，主循环定期喂狗，程序跑飞时自动复位电机。
- **模块化架构**：将电机驱动、PID 算法、任务逻辑独立封装，结构清晰，易于维护。
- **现代化构建**：使用 CMake + Ninja + VS Code 进行开发与编译，摆脱传统 IDE 的束缚。

## 🛠️ 开发环境与工具链
- **主控芯片**：STM32F407IGHx (RoboMaster C型开发板)
- **底层配置**：STM32CubeMX (生成 HAL 库与 CMake 工程)
- **编译器**：arm-none-eabi-gcc (GCC)
- **构建工具**：CMake (v3.22+) + Ninja
- **编辑器**：Visual Studio Code
- **调试器**：SEGGER Ozone (或 Cortex-Debug)

## 📁 目录结构
```text
├── Core/               # CubeMX 生成的底层 HAL 库与 main.c
├── Drivers/            # STM32F4xx 官方外设驱动
├── Tasks/              # 🌟 核心自研代码
│   ├── ControlTask.h   # 任务调度接口声明
│   ├── ControlTask.cpp # 正弦波生成、PID计算、CAN数据收发控制逻辑
│   ├── Gm6020.h        # GM6020 电机面向对象封装
│   ├── Gm6020.cpp      # 电机CAN协议编解码与反馈解析
│   ├── Pid.h           # 位置式PID算法接口
│   └── Pid.cpp         # PID算法实现（含抗积分饱和与输出限幅）
├── cmake/              # CubeMX 生成的 CMake 底层配置
├── CMakeLists.txt      # 项目构建脚本（已配置 Tasks 目录）
└── README.md           # 项目说明文档
