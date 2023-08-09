# Balance_Infantry
> 2023  Balance Infantry USTL COD

## 依赖工具及软硬件环境

依赖工具：Keil5,VsCode

软件环境：Windows11

硬件环境：STM32F407

## 编译器及编译方式

Arm Compiler 5

C/C++编译

## 简介

├── Balance Infantry
│   ├── Chassis : 平衡步兵底盘代码
│   ├── Gimbal : 平衡步兵云台代码
│   ├── balancce.mat : Matlab数据文件，记录上一次计算的权重矩阵和状态反馈控制器等
│   ├── [BalanceInfantry_LQR.md](./BalanceInfantry_LQR.md) ：平衡步兵的状态空间方程及权重矩阵，由MATLAB实现
│   ├── [MATLAB.md](./MATLAB.md) ：简易倒立摆的状态空间方程（一次失败的尝试），由MATLAB实现
│   ├── [Python.md](./Python.md) ：简易倒立摆的状态空间方程（一次无疾而终的尝试），由Python实现