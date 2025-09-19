# 基于STM32F103ZET6与FreeRTOS的麦克纳姆轮四轮速度环闭环小车控制系统

## 项目概述

本项目基于STM32CubeIDE 1.19.0开发，使用的芯片是STM32F103ZET6 最小系统板 + PCA9685 PWM信号发生器 + L298N 有刷电机。当前使用的是JGY370有刷蜗轮蜗杆电机，可换成其它类型的有刷直流电机，当前电机轴最高转速为6000RPM，设定速度与编码器读取速度皆为电机轴速度。

## 电机独立PID控制功能

### 概述
项目已扩展支持每个电机使用独立的PID参数。初始配置为四个电机使用相同的PID参数，但现在可以为每个电机单独调整PID参数以获得更好的控制性能。

### PID参数配置
在 `Core/Inc/robot_types.h` 中为每个电机定义了独立的PID参数：

```c
/* 电机1 PID参数 */
#define MOTOR1_PID_KP     0.015   /* 电机1比例系数*/
#define MOTOR1_PID_KI     0.0025  /* 电机1积分系数*/
#define MOTOR1_PID_KD     0       /* 电机1微分系数*/

/* 电机2 PID参数 */
#define MOTOR2_PID_KP     0.015   /* 电机2比例系数*/
#define MOTOR2_PID_KI     0.0025  /* 电机2积分系数*/
#define MOTOR2_PID_KD     0       /* 电机2微分系数*/

/* 电机3 PID参数 */
#define MOTOR3_PID_KP     0.015   /* 电机3比例系数*/
#define MOTOR3_PID_KI     0.0025  /* 电机3积分系数*/
#define MOTOR3_PID_KD     0       /* 电机3微分系数*/

/* 电机4 PID参数 */
#define MOTOR4_PID_KP     0.015   /* 电机4比例系数*/
#define MOTOR4_PID_KI     0.0025  /* 电机4积分系数*/
#define MOTOR4_PID_KD     0       /* 电机4微分系数*/
```

### 运行时PID参数调整
项目提供了运行时调整PID参数的接口函数：

```c
/**
 * 设置指定电机的PID参数
 * @param motorIndex: 电机索引 (0-3)
 * @param kp: 比例系数
 * @param ki: 积分系数
 * @param kd: 微分系数
 */
void SetMotorPIDParameters(uint8_t motorIndex, float kp, float ki, float kd);

/**
 * 获取指定电机的PID参数
 * @param motorIndex: 电机索引 (0-3)
 * @param kp: 比例系数输出指针
 * @param ki: 积分系数输出指针
 * @param kd: 微分系数输出指针
 */
void GetMotorPIDParameters(uint8_t motorIndex, float *kp, float *ki, float *kd);
```

### 使用示例
```c
// 设置电机1的PID参数
SetMotorPIDParameters(0, 0.02, 0.003, 0.001);

// 获取电机2的PID参数
float kp, ki, kd;
GetMotorPIDParameters(1, &kp, &ki, &kd);
```

### 修改内容
1. **robot_types.h**: 为每个电机定义了独立的PID参数宏
2. **motor_control.h**: 更新了MotorInit函数声明，添加了PID参数设置/获取函数
3. **motor_control.c**: 修改了MotorInit函数实现，支持传入PID参数；添加了PID参数设置/获取函数的实现

### 注意事项
- 修改PID参数时会自动重置积分项和上一次误差，避免参数突变导致的控制不稳定
- 所有PID参数修改都通过互斥量保护，确保线程安全
- 可以通过USB通信或其他通信方式调用这些函数来实现远程PID参数调整

---

## 串口输入格式说明文档

本文档描述了用于控制基于STM32的机器人平台的串口输入格式。通信任务解析通过UART接收的命令并执行相应的操作。

### 1. 电机速度控制命令 (`$SPD`)

该命令用于设置各个电机的目标速度。

**格式:**

```
$SPD,speed0,speed1,speed2,speed3#
```

*   `$SPD`: 命令标识符。
*   `speed0`, `speed1`, `speed2`, `speed3`: 分别表示电机0、1、2和3的目标速度。这些是表示所需速度（RPM）的整数值。负值表示反向。
*   `#`: 井号字符，表示命令结束。

**示例:**

要将电机0设置为50 RPM，电机1设置为-30 RPM，电机2设置为0 RPM，电机3设置为80 RPM，应发送以下命令：

```
$SPD,50,-30,0,80#
```

**响应:**

在接收并处理`$SPD`命令后，系统将为每个电机发送确认消息：

```
ACK,M0,50
ACK,M1,-30
ACK,M2,0
ACK,M3,80
```

*   `ACK`: 确认标识符。
*   `M0`, `M1`, `M2`, `M3`: 电机标识符。
*   `50`, `-30`, `0`, `80`: 为相应电机设置的目标速度。
