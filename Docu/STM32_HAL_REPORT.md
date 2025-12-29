# STM32 HAL / FreeRTOS 项目结构与任务通信报告

> 目标：为未来使用 Rust 重写提供结构化参考。内容聚焦 STM32 HAL 侧与 FreeRTOS 任务结构与通信方式。

## 1) 项目用途与整体功能

- **用途**：基于 STM32F103ZET6 + FreeRTOS 的麦克纳姆轮四轮速度闭环控制系统。
- **硬件组成**：
  - MCU：STM32F103ZET6
  - PWM：PCA9685
  - 电机驱动：L298N
  - 电机：JGY370 有刷蜗轮蜗杆电机（示例）
  - 编码器：定时器编码器模式
- **控制目标**：对 4 个电机执行速度闭环（PID），对外提供串口控制与 USB 遥测输出。

## 2) 目录结构概览（与 HAL/RTOS 相关）

- `Core/Inc` / `Core/Src`
  - 应用与业务逻辑（通信、电机控制、监控、USB 等）
  - HAL 外设初始化（USART/I2C/TIM/USB/CRC/RTC/GPIO）
  - FreeRTOS 任务与同步对象
- `Drivers`
  - STM32 HAL/LL 库与 CMSIS
- `Middlewares`
  - FreeRTOS（CMSIS-RTOS v2 适配层）
- `USB_DEVICE`
  - USB CDC 设备栈与接口
- 其他（`Docu/`, `python/`）
  - 文档/上位机辅助工具等

## 3) 核心模块（Core/Src）

- `main.c`
  - HAL 初始化与外设初始化
  - 启动 FreeRTOS 调度器
- `freertos.c`
  - 任务创建、互斥量/事件组/信号量/队列创建
- `motor_control.c`
  - 电机与 PID 控制核心逻辑
  - PCA9685 I2C 输出
- `communication.c`
  - USART1 命令解析（`$SPD`、`$PID`）
  - UART 中断回调触发任务事件
- `monitor.c`
  - 定期输出电机状态与系统状态（UART1）
- `usb_handler.c`
  - 通过 USB CDC 发送二进制遥测帧
- `usb_comm.c`
  - CRC32、COBS、TLV 等基础通信工具
  - 当前 USB 遥测发送使用 CRC32，但未使用 COBS/TLV 组包

## 4) 外设与对外通信方式

### 4.1 外设初始化（main.c）
- GPIO
- I2C1 / I2C2
- USART1
- TIM2 / TIM3 / TIM4 / TIM5（编码器）
- CRC
- RTC
- USB Device（CDC）

### 4.2 对外通信方式

1) **UART1（USART1）命令通道**
   - 中断方式接收（`HAL_UART_Receive_IT`）
   - 命令格式：
     - 速度：`$SPD,speed0,speed1,speed2,speed3#`
     - PID：`$PID,motorId,kp,ki,kd#`
   - 接收完成后通过 `osEventFlagsSet` 通知通信任务处理

2) **USB CDC 遥测通道**
   - `UsbTxRxTask` 周期发送二进制帧
   - 帧结构（`usb_handler.c`）：
     - Header: sync(0xAA55) + version + length + timestamp
     - 4 个电机记录（目标/当前 RPM + PWM）
     - CRC32 + trail(0x55AA)

3) **I2C（PCA9685）**
   - 由 `motor_control.c` 控制 PWM 输出
   - 用 `i2cMutex` 保护，避免并发访问

4) **GPIO 方向控制**
   - 每个电机两个方向引脚（IN1/IN2）

5) **编码器计数（TIM2/3/4/5）**
   - `TIM_Encoder` 模式
   - 根据计数增量计算 RPM

## 5) FreeRTOS 任务结构

### 5.1 任务列表

| 任务名 | 入口函数 | 栈大小 | 优先级 | 主要职责 |
|---|---|---:|---|---|
| `defaultTask` | `StartDefaultTask` | 128*4 | Low | USB 设备初始化（`MX_USB_DEVICE_Init`），空循环 |
| `MotorControlTas` | `StartMotorControlTask` | 384*4 | Low | PID 控制循环、编码器采样、PWM 输出、事件通知 |
| `CommunicationTa` | `StartCommunicationTask` | 512*4 | Low | UART 命令解析、更新目标速度/PID |
| `MonitorTask` | `StartMonitorTask` | 384*4 | Low | 定期输出电机/系统状态 |
| `UsbTxRxTask` | `StartUsbTask` | 384*4 | Low | 发送 USB 遥测帧 |

> 任务创建位置：`Core/Src/freertos.c`

### 5.2 任务周期/调度特性

- **MotorControlTask**
  - 通过 `vTaskDelayUntil` 进行严格周期调度
  - 周期 `MOTOR_CONTROL_PERIOD = 20ms`
  - 每周期：速度计算 -> 滤波 -> PID -> PWM 输出

- **CommunicationTask**
  - 由 UART 中断通过事件标志触发
  - 超时等待 10ms，空闲时 1ms 休眠

- **MonitorTask**
  - 每 1 秒报告电机状态与系统标志

- **UsbTxRxTask**
  - 正常 20ms 周期发送（失败时 5ms 重试）

## 6) 任务间通信与共享资源

### 6.1 全局共享状态
- `SystemState_t systemState`：
  - 4 个电机状态（目标/当前 RPM、PWM、PID、标志）
  - 系统状态标志（初始化、错误等）

### 6.2 互斥量（Mutex）
- `motorDataMutex`：保护 `systemState` 中的电机数据
- `i2cMutex`：保护 PCA9685 I2C 写入
- `servoDataMutex`：已创建，但当前代码未使用

### 6.3 事件组（Event Flags）
- `systemEventGroup`：
  - `EVENT_COMMUNICATION`：UART 收到完整命令后触发
  - `EVENT_MOTOR_UPDATE`：MotorControlTask 更新完电机数据后触发
  - `EVENT_ERROR`：定义但当前未使用

### 6.4 队列 / 信号量
- `commandQueue`：已创建但当前未使用
- `statusSemaphore`：已创建但当前未使用

> **Rust 重写提示**：可考虑将 `systemState` 封装为共享状态（如 `Arc<Mutex<...>>`），事件组映射为 `Notify`/`Semaphore`/`Channel` 等机制；而 UART ISR -> 任务通知可以替换为中断驱动消息队列。

## 7) 数据流概览（面向 Rust 重写）

1. **UART 输入** → `HAL_UART_RxCpltCallback` → `EVENT_COMMUNICATION` → `CommunicationTask` 解析 → 更新 `systemState`
2. **MotorControlTask** 周期运行 → 读取编码器 → PID → `PCA9685` I2C PWM 输出 → `EVENT_MOTOR_UPDATE`
3. **MonitorTask** 每秒读取 `systemState` → UART1 输出状态
4. **UsbTxRxTask** 周期读取 `systemState` → USB CDC 发送遥测

## 8) 关键重写关注点（建议）

- **任务拆分保持一致**：通信、控制、监控、USB 发送可对应 Rust 的异步任务或 RTOS 线程。
- **共享数据保护**：现有系统依赖 `motorDataMutex`，Rust 侧需要明确锁策略与优先级反转处理。
- **事件驱动机制**：UART ISR 触发事件/队列，可在 Rust HAL 中映射为 WFI + ring buffer + signal/notify。
- **驱动替换点**：
  - USART1 → Rust HAL UART
  - I2C1 → PCA9685 驱动
  - TIMx Encoder → Rust HAL 定时器编码器模式
  - USB CDC → USB device stack / embassy-usb

---

**来源文件（重点）**：
- `Core/Src/freertos.c`
- `Core/Src/motor_control.c`
- `Core/Src/communication.c`
- `Core/Src/monitor.c`
- `Core/Src/usb_handler.c`
- `Core/Src/usb_comm.c`
- `Core/Inc/main.h`, `Core/Inc/robot_types.h`
