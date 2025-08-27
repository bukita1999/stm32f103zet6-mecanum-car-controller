# STM32 工程架构说明

## 1. 系统总体结构

该工程基于 **STM32F1 + FreeRTOS**，主要目标是实现 **四轮电机的速度控制、PID 调节、USB/UART 通信**。
架构分为以下几层：

1. **硬件抽象层 (HAL/CubeMX 生成的驱动)**
   提供 GPIO、I2C、TIM、USART、RTC 等外设的初始化和基础访问。
2. **设备驱动层**

   * `pca9685.c`：PWM 驱动芯片 PCA9685 的 I2C 控制。
   * `usb_comm.c`：USB CDC + 自定义协议 (COBS + CRC32 + TLV) 的通信实现。
3. **任务控制层 (FreeRTOS)**
   在 `freertos.c` 中建立了多个任务，分别负责 **电机控制、通信处理、监控、USB 接收** 等。
4. **应用层逻辑**

   * PID 控制与速度计算。
   * 命令解析 (串口命令 `$SPD`/`$PID`，USB TLV 包)。
   * Telemetry 数据打包发送。

---

## 2. 文件模块说明

### 系统与外设初始化

* **`main.c`**
  入口函数，初始化所有外设 (GPIO、I2C、TIM、USART、CRC 等)，然后启动 FreeRTOS 调度器。

* **`system_stm32f1xx.c` / `stm32f1xx_hal_msp.c` / `stm32f1xx_hal_timebase_tim.c`**
  时钟配置、全局 MSP 初始化、系统滴答定时器实现。

* **`syscalls.c` / `sysmem.c`**
  Newlib 支持 (malloc/printf 等)。

### 外设驱动

* **`gpio.c`**：配置 LED 与电机方向控制引脚。
* **`i2c.c`**：初始化 I2C1、I2C2。
* **`tim.c`**：配置 TIM2–TIM5 为 **编码器接口模式**，用于测速。
* **`usart.c`**：配置 USART1 (波特率 115200)，用于串口通信。
* **`rtc.c`**：RTC 配置。
* **`crc.c`**：CRC 硬件加速配置。

### 功能驱动

* **`pca9685.c`**
  提供对 PCA9685 的封装：初始化、设置 PWM 频率、设置通道占空比、给电机写 PWM 值。

* **`usb_comm.c`**

  * **COBS 编解码**：无 0 字节传输。
  * **CRC32 校验**：保证帧完整性。
  * **TLV 协议**：定义了 `Telemetry(0x10)`、`SetSpeeds(0x01)`、`SetPID(0x02)` 等数据类型。
  * **USB\_HandleFrame**：根据 TLV 解析并执行操作。

### 系统任务 (FreeRTOS，核心文件 `freertos.c`)

1. **DefaultTask**

   * 初始化 USB 设备。
   * 维持空循环。

2. **MotorControlTask**

   * 固定周期调度（例 10ms）。
   * 调用 `CalculateMotorSpeed()` 计算转速。
   * 低通滤波速度，输入 PID 算法。
   * 调整 PWM 占空比驱动电机。

3. **CommunicationTask (UART)**

   * 解析串口命令：

     * `$SPD,x,y,z,w#` 设置电机速度。
     * `$PID,id,kp,ki,kd#` 设置 PID 参数。
   * 发送 ACK/调试信息。

4. **MonitorTask**

   * 周期性输出电机状态到 UART。
   * 每 100ms 打包并通过 USB 发送 Telemetry。

5. **UsbRxTask**

   * 处理 USB 数据接收流 (StreamBuffer)。
   * 解码 COBS，校验 CRC32，调用 `USB_HandleFrame()`。

---

## 3. 数据流与交互

* **电机控制闭环**

  * 编码器 → TIM(计数) → `CalculateMotorSpeed()` → PID 调节 → PCA9685 PWM 输出 → 电机。

* **通信链路**

  * 串口 UART：命令行风格 (`$SPD`, `$PID`)。
  * USB：二进制 TLV 协议，支持高速 Telemetry 与参数设置。

* **数据同步**

  * 使用 FreeRTOS 的 **Mutex (motorDataMutex, i2cMutex)** 来保护共享资源。
  * 使用 **EventFlags** 通知任务（电机数据更新/通信事件）。

---

## 4. 协议说明

### UART 命令格式

* **设置速度**：

  ```
  $SPD,100,-100,80,80#
  ```

  → 分别设置 4 个电机的目标速度。
* **设置 PID**：

  ```
  $PID,0,1.2,0.5,0.05#
  ```

  → 设置电机 0 的 PID 参数。

### USB TLV 格式

* **Frame = \[COBS( Payload + CRC32 ) + 0x00]**
* **Payload = 多个 TLV**

  * `0x01` (SetSpeeds) : int16\[4]
  * `0x02` (SetPID) : {u8 id; float kp,ki,kd}
  * `0x10` (Telemetry) : {每个电机的目标速度、实际速度、PWM、误差}

---

✅ 总结：
这个工程已经形成了 **模块化清晰、任务分工明确的架构**。

* `freertos.c` 是核心：任务分配 + 调度逻辑。
* `pca9685.c` + `usb_comm.c` 是外部设备驱动和通信协议的关键。
* 通过 UART 命令 & USB TLV，你可以在上位机控制电机、调参并获取 Telemetry 数据。


