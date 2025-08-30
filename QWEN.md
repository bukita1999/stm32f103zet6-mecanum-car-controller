# STM32 Small Surface Robot Project Context

## Project Overview

This is an STM32-based embedded C project for a small robot platform. It uses an STM32F103ZET6 microcontroller and is structured around the STM32CubeMX/IDE ecosystem, utilizing HAL, FreeRTOS, and CubeMX-generated initialization code.

The primary functions of the robot are:
- Control of 4 motors with encoders for speed feedback.
- Control of up to 8 servos.
- Communication via UART (115200 baud) to receive commands.
- Implementation of PID control for motor speed regulation.

## Hardware Configuration

- **Microcontroller**: STM32F103ZET6
- **Motor Control**:
  - 4 motors controlled via PCA9685 PWM driver (I2C1).
  - Each motor has two direction control GPIO pins (e.g., MOTOR1_DIR0, MOTOR1_DIR1) for H-bridge control.
  - Encoder feedback for each motor using STM32 timer peripherals (TIM2, TIM3, TIM4, TIM5) in encoder mode.
- **Servo Control**:
  - Up to 8 servos controlled via PCA9685 PWM driver (I2C1).
- **Communication**:
  - UART1 (PA9/TX, PA10/RX) at 115200 baud for receiving commands.
- **Real-Time Clock**: On-chip RTC peripheral.

## Software Architecture

- **RTOS**: FreeRTOS is used for task management.
  - Key tasks include:
    - `MotorControlTas`: Manages motor speed control using PID.
    - `ServoControlTas`: Manages servo position updates.
    - `CommunicationTa`: Handles UART command reception and processing.
    - `MonitorTask`: Periodically reports system status via UART.
- **Concurrency**: Mutexes (`motorDataMutex`, `servoDataMutex`, `i2cMutex`) protect shared data. An event group (`systemEventGroup`) signals between tasks. A queue (`commandQueue`) passes servo commands.
- **Main Control Loop**: Motor control runs at a 10ms period (`MOTOR_CONTROL_PERIOD`).
- **PID Control**: Implemented in `freertos.c` for motor speed regulation based on encoder feedback.

## Communication Protocol

The robot communicates via UART with a host computer using a specific command format:

1.  **Motor Speed Control (`$SPD`)**:
    *   **Format**: `$SPD,speed0,speed1,speed2,speed3#`
    *   Sets target speeds (RPM) for motors 0-3. Negative values indicate reverse direction.
    *   **Response**: `ACK,M{id},{speed}` for each motor.

2.  **PID Parameter Control (`$PID`)**:
    *   **Format**: `$PID,motorId,Kp,Ki,Kd#`
    *   Sets PID parameters for a specific motor (0-3).
    *   **Response**: `PID,M{id},{Kp},{Ki},{Kd}`

3.  **Servo Angle Control (`$SRV`)**:
    *   **Format**: `$SRV,servoId,angle#`
    *   Sets the target angle (0-180 degrees) for a servo (0-7).
    *   **Response**: `SRV,S{id},{angle}`

Commands are case-sensitive, comma-separated, and must end with `#`.

## Key Source Files

- `Core/Src/freertos.c`: Contains the main FreeRTOS task definitions, PID control logic, motor/servo initialization, and communication handling.
- `Core/Src/pca9685.c`: Driver for the PCA9685 PWM controller (motors and servos).
- `Core/Inc/robot_types.h`: Defines core data structures like `Motor_t`, `Servo_t`, `PIDController_t`, and `SystemState_t`.
- `Core/Inc/main.h`: Includes system-wide definitions and function declarations for motor/servo control.
- `Core/Src/main.c`: Main entry point, initializes HAL, system clock, peripherals, and starts the FreeRTOS scheduler.

## Building and Flashing

This project is configured for STM32CubeIDE.
1.  Open the project in STM32CubeIDE.
2.  Build the project using the IDE's build system (typically `Project -> Build Project`).
3.  Connect the STM32 board via a debugger (e.g., ST-Link).
4.  Flash the generated `.elf` or `.hex` file to the microcontroller using the IDE's debug/run functionality.

## Development Conventions

- Code is generated and managed using STM32CubeMX/IDE.
- HAL library is used for peripheral abstraction.
- FreeRTOS manages concurrency with clear separation of concerns via tasks and synchronization primitives (mutexes, queues, event groups).
- Motor and servo operations are thread-safe, protected by mutexes.
- I2C communication (to PCA9685) is protected by a dedicated mutex (`i2cMutex`).
- Data structures for system state (`SystemState_t`) centralize motor and servo information.