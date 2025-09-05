/* Define to prevent recursive inclusion */
#ifndef __ROBOT_TYPES_H
#define __ROBOT_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/* 包含基本标准库 */
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"


/* 电机PID控制参数 - 针对电机轴转速（减速前）- 每个电机独立的PID参数 */
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

/* PID输出范围 - 所有电机相同 */
#define MOTOR_PID_MIN    -100.0f /* PID输出下限 */
#define MOTOR_PID_MAX    100.0f  /* PID输出上限 */

// /* 电机RPM参数 - 电机轴转速（减速前）*/
// #define MOTOR_MAX_RPM    1310     /* 最大RPM值（电机轴，10*131） */
// #define MOTOR_TARGET_RPM 1048     /* 默认目标RPM值（电机轴，8*131） */

/* 用户输入速度限制 - 电机轴转速（减速前）*/
#define MOTOR_USER_MIN_RPM   100   /* 用户可设置的最小RPM值 */
#define MOTOR_USER_MAX_RPM   6000  /* 用户可设置的最大RPM值 */

/* === 速度单位与换算建议（新增） === */
#define SPEED_UNIT_ENC_RPM   1        /* MCU 内部统一使用 RPM */
#define SPEED_UNIT_STR       "rpm"

/* 编码器计数配置 - 电机轴转速（减速前）*/
#ifndef ENCODER_TICKS_PER_REV
/* 这是"定时器实际计数/电机轴机械一转"的 ticks 数：
 * 计算：单通道每转脉冲数 * 四倍频倍数
 * = 11 * 4 = 44 ticks/转（电机轴）
 * 注意：这里测量的是电机轴转速，不包含减速比
 *
 * 在20ms采样周期下：
 * - 60 RPM时：20ms内应产生 44 * 60 * 0.02 ≈ 52.8 个脉冲
 * - 6000 RPM时：20ms内应产生 44 * 6000 * 0.02 ≈ 5280 个脉冲
 * - 最大测量范围约为 ±3276 RPM（考虑16位定时器溢出）
 */
#define ENCODER_TICKS_PER_REV (11 * 4)  /* 44 ticks per motor shaft revolution */
#endif

static inline int32_t enc_delta_to_cps(int32_t delta, uint32_t dt_ms){
  /* delta: 采样周期内的计数增量（带符号） */
  return (int32_t)((1000LL * delta) / (int32_t)dt_ms);
}
static inline int32_t enc_delta_to_rpm(int32_t delta, uint32_t dt_ms){
  /* delta: 采样周期内的计数增量（带符号） */
  /* 使用64位整数进行计算以避免溢出 */
  /* 返回电机轴转速(RPM，减速前) */
  /* 公式推导：rpm = (delta / dt_ms) * (1000 / ENCODER_TICKS_PER_REV) * 60 */
  /*         = delta * 60 * 1000 / (ENCODER_TICKS_PER_REV * dt_ms) */
  if (dt_ms == 0) return 0;  /* 避免除零错误 */

  int64_t rpm = ((int64_t)delta * 60LL * 1000LL) / ((int64_t)ENCODER_TICKS_PER_REV * dt_ms);
  return (int32_t)rpm;
}
static inline float cps_to_rpm(int32_t cps){
  return (float)cps * 60.0f / (float)ENCODER_TICKS_PER_REV;
}
static inline int32_t rpm_to_cps(float rpm){
  return (int32_t)(rpm * (float)ENCODER_TICKS_PER_REV / 60.0f);
}


/* PID控制器结构体 */
typedef struct {
    float Kp;                  /* 比例系数 */
    float Ki;                  /* 积分系数 */
    float Kd;                  /* 微分系数 */
    float targetValue;         /* 目标值 */
    float currentValue;        /* 当前值 */
    float error;               /* 当前误差 */
    float lastError;           /* 上一次误差 */
    float errorSum;            /* 误差累积（积分项） */
    float output;              /* PID输出 */
    float outputMax;           /* 输出上限 */
    float outputMin;           /* 输出下限 */
} PIDController_t;

/* 电机状态枚举 */
typedef enum {
    MOTOR_STOP = 0,            /* 电机停止 */
    MOTOR_FORWARD = 1,         /* 电机正向运行 */
    MOTOR_BACKWARD = 2,        /* 电机反向运行 */
    MOTOR_ERROR = 3            /* 电机错误状态 */
} MotorState_t;

/* 电机方向枚举 */
typedef enum {
    MOTOR_DIR_FORWARD = 0,
    MOTOR_DIR_BACKWARD = 1
} MotorDirection_t;

/* 电机结构体 */
typedef struct {
    uint8_t id;                /* 电机ID (0-3) */
    MotorState_t state;        /* 电机状态 */
    MotorDirection_t direction; /* 电机方向 */
    int16_t targetSpeed;       /* 目标速度 (RPM, 电机轴转速-减速前) */
    int16_t currentSpeed;      /* 当前速度 (RPM, 电机轴转速-减速前) */
    uint8_t pwmPercent;        /* PWM百分比 (0-100) */
    int32_t encoderCount;      /* 编码器计数 */
    int32_t lastEncoderCount;  /* 上次编码器计数 */
    uint32_t lastUpdateTime;   /* 上次更新时间 (ms) */
    TIM_HandleTypeDef* encoderTimer; /* 编码器定时器 */
    uint8_t pwmChannel;        /* PCA9685 PWM通道 */
    
    /* 修改方向控制为两个引脚 */
    GPIO_TypeDef* dir0Port;    /* 方向0控制GPIO端口 (IN1) */
    uint16_t dir0Pin;          /* 方向0控制GPIO引脚 (IN1) */
    GPIO_TypeDef* dir1Port;    /* 方向1控制GPIO端口 (IN2) */
    uint16_t dir1Pin;          /* 方向1控制GPIO引脚 (IN2) */
    
    PIDController_t pidController; /* 速度PID控制器 */
    uint8_t errorCounter;      /* 错误计数器 */
    struct {
        uint8_t encoderError : 1;  /* 编码器错误标志 */
        uint8_t overCurrent : 1;   /* 过流标志 */
        uint8_t stalled : 1;       /* 堵转标志 */
        uint8_t reserved : 5;      /* 保留位 */
    } flags;                   /* 使用位域优化内存 */
} Motor_t;

/* 系统状态结构体 */
typedef struct {
    Motor_t motors[4];         /* 四个电机 */
    struct {
        uint8_t initialized : 1;   /* 系统初始化标志 */
        uint8_t motorError : 1;    /* 电机错误标志 */
        uint8_t communicationError : 1; /* 通信错误标志 */
        uint8_t lowBattery : 1;    /* 低电量标志 */
        uint8_t emergencyStop : 1; /* 紧急停止标志 */
        uint8_t pca9685Error : 1;  /* PCA9685错误标志 */
        uint8_t reserved : 2;      /* 保留位 */
    } systemFlags;             /* 系统状态标志 */
} SystemState_t;

#ifdef __cplusplus
}
#endif

#endif /* __ROBOT_TYPES_H */
