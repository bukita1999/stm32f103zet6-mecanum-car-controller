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


/* 电机PID控制参数 */
#define MOTOR_PID_KP     0.029    /* 初始比例系数 */
#define MOTOR_PID_KI     0.00146    /* 初始积分系数 */
#define MOTOR_PID_KD     0.000146   /* 初始微分系数 */
#define MOTOR_PID_MIN    -100.0f /* PID输出下限 */
#define MOTOR_PID_MAX    100.0f  /* PID输出上限 */

/* 电机RPM参数 */
#define MOTOR_MAX_RPM    10      /* 最大RPM值 */
#define MOTOR_TARGET_RPM 8       /* 默认目标RPM值 */

/* === 速度单位与换算建议（新增） === */
#define SPEED_UNIT_ENC_CPS   1        /* MCU 内部统一使用 CPS */
#define SPEED_UNIT_STR       "enc_cps"

/* 如果需要把 CPS 打印为 RPM，仅用于调试（不用于控制），请设定每转 ticks 数 */
#ifndef ENCODER_TICKS_PER_REV
/* 这是“定时器实际计数/机械一转”的 ticks 数：
 * 强烈建议：靠实测（手动转一圈读取计数差）确认，而不是拍脑袋。
 */
#define ENCODER_TICKS_PER_REV 4096
#endif

static inline int32_t enc_delta_to_cps(int32_t delta, uint32_t dt_ms){
  /* delta: 采样周期内的计数增量（带符号） */
  return (int32_t)((1000LL * delta) / (int32_t)dt_ms);
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
    int16_t targetSpeed;       /* 目标速度 (RPM) */
    int16_t currentSpeed;      /* 当前速度 (RPM) */
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
