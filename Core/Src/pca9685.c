/**
  ******************************************************************************
  * @file           : pca9685.c
  * @brief          : PCA9685 PWM控制器驱动实现
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "pca9685.h"
#include <math.h>
#include "cmsis_os.h"  /* 添加RTOS头文件 */
#include "i2c.h"       /* 添加I2C头文件 */
#include "robot_types.h" /* 添加机器人类型头文件 */

/**
 * @brief 写入单字节到PCA9685寄存器
 * @param hi2c: I2C句柄
 * @param reg: 寄存器地址
 * @param data: 要写入的数据
 * @return HAL_StatusTypeDef: 操作状态
 */
HAL_StatusTypeDef PCA9685_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t data) {
    return HAL_I2C_Mem_Write(hi2c, PCA9685_I2C_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

/**
 * @brief 读取单字节从PCA9685寄存器
 * @param hi2c: I2C句柄
 * @param reg: 寄存器地址
 * @param data: 存储读取数据的指针
 * @return HAL_StatusTypeDef: 操作状态
 */
HAL_StatusTypeDef PCA9685_ReadRegister(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data) {
    return HAL_I2C_Mem_Read(hi2c, PCA9685_I2C_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 100);
}

/**
 * @brief 初始化PCA9685
 * @param hi2c: I2C句柄
 * @return HAL_StatusTypeDef: 操作状态
 */
HAL_StatusTypeDef PCA9685_Init(I2C_HandleTypeDef *hi2c) {
    HAL_StatusTypeDef status;
    
    /* 重置PCA9685 */
    status = PCA9685_WriteRegister(hi2c, PCA9685_MODE1, 0x80);
    if (status != HAL_OK) return status;
    
    HAL_Delay(10); /* 等待重置完成 */
    
    /* 设置模式：
     * 0x00 = 普通模式
     * 内部时钟
     * 寄存器自动递增
     */
    status = PCA9685_WriteRegister(hi2c, PCA9685_MODE1, 0x20);
    if (status != HAL_OK) return status;
    
    /* 设置默认PWM频率为50Hz(标准频率) */
    status = PCA9685_SetPWMFreq(hi2c, 50.0f);
    
    return status;
}

/**
 * @brief 设置PCA9685的PWM频率
 * @param hi2c: I2C句柄
 * @param freq: 频率(Hz)
 * @return HAL_StatusTypeDef: 操作状态
 */
HAL_StatusTypeDef PCA9685_SetPWMFreq(I2C_HandleTypeDef *hi2c, float freq) {
    HAL_StatusTypeDef status;
    uint8_t mode1;
    
    /* 频率范围限制为24Hz~1526Hz */
    if (freq < 24) freq = 24;
    if (freq > 1526) freq = 1526;
    
    /* 计算预分频值 */
    float prescaleval = 25000000.0; /* 25MHz内部振荡器 */
    prescaleval /= 4096.0;          /* 12位分辨率 */
    prescaleval /= freq;
    prescaleval -= 1.0;
    
    uint8_t prescale = (uint8_t)(prescaleval + 0.5); /* 四舍五入 */
    
    /* 读取当前MODE1寄存器 */
    status = PCA9685_ReadRegister(hi2c, PCA9685_MODE1, &mode1);
    if (status != HAL_OK) return status;
    
    /* 进入睡眠模式设置预分频 */
    status = PCA9685_WriteRegister(hi2c, PCA9685_MODE1, (mode1 & 0x7F) | 0x10);
    if (status != HAL_OK) return status;
    
    /* 设置预分频值 */
    status = PCA9685_WriteRegister(hi2c, PCA9685_PRESCALE, prescale);
    if (status != HAL_OK) return status;
    
    /* 恢复MODE1寄存器 */
    status = PCA9685_WriteRegister(hi2c, PCA9685_MODE1, mode1);
    if (status != HAL_OK) return status;
    
    HAL_Delay(5); /* 等待振荡器恢复 */
    
    /* 使能自动增量功能 */
    status = PCA9685_WriteRegister(hi2c, PCA9685_MODE1, mode1 | 0x20);
    
    return status;
}

/**
 * @brief 设置PCA9685的PWM信号
 * @param hi2c: I2C句柄
 * @param channel: 通道(0-15)
 * @param on: 开始计数点(0-4095)
 * @param off: 结束计数点(0-4095)
 * @return HAL_StatusTypeDef: 操作状态
 */
HAL_StatusTypeDef PCA9685_SetPWM(I2C_HandleTypeDef *hi2c, uint8_t channel, uint16_t on, uint16_t off) {
    uint8_t buffer[4];
    
    /* 检查通道范围 */
    if (channel > 15) return HAL_ERROR;
    
    /* 准备数据 */
    buffer[0] = on & 0xFF;         /* ON 低字节 */
    buffer[1] = (on >> 8) & 0x0F;  /* ON 高字节 */
    buffer[2] = off & 0xFF;        /* OFF 低字节 */
    buffer[3] = (off >> 8) & 0x0F; /* OFF 高字节 */
    
    /* 写入数据 */
    return HAL_I2C_Mem_Write(hi2c, PCA9685_I2C_ADDR << 1, 
                              PCA9685_LED0_ON_L + (channel * 4), 
                              I2C_MEMADD_SIZE_8BIT, buffer, 4, 100);
}

/**
 * @brief 设置PCA9685通道的PWM值(简化版本)
 * @param hi2c: I2C句柄
 * @param channel: 通道(0-15)
 * @param value: PWM值(0-4095)
 * @return HAL_StatusTypeDef: 操作状态
 */
HAL_StatusTypeDef PCA9685_SetPin(I2C_HandleTypeDef *hi2c, uint8_t channel, uint16_t value) {
    /* 限制PWM值范围 */
    if (value > 4095) value = 4095;
    
    /* 设置PWM: 从0开始，在value结束 */
    return PCA9685_SetPWM(hi2c, channel, 0, value);
}

/**
 * @brief 设置电机PWM值
 * @param hi2c: I2C句柄
 * @param channel: 通道(0-15)
 * @param value: PWM值(0-4095)
 */
void SetMotorPWM(I2C_HandleTypeDef *hi2c, uint8_t channel, uint16_t value) {
    PCA9685_SetPin(hi2c, channel, value);
}
