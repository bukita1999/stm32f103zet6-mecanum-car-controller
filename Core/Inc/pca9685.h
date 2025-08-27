/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PCA9685_H
#define __PCA9685_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "robot_types.h"

/* PCA9685定义 */
#define PCA9685_I2C_ADDR        0x40    /* PCA9685默认I2C地址 */
#define PCA9685_MODE1           0x00    /* 模式寄存器1 */
#define PCA9685_MODE2           0x01    /* 模式寄存器2 */
#define PCA9685_SUBADR1         0x02    /* I2C子地址1 */
#define PCA9685_SUBADR2         0x03    /* I2C子地址2 */
#define PCA9685_SUBADR3         0x04    /* I2C子地址3 */
#define PCA9685_PRESCALE        0xFE    /* 预分频器 */
#define PCA9685_LED0_ON_L       0x06    /* LED0 ON 低字节 */
#define PCA9685_LED0_ON_H       0x07    /* LED0 ON 高字节 */
#define PCA9685_LED0_OFF_L      0x08    /* LED0 OFF 低字节 */
#define PCA9685_LED0_OFF_H      0x09    /* LED0 OFF 高字节 */
#define PCA9685_ALL_LED_ON_L    0xFA    /* 所有LED ON 低字节 */
#define PCA9685_ALL_LED_ON_H    0xFB    /* 所有LED ON 高字节 */
#define PCA9685_ALL_LED_OFF_L   0xFC    /* 所有LED OFF 低字节 */
#define PCA9685_ALL_LED_OFF_H   0xFD    /* 所有LED OFF 高字节 */




/* 函数声明 */
HAL_StatusTypeDef PCA9685_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t data);
HAL_StatusTypeDef PCA9685_ReadRegister(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef PCA9685_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef PCA9685_SetPWMFreq(I2C_HandleTypeDef *hi2c, float freq);
HAL_StatusTypeDef PCA9685_SetPWM(I2C_HandleTypeDef *hi2c, uint8_t channel, uint16_t on, uint16_t off);
HAL_StatusTypeDef PCA9685_SetPin(I2C_HandleTypeDef *hi2c, uint8_t channel, uint16_t value);

/* 电机PWM控制函数 */
void SetMotorPWM(I2C_HandleTypeDef *hi2c, uint8_t channel, uint16_t value);

#ifdef __cplusplus
}
#endif

#endif /* __PCA9685_H */