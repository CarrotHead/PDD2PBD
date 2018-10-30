#ifndef __motor_driver_H

#define __motor_driver_H

#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal_tim.h"
#include "stm32l0xx_hal_tim_ex.h"

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm);
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);

void motor_init (void);
void set_motor_speed (uint16_t speed);
void motor_on (uint8_t channel);
void motor_off(uint8_t channel);
void motor_dir(uint8_t dir);
#endif
