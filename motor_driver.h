#ifndef __motor_driver_H

#define __motor_driver_H

#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal_tim.h"
#include "stm32l0xx_hal_tim_ex.h"

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm);
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base);
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base);

void dc_motor_init (void);
//void set_motor_speed (uint16_t speed);
void dc_motor_on (uint16_t pulse,uint8_t dir);
void dc_motor_off(void);
//void motor_dir(uint8_t dir);
void stepper_motor_on(uint8_t dir);
void stepper_motor_off(void);
void stepper_init(void);

#endif
