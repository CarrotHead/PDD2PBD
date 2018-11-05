#include "motor_driver.h"
// go into stm32l0xx_hal_conf.h and uncommment #define HAL_TIM_MODULE_ENABLED   
// systemcoreclock = 16Mhz
// Channel 1 PA0
// Channel 2 PA1
// Dir PA4
TIM_OC_InitTypeDef DCConfig;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim21;
//initalize at 50% duty cycle
/*

*/
void dc_motor_init (void)
{
  GPIO_InitTypeDef gpiodef;
  __HAL_RCC_GPIOA_CLK_ENABLE();
  gpiodef.Mode = GPIO_MODE_OUTPUT_PP;
  gpiodef.Pin = GPIO_PIN_4;
  HAL_GPIO_Init(GPIOA,&gpiodef);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
  TIM_MasterConfigTypeDef sMasterConfig;
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = SystemCoreClock/16000000 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1600-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }

  DCConfig.OCMode = TIM_OCMODE_PWM1;
  DCConfig.Pulse = 800;
  DCConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  DCConfig.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &DCConfig, TIM_CHANNEL_1) != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }
  HAL_TIM_MspPostInit(&htim2);

}
void stepper_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 0;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 31999;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim21) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim21, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim21) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 16000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim21, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim21);

}
// void set_motor_speed (uint16_t speed)
// {
// 	pulse = speed;
// 	DCConfig.Pulse = pulse;
// 	if (HAL_TIM_PWM_ConfigChannel(&htim2, &DCConfig, TIM_CHANNEL_1) != HAL_OK)
// 	{
// 		_Error_Handler(__FILE__, __LINE__);
// 	}
// }

void dc_motor_on (uint16_t pulse,uint8_t dir)
{		
	  GPIO_PinState dirstate;
  	if (dir == 1)
		dirstate = GPIO_PIN_SET;
  	else
		dirstate = GPIO_PIN_RESET;
  	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,dirstate);
		DCConfig.Pulse = pulse;
		if (HAL_TIM_PWM_ConfigChannel(&htim2, &DCConfig, TIM_CHANNEL_1) != HAL_OK)
		{
		_Error_Handler(__FILE__, __LINE__);
		}
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}
void stepper_motor_on(uint8_t dir)
{
	GPIO_PinState dirstate;
  if (dir == 1)
	dirstate = GPIO_PIN_SET;
	 else
	dirstate = GPIO_PIN_RESET;
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,dirstate);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
	HAL_TIM_PWM_Start(&htim21, TIM_CHANNEL_1);
}
void stepper_motor_off(void)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
	HAL_TIM_PWM_Stop(&htim21, TIM_CHANNEL_1);
}
void dc_motor_off(void)
{
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}
// void motor_dir(uint8_t dir)
// {
// 	GPIO_PinState dirstate;
//   if (dir == 1)
// 	dirstate = GPIO_PIN_SET;
//   else
// 	dirstate = GPIO_PIN_RESET;
//   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,dirstate);
// }
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{

  if(htim_pwm->Instance==TIM2)
  {
	__HAL_RCC_TIM2_CLK_ENABLE();
  }

}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(htim->Instance==TIM2)
  {

	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  if(htim->Instance==TIM21)
  {
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF6_TIM21;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm)
{

  if(htim_pwm->Instance==TIM2)
  {
		__HAL_RCC_TIM2_CLK_DISABLE();
  }

}
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM21)
 {
		__HAL_RCC_TIM21_CLK_ENABLE();
 }
}
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM21)
  {
  	__HAL_RCC_TIM21_CLK_DISABLE();
  }
}
