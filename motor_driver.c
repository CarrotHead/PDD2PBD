#include "motor_driver.h"
// go into stm32l0xx_hal_conf.h and uncommment #define HAL_TIM_MODULE_ENABLED   
// systemcoreclock = 16Mhz
// Channel 1 PA0
// Channel 2 PA1
// Dir PA4
TIM_OC_InitTypeDef sConfigOC;
TIM_HandleTypeDef htim2;
//initalize at 50% duty cycle
/*

*/
void motor_init (void)
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

 	sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 800;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

//  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }

  HAL_TIM_MspPostInit(&htim2);

}
// implement set 1-10 speed,this is in case of +5v V+.
// for now set raw duty cycle speed/1600 1600==100% duty
void set_motor_speed (uint16_t speed)
{
	uint16_t pulse;
/*	if(speed==1)
		pulse = 700;
	else if(speed==2)
		pulse = 800;
	else if(speed==3)
		pulse = 900;
	else if(speed==4)
		pulse = 1000;
	else if(speed==5)
		pulse = 1100;
	else if(speed==6)
		pulse = 1200;
	else if(speed==7)
		pulse = 1300;
	else if(speed==8)
		pulse = 1400;
	else if(speed==9)
		pulse = 1500;
	else if(speed==10)
		pulse = 1600; */
    pulse = speed;
	sConfigOC.Pulse = pulse;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
 	{
   		_Error_Handler(__FILE__, __LINE__);
  	}
//  	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
// 	{
//   		_Error_Handler(__FILE__, __LINE__);
//  	}
}

void motor_on (uint8_t channel)
{	
	if (channel == 1)
	{
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	}
//	else if (channel == 2)
//	{
//		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
//	}//
}

void motor_off(uint8_t channel)
{
	if (channel == 1)
	{
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	}
//	else if (channel == 2)
//	{
//		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
//	}
}
void motor_dir(uint8_t dir)
{
  GPIO_PinState dirstate;
  if (dir == 1)
    dirstate = GPIO_PIN_SET;
  else
    dirstate = GPIO_PIN_RESET;
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,dirstate);
}
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
    /**TIM2 GPIO Configuration    
    PA0     ------> TIM2_CH1
    PA1     ------> TIM2_CH2 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    //GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }

}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm)
{

  if(htim_pwm->Instance==TIM2)
  {
    __HAL_RCC_TIM2_CLK_DISABLE();
  }

}

