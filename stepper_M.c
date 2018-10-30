
#include "stm32l0xx_hal.h"
#include "stepper_M.h"
#include "main.h"

//void Motorpins_init(void)
//{
//	GPIO_InitTypeDef MotorGPIO;
//	
//  __HAL_RCC_GPIOA_CLK_ENABLE();    
//	MotorGPIO.Pin = GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_2;
//  MotorGPIO.Mode = GPIO_MODE_OUTPUT_PP;
//  MotorGPIO.Pull = GPIO_PULLUP; 
//	MotorGPIO.Speed= GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOA,&MotorGPIO);
//	
////	  __HAL_RCC_GPIOB_CLK_ENABLE();    
////	MotorGPIO.Pin = GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_1;
////  MotorGPIO.Mode = GPIO_MODE_OUTPUT_PP;
////  MotorGPIO.Pull = GPIO_PULLUP; 
////	MotorGPIO.Speed= GPIO_SPEED_FREQ_LOW;
////  HAL_GPIO_Init(GPIOB,&MotorGPIO);
//	
//		  __HAL_RCC_GPIOB_CLK_ENABLE();    
//	MotorGPIO.Pin = GPIO_PIN_3|GPIO_PIN_0;
//  MotorGPIO.Mode = GPIO_MODE_OUTPUT_PP;
//  MotorGPIO.Pull = GPIO_PULLUP; 
//	MotorGPIO.Speed= GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOB,&MotorGPIO);
//	
//}

GPIO_PinState binaryConvert1(uint8_t x)
{
  if(x == 1)
    return GPIO_PIN_SET;
  else
    return GPIO_PIN_RESET;
}

void Motor_writeIN(uint8_t IN1, uint8_t IN2, uint8_t IN3, uint8_t IN4)
{

  HAL_GPIO_WritePin(Motor_PIN_IN1,binaryConvert1(IN1));
  HAL_GPIO_WritePin(Motor_PIN_IN2,binaryConvert1(IN2));
  HAL_GPIO_WritePin(Motor_PIN_IN3,binaryConvert1(IN3));
  HAL_GPIO_WritePin(Motor_PIN_IN4,binaryConvert1(IN4));
	
}

void Motor_CW(uint16_t t)
{
//	MotorEN;
	uint16_t i;
	for(i=0; i<t*13; i++)
	{
		Motor_writeIN(1,0,0,0);
		HAL_Delay(1);
		//Motor_writeIN(1,0,1,0);
    //HAL_Delay(5);
		Motor_writeIN(0,0,1,0);
		HAL_Delay(1);
		//Motor_writeIN(0,1,1,0);
		//HAL_Delay(5);
		Motor_writeIN(0,1,0,0);
		HAL_Delay(1);
		//Motor_writeIN(0,1,0,1);
		//HAL_Delay(5);
		Motor_writeIN(0,0,0,1);
		HAL_Delay(1);
	  //Motor_writeIN(1,0,0,1);
		//HAL_Delay(5);
		
	}	
	Motor_writeIN(0,0,0,0);
}

void Motor_CCW(uint16_t t)
{
//	MotorEN;
	uint16_t i;
	for(i=0; i<t*13; i++)
	{
		Motor_writeIN(1,0,0,1);
		HAL_Delay(1);
		//Motor_writeIN(0,0,0,1);
		//HAL_Delay(10);
		Motor_writeIN(0,1,0,1);
		HAL_Delay(1);
		//Motor_writeIN(0,1,0,0);
		//HAL_Delay(10);
		Motor_writeIN(0,1,1,0);
		HAL_Delay(1);
		//Motor_writeIN(0,0,1,0);
		//HAL_Delay(10);
		Motor_writeIN(1,0,1,0);
		HAL_Delay(1);
		//Motor_writeIN(1,0,0,0);
		//HAL_Delay(10);
		
	}	
	Motor_writeIN(0,0,0,0);
}



