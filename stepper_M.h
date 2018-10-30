
//Pins config
//#define Motor_PIN_EN GPIOA,GPIO_PIN_7
#define Motor_PIN_IN1 STEPM_1_GPIO_Port,STEPM_1_Pin
//#define Motor_PIN_IN2 GPIOB,GPIO_PIN_4
#define Motor_PIN_IN2 STEPM_2_GPIO_Port,STEPM_2_Pin
//#define Motor_PIN_IN4 GPIOB,GPIO_PIN_1

#define Motor_PIN_IN3  STEPM_3_GPIO_Port,STEPM_3_Pin
#define Motor_PIN_IN4 STEPM_4_GPIO_Port,STEPM_4_Pin

//#define MotorEN HAL_GPIO_WritePin(Motor_PIN_EN,binaryConvert1(1))


void Motorpins_init(void);
void Motor_writeIN ( uint8_t IN1, uint8_t IN2, uint8_t IN3, uint8_t IN4);
void Motor_CW(uint16_t t);
void Motor_CCW(uint16_t t);

