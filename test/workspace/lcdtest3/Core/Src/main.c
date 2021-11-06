
#include "main.h"
#include "uit.h"
#include "ssd1306.h"
#include "stdio.h"
#include "max30100.h"
#include <string.h>
#include <string.h>

#define  MPU_ADDR 0x68
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B

#define INT_PIN_CFG	0x37
#define INT_ENABLE 	0x38
#define INT_STATUS  0x3A
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

#define LATCH_INT_EN	(1<<5)
#define INT_RD_CLEAR  	(1<<4)

#define DATA_RDY_EN   	(1<<0)


I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;

UART_HandleTypeDef huart2;

uint16_t _max30100_ir_sample[16];
uint16_t _max30100_red_sample[16];
float heart = 0;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);

struct MPU_DATA{
	float Accel_x;
	float Accel_y;
	float Accel_z;
	float Gyro_x;
	float Gyro_y;
	float Gyro_z;
};
struct MPU_DATA Mpu_data;
int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

int _write(int file, char *ptr, int len)
{
  /* Implement your write code here, this is used by puts and printf for example */
  int i=0;
  for(i=0 ; i<len ; i++)
    ITM_SendChar((*ptr++));
  return len;
}

void Update_Accel_Gyro_to_SSD1306(void);

void MPU_Init(void){
	uint8_t data_sent=0;
	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR<<1, PWR_MGMT_1_REG, I2C_MEMADD_SIZE_8BIT, &data_sent, 1, 1000);

	data_sent = 0x07;
	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR<<1, SMPLRT_DIV_REG, I2C_MEMADD_SIZE_8BIT, &data_sent, 1, 1000);

	data_sent = 3<<3;
	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR<<1, ACCEL_CONFIG_REG, I2C_MEMADD_SIZE_8BIT, &data_sent, 1, 1000);

	data_sent = 3<<3;
	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR<<1, GYRO_CONFIG_REG, I2C_MEMADD_SIZE_8BIT, &data_sent, 1, 1000);

//	data_sent = LATCH_INT_EN;
//	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR << 1, INT_PIN_CFG,
//	I2C_MEMADD_SIZE_8BIT, &data_sent, 1, 100);

	data_sent = DATA_RDY_EN;
//	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR << 1, INT_ENABLE,
//	I2C_MEMADD_SIZE_8BIT, &data_sent, 1, 100);
}

void MPU_Read_Accel(void){
	uint8_t Rec[6];
	HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR << 1, INT_STATUS, I2C_MEMADD_SIZE_8BIT, Rec, 1, 50);
	HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR<<1, ACCEL_XOUT_H_REG, I2C_MEMADD_SIZE_8BIT, Rec, 6, 100);

	Accel_X_RAW = (int16_t)((int16_t)Rec[0] << 8 | Rec[1]);
	Accel_Y_RAW = (int16_t)((int16_t)Rec[2] << 8 | Rec[3]);
	Accel_Z_RAW = (int16_t)((int16_t)Rec[4] << 8 | Rec[5]);

	Mpu_data.Accel_x = Accel_X_RAW/2048.0;
	Mpu_data.Accel_y = Accel_Y_RAW/2048.0;
	Mpu_data.Accel_z = Accel_Z_RAW/2048.0;
}
void MPU_Read_Gyro(void){
	uint8_t Rec[6];
	HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR<<1, GYRO_XOUT_H_REG, I2C_MEMADD_SIZE_8BIT, Rec, 6, 1000);

	Gyro_X_RAW = (int16_t)((int16_t)Rec[0] << 8 | Rec[1]);
	Gyro_Y_RAW = (int16_t)((int16_t)Rec[2] << 8 | Rec[3]);
	Gyro_Z_RAW = (int16_t)((int16_t)Rec[4] << 8 | Rec[5]);

	Mpu_data.Gyro_x = Gyro_X_RAW/16.4;
	Mpu_data.Gyro_y = Gyro_Y_RAW/16.4;
	Mpu_data.Gyro_z = Gyro_Z_RAW/16.4;
}

char s[40];


int main(void)
{
 uint8_t len = 0;
  HAL_Init();

  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();

  MPU_Init();

  MAX30100_Init(&hi2c1, &huart2);
  MAX30100_SetSpO2SampleRate(MAX30100_SPO2SR_DEFAULT);
  MAX30100_SetLEDPulseWidth(MAX30100_LEDPW_DEFAULT);
  MAX30100_SetLEDCurrent(MAX30100_LEDCURRENT_DEFAULT, MAX30100_LEDCURRENT_DEFAULT);
  MAX30100_SetMode(MAX30100_SPO2_MODE);

  while(!SSD1306_Init()){
	  HAL_Delay(1000);
  }
  	 SSD1306_GotoXY(0,0);
     SSD1306_Puts ("HELLO", &Font_11x18, 1);
     SSD1306_GotoXY (10, 30);
     SSD1306_Puts ("  WORLD :)", &Font_11x18, 1);
     SSD1306_UpdateScreen(); //display
     HAL_Delay(2000);

     SSD1306_ScrollRight(0,7);  // scroll entire screen
     HAL_Delay(2000);  // 2 sec

     SSD1306_ScrollLeft(0,7);  // scroll entire screen
     HAL_Delay(2000);  // 2 sec

     SSD1306_Stopscroll();
     SSD1306_Clear();
     SSD1306_UpdateScreen();
     int last = 0;
     int last1=0;
  while (1)
  {
	  	  if(HAL_GetTick()-last1>5){
	  		last1=HAL_GetTick();
	  		MPU_Read_Accel();
	  			  	 MPU_Read_Gyro();
	  			  	 heart =  heart_rate();
	  	  }

	  	 //MAX30100_PlotBothToUART(&huart2, (float *)_max30100_red_sample, _max30100_red_meandiff, 16);

	  	 if(HAL_GetTick()-last > 1000){
	  		 last = HAL_GetTick();
	  		 Update_Accel_Gyro_to_SSD1306();
	  	 }

  }

}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){

	return;
}

void Update_Accel_Gyro_to_SSD1306(void){
			static volatile uint8_t flag = 0;
			uint8_t len;
			SSD1306_Clear();
			if(flag == 0){
			SSD1306_GotoXY(0, 0);
		  	 len = sprintf(s, "Accel_X: %f", Mpu_data.Accel_x);
		  	 SSD1306_Puts(s, &Font_7x10, 1);

		  	SSD1306_GotoXY(0,12);
		  	sprintf(s, "Accel_Y: %f", Mpu_data.Accel_y);
		  	SSD1306_Puts(s, &Font_7x10, 1);

		  	SSD1306_GotoXY(0, 24);
		  	sprintf(s, "Accel_Z: %f", Mpu_data.Accel_z);
		  	SSD1306_Puts(s, &Font_7x10, 1);

		  	SSD1306_GotoXY(0, 36);
		  	sprintf(s, "Gyro_X: %f", Mpu_data.Gyro_z);
		  	SSD1306_Puts(s, &Font_7x10, 1);

		  	SSD1306_GotoXY(0, 48);
		  	sprintf(s, "Gyro_Y: %f", Mpu_data.Gyro_y);
		  	SSD1306_Puts(s, &Font_7x10, 1);
		  	flag = 1;
			}

			else{

		  	SSD1306_GotoXY(0, 0);
		  	sprintf(s, "Gyro_Z: %f", Mpu_data.Gyro_z);
		  	SSD1306_Puts(s, &Font_7x10, 1);

		  	SSD1306_GotoXY(0, 12);
		  	sprintf(s, "ir sample : %d", _max30100_ir_sample[15]);
		  	SSD1306_Puts(s, &Font_7x10, 1);

		  	SSD1306_GotoXY(0, 24);
		  	sprintf(s, "red sample : %d", _max30100_red_sample[15]);
		  	SSD1306_Puts(s, &Font_7x10, 1);

		  	SSD1306_GotoXY(0, 36);
		  	sprintf(s, "heart rate : %f", heart);
		  	SSD1306_Puts(s, &Font_7x10, 1);
		  	flag = 0;
			}
		  	SSD1306_UpdateScreen();
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{


  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
