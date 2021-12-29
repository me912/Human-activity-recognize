#include "main.h"
#include "cmsis_os.h"
#include "string.h"
#include "mpu6050.h"
#include "max30100.h"
#include "ssd1306.h"
#include "ai_platform.h"
#include "ai_datatypes_defines.h"
#include "har.h"
#include "har_data.h"
#include <stdio.h>
#include <math.h>
#define  DATA_RDY 1
#define  BUSY_PREDICT (1<<1)
#define  WINDOW_SIZE  	(count_delay/data_cycle)
#define  MAG_THERSHOLD  1.3
#define  GM				9.8
#define 	data_cycle 	10
#define 	count_delay 300
#define 	K			2
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* Definitions for GetValue */
osThreadId_t GetValueHandle;
const osThreadAttr_t GetValue_attributes = { .name = "GetValue", .stack_size =
		128 * 4 * 8, .priority = (osPriority_t) osPriorityHigh, };
/* Definitions for RunNeural */
osThreadId_t RunNeuralHandle;
const osThreadAttr_t RunNeural_attributes = { .name = "RunNeural", .stack_size =
		128 * 4 * 20, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for CountStep */
osThreadId_t CountStepHandle;
const osThreadAttr_t CountStep_attributes = { .name = "CountStep", .stack_size =
		128 * 4 * 4, .priority = (osPriority_t) osPriorityHigh, };
/* Definitions for ValueQueue */
osMessageQueueId_t ValueQueueHandle;
const osMessageQueueAttr_t ValueQueue_attributes = { .name = "ValueQueue" };
/* Definitions for Neural_permit */
osMutexId_t Neural_permitHandle;
const osMutexAttr_t Neural_permit_attributes = { .name = "Neural_permit" };
/* Definitions for ValueReady */
osSemaphoreId_t ValueReadyHandle;
const osSemaphoreAttr_t ValueReady_attributes = { .name = "ValueReady" };
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C1_Init(void);
float gravitystatic(void);
void StartTask1(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void printresult(float *y_val);
void MutexAcquire(osMutexId_t *mutex);
void MutexRelease(osMutexId_t *mutex);
void debug(Har_InputTypeDef *newItem);
void DisplayStep(int stepcount, float step_length);
int max_index(float *y_val);

char buf[500];
int buf_len = 0;
float window_data[WINDOW_SIZE];
int current_predict;

// Buffers used to store input and output tensors
AI_ALIGNED(4) ai_i8 in_data[AI_HAR_IN_1_SIZE_BYTES] = { 0 };
AI_ALIGNED(4) ai_i8 out_data[AI_HAR_OUT_1_SIZE_BYTES];

// Pointer to our model
ai_handle har_model = AI_HANDLE_NULL;

// Initialize wrapper structs that hold pointers to data and info about the
// data (tensor height, width, channels)
ai_buffer ai_input[AI_HAR_IN_NUM] = AI_HAR_IN;
ai_buffer ai_output[AI_HAR_OUT_NUM] = AI_HAR_OUT;
float y_val[6];
int step_count = 0;
double step_length = 0;
int main(void) {
	ai_error ai_err;
	ai_i32 nbatch;
	uint32_t timestamp;
	//Har_InputTypeDef *mpudata = (Har_InputTypeDef*) malloc(sizeof(Har_InputTypeDef)); //newest input sensor

	AI_ALIGNED(4) ai_u8 activations[AI_HAR_DATA_ACTIVATIONS_SIZE];

	//Har_InputTypeDef *in_data = (Har_InputTypeDef *)malloc(128*sizeof(Har_InputTypeDef)); //input tensor

	ai_network_params ai_params = { .params =
	AI_HAR_DATA_WEIGHTS(ai_har_data_weights_get()), .activations =
	AI_HAR_DATA_ACTIVATIONS(activations) };

	ai_input[0].n_batches = 1;
	ai_input[0].data = AI_HANDLE_PTR(in_data);
	ai_output[0].n_batches = 1;
	ai_output[0].data = AI_HANDLE_PTR(out_data);
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_CRC_Init();
	MX_I2C1_Init();

	ai_err = ai_har_create(&har_model, AI_HAR_DATA_CONFIG);
	if (ai_err.type != AI_ERROR_NONE) {
		buf_len = sprintf(buf, "Error: could not create NN instance\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) buf, buf_len, 100);
		while (1)
			;
	}
	if (!ai_har_init(har_model, &ai_params)) {
		buf_len = sprintf(buf, "Error: could not initialize NN\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) buf, buf_len, 100);
		while (1)
			;
	}

	buf_len = sprintf(buf, "Create success\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) buf, buf_len, 100);

//	MAX30100_Init(&hi2c1, &huart2);
//	MAX30100_SetSpO2SampleRate(MAX30100_SPO2SR_DEFAULT);
//	MAX30100_SetLEDPulseWidth(MAX30100_LEDPW_DEFAULT);
//	MAX30100_SetLEDCurrent(MAX30100_LEDCURRENT_DEFAULT,
//			MAX30100_LEDCURRENT_DEFAULT);
//	MAX30100_SetMode(MAX30100_SPO2_MODE);

	while (!SSD1306_Init()) {
		HAL_Delay(1000);
	}

	SSD1306_Clear();
	MPU_Init();

	const float M = gravitystatic();

	buf_len = sprintf(buf, "M: %f\r\n", M);
	HAL_UART_Transmit(&huart2, buf, buf_len, 100);

	Har_InputTypeDef mpudata;
	for (int i = 0; i < 128; i++) {
		MPU_Read_Data_forHAR(&mpudata);
		((Har_InputTypeDef*) in_data)[i] = mpudata;
	}

	ai_har_run(har_model, &ai_input[0], &ai_output[0]);
	osKernelInitialize();
	/* Create the mutex(es) */
	/* creation of Neural_permit */
	Neural_permitHandle = osMutexNew(&Neural_permit_attributes);

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* creation of ValueReady */
	ValueReadyHandle = osSemaphoreNew(1, 0, &ValueReady_attributes);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* creation of ValueQueue */
	ValueQueueHandle = osMessageQueueNew(128, sizeof(Har_InputTypeDef),
			&ValueQueue_attributes);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of GetValue */
	GetValueHandle = osThreadNew(StartTask1, NULL, &GetValue_attributes);

	/* creation of RunNeural */
	RunNeuralHandle = osThreadNew(StartTask02, NULL, &RunNeural_attributes);

	/* creation of CountStep */
	CountStepHandle = osThreadNew(StartTask03, NULL, &CountStep_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
	buf_len = sprintf(buf, "I2C error!");
	HAL_UART_Transmit(&huart2, buf, buf_len, 100);

}
/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 100;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void) {

	/* USER CODE BEGIN CRC_Init 0 */

	/* USER CODE END CRC_Init 0 */

	/* USER CODE BEGIN CRC_Init 1 */

	/* USER CODE END CRC_Init 1 */
	hcrc.Instance = CRC;
	if (HAL_CRC_Init(&hcrc) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CRC_Init 2 */

	/* USER CODE END CRC_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

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

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask1 */
/**
 * @brief  Function implementing the Task1 thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask1 */
void StartTask1(void *argument) {
	Har_InputTypeDef mpudata;
	int i = 0;
	volatile double max_mag = 0;
	volatile double min_mag = 1000;
	double prev_mag = 1;
	double curr_mag = 0;
	double sum = 0;
	volatile int dem = 0;
	for (;;) {
		dem++;
		MPU_Read_Data_forHAR(&mpudata);
		if (i++ % 50 == 0) {
			debug(&mpudata);
		}
		MutexAcquire(&Neural_permitHandle);
		osMessageQueuePut(ValueQueueHandle, &mpudata, NULL, 0);
		MutexRelease(&Neural_permitHandle);
		curr_mag = MPU_mag2(&mpudata);
		sum += fabs((double) curr_mag);
		if (max_mag < curr_mag)
			max_mag = curr_mag;
		if (min_mag > curr_mag)
			min_mag = curr_mag;
		if (curr_mag > 12.0/ 9.8 && current_predict <= 2) {
			if(dem > 10)
			{
			step_count++;
			step_length += K * (sum / dem - min_mag) / (max_mag - min_mag);
			buf_len = sprintf(buf, "Step count = %d\r\nStep length: %f\r\n",
					step_count, step_length);
			HAL_UART_Transmit(&huart2, buf, buf_len, 100);
			min_mag = 10000;
			max_mag = -1;
			sum = 0;
			dem = 0;
			}
		}
		prev_mag = curr_mag;
		osDelay(20);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * @brief Function implementing the RunNeural thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument) {
	ai_i32 nbatch;
	Har_InputTypeDef data;
	for (;;) {
		MutexAcquire(&Neural_permitHandle);
		uint32_t count = osMessageQueueGetCount(ValueQueueHandle);
		memmove((Har_InputTypeDef*) in_data,
				((Har_InputTypeDef*) in_data + count),
				sizeof(Har_InputTypeDef) * (128 - count));

		for (int i = 128 - count; i < 128; i++) {
			osMessageQueueGet(ValueQueueHandle, &data, NULL, 1);
			*((Har_InputTypeDef*) in_data + i) = data;

		}
		osMessageQueueReset(ValueQueueHandle);
		MutexRelease(&Neural_permitHandle);

//		volatile float a[1000] = {0};
		nbatch = ai_har_run(har_model, &ai_input[0], &ai_output[0]); //block 100ms
		if (nbatch != 1) {
			buf_len = sprintf(buf, "Error: could not run inference\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) buf, buf_len, 100);
		}
		memcpy(y_val, out_data, sizeof(float)*6);

		//printresult(y_val);
		current_predict = max_index(y_val);
		osDelay(200);
	}
	/* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
 * @brief Function implementing the CountStep thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument) {
	for (;;) {
		DisplayStep(step_count, step_length);
		osDelay(1500);
	}
	/* USER CODE END StartTask03 */
}

float gravitystatic(void) {
	MPU_DATA *mpu_data = (MPU_DATA*) malloc(sizeof(MPU_DATA));
	buf_len = sprintf(buf, "Leave the device stationary, wait.\r\n");
	HAL_UART_Transmit(&huart2, buf, buf_len, 100);
	float x = 0;
	for (int i = 0; i < 20; i++) {

		MPU_Read_Data(mpu_data);
		x += mpu_data->Accel_x;
		HAL_Delay(5);
	}
	buf_len = sprintf(buf, "Finished!\r\n");
	HAL_UART_Transmit(&huart2, buf, buf_len, 100);
	free(mpu_data);
	return x / 20.0;

}

int max_index(float *y_val){
	volatile int max_index = 0;
		for (int i = 0; i < 6; i++) {
			if (y_val[max_index] < y_val[i])
				max_index = i;
		}
	return max_index;
}

void printresult(float *y_val) {
	volatile int max_index = 0;
	for (int i = 0; i < 6; i++) {
		if (y_val[max_index] < y_val[i])
			max_index = i;
	}

	buf_len = sprintf(buf, "Output: ");
	SSD1306_GotoXY(0, 0);
	SSD1306_Puts(buf, &Font_7x10, 1);
	switch (max_index) {
	case 0:
		buf_len = sprintf(buf, "WALKING");
		break;
	case 1:
		buf_len = sprintf(buf, "WALKING_UPSTAIRS");
		break;
	case 2:
		buf_len = sprintf(buf, "WALKING_DOWNSTAIRS");
		break;
	case 3:
		buf_len = sprintf(buf, "SITTING");
		break;
	case 4:
		buf_len = sprintf(buf, "STANDING");
		break;
	case 5:
		buf_len = sprintf(buf, "LAYING");
		break;
	}
	HAL_UART_Transmit(&huart2, buf, buf_len, 100);
	char *newline = "\r\n";
	HAL_UART_Transmit(&huart2, newline, 2, 100);
	SSD1306_GotoXY(0, 12);
	SSD1306_Puts(buf, &Font_7x10, 1);
	SSD1306_UpdateScreen();
}

void MutexAcquire(osMutexId_t *mutex) {
	osStatus status = osMutexAcquire(*mutex, osWaitForever);
	if (status != osOK) {
		buf_len = sprintf(buf, "Mutex error in MutexAcquire function!");
		HAL_UART_Transmit(&huart2, buf, buf_len, 5);
	}
}
void debug(Har_InputTypeDef *newItem) {
	buf_len = sprintf(buf, "(%f, %f, %f, %f, %f, %f, %f, %f, %f)\r\n", newItem->total_acc_x,
			newItem->total_acc_y, newItem->total_acc_z, newItem->body_gyro_x,
			newItem->body_gyro_y, newItem->body_gyro_z, newItem->body_acc_x, newItem->body_acc_y, newItem->body_acc_z);
	HAL_UART_Transmit(&huart2, buf, buf_len, 10);
}
void DisplayStep(int stepcount, float step_length) {
	SSD1306_Clear();
	buf_len = sprintf(buf, "Step count: %d", stepcount);
	SSD1306_GotoXY(0, 24);
	SSD1306_Puts(buf, &Font_7x10, 1);
	SSD1306_UpdateScreen();

	buf_len = sprintf(buf, "Total step length:", step_length);
	SSD1306_GotoXY(0, 36);
	SSD1306_Puts(buf, &Font_7x10, 1);
	SSD1306_UpdateScreen();

	buf_len = sprintf(buf, "%f", step_length);
		SSD1306_GotoXY(0, 48);
		SSD1306_Puts(buf, &Font_7x10, 1);
		SSD1306_UpdateScreen();

	printresult(y_val);
}
void MutexRelease(osMutexId_t *mutex) {
	osStatus status = osMutexRelease(*mutex);
	if (status != osOK) {
		buf_len = sprintf(buf, "Mutex error in MutexRelease function!");
		HAL_UART_Transmit(&huart2, buf, buf_len, 5);
	}
}
/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
