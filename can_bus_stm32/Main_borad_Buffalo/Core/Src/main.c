/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
CAN_RxHeaderTypeDef RxHeader;
CAN_TxHeaderTypeDef TxHeader;
#define Rotary1 0
#define Rotary2 1
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//----------CAN----------------//
uint8_t TxData[8];
uint32_t TxMailbox;
uint16_t V1_out = 0;
uint16_t V2_out = 0;
uint16_t V3_out = 0;
uint16_t V4_out = 0;

uint8_t RxData[8];
//uint8_t TxData[8];
//uint32_t TxMailbox;
float RxData1 = 0;
float RxData2 = 0;
float RxData3 = 0;
float RxData4 = 0;
float V1 = 0; // target speed of motor1
float V2 = 0; // target speed of motor2
float V3 = 0; // target speed of motor1
float V4 = 0; // target speed of motor2

uint8_t datacheck = 0;
uint8_t cntt;
float V_back;
float V1_back;
float V2_back;
float V3_back;
float V4_back;
//NRF
extern float Speed;
extern float Vx;
extern float Vy;
extern float Omega;
//Varible to read Encoder
float v;
uint8_t nowA[4];
uint8_t nowB[4];
uint8_t lastA[4];
uint8_t lastB[4];
uint8_t dir[4];
uint16_t cnt[4];
uint16_t Enc_count[4];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
float map(float Input, float Min_Input , float Max_Input ,float Min_Output, float Max_Output){

	return (float) ((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	cntt++;
	while (cntt - 100 > 0) {
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		cntt = 0;
	}

	if (RxHeader.StdId == 0x215) {
			RxData1 = (RxData[0] << 8) | RxData[1];
			RxData2 = (RxData[2] << 8) | RxData[3];
			V1_back = map(RxData1, 0, 65535, -30.0, 30.0);
			V2_back = map(RxData2, 0, 65535, -30.0, 30.0);
			datacheck = 1;
	}
	else if (RxHeader.StdId == 0x211) {
			RxData3 = (RxData[0] << 8) | RxData[1];
			RxData4 = (RxData[2] << 8) | RxData[3];
			V3_back = map(RxData3, 0, 65535, -30.0, 30.0);
			V4_back = map(RxData4, 0, 65535, -30.0, 30.0);
			datacheck = 1;
	}
}

float encoder(int i)
{
	if (nowA[i] != lastA[i])
	{
		lastA[i] = nowA[i];
		if (lastA[i] == 0)
		{
			if (nowB[i] == 0)
			{
				dir[i] = 0;
				cnt[i]--;
			}
			else
			{
				dir[i] = 1;
				cnt[i]++;
			}
		}
		else
		{
			if (nowB[i] == 1)
			{
				dir[i] = 0;
				cnt[i]--;
			}
			else
			{
				dir[i] = 1;
				cnt[i]++;
			}
		}
	}
	if (nowB[i] != lastB[i])
	{
		lastB[i] = nowB[i];
		if (lastB[i] == 0)
		{
			if (nowA[i] == 1)
			{
				dir[i] = 0;
				cnt[i]--;
			}
			else
			{
				dir[i] = 1;
				cnt[i]++;
			}
		}
		else
		{
			if (nowA[i] == 0)
			{
				dir[i] = 0;
				cnt[i]--;
			}
			else
			{
				dir[i] = 1;
				cnt[i]++;
			}
		}
	}
	return cnt[i];
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
  // CAN _Transmition
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	TxHeader.DLC = 8; // data length
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = 0x407; //Id 0x7FF
	// TIMER Internal clock
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim9);
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	if (GPIO_Pin == E1_C1_Pin || E1_C2_Pin)
//	{ // ENCODER Motor 1
//		nowA[0] = HAL_GPIO_ReadPin(E1_C1_GPIO_Port, E1_C1_Pin);
//		nowB[0] = HAL_GPIO_ReadPin(E1_C2_GPIO_Port, E1_C2_Pin);
//		Enc_count[0] = encoder(0);
//
//	}
//	if (GPIO_Pin == E2_C1_Pin || E2_C2_Pin)
//	{
//		nowA[1] = HAL_GPIO_ReadPin(E2_C1_GPIO_Port, E2_C1_Pin);
//		nowB[1] = HAL_GPIO_ReadPin(E2_C2_GPIO_Port, E2_C2_Pin);
//		Enc_count[1] = encoder(1);
//	}
//
//}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM3) {

		V1_out = map(Vx, -2.0, 2.0, 0, 65535);
		V2_out = map(Vy, -2.0, 2.0, 0, 65535);
		V3_out = map(Omega, -3.14, 3.14, 0, 65535);
		V4_out = map(Speed, 0.0, 2.0, 0, 65535);
		TxData[0] = ((V1_out & 0xFF00) >> 8);
		TxData[1] = (V1_out & 0x00FF);
		TxData[2] = ((V2_out & 0xFF00) >> 8);
		TxData[3] = (V2_out & 0x00FF);
		TxData[4] = ((V3_out & 0xFF00) >> 8);
		TxData[5] = (V3_out & 0x00FF);
		TxData[6] = ((V4_out & 0xFF00) >> 8);
		TxData[7] = (V4_out & 0x00FF);
		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
		}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
