/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
#include "nrf.h"
#include "spi.h"
#include "bno055.h"
#include "bno055_stm32.h"
#include "PID_Position.h"
#include "Mecanum.h"
//#include "EKF.h"
#include "math.h"
#include "fonts.h"
#include "ssd1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
Mecanum mec;
PIDController pid;
typedef struct {
	double Roll;
	double Pitch;
	double Yaw;
}Rotation;

typedef struct{
	uint32_t counter;
	uint32_t new_counter;
	uint8_t counter_status;
	float speed;
	float rdps;
	double distant;
}Encoder;

Encoder encoder0;
Encoder encoder1;

Rotation Angle;
bno055_vector_t V;
bno055_vector_t Q;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.1456
#define r 0.03 //[Radius m]
#define D 2*0.025  //[Diameter m]
#define CPR_X 1442
#define CPR_Y 1442
#define C   2*PI*r //[circumference  C=2*Pi*R]
#define sampling_time  10 //[10ms]
#define dt 0.01
#define Rotary1 0
#define Rotary2 1
//Wheel
#define R 0.05
#define lx 0.165  //m
#define ly 0.225  //m
#define d1 0.05
#define d2 0.15
#define X 0
#define Y 1
#define YAW 2


// Gain X
#define Kp_X 1.5
#define Ki_X 0.002
#define Kd_X 0.02
// Gain Y
#define Kp_Y 1.5
#define Ki_Y 0.002
#define Kd_Y 0.02
// Gain Yaw
#define Kp_Yaw 1.0
#define Ki_Yaw 0.001
#define Kd_Yaw 0.02
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
//------------------------------uart-functions--------------------------------------
volatile extern int flag;
volatile int flag = 0 ;
uint64_t RxpipeAddrs = 0x11223344BB;
const void* myAckPayload ;
int input_num = 0 ;
char input_data[32] ;
char c1;
char* c = &c1;
char c2;
char* c3 = &c2;
uint64_t TxpipeAddrs = 0x11223344AA;
char myTxData[32];
char AckPayload[32];
char myRxData[50];
float Speed;
float Vx;
float Vy;
float Omega;
float X_Start = 0.0;
float Y_Start = 0.0;
float Yaw_Start = 0.0;
//float X = 0.0;
//float Y = 0.0;
//float Yaw = 0.0;
float theta;

float X_end = 0.0;
float Y_end = 0.0;
float Yaw_end = 0.0;
float X_next = 0.0;
float Y_next = 0.0;
float Yaw_next = 0.0;

float Output_Vx;
float Output_Vy;
float Output_Omega;
float vx;
float vy;
float vyaw;
float VxM;
float VyM;
float VyawM;
extern float V1_back;
extern float V2_back;
extern float V3_back;
extern float V4_back;


//Variable of Calculate Position Robot from Rotary encoder wheel
extern float rdps[2];
float output_Vx;
float output_Vy;
float output_Omega;
float W1;
float W2;
float theta;
float Vx_enR;
float Vy_enR;
float Vyaw_IMU;
float X_old_enR;
float Y_old_enR;
float Yaw_old_IMU;
float yaw_old_enR;
float X_enR;
float Y_enR;
float Yaw_IMU = 0.0;
float yaw_enR;
float wheel_velocity_encoder[2];
float P_enR[3];
float old_count[2];
extern uint16_t Enc_count[2];
extern uint8_t dir[4];
uint16_t count[2]; // count pulse from encoder
uint16_t new_count[2];
uint8_t count_state[2];
uint16_t diff[2]; // difference between count and new_count in a sample time
float speed_rotary[2];
float rdps[2];
double position;

// IMU
float Radian;
double sinr_cosp;
double cosr_cosp;
double sinp;
double siny_cosp;
double cosy_cosp;
float  Heading;
float Heading_old;
float dyaw;
double previous_heading =0.0;
float heading;
double angle; // Angle in degrees
double radians;

//OLED
char XRum[10];
char YRum[10];
char Ium[10];
int X_R;
int Y_R;
int Yaw_R;
int Yaw_I;
int X_r;
int Y_r;
int Yaw_i;
int yaw_i;
uint8_t mode;

/* USER CODE END Variables */
osThreadId Position_TaskHandle;
osThreadId NRF_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

float Map(float Input, float Min_Input , float Max_Input ,float Min_Output, float Max_Output){

	return (float) ((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output);
}

float Speed_encoder(int i, float sample_time, float CPR)
{
	new_count[i] = Enc_count[i];
	wheel_velocity_encoder[i] = 2.0 *PI * r*(new_count[i] - old_count[i]) / (CPR * sample_time);
	old_count[i] = new_count[i];
	return wheel_velocity_encoder[i];
}

void read_encoder(Encoder *enc, TIM_HandleTypeDef* timer){
	enc->new_counter = __HAL_TIM_GET_COUNTER(timer);
	enc->counter_status = __HAL_TIM_IS_TIM_COUNTING_DOWN(timer);
	int16_t count_change = enc->new_counter - enc->counter;
	if(enc->counter_status && count_change <0){
		count_change += 65536;
	}else if (!enc->counter_status && count_change > 0){
		count_change -= 65536;
	}
	enc->counter = enc->new_counter;
	enc->counter_status = (count_change >=0);
	enc->speed = (float)count_change*1000.0f/(CPR_X * sampling_time);
	enc->rdps = (float)count_change*2*PI*1000.0f/(CPR_X * sampling_time);
}
void ReceiveMode(void){

	if (NRF24_available()) {
		NRF24_read(myRxData, 32);

		NRF24_writeAckPayload(1, myAckPayload, 32);
		myRxData[32] = '\r';
		myRxData[32 + 1] = '\n';

	}
}
/* USER CODE END FunctionPrototypes */

void Position_Init(void const * argument);
void NRF_Init(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Position_Task */
  osThreadDef(Position_Task, Position_Init, osPriorityNormal, 0, 2200);
  Position_TaskHandle = osThreadCreate(osThread(Position_Task), NULL);

  /* definition and creation of NRF_Task */
  osThreadDef(NRF_Task, NRF_Init, osPriorityIdle, 0, 1400);
  NRF_TaskHandle = osThreadCreate(osThread(NRF_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Position_Init */
/**
  * @brief  Function implementing the Position_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Position_Init */
void Position_Init(void const * argument)
{
  /* USER CODE BEGIN Position_Init */
	//PID_Controller
	PID_Position_Init(&pid, 3);
	pid.limMax = 1.5;
	pid.limMin = -1.5;
	pid.limMaxInt = 1.5;
	pid.limMinInt = -1.5;
	pid.T = dt;
	pid.alpha = 0.8;

	//Sensor Feed_back
//	Mecanum_Init(&mec, r, lx, ly);
	bno055_assignI2C(&hi2c1);
	bno055_setup();
	bno055_setOperationModeNDOF();
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);
//	SSD1306_Init();
//	SSD1306_GotoXY(10, 10); // goto 10, 10
//	SSD1306_Puts("mode:", &Font_7x10, 1); //
//	SSD1306_GotoXY(80, 10); //
//	SSD1306_Puts("cm.", &Font_7x10, 1); //
//	SSD1306_GotoXY(10, 30);
//	SSD1306_Puts("Y:", &Font_7x10, 1);
//	SSD1306_GotoXY(80, 30); //
//	SSD1306_Puts("cm.", &Font_7x10, 1); //
//	SSD1306_GotoXY(10, 50);
//	SSD1306_Puts("Yaw:", &Font_7x10, 1);
//	SSD1306_GotoXY(80, 50); //
//	SSD1306_Puts("rd.", &Font_7x10, 1); //
//	SSD1306_UpdateScreen();
  /* Infinite loop */
  for(;;)
  {
	  //*** Calculate PID ******//
		Output_Vx = PID_Position(&pid, X_end, X_enR, Kp_X, Ki_X, Kd_X, X);
		Output_Vy = PID_Position(&pid, Y_end, Y_enR, Kp_Y, Ki_Y, Kd_Y, Y);
		Output_Omega = PID_Position(&pid, Yaw_end, theta, Kp_Yaw, Ki_Yaw,Kd_Yaw, YAW);
		Vx = Output_Vx * cos(theta) - Output_Vy * sin(theta);
		Vy = Output_Vx * sin(theta) + Output_Vy * cos(theta);
		Omega = Output_Omega;
//		if (mode == 1 ){
//			Vx = vx;
//			Vy = vy;
//			Omega = vyaw;
//
//		}
//		else if (mode == 0)
//		{
//			Vx = VxM;
//			Vy = VyM;
//			Omega = VyawM;
//		}

	//**** Position From Rotary encoder and IMU   *****//

		// Qauternion_to_Euler(Angle);
		Q = bno055_getVectorQuaternion();
		// yaw (z-axis rotation)
		siny_cosp = 2 * (Q.w * Q.z + Q.x * Q.y);
		cosy_cosp = 1 - 2 * (Q.y * Q.y + Q.z * Q.z);
		Angle.Yaw = atan2(siny_cosp, cosy_cosp);
		theta = Angle.Yaw; // [radians]
		// Encoder Feedback
		read_encoder(&encoder0, &htim1);
		read_encoder(&encoder1, &htim2);
		W1 = (double)encoder1.rdps*r;
		W2 = (double)encoder0.rdps*r;
		Vx_enR = W1 * cosf(theta) - W2 * sinf(theta);
		Vy_enR = W1 * sinf(theta) + W2 * cosf(theta);
		X_enR = X_old_enR + Vx_enR * dt;
		Y_enR = Y_old_enR + Vy_enR * dt;

		X_old_enR = X_enR;
		Y_old_enR = Y_enR;


		//OLED
//		X_R = X_enR*10 ; //[cm]
//		Y_R = Y_enR*10 ; //[cm]
//		Yaw_I = theta*10 ;
//
//		itoa(mode, XRum, 10);
//		itoa(Y_R, YRum, 10);
//		itoa(Yaw_I, Ium, 10);
//
//
//		SSD1306_GotoXY(50, 10);
//		SSD1306_Puts(XRum, &Font_7x10, 1); // print Hello
//		SSD1306_GotoXY(50, 30);
//		SSD1306_Puts(YRum, &Font_7x10, 1);
//		SSD1306_GotoXY(50, 50);
//		SSD1306_Puts(Ium, &Font_7x10, 1);
//		SSD1306_UpdateScreen();
    osDelay(10);
  }
  /* USER CODE END Position_Init */
}

/* USER CODE BEGIN Header_NRF_Init */
/**
* @brief Function implementing the NRF_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_NRF_Init */
void NRF_Init(void const * argument)
{
  /* USER CODE BEGIN NRF_Init */
	//-----------------------------nrf-startup----------------------------------
	NRF24_begin(GPIOA, GPIOA, GPIO_PIN_3, GPIO_PIN_4, hspi1);
	//-----------------------------Rx-setting----------------------------------
	NRF24_setAutoAck(true);
	NRF24_setChannel(52);
	NRF24_setPayloadSize(32);
	NRF24_openReadingPipe(1, RxpipeAddrs);
	NRF24_enableDynamicPayloads();
	NRF24_enableAckPayload();
	NRF24_startListening();
  /* Infinite loop */
  for(;;)
  {
	  ReceiveMode();

	  /** Manual ***/
//	  Speed = myRxData[3];
//		if (myRxData[0] == 20) {
//
//			VxM = 0.8 + (Speed / 10.0);
//		} else if (myRxData[0] == 10) {
//			VxM = -(0.8 + (Speed / 10));
//		} else {
//			VxM = 0;
//		}
//		if (myRxData[1] == 20) {
//
//			VyM = (0.8 + (Speed / 10));
//		} else if (myRxData[1] == 10) {
//			VyM = -(0.8 + (Speed / 10));
//		} else {
//			VyM = 0;
//		}
//		if (myRxData[2] == 20) {
//
//			VyawM = 1.57 + (Speed / 10);
//		} else if (myRxData[2] == 10) {
//			VyawM = -(1.57 + (Speed / 10));
//		} else {
//			VyawM = 0;
//		}


		/**** Auto ***********/
		if (myRxData[4] == 10) { // goal 1
			X_Start = X_enR;
			Y_Start = Y_enR;
			Yaw_Start = theta;
			X_end = 6.05;
			Y_end = 0.0;
			Yaw_end = 0.0;
		}
		if (myRxData[5] == 20) { // goal2
			X_Start = X_enR;
			Y_Start = Y_enR;
			Yaw_Start = theta;
			X_end = 6.05;
			Y_end = -3.8;
			Yaw_end = 0.0;
		}
		if (myRxData[6] == 30) { // goal3
			X_Start = X_enR;
			Y_Start = Y_enR;
			Yaw_Start = theta;
			X_end = 9.0;
			Y_end = -3.8;
			Yaw_end = 0.0;
		}
		if (myRxData[7] == 40) { // goal 4
			X_Start = X_enR;
			Y_Start = Y_enR;
			Yaw_Start = theta;
			X_end = 10.0;
			Y_end = -1.2;
			Yaw_end = 0.8;
		}
		if (myRxData[8] == 50) // back (0,0,0)
		{
			X_Start = X_enR;
			Y_Start = Y_enR;
			Yaw_Start = theta;
			X_end = 0.0;
			Y_end = 0.0;
			Yaw_end = 0.0;
		}
//		if (myRxData[9] == 0)
//		{
//			mode = 0;
//		}
//		if (myRxData[9] == 1)
//		{
//			mode = 1;
//		}


    osDelay(20);
  }
  /* USER CODE END NRF_Init */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
