/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//PID Structure
typedef struct PID {
	float kp,ki,kd;						//Gain
	float integral, prev_error;			//State Variables
	float filtered_derivative;			//for smoothed derivative
	float setpoint, measured_val;		//desired and current values
	float dt;							//time step
	float min_output, max_output;		//output limits
	float min_integral, max_integral;	//integral limits
} PID;

//Command Bank
typedef enum {
	CMD_INVALID,     	// Used for unknown/invalid commands
    CMD_LEFT,       		// Unique identifier for "tilt left"
    CMD_RIGHT,      		// Unique identifier for "tilt right"
    CMD_FORWARD,    		// Unique identifier for "tilt forward"
    CMD_BACK,       		// Unique identifier for "tilt backward"
    CMD_ASCEND,     		// Unique identifier for "ascend"
    CMD_DESCEND,		// Unique identifier for "descend"
    CMD_EMERGENCY_SHUTOFF	// Immediate shutoff
} DroneCommand;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_START_SPEED 0				//Motor Speed at Initialization (Max. 2000)
#define MOTOR_DESCEND_SPEED 100			//Motor speed to use when descending (Max. 2000)
#define MOTOR_ASCEND_SPEED 350			//Motor speed to use when ascending (Max. 2000)
#define MOTOR_DIRECTION_DECREASE 180	 	//Lower motor speed to use when moving in a lateral direction (Max. 2000)
#define MOTOR_DIRECTION_INCREASE 220	//Higher motor speed to use when moving in a lateral direction (Max. 2000)
#define MOTOR_HOVER_SPEED 200			//Motor speed to use for hovering (Max. 2000)

#define MPU6500_ADDR 0xD0				//MPU6500 I2C Address (AD0 = 0)
#define ACCEL_XOUT_H 0x3B				//accel X-axis high byte register address
#define GYRO_SAMPLES 100				//Number of samples to take for calibration

#define RECV_SIZE 1						//buffer size to hold received UART data

#define MIN_OUTPUT -100					//
#define MAX_OUTPUT 100					//
#define MIN_INTEGRAL -50.0f				//
#define MAX_INTEGRAL 50.0f				//
#define RAD_TO_DEG 57.2958				//Rad to Deg conversion factor
#define ALPHA 0.98					//Filter coefficient (higher = gyro trust)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
void blinkLEDs(int count);

void Update_Motors();
void Handle_Input();
void Execute_Command(DroneCommand command);

void MPU6500_Init(void);
void Read_Gyro_Accel_Data(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);
void Calibrate_Gyro(void);

void init_PID(PID *pid, float kp, float ki, float kd, float dt);

float Get_AccelPitch(int16_t ax, int16_t ay, int16_t az);
float Get_AccelRoll(int16_t ax, int16_t ay, int16_t az);
void Update_GyroAngle(float *roll, float *pitch, int16_t gx, int16_t gy, float dt);
void Update_FilteredAngle(float *roll, float *pitch, int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, float dt);

float Compute_PID(PID *pid);

void print_sensor_data(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t RxBuf[RECV_SIZE]={0};	//Instruction Receive Buffer

int16_t motor_speeds[4] = {MOTOR_START_SPEED,MOTOR_START_SPEED,0,MOTOR_START_SPEED};	//Speeds of each individual motor

int16_t gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;

float current_roll = 0.0, current_pitch = 0.0;
float roll_setPoint = 0.0, pitch_setPoint = 0.0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	int16_t ax,ay,az,gx,gy,gz;

	PID pid_roll, pid_pitch;
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
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
  MPU6500_Init();
  Calibrate_Gyro();
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);

  HAL_UART_Receive_IT(&huart6,RxBuf,RECV_SIZE);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);		//Motor1
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);		//Motor2
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);		//Motor3
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);		//Motor4

  //Initializing the PID controllers
  init_PID(&pid_roll, 0.15, 0.0008, 0.01, 0.01);
  init_PID(&pid_pitch, 0.15, 0.0008, 0.01, 0.01);

  uint32_t last_time = HAL_GetTick();		//track time
  uint32_t current_time = 0;
  float dt = 0.01;
  int16_t roll_output = 0;
  int16_t pitch_output = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  current_time = HAL_GetTick();
	  dt = (current_time - last_time)/1000.0f;		//converting ms to s
	  if(dt>0.1) dt = 0.1;
	  last_time = current_time;

	  Read_Gyro_Accel_Data(&ax, &ay, &az, &gx, &gy, &gz);
	  Update_FilteredAngle(&current_roll, &current_pitch, ax, ay, az, gx, gy, dt);

	  //Print out gyro and accelerometer data for debugging
	  //print_sensor_data(ax,ay,az,gx,gy,gz);

	  //update dt in the PID structs
	  pid_roll.dt = dt;
	  pid_pitch.dt = dt;

	  pid_roll.setpoint = roll_setPoint;
	  pid_pitch.setpoint = pitch_setPoint;

	  pid_roll.measured_val = current_roll;
	  pid_pitch.measured_val = current_pitch;

	  roll_output = (int16_t)Compute_PID(&pid_roll);
	  pitch_output = (int16_t)Compute_PID(&pid_pitch);

	  Handle_Input();
	  motor_speeds[0] = motor_speeds[0] + roll_output;// - pitch_output;
	  motor_speeds[1] = motor_speeds[1] + roll_output;// + pitch_output;
	  motor_speeds[2] = motor_speeds[2] - roll_output;// + pitch_output;
	  motor_speeds[3] = motor_speeds[3] - roll_output;// - pitch_output;
	  Update_Motors();

	  HAL_Delay(50);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 499;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
float Compute_PID(PID *pid)
{
	float alpha = 0.8;		//smoothing factor for D term
	float error = pid->setpoint - pid->measured_val;		//Calculate error
	//P term
	float termP =pid->kp * error;
	//I term (with anti-windup)
	pid->integral += error * pid->dt;
	if(pid->integral > pid->max_integral) pid->integral = pid->max_integral;
	if(pid->integral < pid->min_integral) pid->integral = pid->min_integral;
	if(fabs(error)<0.5) pid->integral *= 0.95;
	float termI = pid->ki * pid->integral;
	//D term (with smoothing)
	float raw_derivative = (error - pid->prev_error)/ pid->dt;
	//LPF
	pid->filtered_derivative = (alpha * raw_derivative) + ((1-alpha) * pid->filtered_derivative);

	// Clamp filtered derivative to prevent runaway values
	if (pid->filtered_derivative > 1000.0f) pid->filtered_derivative = 1000.0f;
	if (pid->filtered_derivative < -1000.0f) pid->filtered_derivative = -1000.0f;

	float termD = pid->kd * pid->filtered_derivative;
	//update previous error
	if (error * pid->prev_error < 0){
		pid->integral = 0;
	}
	pid->prev_error = error;
	//Combine the terms
	float outputPID = termP + termI + termD;
	//Clamping output to prevent saturation
	if(outputPID > pid->max_output) outputPID = pid->max_output;
	if(outputPID < pid->min_output) outputPID = pid->min_output;

	//Print the values for troubleshooting
	//printf("Error: %.2f | P: %.2f | I: %.2f | D: %.2f | Output: %.2f\r\n", error, termP, termI, termD, outputPID);
	//printf("Setpoint: %.2f | Measured: %.2f | Error: %.2f\r\n", pid->setpoint, pid->measured_val, error);
	//printf("dt: %.6f\r\n", pid->dt);

	return outputPID;
}

void Update_FilteredAngle(float *roll, float *pitch, int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, float dt)
{
	float accel_roll = Get_AccelRoll(ax,ay,az);
	float accel_pitch = Get_AccelPitch(ax,ay,az);

	Update_GyroAngle(roll,pitch,gx,gy,dt);

	*roll = ALPHA*(*roll) + (1-ALPHA)*accel_roll;
	*pitch = ALPHA*(*pitch) + (1-ALPHA)*accel_pitch;
}

void Update_GyroAngle(float *roll, float *pitch, int16_t gx, int16_t gy, float dt)
{
	*roll += (gx/131.0f)*dt;		//131 LSB/deg/s (±250°/s range)
	*pitch += (gy/131.0f)*dt;
}

float Get_AccelPitch(int16_t ax, int16_t ay, int16_t az)
{
	return atan2(-ax, sqrt(ay*ay + az*az)) * RAD_TO_DEG;
}

float Get_AccelRoll(int16_t ax, int16_t ay, int16_t az)
{
	return atan2(ay,az) * RAD_TO_DEG;
}

void init_PID(PID *pid, float kp, float ki, float kd, float dt)
{
	pid->kp = kp;
	pid->kd = kd;
	pid->ki = ki;
	pid->dt = dt;

	pid->integral = 0.0;
	pid->prev_error = 0.0;
	pid->filtered_derivative = 0.0;
	pid->setpoint = 0.0;
	pid->measured_val = 0.0;

	pid->min_output = MIN_OUTPUT;
	pid->max_output = MAX_OUTPUT;
	pid->min_integral = MIN_INTEGRAL;
	pid->max_integral = MAX_INTEGRAL;
}

void Calibrate_Gyro(void)
{
	int32_t sum_x = 0, sum_y = 0, sum_z = 0;
	int16_t gx,gy,gz;
	int samples = GYRO_SAMPLES;

	for (int i=0; i<samples; i++)
	{
		Read_Gyro_Accel_Data(NULL,NULL,NULL,&gx,&gy,&gz);
		sum_x += gx;
		sum_y += gy;
		sum_z += gz;
		HAL_Delay(10);
	}

	gyro_offset_x = sum_x/samples;
	gyro_offset_y = sum_y/samples;
	gyro_offset_z = sum_z/samples;
}

void Read_Gyro_Accel_Data(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
	uint8_t rawData[14];

	// Read 14 bytes starting from ACCEL_XOUT_H
	if (HAL_I2C_Mem_Read(&hi2c1, MPU6500_ADDR, ACCEL_XOUT_H, 1, rawData, 14, HAL_MAX_DELAY) != HAL_OK)
	{
		return;
	}
	//Accelerometer Data
	*ax = ((int16_t)rawData[0]<<8) | rawData[1];
	*ay = ((int16_t)rawData[2]<<8) | rawData[3];
	*az = ((int16_t)rawData[4]<<8) | rawData[5];

	//Gyro data (apply offsets)
	*gx = (((int16_t)rawData[8]<<8) | rawData[9]) - gyro_offset_x;
	*gy = (((int16_t)rawData[10]<<8) | rawData[11]) - gyro_offset_y;
	*gz = (((int16_t)rawData[12]<<8) | rawData[13]) - gyro_offset_z;
}

void MPU6500_Init(void) {
    uint8_t data;
    // Wake up MPU6500 (remove sleep mode)
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6500_ADDR, 0x6B, 1, &data, 1, HAL_MAX_DELAY);
    // Set Gyro full-scale range to ±250°/s
//    data2 = 0x08;
    HAL_I2C_Mem_Write(&hi2c1, MPU6500_ADDR, 0x1B, 1, &data, 1, HAL_MAX_DELAY);
    // Set Accelerometer full-scale range to ±2g
    HAL_I2C_Mem_Write(&hi2c1, MPU6500_ADDR, 0x1C, 1, &data, 1, HAL_MAX_DELAY);
    // Configure Low-Pass Filter (DLPF = 4 → 20Hz)
    data = 0x04;
    HAL_I2C_Mem_Write(&hi2c1, MPU6500_ADDR, 0x1A, 1, &data, 1, HAL_MAX_DELAY);
}

void Execute_Command(DroneCommand command)
{
	switch (command) {
		case CMD_ASCEND:
			motor_speeds[0] = MOTOR_ASCEND_SPEED;
			motor_speeds[1] = MOTOR_ASCEND_SPEED;
			motor_speeds[2] = MOTOR_ASCEND_SPEED;
			motor_speeds[3] = MOTOR_ASCEND_SPEED;
			Update_Motors();
			break;
		case CMD_DESCEND:
			motor_speeds[0] = MOTOR_DESCEND_SPEED;
			motor_speeds[1] = MOTOR_DESCEND_SPEED;
			motor_speeds[2] = MOTOR_DESCEND_SPEED;
			motor_speeds[3] = MOTOR_DESCEND_SPEED;
			Update_Motors();
			break;
		case CMD_EMERGENCY_SHUTOFF:
			// Instantly stop all motors
			motor_speeds[0] = 0;
			motor_speeds[1] = 0;
			motor_speeds[2] = 0;
			motor_speeds[3] = 0;
			Update_Motors();
			break;
		case CMD_LEFT:
			motor_speeds[0] = MOTOR_DIRECTION_DECREASE;
			motor_speeds[1] = MOTOR_DIRECTION_DECREASE;
			motor_speeds[2] = MOTOR_DIRECTION_INCREASE;
			motor_speeds[3] = MOTOR_DIRECTION_INCREASE;
			Update_Motors();
			break;
		case CMD_RIGHT:
			motor_speeds[0] = MOTOR_DIRECTION_INCREASE;
			motor_speeds[1] = MOTOR_DIRECTION_INCREASE;
			motor_speeds[2] = MOTOR_DIRECTION_DECREASE;
			motor_speeds[3] = MOTOR_DIRECTION_DECREASE;
			Update_Motors();
			break;
		case CMD_FORWARD:
			motor_speeds[0] = MOTOR_DIRECTION_INCREASE;
			motor_speeds[1] = MOTOR_DIRECTION_DECREASE;
			motor_speeds[2] = MOTOR_DIRECTION_DECREASE;
			motor_speeds[3] = MOTOR_DIRECTION_INCREASE;
			Update_Motors();
			break;
		case CMD_BACK:
			motor_speeds[0] = MOTOR_DIRECTION_DECREASE;
			motor_speeds[1] = MOTOR_DIRECTION_INCREASE;
			motor_speeds[2] = MOTOR_DIRECTION_INCREASE;
			motor_speeds[3] = MOTOR_DIRECTION_DECREASE;
			Update_Motors();
			break;
		default: //MAY CHANGE TO HOVER
			break;
		}
}

void Handle_Input()
{
	DroneCommand command;
	int8_t input = atoi((const char*)RxBuf);
	switch(input)
	{
		case 0:
			command = CMD_EMERGENCY_SHUTOFF;
			break;
		case 1:
			command = CMD_ASCEND;
			break;
		case 2:
			command = CMD_DESCEND;
			break;
		case 3:
			command = CMD_LEFT;
			break;
		case 4:
			command = CMD_RIGHT;
			break;
		case 5:
			command = CMD_FORWARD;
			break;
		case 6:
			command = CMD_BACK;
			break;
		case 7:
			command = CMD_EMERGENCY_SHUTOFF;
			break;
		default:
			break;
	}
	Execute_Command(command);
}

void Update_Motors()
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, motor_speeds[0]);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, motor_speeds[1]);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, motor_speeds[2]);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, motor_speeds[3]);
}

//Function for troubleshooting, blink built in LED a certain number of times
void blinkLEDs(int count)
{
	for(int i = 0; i < count; i++)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); //Turn on LED
		HAL_Delay(500); //Wait 0.5s
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); //Turn of LED
		HAL_Delay(500);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(&huart6, RxBuf, RECV_SIZE);
}

void print_sensor_data(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz)
{
	float accel_x = ax/16384.0f;
	float accel_y = ay/16384.0f;
	float accel_z = az/16384.0f;
	float gyro_x = gx/131.0f;
	float gyro_y = gy/131.0f;
	float gyro_z = gz/131.0f;

	printf("AX:%.2f AY:%.2f AZ:%.2f] GX:%.2f GY:%.2f GZ:%.2f]\r\n",accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z);
}

int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart6, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	return len;
}
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
