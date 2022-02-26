/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
//MD Drone - Code for drone
//Author: Lukasz Halbiniak
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

//Variable defined by user

bool Start_var = false;
bool Test_var = false;
uint8_t Received_from_Android[38];
uint8_t Data_to_UART[40];

uint32_t ADC1_Variables[2];
uint16_t PWM1_Duty = 0;
uint16_t PWM2_Duty = 0;
uint16_t PWM3_Duty = 0;
uint16_t PWM4_Duty = 0;

int16_t F_Var_from_Android =0;
int16_t T_Var_from_Android =0;
int16_t P_Var_from_Android =0;
int16_t H_Var_from_Android =0;
int16_t D_Var_from_Android =0;
int16_t R_Var_from_Android =0;
int16_t M1_Var_from_Android =0;
int16_t M2_Var_from_Android =0;
int16_t M3_Var_from_Android =0;
int16_t M4_Var_from_Android =0;
int8_t X_to_Android =1;
int8_t Y_to_Android =2;
int8_t Z_to_Android =3;
int8_t F_to_Android =4;
int8_t T_to_Android =5;
int8_t P_to_Android =6;
int8_t B_to_Android =7;
double Accel_Output [3] = {0,0,0};
double X_Calib =0;
double Y_Calib =0;


//Variables defined by ACC and GYRO
//CTRL

uint8_t CTRL1_MSG_W[2] = {0x10,0x66};
uint8_t CTRL2_MSG_W[2] = {0x11,0x6C};
uint8_t CTRL4_MSG_W[2] = {0x13,0x80};
uint8_t CTRL_REC[2] = {0x00,0x00};
//ACC X
uint8_t OUTXLA_MSG[2]= {0xA8,0xFF};
uint8_t OUTXHA_MSG[2]= {0xA9,0xFF};
uint8_t OUTXLA_REC[2]= {0xFF,0x00};
uint8_t OUTXHA_REC[2]= {0xFF,0x00};
//ACC Y
uint8_t OUTYLA_MSG[2]= {0xAA,0xFF};
uint8_t OUTYHA_MSG[2]= {0xAB,0xFF};
uint8_t OUTYLA_REC[2]= {0xFF,0x00};
uint8_t OUTYHA_REC[2]= {0xFF,0x00};
//ACC Z
uint8_t OUTZLA_MSG[2]= {0xAC,0xFF};
uint8_t OUTZHA_MSG[2]= {0xAD,0xFF};
uint8_t OUTZLA_REC[2]= {0xFF,0x00};
uint8_t OUTZHA_REC[2]= {0xFF,0x00};
//GYRO X
uint8_t OUTXLG_MSG[2]= {0xA2,0xFF};
uint8_t OUTXHG_MSG[2]= {0xA3,0xFF};
uint8_t OUTXLG_REC[2]= {0xFF,0x00};
uint8_t OUTXHG_REC[2]= {0xFF,0x00};
//GYRO Y
uint8_t OUTYLG_MSG[2]= {0xA4,0xFF};
uint8_t OUTYHG_MSG[2]= {0xA5,0xFF};
uint8_t OUTYLG_REC[2]= {0xFF,0x00};
uint8_t OUTYHG_REC[2]= {0xFF,0x00};
//GYRO Z
uint8_t OUTZLG_MSG[2]= {0xA6,0xFF};
uint8_t OUTZHG_MSG[2]= {0xA7,0xFF};
uint8_t OUTZLG_REC[2]= {0xFF,0x00};
uint8_t OUTZHG_REC[2]= {0xFF,0x00};

float A_Lin_D_1[6] = {1,0,0,0,0,0};
float A_Lin_D_2[6] = {0,1,0,0,0,0};
float A_Lin_D_3[6] = {0,0,1,0,0,0};
float A_Lin_D_4[8] = {0.2,0,0,1,0,0};
float A_Lin_D_5[8] = {0,0.2,0,0,1,0};
float A_Lin_D_6[8] = {0,0,0.2,0,0,1};

float B_Lin_D_1[4] = {2.22,2.22,2.22,2.22};
float B_Lin_D_2[4] = {0, 293.04 ,0,-293.04};
float B_Lin_D_3[4] = {300.18, 0, -300.18, 0};
float B_Lin_D_4[4] = {0,0,0,0};
float B_Lin_D_5[4] = {0,0,0,0};
float B_Lin_D_6[4] = {0,0,0,0};

float C_Lin_D_1[6] = {1,0,0,0,0,0};
float C_Lin_D_2[6] = {0,0,0,0,0,0};
float C_Lin_D_3[6] = {0,0,0,0,0,0};
float C_Lin_D_4[6] = {0,0,0,0,0,0};
float C_Lin_D_5[6] = {0,0,0,0,1,0};
float C_Lin_D_6[6] = {0,0,0,0,0,1};

float D_Lin_D = 1.964;
float L_Kalman_1[6] = {0.000334, 0,      0,     0.1246, 0,         0};
float L_Kalman_2[6] = {0,        1,      0,     0,      0.000171, 0};
float L_Kalman_3[6] = {0,        0,      1,     0,      0,         0.000163};
float L_Kalman_4[6] = {0.000316, 0,      0,     0.2360, 0,         0};
float L_Kalman_5[6] = {0,        0.2,    0,     0,      0.109,     0};
float L_Kalman_6[6] = {0,        0,      0.2,   0,      0,         0.109};

float K_LQR_1[6] = {0.1277, 0,      0.0034,  0.0972,  0,       0.0083 };
float K_LQR_2[6] = {0.1277, 0.0034, 0,       0.0972,  0.0085,  0};
float K_LQR_3[6] = {0.1277, 0,      -0.0033, 0.0972,  0,       -0.0083};
float K_LQR_4[6] = {0.1277,-0.0034, 0,       0.0972,  -0.0085, 0};
float Y_Output_to_Kalman[6] = {0,0,0,0,0,0};
float X_Prior_Kalman_Buff[6] = {0,0,0,0,0,0};
float U_input_Buff[4] = {0,0,0,0};
float X_Reg_Var[6]= {0,0,0,0,0,0};
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void Reset_function()
{
	//Stop timer
	HAL_TIM_Base_Stop_IT(&htim3);

	//Reset every variables
	PWM1_Duty = 0;
	PWM2_Duty = 0;
	PWM3_Duty = 0;
	PWM4_Duty = 0;
	Start_var = false;
	Test_var = false;
	F_Var_from_Android =0;
	T_Var_from_Android =0;
	P_Var_from_Android =0;
	H_Var_from_Android =0;
	D_Var_from_Android =0;
	R_Var_from_Android =0;
	M1_Var_from_Android =0;
	M2_Var_from_Android =0;
	M3_Var_from_Android =0;
	M4_Var_from_Android =0;
	X_to_Android =1;
	Y_to_Android =2;
	Z_to_Android =3;
	F_to_Android =-4;
	T_to_Android =5;
	P_to_Android =6;
	B_to_Android =7;
	Accel_Output [0] = 0;
	Accel_Output [1] = 0;
	Accel_Output [2] = 0;
	Y_Output_to_Kalman[0] = 0;
	Y_Output_to_Kalman[1] = 0;
	Y_Output_to_Kalman[2] = 0;
	Y_Output_to_Kalman[3] = 0;
	Y_Output_to_Kalman[4] = 0;
	Y_Output_to_Kalman[5] = 0;
	X_Prior_Kalman_Buff[0] = 0;
	X_Prior_Kalman_Buff[1] = 0;
	X_Prior_Kalman_Buff[2] = 0;
	X_Prior_Kalman_Buff[3] = 0;
	X_Prior_Kalman_Buff[4] = 0;
	X_Prior_Kalman_Buff[5] = 0;
	U_input_Buff[0] = 0;
	U_input_Buff[1] = 0;
	U_input_Buff[2] = 0;
	U_input_Buff[3] = 0;
	X_Reg_Var[0]= 0;
	X_Reg_Var[1]= 0;
	X_Reg_Var[2]= 0;
	X_Reg_Var[3]= 0;
	X_Reg_Var[4]= 0;
	X_Reg_Var[5]= 0;
	//Set PWM to zero
	TIM2->CCR1 = PWM1_Duty;
	TIM2->CCR2 = PWM2_Duty;
	TIM2->CCR3 = PWM3_Duty;
	TIM2->CCR4 = PWM4_Duty;

	//Information for user
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

}
void Mode_function()
{
	//Start drone mode
	if (D_Var_from_Android == 1 && R_Var_from_Android == 0 && Start_var == false && Test_var == false)
	{
		Start_var = true;
		Test_var = false;
		HAL_TIM_Base_Start_IT(&htim3);

	//Test drone mode
	}else if (D_Var_from_Android == 0 && R_Var_from_Android == 1 && Start_var == false && Test_var == false)
	{
		Start_var = false;
		Test_var = true;
		HAL_TIM_Base_Start_IT(&htim3);

	//Reset drone mode and stop
	}else if (D_Var_from_Android == 0 && R_Var_from_Android == 0)
	{
		Start_var = false;
		Test_var = false;
		Reset_function();
	}

	//Waiting for commands from Android
	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	HAL_UART_Receive_DMA(&huart2, Received_from_Android, 37);
}

int16_t Cut_Char(uint8_t Input_char[], uint8_t From_char, uint8_t Many)
{
	//Function for cutting char tab
	int16_t Result =0;
	uint8_t Output_char[3];
	if (Many == (uint8_t)3)
	{

		for (uint8_t a = 0; a <= Many; ++a)
		{
			Output_char[a]=Input_char[From_char +a +1];
		}

		if (Output_char[0] == '0' )
		{
			if (Output_char[1] == '0')
			{
				Result = (int16_t)Output_char[2] - '0';
			}else
			{
				uint8_t buff[2];
				buff[0] = Input_char[From_char +1 +1];
				buff[1] = Input_char[From_char +2 +1];
				Result = atoi(buff);
			}

		}else
		{
			Result = atoi(Output_char);
		}
	}else if (Many == (uint8_t)1)
	{
		uint8_t Buff = Input_char[From_char+1];
		Result = (int16_t)Buff- '0';
	}

	return Result;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {

		//Message search to int
		F_Var_from_Android = Cut_Char(Received_from_Android, 0,3);
		T_Var_from_Android = Cut_Char(Received_from_Android, 4,3);
		P_Var_from_Android = Cut_Char(Received_from_Android, 8,3);
		H_Var_from_Android = Cut_Char(Received_from_Android, 12,3);
		D_Var_from_Android = Cut_Char(Received_from_Android, 16, 1);
		R_Var_from_Android = Cut_Char(Received_from_Android, 18, 1);
		M1_Var_from_Android = Cut_Char(Received_from_Android, 20,3);
		M2_Var_from_Android = Cut_Char(Received_from_Android, 24,3);
		M3_Var_from_Android = Cut_Char(Received_from_Android, 28,3);
		M4_Var_from_Android = Cut_Char(Received_from_Android, 32,3);

		//Making Variables
		//Sending messages
		uint16_t Message_size = sprintf(Data_to_UART, "X%dY%dZ%dF%dT%dP%dB%dQ",X_to_Android,Y_to_Android, Z_to_Android, F_to_Android, T_to_Android, P_to_Android,(ADC1_Variables[0])/10 );
		HAL_UART_Transmit(&huart1, Data_to_UART, Message_size,HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart2, Data_to_UART, Message_size,HAL_MAX_DELAY);
	}
	//Mode change
	Mode_function();
}
void Accel_Calib()
{
	//Receiving data from ACCEL
			//ACC X
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
			HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)OUTXLA_MSG,(uint8_t *)OUTXLA_REC,2,HAL_MAX_DELAY);
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);

			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
			HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)OUTXHA_MSG,(uint8_t *)OUTXHA_REC,2,HAL_MAX_DELAY);
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);
			//ACC Y
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
			HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)OUTYLA_MSG,(uint8_t *)OUTYLA_REC,2,HAL_MAX_DELAY);
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);

			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
			HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)OUTYHA_MSG,(uint8_t *)OUTYHA_REC,2,HAL_MAX_DELAY);
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);
			//ACC Z
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
			HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)OUTZLA_MSG,(uint8_t *)OUTZLA_REC,2,HAL_MAX_DELAY);
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);

			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
			HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)OUTZHA_MSG,(uint8_t *)OUTZHA_REC,2,HAL_MAX_DELAY);
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);

			//Consolidation
			int16_t AX = (OUTXHA_REC[1]<<8) | OUTXLA_REC[1];
			int16_t AY = (OUTYHA_REC[1]<<8) | OUTYLA_REC[1];

			double ACC_X = ((double)AX * 0.488/1000);
			double ACC_Y = ((double)AY * 0.488/1000);

			//Operations for data
			X_Calib = atan2(ACC_Y, 1) *180/3.14;
			Y_Calib = -atan2(-ACC_X, sqrt(ACC_Y*ACC_Y + 1*1))  *180/3.14;
}
void Accel_get_var()
{
		//Receiving data from ACCEL
		//ACC X
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)OUTXLA_MSG,(uint8_t *)OUTXLA_REC,2,HAL_MAX_DELAY);
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);

		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)OUTXHA_MSG,(uint8_t *)OUTXHA_REC,2,HAL_MAX_DELAY);
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);
		//ACC Y
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)OUTYLA_MSG,(uint8_t *)OUTYLA_REC,2,HAL_MAX_DELAY);
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);

		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)OUTYHA_MSG,(uint8_t *)OUTYHA_REC,2,HAL_MAX_DELAY);
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);
		//ACC Z
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)OUTZLA_MSG,(uint8_t *)OUTZLA_REC,2,HAL_MAX_DELAY);
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);

		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)OUTZHA_MSG,(uint8_t *)OUTZHA_REC,2,HAL_MAX_DELAY);
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);

		//Consolidation
		int16_t AX = (OUTXHA_REC[1]<<8) | OUTXLA_REC[1];
		int16_t AY = (OUTYHA_REC[1]<<8) | OUTYLA_REC[1];
		int16_t AZ = (OUTZHA_REC[1]<<8) | OUTZLA_REC[1];

		double ACC_X = ((double)AX * 0.488/1000);
		double ACC_Y = ((double)AY * 0.488/1000);
		double ACC_Z = ((double)AZ * 0.488/1000);
		//Operations for data

		Accel_Output[1] = ((atan2(ACC_Y, 1) *180/3.14)-X_Calib)*2;
		Accel_Output[2] = ((-atan2(-ACC_X, sqrt(ACC_Y*ACC_Y + 1*1))  *180/3.14)- Y_Calib)*2;
		Accel_Output[0] = sqrt((ACC_Z*ACC_Z)+ (ACC_X *ACC_X )+ (ACC_Y*ACC_Y))*9.81;
}

void Motor_transition()
{
	//PWM1_Duty ;
	//Conversion N to each motor
	float PWM1_Buff = U_input_Buff[0];
	float PWM2_Buff = U_input_Buff[1];
	float PWM3_Buff = U_input_Buff[2];
	float PWM4_Buff = U_input_Buff[3];

	//Conversion N to gram thrust
	PWM1_Buff = 102*PWM1_Buff;
	PWM2_Buff = 102*PWM2_Buff;
	PWM3_Buff = 102*PWM3_Buff;
	PWM4_Buff = 102*PWM4_Buff;

	//Conversion gram thrust to duty

	PWM1_Buff = 3.5 * PWM1_Buff;
	PWM2_Buff = 3.5 * PWM2_Buff;
	PWM3_Buff = 3.5 * PWM3_Buff;
	PWM4_Buff = 3.5 * PWM4_Buff;
	if (PWM1_Buff>=100)
	{
		PWM1_Buff = 100;
	}
	if (PWM2_Buff>=100)
	{
		PWM2_Buff = 100;
	}
	if (PWM3_Buff>=100)
	{
		PWM3_Buff = 100;
	}
	if (PWM4_Buff>=100)
	{
		PWM4_Buff = 100;
	}
	if (PWM1_Buff<=0)
	{
		PWM1_Buff = 0;
	}
	if (PWM2_Buff<=0)
	{
		PWM2_Buff = 0;
	}
	if (PWM3_Buff<=0)
	{
		PWM3_Buff = 0;
	}
	if (PWM4_Buff<=0)
	{
		PWM4_Buff = 0;
	}
	PWM1_Duty = (uint16_t)PWM1_Buff;
	PWM2_Duty = (uint16_t)PWM2_Buff;
	PWM3_Duty = (uint16_t)PWM3_Buff;
	PWM4_Duty = (uint16_t)PWM4_Buff;
}

float Matrix_operation(float FM[], float SM[], uint8_t length)
{
	//Function for operations on matrix
	float result =0;
	for (uint8_t a=0; a<=length; a++)
	{
		result = result + (FM[a]*SM[a]);
	}
	return result;
}
void Kalman_filter()
{
	//Kalman filter
	// Equation for Kalman
	//X_Prior = A_Lin_D *X_Prior_L + B_Lin_D * U_Input_Buff - (D_Lin_D*[1; 0; 0; 0]*Tp) + L *(Y_Output_Noise_Buff-C_Lin_D*X_Prior_L );
	float X_Prior_Kalman[6] = {0,0,0,0,0,0};
	X_Prior_Kalman[0] = Matrix_operation( A_Lin_D_1, X_Prior_Kalman_Buff, 5);
	X_Prior_Kalman[1] = Matrix_operation( A_Lin_D_2, X_Prior_Kalman_Buff, 5);
	X_Prior_Kalman[2] = Matrix_operation( A_Lin_D_3, X_Prior_Kalman_Buff, 5);
	X_Prior_Kalman[3] = Matrix_operation( A_Lin_D_4, X_Prior_Kalman_Buff, 5);
	X_Prior_Kalman[4] = Matrix_operation( A_Lin_D_5, X_Prior_Kalman_Buff, 5);
	X_Prior_Kalman[5] = Matrix_operation( A_Lin_D_6, X_Prior_Kalman_Buff, 5);

	X_Prior_Kalman[0] = X_Prior_Kalman[0] + Matrix_operation( B_Lin_D_1, U_input_Buff, 3);
	X_Prior_Kalman[1] = X_Prior_Kalman[1] + Matrix_operation( B_Lin_D_2, U_input_Buff, 3);
	X_Prior_Kalman[2] = X_Prior_Kalman[2] + Matrix_operation( B_Lin_D_3, U_input_Buff, 3);
	X_Prior_Kalman[3] = X_Prior_Kalman[3] + Matrix_operation( B_Lin_D_4, U_input_Buff, 3);
	X_Prior_Kalman[4] = X_Prior_Kalman[4] + Matrix_operation( B_Lin_D_5, U_input_Buff, 3);
	X_Prior_Kalman[5] = X_Prior_Kalman[5] + Matrix_operation( B_Lin_D_6, U_input_Buff, 3);

	X_Prior_Kalman[0] = X_Prior_Kalman[0] - D_Lin_D;

	float Buffer_L[6] = {0,0,0,0,0,0};
	Buffer_L[0] = Matrix_operation( C_Lin_D_1, X_Prior_Kalman_Buff, 5);
	Buffer_L[1] = Matrix_operation( C_Lin_D_2, X_Prior_Kalman_Buff, 5);
	Buffer_L[2] = Matrix_operation( C_Lin_D_3, X_Prior_Kalman_Buff, 5);
	Buffer_L[3] = Matrix_operation( C_Lin_D_4, X_Prior_Kalman_Buff, 5);
	Buffer_L[4] = Matrix_operation( C_Lin_D_5, X_Prior_Kalman_Buff, 5);
	Buffer_L[5] = Matrix_operation( C_Lin_D_6, X_Prior_Kalman_Buff, 5);

	Buffer_L[0] =  Y_Output_to_Kalman[0] - Buffer_L[0];
	Buffer_L[1] =  Y_Output_to_Kalman[1] - Buffer_L[1];
	Buffer_L[2] =  Y_Output_to_Kalman[2] - Buffer_L[2];
	Buffer_L[3] =  Y_Output_to_Kalman[3] - Buffer_L[3];
	Buffer_L[4] =  Y_Output_to_Kalman[4] - Buffer_L[4];
	Buffer_L[5] =  Y_Output_to_Kalman[5] - Buffer_L[5];


	float Kalman_transition[8] = {0,0,0,0,0,0};
	Kalman_transition[0] = Matrix_operation( L_Kalman_1,Buffer_L , 5);
	Kalman_transition[1] = Matrix_operation( L_Kalman_2,Buffer_L , 5);
	Kalman_transition[2] = Matrix_operation( L_Kalman_3,Buffer_L , 5);
	Kalman_transition[3] = Matrix_operation( L_Kalman_4,Buffer_L , 5);
	Kalman_transition[4] = Matrix_operation( L_Kalman_5,Buffer_L , 5);
	Kalman_transition[5] = Matrix_operation( L_Kalman_6,Buffer_L , 5);


	X_Prior_Kalman_Buff[0] = X_Prior_Kalman[0] + Kalman_transition[0];
	X_Prior_Kalman_Buff[1] = X_Prior_Kalman[1] + Kalman_transition[1];
	X_Prior_Kalman_Buff[2] = X_Prior_Kalman[2] + Kalman_transition[2];
	X_Prior_Kalman_Buff[3] = X_Prior_Kalman[3] + Kalman_transition[3];
	X_Prior_Kalman_Buff[4] = X_Prior_Kalman[4] + Kalman_transition[4];
	X_Prior_Kalman_Buff[5] = X_Prior_Kalman[5] + Kalman_transition[5];


}
void LQR_regulator()
{
	//LQR regulator
	//Equation
	//U_Input_Buff = -(K_LQR * (Y_Output_Filtered_Buff-X_Reg(:,s)));//Nastawy
	float LQR_Buff[6] = {0,0,0,0,0,0};
	LQR_Buff[0] = X_Prior_Kalman_Buff[0]- X_Reg_Var[0];
	LQR_Buff[1] = X_Prior_Kalman_Buff[1]- X_Reg_Var[1];
	LQR_Buff[2] = X_Prior_Kalman_Buff[2]- X_Reg_Var[2];
	LQR_Buff[3] = X_Prior_Kalman_Buff[3]- X_Reg_Var[3];
	LQR_Buff[4] = X_Prior_Kalman_Buff[4]- X_Reg_Var[4];
	LQR_Buff[5] = X_Prior_Kalman_Buff[5]- X_Reg_Var[5];

	U_input_Buff[0]= -Matrix_operation( K_LQR_1,LQR_Buff , 6);
	U_input_Buff[1]= -Matrix_operation( K_LQR_2,LQR_Buff , 6);
	U_input_Buff[2]= -Matrix_operation( K_LQR_3,LQR_Buff , 6);
	U_input_Buff[3]= -Matrix_operation( K_LQR_4,LQR_Buff , 6);


}

void Break_function()
{
	if (X_Prior_Kalman_Buff[3]>15 || X_Prior_Kalman_Buff[3]<-15)
	{
		Reset_function();
	}
	if (X_Prior_Kalman_Buff[4]>40 || X_Prior_Kalman_Buff[4]<-40)
	{
		Reset_function();
	}
	if (X_Prior_Kalman_Buff[5]>40 || X_Prior_Kalman_Buff[5]<-40)
	{
		Reset_function();
	}
}
void Drone_flight_function()
{
	if (ADC1_Variables[0]>1000)
	{
		//Receive and send accel information to user
		Accel_get_var();

		//Making X_Reg
		//X_Reg_Var[0] =0;
		//X_Reg_Var[1] =0;
		//X_Reg_Var[2] =0;
		//X_Reg_Var[3] =1;
		//X_Reg_Var[4] =20;
		//X_Reg_Var[5] =30;

		X_Reg_Var[0] =0;
		X_Reg_Var[1] =0;
		X_Reg_Var[2] =0;
		X_Reg_Var[3] =(H_Var_from_Android)/10;
		X_Reg_Var[4] =F_Var_from_Android;
		X_Reg_Var[5] =T_Var_from_Android;

		//Making input for regulator
		//Y_Output_to_Kalman[0]=0.5;
		//Y_Output_to_Kalman[1]=0;
		//Y_Output_to_Kalman[2]=0;
		//Y_Output_to_Kalman[3]=0;
		//Y_Output_to_Kalman[4]=10;
		//Y_Output_to_Kalman[5]=15;

		Y_Output_to_Kalman[0]=Accel_Output[0]-9.81;
		Y_Output_to_Kalman[1]=0;
		Y_Output_to_Kalman[2]=0;
		Y_Output_to_Kalman[3]=0;
		Y_Output_to_Kalman[4]=Accel_Output[2];
		Y_Output_to_Kalman[5]=Accel_Output[1];

		Kalman_filter();
		LQR_regulator();
		Motor_transition();

		//Saving results to android
		X_to_Android =0;
		Y_to_Android =0;
		Z_to_Android =X_Prior_Kalman_Buff[3];
		F_to_Android =X_Prior_Kalman_Buff[4];
		T_to_Android =X_Prior_Kalman_Buff[5];
		P_to_Android =0;

		Break_function();

		//Set PWM for user
		TIM2->CCR1 = PWM4_Duty;
		TIM2->CCR2 = PWM1_Duty;
		TIM2->CCR3 = PWM2_Duty;
		TIM2->CCR4 = PWM3_Duty;

		//TIM2->CCR1 = M1_Var_from_Android;
		//TIM2->CCR2 = M2_Var_from_Android;
		//TIM2->CCR3 = M3_Var_from_Android;
		//TIM2->CCR4 = M4_Var_from_Android;

		//Diode blink
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);

	}else
	{
		Reset_function();
	}
}
void Test_function()
{
	if (ADC1_Variables[0]>1000)
	{
		//Receive and send accel information to user
		Accel_get_var();
		F_to_Android =(int8_t)Accel_Output[1];
		T_to_Android =(int8_t)Accel_Output[2];
		X_to_Android =(int8_t)Accel_Output[0];

		//Set PWM for user
		TIM2->CCR1 = M4_Var_from_Android;
		TIM2->CCR2 = M1_Var_from_Android;
		TIM2->CCR3 = M2_Var_from_Android;
		TIM2->CCR4 = M3_Var_from_Android;

		//Diode blink
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);

	}else
	{
		Reset_function();
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM3){
		if (D_Var_from_Android == 1 && R_Var_from_Android == 0)
			{
				//Initialize drone flight regular function
				Drone_flight_function();
			}else if (D_Var_from_Android == 0 && R_Var_from_Android == 1)
			{
				//Initialize test function
				Test_function();
			}
	}
}

//Initialization of system
void Init_system()
{
	//Init diodes
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

	//Init ADC in DMA
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1_Variables, 2);

	//Init PWM
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	//Set PWM to 0
	TIM2->CCR1 = PWM1_Duty;
	TIM2->CCR2 = PWM2_Duty;
	TIM2->CCR3 = PWM3_Duty;
	TIM2->CCR4 = PWM4_Duty;

	//Init ACCEL + GYRO
	//INIT CTRL2
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)CTRL2_MSG_W,(uint8_t *)CTRL_REC,2,HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);

	//INIT CTRL1
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)CTRL1_MSG_W,(uint8_t *)CTRL_REC,2,HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);

	//INIT CTRL4
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)CTRL4_MSG_W,(uint8_t *)CTRL_REC,2,HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);
	//Calibration
	Accel_Calib();

	//Toogle pin
	HAL_Delay(1000);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

	//Waiting for commands from Android
	HAL_UART_Receive_DMA(&huart2, Received_from_Android, 37);

}

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  Init_system();
  /* USER CODE END 2 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 639;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 639;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI1_INT1_Pin|SPI1_INT2_Pin|LED1_Pin|LED2_Pin
                          |LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_INT1_Pin SPI1_INT2_Pin LED1_Pin LED2_Pin
                           LED3_Pin */
  GPIO_InitStruct.Pin = SPI1_INT1_Pin|SPI1_INT2_Pin|LED1_Pin|LED2_Pin
                          |LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
