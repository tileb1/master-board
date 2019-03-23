/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include <math.h>

#include "CAN_communication.h"
#include "Misc/Common.h"
#include "Misc/rocket_constants.h"
#include "led.h"
extern CAN_HandleTypeDef hcan1;
extern void TK_state_machine(void const * argument);
extern float IMUb[6];
extern float zdata[4];
volatile int IMU_avail = 0;
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
/* USER CODE BEGIN Variables */
IMU_data IMU_buffer[CIRC_BUFFER_SIZE] CCMRAM;
BARO_data BARO_buffer[CIRC_BUFFER_SIZE] CCMRAM;
osThreadId state_machineHandle;
osThreadId kalman_handle;
extern void TK_kalman(void const * argument);
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId readCANHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
inline float altitudeFromPressure(float pressure_hPa)
{
	return 44330 * (1.0 - pow (pressure_hPa / ADJUSTED_SEA_LEVEL_PRESSURE, 0.1903));
}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartReadCAN(void const * argument);

extern void MX_FATFS_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of readCAN */
  osThreadDef(readCAN, StartReadCAN, osPriorityNormal, 0, 128);
  readCANHandle = osThreadCreate(osThread(readCAN), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* definition and creation of state_machine */
  osThreadDef(state_machine, TK_state_machine, osPriorityHigh, 0, 1024);
  state_machineHandle = osThreadCreate(osThread(state_machine), NULL);

  /* definition and creation of kalman */
  osThreadDef(kalman, TK_kalman, osPriorityHigh, 0, 1024);
  kalman_handle = osThreadCreate(osThread(kalman), NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for FATFS */
  MX_FATFS_Init();

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartReadCAN */
/**
* @brief Function implementing the readCAN thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadCAN */
void StartReadCAN(void const * argument)
{
  /* USER CODE BEGIN StartReadCAN */
  /* Infinite loop */
	BARO_data* new_baro_data = 0;
	IMU_data* new_imu_data = 0;
	uint32_t timestamp = HAL_GetTick();
	uint32_t a = HAL_GetTick();
	uint32_t last_imu_meas = HAL_GetTick();
	uint8_t oldState = currentState;
	while (1) {
		// Send the state of the rocket
		a = HAL_GetTick();
		if (a - timestamp > 2000) {
			setFrame(0, currentState, a);
			timestamp = a;
		}
		if (currentState != oldState) {
			setFrame(0, currentState, a);
			oldState = currentState;
		}
		if (readFrame() > 0) {
			if (current_msg.id_CAN == ID_GPS_SENSOR) {
				if (current_msg.id == DATA_ID_ALTITUDE) {
					new_baro_data = &BARO_buffer[(currentBaroSeqNumber + 1) % CIRC_BUFFER_SIZE];
					new_imu_data = &IMU_buffer[(currentImuSeqNumber + 1) % CIRC_BUFFER_SIZE];
					new_baro_data->pressure = ((float32_t) ((int32_t) current_msg.data)) / 100; // convert from Pa to hPa
					new_baro_data->altitude = altitudeFromPressure(new_baro_data->pressure);
					zdata[3] = new_baro_data->altitude;

					// mock gps estimate
					zdata[0] = 0;
					zdata[1] = 0;
					zdata[2] = zdata[3];
				}
				//IMUb[] acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z //zdata gps_x, gps_y, gps_z, alt
				else if (current_msg.id == DATA_ID_ACCELERATION_X) {
					new_imu_data->acceleration.x = ((float32_t) ((int32_t) current_msg.data)) / 1000 * 9.81; // convert from m-g to g to m/s;
					IMUb[0] = new_imu_data->acceleration.x;
				}
				else if (current_msg.id == DATA_ID_ACCELERATION_Y) {
					new_imu_data->acceleration.y = ((float32_t) ((int32_t) current_msg.data)) / 1000 * 9.81; // convert from m-g to g to m/s;
					IMUb[1] = new_imu_data->acceleration.y;
				}
				else if (current_msg.id == DATA_ID_ACCELERATION_Z) {
					new_imu_data->acceleration.z = ((float32_t) ((int32_t) current_msg.data)) / 1000 * 9.81; // convert from m-g to g to m/s;
					IMUb[2] = new_imu_data->acceleration.z;
				}
				else if (current_msg.id == DATA_ID_GYRO_X) {
					new_imu_data->gyro_rps.x = current_msg.data;
					IMUb[3] = 0*current_msg.data; // discard gyro
				}
				else if (current_msg.id == DATA_ID_GYRO_Y) {
					new_imu_data->gyro_rps.y = current_msg.data;
					IMUb[4] = 0*current_msg.data; // discard gyro
				}
				else if (current_msg.id == DATA_ID_GYRO_Z) {
					new_imu_data->gyro_rps.z = current_msg.data;
					IMUb[5] = 0*current_msg.data; // discard gyro
					if (HAL_GetTick() - last_imu_meas > 100) { // 10 Hz update
						IMU_avail = 1; // assume gyro is the last data
						last_imu_meas = HAL_GetTick();
					}
				    currentBaroSeqNumber++;
				    currentImuSeqNumber++;
				}
			}

		}
	}
  /* USER CODE END StartReadCAN */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
