/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
typedef struct {
    float LPGCurve[3];
    float COCurve[3];
    float SmokeCurve[3];
    float Ro;
    float values[3];
    int lastReadTime;
} MQ2;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void MQ2_create(MQ2 *sensor);
void MQ2_begin(MQ2 *sensor);
void MQ2_close(MQ2 *sensor);
bool MQ2_checkCalibration(MQ2 *sensor);
float* MQ2_read(MQ2 *sensor, bool print);
float MQ2_readLPG(MQ2 *sensor);
float MQ2_readCO(MQ2 *sensor);
float MQ2_readSmoke(MQ2 *sensor);
float MQ2_MQResistanceCalculation(int raw_adc);
float MQ2_MQCalibration(MQ2 *sensor);
float MQ2_MQRead(MQ2 *sensor);
float MQ2_MQGetPercentage(float *pcurve, MQ2 *sensor);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

// define the load resistance on the board, in kilo ohms
# define RL_VALUE 5.0
// given constant
# define RO_CLEAN_AIR_FACTOR 9.83

// reads 10 times the sensor every 50ms and takes the average
// NOTE: it is encouraged to take more samples during the calibration
# define CALIBARAION_SAMPLE_TIMES 10
# define CALIBRATION_SAMPLE_INTERVAL 50

// reads 5 times the sensor every 50ms and takes the average
# define READ_SAMPLE_TIMES 5
# define READ_SAMPLE_INTERVAL 50

// 10s, time elapsed before new data can be read.
# define READ_DELAY 10000

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
