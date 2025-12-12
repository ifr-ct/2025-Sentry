/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2022, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_pwm.h
  * Version		: v2.1
  * Author		: LiuHao Lijiawei
  * Date		: 2022-09-27
  * Description	:

  *********************************************************************
  */

#ifndef __IFR_PWM_H_
#define __IFR_PWM_H_

/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif

#ifdef __cplusplus
 }
#endif
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#ifdef HAL_TIM_MODULE_ENABLED
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

class IFR_OPWM_ClassDef
{
	public:
		void OPWM_Init(TIM_HandleTypeDef *htim, uint32_t Channel);
		void OPWM_SetCompare(uint16_t __Compare);
	private:
		TIM_HandleTypeDef *__htim;
		uint32_t __Channel;
};
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void IFR_PWM_TIM_Overflow_Callback(TIM_HandleTypeDef *htim);
void IFR_TIM_PWMCapture_Callback(TIM_HandleTypeDef *htim);
static uint8_t IFR_PWMChannel_ID_GET(TIM_HandleTypeDef *htim);
static uint8_t IFR_PWMTIM_ID_GET(TIM_HandleTypeDef *htim);
/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#endif
#endif

