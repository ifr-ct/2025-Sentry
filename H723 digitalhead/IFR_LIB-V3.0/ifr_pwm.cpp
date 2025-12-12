/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2024, China,IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_pwm.cpp
  * Version		: v2.1
  * Author		: LiuHao Lijiawei Albert
  * Date		: 2022-09-27
  * Description	:

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "ifr_pwm.h"

/* Private variables -------------------------------------------------*/
#ifdef HAL_TIM_MODULE_ENABLED//如果底下是虚的说明你没使用任何定时器
/**
  * @概述	DJI遥控器解析函数，无键盘数据解析。
  * @参数1	定时器句柄
  * @参数2  定时器通道
  * @返回值 void
  */
void IFR_OPWM_ClassDef::OPWM_Init(TIM_HandleTypeDef *htim, uint32_t Channel)
{
	__htim = htim; __Channel = Channel;
	HAL_TIM_PWM_Start(htim,Channel);
}
/**
  * @概述	修改占空比。
  * @参数1	比较值
  * @返回值 void
  */
void IFR_OPWM_ClassDef::OPWM_SetCompare(uint16_t __Compare)
{
	__HAL_TIM_SET_COMPARE(__htim,__Channel,__Compare);
}

#endif
