/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2022, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_gpio.cpp
  * Version		: v2.1
  * Author		: LiuHao Lijiawei
  * Date		: 2022-09-27
  * Description	: IFR库GPIO类封装

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "ifr_gpio.h"

/* Private variables -------------------------------------------------*/
/**
  * @概述	GPIO实例化的构造函数，初始化GPIO对象信息。
  * @参数1	GPIOx where x can be (A..K) to select the GPIO peripheral
  *         for STM32F429X device or x can be (A..I) to select the GPIO
  *         peripheral for STM32F40XX and STM32F427X devices.
  * @参数2  GPIO_Pin specifies the port bit to be written.
  *         This parameter can be one of GPIO_PIN_x where x can be (0..15).
  */
IFR_GPIO_ClassDef::IFR_GPIO_ClassDef(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	PinState  = HAL_GPIO_ReadPin(GPIOX,GPIO_PIN);
	GPIOX = GPIOx;
	GPIO_PIN = GPIO_Pin;
}
/**
  * @概述	调整GPIO输出为高电平。
  * @返回值 void
  */
void IFR_GPIO_ClassDef::High(void)
{
	PinState  = GPIO_PIN_SET;
	HAL_GPIO_WritePin(GPIOX,GPIO_PIN,GPIO_PIN_SET);
}
/**
  * @概述	调整GPIO输出为低电平。
  * @返回值 void
  */
void IFR_GPIO_ClassDef::Low(void)
{
	PinState  = GPIO_PIN_RESET;
	HAL_GPIO_WritePin(GPIOX,GPIO_PIN,GPIO_PIN_RESET);
}
/**
  * @概述	翻转GPIO输出。
  * @返回值 void
  */
void IFR_GPIO_ClassDef::Turn(void)
{
	HAL_GPIO_TogglePin(GPIOX,GPIO_PIN);
	PinState  =  HAL_GPIO_ReadPin(GPIOX,GPIO_PIN);
}
/**
  * @概述	读取GPIO输入。
  * @返回值 GPIO_PinState(GPIO_PIN_SET/GPIO_PIN_RESET)
  */
GPIO_PinState IFR_GPIO_ClassDef::Read(void)
{
	PinState = HAL_GPIO_ReadPin(GPIOX,GPIO_PIN);
	return PinState;
}

