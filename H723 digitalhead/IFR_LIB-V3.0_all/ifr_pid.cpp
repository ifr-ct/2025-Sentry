/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2024, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_pid.cpp
  * Version		: v2.3
  * Author		: LiuHao Lijiawei Albert
  * Date		: 2023-10-30
  * Description	:

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "ifr_pid.h"

/* Private variables -------------------------------------------------*/
/**
  * @概述	PID参数初始化。
  * @参数1	Kp
  * @参数2	Ki
  * @参数3	Kd
  * @参数4	输出最大值
  * @参数5	误差最大值
  * @参数6	积分最大值
  * @参数6	死区
  * @返回值 void
  */
void IFR_PID::PID_Init(float kp,float ki,float kd,float output_max,float error_max,float integral_max,float dead_zone)
{
	Kp=kp;Ki=ki;Kd=kd;
	Output_Max=output_max;
	Error_Max=error_max;
	Integral_Max=integral_max;
	Dead_Zone=dead_zone;
}
/**
  * @概述	位置式PID计算函数。
  * @参数1	目标值
  * @参数2	现在值
  * @返回值 PID计算的输出
  */
float IFR_PID::Positional_PID(float Target_Value,float Actual_Value)
{
	Error = Target_Value-Actual_Value;
	if(Error >  Error_Max) Error =  Error_Max;
	if(Error < -Error_Max) Error = -Error_Max;
	if(abs(Error) <= Dead_Zone) Error=0;

	Integral=Integral+Error;
//	if(Error*Integral<0) 				Integral/=2;
	if(Integral > Integral_Max) Integral=Integral_Max;
	if(Integral < -Integral_Max)Integral=-Integral_Max;

	Diffrential=Error-Last_Error;

	Output=Kp*Error+Ki*Integral+Kd*Diffrential;
	if(Output> Output_Max) Output=Output_Max;
	if(Output<-Output_Max) Output=-Output_Max;

	Last_Error=Error;

	return Output;
}

/**
  * @概述	增量式PID计算函数。
  * @参数1	目标值
  * @参数2	现在值
  * @返回值 PID计算的输出
  */
float IFR_PID::Incremental_PID(float Target_Value,float Actual_Value)
{
	Error = Target_Value-Actual_Value;

	if(Error >  Error_Max) Error =  Error_Max;
	if(Error < -Error_Max) Error = -Error_Max;
	if(abs(Error) <= Dead_Zone) Error=0;

	Output+=Kp*(Error-Last_Error)+Ki*Error+Kd*(Error-2*Last_Error+Last_Last_Error);

	if(Output>Output_Max)Output=Output_Max;
	if(Output<-Output_Max)Output=-Output_Max;

	Last_Last_Error=Last_Error;
	Last_Error = Error;

	return Output;
}
/**
  * @概述 	PID最大输出设置
  * @参数1	最大输出设置值
  * @返回值 void
  */
void IFR_PID::OutputMax_Set(float output_max)
{
	Output_Max=output_max;
}
/**
  * @概述		获取输出值
  * @返回值 输出值(float)
  */
float IFR_PID::Output_Get(void)
{
	return Output;
}

