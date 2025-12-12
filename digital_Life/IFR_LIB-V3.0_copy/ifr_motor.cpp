/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2022, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_motor.cpp
  * Version		: v2.1
  * Author		: LiuHao Lijiawei
  * Date		: 2022-09-28
  * Description	:

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "ifr_motor.h"
#include "function.h"
/* Private variables -------------------------------------------------*/
 /**
  * @概述	DJI电机实例化的构造函数，初始化电机ID。
  * @参数1	电机ID
  */
IFR_DJI_Motor::IFR_DJI_Motor(uint16_t Num)
{
	Motor_Num = Num;
	Pos_Info.Last_Angle = 65535;
}
 /**
  * @概述	电机标定。
  * @参数1	电机标定的位置
  * @返回值 void
  */
void IFR_DJI_Motor::Motor_PosInit(int32_t PosInit)
{
	Pos_Info.Abs_Angle = PosInit;
	Pos_Info.Last_Angle = 65535;
}
uint8_t rxdata_209[8];
 /**
  * @概述	电机数据解析函数。
  * @参数1	数据头指针
  * @参数2	CAN消息Stdid
  * @返回值 void
  */
void IFR_DJI_Motor::DJI_Motor_Analysis(uint8_t *Data,uint32_t stdid)
{
	if(stdid == Motor_Num)
	{
		Pos_Info.Angle=(uint16_t)Data[0]<<8|Data[1];
		Pos_Info.Speed=(uint16_t)Data[2]<<8|Data[3];
		Pos_Info.Electric=(uint16_t)Data[4]<<8|Data[5];
		Pos_Info.Temperature=Data[6];

		if(Pos_Info.Last_Angle == 65535) Pos_Info.Last_Angle = Pos_Info.Angle;
		if (Pos_Info.Speed >5 || Pos_Info.Speed <-5)
		{
			int16_t Error=Pos_Info.Angle-Pos_Info.Last_Angle;
			Pos_Info.Abs_Angle += Error;
			if (Error < -4096) Pos_Info.Abs_Angle += 8192;
			else if (Error > 4096)  Pos_Info.Abs_Angle -= 8192;
		}Pos_Info.Last_Angle=Pos_Info.Angle;
	}
}
 /**
  * @概述	获取该电机ID。
  * @返回值 (uint16_t)电机号
  */
uint16_t IFR_DJI_Motor::Get_DJIMotor_Num(void)
{
	return Motor_Num;
}
 /**
  * @概述	获取该电机输出。
  * @返回值 (int16_t)电机输出
  */
int16_t IFR_DJI_Motor::Get_DJIMotor_Output(void)
{
	return output;
}
 /**
  * @概述	设置电机输出值。
  * @参数1	输出设置值
  * @返回值 void
  */
void IFR_DJI_Motor::Set_DJIMotor_Output(int16_t set_output)
{
	output = set_output;
}
 /**
  * @概述	速度环电机实例化的构造函数，初始化电机ID。
  * @参数1	电机ID
  */
IFR_Speed_Motor::IFR_Speed_Motor(uint16_t Num):IFR_DJI_Motor(Num){ }
 /**
  * @概述	电机速度设置。
  * @参数1	速度目标值
  * @返回值 void
  */
void IFR_Speed_Motor::Motor_Speed_Set(int16_t Speed_Tar)
{
	Tar_Speed = Speed_Tar;
	output = Speed_PID.Positional_PID(Speed_Tar,Pos_Info.Speed);
}
 /**
  * @概述	位置环电机实例化的构造函数，初始化电机ID。
  * @参数1	电机ID
  */
IFR_Pos_Motor::IFR_Pos_Motor(uint16_t Num):IFR_DJI_Motor(Num){}
 /**
  * @概述	位置环电机实例化的构造函数，初始化电机ID和重力补偿参数。
  * @参数1	重力补偿函数指针
  * @参数2	电机ID
  */
IFR_Pos_Motor::IFR_Pos_Motor(float (*Offset_MotoFunc)(int32_t Motor_Tar),uint16_t Num):IFR_DJI_Motor(Num)
{
	Motor_Offset_MotoFunc = Offset_MotoFunc;
}
 /**
  * @概述	电机绝对位置设置。
  * @参数1	绝对位置目标值
  * @返回值 void
  */
void IFR_Pos_Motor::Motor_AbsPos_Set(int32_t Pos_Tar)
{
	Tar_Pos = Pos_Tar;
	int16_t Speed_Tar = Pos_PID.Positional_PID(Tar_Pos,Pos_Info.Abs_Angle);
	output = Speed_PID.Positional_PID(Speed_Tar,Pos_Info.Speed);
	if(Motor_Offset_MotoFunc !=NULL) output +=(*Motor_Offset_MotoFunc)(Tar_Pos);
}
 /**
  * @概述	电机位置设置。
  * @参数1	位置目标值
  * @返回值 void
  */
void IFR_Pos_Motor::Motor_Pos_Set(int32_t Pos_Tar)
{
	Tar_Pos = Pos_Tar;
	int16_t Speed_Tar = Pos_PID.Positional_PID(Tar_Pos,Pos_Info.Angle);
	output = Speed_PID.Positional_PID(Speed_Tar,Pos_Info.Speed);
	if(Motor_Offset_MotoFunc !=NULL) output +=(*Motor_Offset_MotoFunc)(Tar_Pos);
}
 /**
  * @概述	电机速度设置。
  * @参数1	速度目标值
  * @返回值 void
  */
void IFR_Pos_Motor::Motor_Speed_Set(int16_t Speed_Tar)
{
	output = Speed_PID.Positional_PID(Speed_Tar,Pos_Info.Speed);
}

 /**
  * @概述	位置环电机实例化的构造函数，初始化电机ID。
  * @参数1	电机ID
  */
IFR_GyroControl_Motor::IFR_GyroControl_Motor(uint16_t Num):IFR_DJI_Motor(Num){Angle_Kd=0;}
 /**
  * @概述	角度环电机实例化的构造函数，初始化电机ID和重力补偿参数。
  * @参数1	重力补偿函数指针
  * @参数2	电机ID
  */
IFR_GyroControl_Motor::IFR_GyroControl_Motor(float (*Offset_MotoFunc)(int32_t Motor_Tar),uint16_t Num):IFR_DJI_Motor(Num)
{
	Motor_Offset_MotoFunc = Offset_MotoFunc;
	Angle_Kd=0;
}

 /**
  * @概述	角度环角度数据更新。
  * @参数1	陀螺仪角速度信息
  * @参数2	陀螺仪角度信息
  * @返回值 void
  */
void IFR_GyroControl_Motor::IMU_Data_Get(float imu_speed,float imu_angle)
{
	IMU_Speed = imu_speed;
	IMU_Angle = imu_angle;
}
 /**
  * @概述	电机角度设置。
  * @参数1	电机目标角度设置
  * @返回值 void
  */
void IFR_GyroControl_Motor::Motor_Angle_Set(int32_t Angle_Tar)
{
	Tar_Angle = Angle_Tar;
	float Speed_Tar = Angle_PID.Positional_PID(Tar_Angle,IMU_Angle) + IMU_Speed * Angle_Kd;
	output = Speed_PID.Positional_PID(Speed_Tar,IMU_Speed);
	if(Motor_Offset_MotoFunc !=NULL) output +=(*Motor_Offset_MotoFunc)(Pos_Info.Angle);
}

/******************MF9015**********************/
#if USE_MF9015_MOTOR

 /**
  * @概述	电机数据解析函数。
  * @参数1	数据头指针
  * @参数2	CAN消息Stdid
  * @返回值 void
  */
void MF_MotorClassDef::MF_Motor_Analysis(uint8_t *pData, uint16_t MFStdId)
{
	uint8_t mf_command_id = *pData;
	if (MFStdId != MFMotor_Num)return;
	switch (mf_command_id)
	{
		case MF_COMMAND_R_ABS_ANGLE:

			MFMotor_Info.MFMotor_Abs_Angle  = *(int32_t*)pData;
			MFMotor_Info.MFMotor_Abs_Angle |= ((int64_t)(*(int32_t*)(pData+4)))<<32;
			MFMotor_Info.MFMotor_Abs_Angle >>= 8;
			break;

		case MF_COMMAND_R_ACCEL:
			MFMotor_Info.MFMotor_Accel = *((int32_t*)(pData+4));
			break;

		case MF_COMMAND_R_ENCODER:
			MFMotor_Info.MFMotor_Encoder = *((uint16_t*)(pData+2));
			break;

		case MF_COMMAND_R_MOTOR_ANGLE:
			MFMotor_Info.MFMotor_Angle = *((uint32_t*)(pData+4));
			break;

		case MF_COMMAND_R_PID:
			MFMotor_Info.MFMotor_PidInfo.AnglePid_Kp = pData[2];
			MFMotor_Info.MFMotor_PidInfo.AnglePid_Ki = pData[3];
			MFMotor_Info.MFMotor_PidInfo.SpeedPid_Kp = pData[4];
			MFMotor_Info.MFMotor_PidInfo.SpeedPid_Ki = pData[5];
			MFMotor_Info.MFMotor_PidInfo.IqPid_Kp	  = pData[6];
			MFMotor_Info.MFMotor_PidInfo.IqPid_Ki	  = pData[7];
			break;

		case MF_COMMAND_R_MOTOR_STATE1:
		case MF_COMMAND_W_MOTOR_CLEAR_ERROR_STATE:
			MFMotor_Info.MFMotor_Temperature = (int8_t)pData[1];
			MFMotor_Info.MFMotor_Voltage = *(pData+3);
			MFMotor_Info.MFMotor_ErrorState = (ErrorStateEnumTypedef)pData[7];
			break;

		case MF_COMMAND_R_MOTOR_STATE2:
		case MF_COMMAND_C_MOTOR_Iq:
		case MF_COMMAND_C_MOTOR_Speed:
			MFMotor_Info.MFMotor_Temperature = (int8_t)pData[1];
			MFMotor_Info.MFMotor_Iq = *((int16_t*)(pData+2));
			MFMotor_Info.MFMotor_Speed = *((int16_t*)(pData+4));
			MFMotor_Info.MFMotor_Encoder = *((uint16_t*)(pData+6));
			break;

		default:
			return;
	}
}

void MF_MotorClassDef::Set_Output(int16_t MF_OutputTar)
{
	if (MF_OutputTar > 2000)
	{
		MF_OutputTar = 2000;
	}
	else if (MF_OutputTar < -2000)
	{
		MF_OutputTar = -2000;
	}
	MFMotor_Output = MF_OutputTar;

}

#endif //#if USE_MF9015_MOTOR
