/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2024, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_motor.h
  * Version		: v2.3
  * Author		: LiuHao Lijiawei Albert
  * Date		: 2023-09-28
  * Description	:

  *********************************************************************
  */

#ifndef __IFR_MOTOR_H_
#define __IFR_MOTOR_H_

/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif

#ifdef __cplusplus
 }
#endif
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ifr_pid.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
//#define USE_DJI_MOTOR 1
#define USE_MF9015_MOTOR 0

typedef struct
{
  int16_t Speed;
  uint16_t Angle;
  int32_t Abs_Angle;
  uint8_t Temperature;
  int16_t Electric;
  uint16_t Last_Angle;
}Motor_Pos_Info;

class IFR_DJI_Motor
{
	public:
		uint16_t Get_DJIMotor_Num(void);
		int16_t Get_DJIMotor_Output(void);
		void Set_DJIMotor_Output(int16_t set_output);
		Motor_Pos_Info Pos_Info;
		void Motor_PosInit(int32_t PosInit);
		void DJI_Motor_Analysis(uint8_t *Data,uint32_t stdid);

	protected:
		int16_t output;
		uint16_t Motor_Num;
		IFR_DJI_Motor(uint16_t Num);
};

class IFR_Speed_Motor:public IFR_DJI_Motor
{
	public:
		IFR_Speed_Motor(uint16_t Num);
		void Motor_Speed_Set(int16_t Speed_Tar);
		IFR_PID Speed_PID;
	private:
		int16_t Tar_Speed;
};

class IFR_Pos_Motor:public IFR_DJI_Motor
{
	public:
		IFR_Pos_Motor(uint16_t Num);
		IFR_Pos_Motor(float (*Offset_MotoFunc)(int32_t Motor_Tar),uint16_t Num);
		void Motor_AbsPos_Set(int32_t Pos_Tar);
		void Motor_Pos_Set(int32_t Pos_Tar);
		int32_t Motor_TarPos_Get(void) {return Tar_Pos;}
		int32_t Get_Tar_Pos(void) {return Tar_Pos;}

		void Motor_Speed_Set(int16_t Speed_Tar);

		IFR_PID Pos_PID;
		IFR_PID Speed_PID;
	private:
		int32_t Tar_Pos;
		float (*Motor_Offset_MotoFunc)(int32_t Motor_Tar);
};

class IFR_GyroControl_Motor:public IFR_DJI_Motor
{
	public:
		IFR_GyroControl_Motor(uint16_t Num);
		IFR_GyroControl_Motor(float (*Offset_MotoFunc)(int32_t Motor_Tar),uint16_t Num);

		void IMU_Data_Get(float IMU_Speed,float IMU_Angle);
		void Motor_Angle_Set(int32_t Angle_Tar);
		void Fliter_Motor_Angle_Set(int32_t Angle_Tar);
		int32_t Motor_Angle_Get(void) {return Tar_Angle;}
		int32_t Get_Tar_Angle(void) {return Tar_Angle;}

		float IMU_Speed;
		float IMU_Angle;

		float Angle_Kd;

		IFR_PID Angle_PID;
		IFR_PID Speed_PID;

	private:
		int32_t Tar_Angle;
		float (*Motor_Offset_MotoFunc)(int32_t Motor_Tar);
};

#if USE_MF9015_MOTOR

/***********MF9015 Motor*************/
#define MF_COMMAND_R_PID 											0x30
#define MF_COMMAND_W_PID2RAM									0x31
#define MF_COMMAND_W_PID2ROM									0x32
#define MF_COMMAND_R_ACCEL										0x33
#define MF_COMMAND_W_ACCEL2RAM								0x34
#define MF_COMMAND_R_ENCODER									0x90//编码器
#define MF_COMMAND_W_ENCODER_OFFSET2RAM				0x91
#define MF_COMMAND_W_ENCODER_OFFSET2ROM				0x19
#define MF_COMMAND_R_ABS_ANGLE								0x92
#define MF_COMMAND_R_MOTOR_ANGLE							0x94
#define MF_COMMAND_R_MOTOR_STATE1							0x9A//温度，电压，错误标志
#define MF_COMMAND_W_MOTOR_CLEAR_ERROR_STATE	0x9B//清除错误状态,并返回温度，电压，错误标志
#define MF_COMMAND_R_MOTOR_STATE2							0x9C//温度，电压，转速，位置

/*******************Ctrl**************************/
#define MF_COMMAND_C_MOTOR_OFF								0x80
#define MF_COMMAND_C_MOTOR_STOP								0x81
#define MF_COMMAND_C_MOTOR_START							0x88
#define MF_COMMAND_C_MOTOR_Iq									0xA1//转矩控制
#define MF_COMMAND_C_MOTOR_Speed							0xA2//速度控制

typedef struct
{
	uint8_t AnglePid_Kp;
	uint8_t AnglePid_Ki;

	uint8_t SpeedPid_Kp;
	uint8_t SpeedPid_Ki;

	uint8_t IqPid_Kp;
	uint8_t IqPid_Ki;

}MFMotor_PidInfoTypedef;

typedef struct
{
	uint8_t AnglePid_Kp;
	uint8_t AnglePid_Ki;

	uint8_t SpeedPid_Kp;
	uint8_t SpeedPid_Ki;

	uint8_t IqPid_Kp;
	uint8_t IqPid_Ki;

}MFMotor_AccelInfoTypedef;

typedef enum
{

	Normal_State 						= 0x00,
	LowVoltage_State  			= 0x01,
	HighTemperature_State 	= 0x08,
	Tem_Vol_Error_State			= 0x09,

}ErrorStateEnumTypedef;

typedef struct
{
	MFMotor_PidInfoTypedef 	MFMotor_PidInfo;
	int32_t									MFMotor_Accel;
	uint16_t								MFMotor_Encoder;//0-16383
	uint16_t 								MFMotor_Angle;//0.01d, 0 ~ (36000-1)
	int64_t									MFMotor_Abs_Angle;//0.01d
	int8_t									MFMotor_Temperature;
	uint16_t								MFMotor_Voltage;
	int16_t									MFMotor_Iq;//转矩电流，-2048~2048 -> -33A~33A
	int16_t									MFMotor_Speed;//1dps
	ErrorStateEnumTypedef		MFMotor_ErrorState;

}MFMotor_InfoTypedef;

class MF_MotorClassDef
{
	public:
		MF_MotorClassDef(uint16_t MFMotor_Id){MFMotor_Num = MFMotor_Id;}
		MFMotor_InfoTypedef MFMotor_Info;
		uint16_t Get_MotorId(void){ return MFMotor_Num; }
		int16_t Get_MotorOutput(void){ return MFMotor_Output; }
		void MF_Motor_Analysis(uint8_t *pData, uint16_t MFStdId);
		void Set_Output(int16_t MF_OutputTar);

	private:
		uint16_t 						MFMotor_Num;
		int16_t							MFMotor_Output;

};

#endif	//#if USE_MF9015_MOTOR

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#endif

