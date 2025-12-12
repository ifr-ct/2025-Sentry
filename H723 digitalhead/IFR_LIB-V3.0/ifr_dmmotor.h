#ifndef __IFR_DMMOTOR_H_
#define __IFR_DMMOTOR_H_

#include "main.h"
#include "ifr_pid.h"

#ifndef USE_DM_MOTOR
#define USE_DM_MOTOR 	(1)
#endif

/*******************************/
#define MasterCanID 0x00

//#define PMAX  3.1415927f
#define PMAX  12.5f
#define VMAX  30
#define TMAX  10

#define KpMAX 500
#define KdMAX 5

typedef enum
{
	DMState_Disable = 0x00,
	DMState_Enable = 0x01,
	DMError_overV = 0x08,
	DMError_qianya = 0x09,
	DMError_overI = 0x0A,
	DMError_MosHot = 0x0B,
	DMError_MotorHot = 0x0C,
	DMError_loss	= 0x0D,
	DMError_overF = 0x0E,
}ErrCodeTypedef;

typedef enum
{
	DM_Enable = 0xFC,//使能(上电后800ms)
	DM_Disable = 0xFD,//失能
	DM_SaveZero = 0xFE,//保存零点
	DM_ClearError = 0xFB,
}DM_Command_t;

typedef struct
{
	//uint8_t Motor_ID;
	ErrCodeTypedef DM_State;
	
	float Angle;//表示电机的位置信息
	float Speed;//表示电机的速度信息
	float Torque;//表示电机的扭矩信息
	uint8_t T_MOS;//表示驱动上MOS 的平均温度，单位℃
	uint8_t T_Rotor;//表示电机内部线圈的平均温度，单位℃
	
	float Last_Angle;
	float Abs_Angle;//不建议使用
}DM_Info_t;

class IFR_DM_Motor
{
	public:
		inline float uint_to_float(int x_int, float x_max, int bits);
		inline int float_to_uint(float x, float x_max, int bits);
		inline float uint_to_float(int x_int, float x_min, float x_max, int bits);
		inline int float_to_uint(float x, float x_min, float x_max, int bits);
	
		uint8_t Get_DMMotor_Id(void);
		float Get_DMMotor_Output(void);
		void Set_DMMotor_Output(float set_output);
		DM_Info_t DM_Info;
		void DM_Motor_PosInit(float PosInit);
		void DM_Motor_Analysis(uint8_t *Data,uint32_t stdid);
		void (*DM_Error_Callback)(void);
		
		void DM_Motor_Command(DM_Command_t Command);
	
		uint8_t pTxCommandData[8];
	
		uint16_t get_t_ff(void){return _t_ff;}
	protected:
		float  _output;//扭矩
		uint16_t _t_ff;//转化后的数据
		float _t_max;//最大扭矩
		uint8_t _motor_id;
		IFR_DM_Motor(uint8_t MotorId);
		IFR_DM_Motor(uint8_t MotorId, float T_Max);
};

class IFR_Speed_DMMotor:public IFR_DM_Motor
{
	public:
		IFR_Speed_DMMotor(uint8_t Motor_id);
		IFR_Speed_DMMotor(uint8_t Motor_id, float T_Max);
		void Motor_Speed_Set(float Speed_Tar);
		IFR_PID Speed_PID;
		float Get_TarSpeed(void) {return Tar_Speed;}
	private:
		float Tar_Speed;
};

class IFR_Pos_DMMotor:public IFR_Speed_DMMotor
{
	public:
		IFR_Pos_DMMotor(uint8_t Motor_id);
		IFR_Pos_DMMotor(float (*Offset_MotoFunc)(float Motor_Tar),uint8_t Motor_id);
		IFR_Pos_DMMotor(uint8_t Motor_id, float T_Max);
		IFR_Pos_DMMotor(float (*Offset_MotoFunc)(float Motor_Tar),uint8_t Motor_id, float T_Max);
	
		void Motor_AbsPos_Set(float Pos_Tar);
		void Motor_Pos_Set(float Pos_Tar);
		float Motor_TarPos_Get(void) {return Tar_Pos;}
		float Get_Tar_Pos(void) {return Tar_Pos;}

		IFR_PID Pos_PID;

	private:
		float Tar_Pos;
		float (*Motor_Offset_MotoFunc)(float Motor_Tar);
};

class IFR_GyroControl_DMMotor:public IFR_Speed_DMMotor
{
	public:
		IFR_GyroControl_DMMotor(uint8_t Motor_id);
		IFR_GyroControl_DMMotor(float (*Offset_MotoFunc)(float Motor_Tar),uint8_t Motor_id);
		IFR_GyroControl_DMMotor(uint8_t Motor_id, float T_Max);
		IFR_GyroControl_DMMotor(float (*Offset_MotoFunc)(float Motor_Tar),uint8_t Motor_id, float T_Max);

		void IMU_Data_Get(float IMU_Speed,float IMU_Angle);
		void Motor_Angle_Set(float Angle_Tar);

		float Motor_Angle_Get(void) {return Tar_Angle;}
		float Get_Tar_Angle(void) {return Tar_Angle;}

		float IMU_Speed;
		float IMU_Angle;

		float Angle_Kd;//使用速度作为角度的微分时的微分系数

		IFR_PID Angle_PID;

	private:
		float Tar_Angle;
		float (*Motor_Offset_MotoFunc)(float Motor_Tar);
};

#endif



