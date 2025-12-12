#include "ifr_dmmotor.h"

/**
  * @概述	DJI电机实例化的构造函数，初始化电机ID。
  * @参数1	电机ID
  */
IFR_DM_Motor::IFR_DM_Motor(uint8_t MotorId)
{
	_motor_id = MotorId;
	_t_max = TMAX;
	Set_DMMotor_Output(0);
	//DM_Info.Last_Angle = 65535;
}
/**
  * @概述	DJI电机实例化的构造函数，初始化电机ID。
  * @参数1	电机ID
  */
IFR_DM_Motor::IFR_DM_Motor(uint8_t MotorId, float T_Max)
{
	_motor_id = MotorId;
	
	_t_max = (T_Max<0)?-T_Max:T_Max;
	Set_DMMotor_Output(0);
	//DM_Info.Last_Angle = 65535;
}
 /**
  * @概述	电机标定。
  * @参数1	电机标定的位置
  * @返回值 void
  */
void IFR_DM_Motor::DM_Motor_PosInit(float PosInit)
{
	DM_Info.Abs_Angle = PosInit;
	DM_Info.Last_Angle = PosInit;
}
 /**
  * @概述	电机数据解析函数。
  * @参数1	数据头指针
  * @参数2	CAN消息Stdid
  * @返回值 void
  */
void IFR_DM_Motor::DM_Motor_Analysis(uint8_t *Data,uint32_t stdid)
{
	if(stdid == MasterCanID && (Data[0]&0x0F) == Get_DMMotor_Id())
	{
		uint16_t p_int, v_int, t_int;
		
		p_int = (((uint16_t)Data[1]<<8)&0xFF00) | (((uint16_t)Data[2]) & 0x00FF);
		v_int = (((uint16_t)Data[3]<<4)&0x0FF0) | (((uint16_t)Data[4]>>4) & 0x000F);
		t_int = (((uint16_t)Data[4]<<8)&0x0F00) | (((uint16_t)Data[5]) & 0x00FF);
		
		DM_Info.Angle	= uint_to_float(p_int, PMAX, 16);
		DM_Info.Speed = uint_to_float(v_int, VMAX, 12);
		DM_Info.Torque = uint_to_float(t_int, _t_max, 12);
		
		DM_Info.DM_State = (ErrCodeTypedef)((Data[0]>>4)&0x0F);
		if (DM_Info.DM_State < 8 && DM_Error_Callback != NULL) DM_Error_Callback();
		
		DM_Info.T_MOS = Data[6];
		DM_Info.T_Rotor = Data[7];
		
		if (DM_Info.Speed > 0.07f || DM_Info.Speed < -0.07f)
		{
			float Error=DM_Info.Angle-DM_Info.Last_Angle;
			DM_Info.Abs_Angle += Error;
			if (Error < -PMAX) DM_Info.Abs_Angle += PMAX*2.0f;
			else if (Error > PMAX)  DM_Info.Abs_Angle -= PMAX*2.0f;
		}DM_Info.Last_Angle=DM_Info.Angle;
	}
}
 /**
  * @概述	获取该电机ID。
  * @返回值 (uint8_t)电机号
  */
uint8_t IFR_DM_Motor::Get_DMMotor_Id(void)
{
	return _motor_id;
}
 /**
  * @概述	获取该电机目标扭矩。
  * @返回值 (float)电机目标扭矩
  */
float IFR_DM_Motor::Get_DMMotor_Output(void)
{
	return _output;
}
 /**
  * @概述	设置电机输出值。
  * @参数1	输出设置值
  * @返回值 void
  */
void IFR_DM_Motor::Set_DMMotor_Output(float set_output)
{
	if (_output > _t_max)
		_output = -_t_max;
	else if (_output < -_t_max)
		_output = _t_max;
	else _output = set_output;
	
	_t_ff = float_to_uint(_output, _t_max, 12);

}
 /**
  * @概述	设置电机。
  * @参数1	设置命令
  * @返回值 void
  */
void IFR_DM_Motor::DM_Motor_Command(DM_Command_t Command)
{
//	CAN_TxHeaderTypeDef tx_header;
	for (uint8_t i=0;i<7;i++) pTxCommandData[i] = 0xFF;
	pTxCommandData[7] = (uint8_t)Command;
	
//	tx_header.DLC = 8;
//	tx_header.ExtId = 0;
//	tx_header.IDE = CAN_ID_STD;
//	tx_header.RTR = CAN_RTR_DATA;
//	tx_header.StdId = _motor_id;
//	tx_header.TransmitGlobalTime = DISABLE;
//	
//	pCan->CAN_Transmit(&tx_header, pTxData);
}
 /**
  * @概述	速度环电机实例化的构造函数，初始化电机ID。
  * @参数1	电机ID
  */
IFR_Speed_DMMotor::IFR_Speed_DMMotor(uint8_t Motor_id):IFR_DM_Motor(Motor_id){ }

 /**
  * @概述	速度环电机实例化的构造函数，初始化电机ID和最大扭矩（上位机设置的值）。
  * @参数1	电机ID
  * @参数2	最大扭矩
  */
IFR_Speed_DMMotor::IFR_Speed_DMMotor(uint8_t Motor_id, float T_Max):IFR_DM_Motor(Motor_id, T_Max){ }

 /**
  * @概述	电机速度设置。
  * @参数1	速度目标值
  * @返回值 void
  */
void IFR_Speed_DMMotor::Motor_Speed_Set(float Speed_Tar)
{
	if (Speed_Tar > VMAX) Speed_Tar = VMAX;
	else if (Speed_Tar < -VMAX) Speed_Tar = -VMAX;
	Tar_Speed = Speed_Tar;
	_output = Speed_PID.Positional_PID(Tar_Speed,DM_Info.Speed);
	
	Set_DMMotor_Output(_output);
}
 /**
  * @概述	位置环电机实例化的构造函数，初始化电机ID。
  * @参数1	电机ID
  */
IFR_Pos_DMMotor::IFR_Pos_DMMotor(uint8_t Motor_id):IFR_Speed_DMMotor(Motor_id){};
	
/**
	* @概述	位置环电机实例化的构造函数，初始化电机ID和最大扭矩（上位机设置的值）。
	* @参数1	电机ID
	* @参数2	最大扭矩
*/
IFR_Pos_DMMotor::IFR_Pos_DMMotor(uint8_t Motor_id, float T_Max):IFR_Speed_DMMotor(Motor_id, T_Max){};
 /**
  * @概述	位置环电机实例化的构造函数，初始化电机ID和重力补偿函数。
  * @参数1	重力补偿函数指针
  * @参数2	电机ID
  */
IFR_Pos_DMMotor::IFR_Pos_DMMotor(float (*Offset_MotoFunc)(float Motor_Tar),uint8_t Motor_id):IFR_Speed_DMMotor(Motor_id)
{
	Motor_Offset_MotoFunc = Offset_MotoFunc;
}

 /**
  * @概述	位置环电机实例化的构造函数，初始化电机ID、重力补偿函数和最大扭矩（上位机设置的值）。
  * @参数1	重力补偿函数指针
  * @参数2	电机ID
  * @参数3	最大扭矩
  */
IFR_Pos_DMMotor::IFR_Pos_DMMotor(float (*Offset_MotoFunc)(float Motor_Tar),uint8_t Motor_id, float T_Max):IFR_Speed_DMMotor(Motor_id, T_Max)
{
	Motor_Offset_MotoFunc = Offset_MotoFunc;
}

 /**
  * @概述	电机绝对位置设置。
  * @参数1	绝对位置目标值
  * @返回值 void
  */
void IFR_Pos_DMMotor::Motor_AbsPos_Set(float Pos_Tar)
{
	Tar_Pos = Pos_Tar;
	float Speed_Tar = Pos_PID.Positional_PID(Tar_Pos,DM_Info.Abs_Angle);
	Motor_Speed_Set(Speed_Tar);

	if(Motor_Offset_MotoFunc !=NULL) _output +=(*Motor_Offset_MotoFunc)(Tar_Pos);
	
	Set_DMMotor_Output(_output);
}
 /**
  * @概述	电机位置(rad)设置。
  * @参数1	位置目标值
  * @返回值 void
  */
void IFR_Pos_DMMotor::Motor_Pos_Set(float Pos_Tar)
{
	if (abs(Pos_Tar) <= PMAX)//超限保护
	{
		Tar_Pos = Pos_Tar;
	}
	
	float Speed_Tar = Pos_PID.Positional_PID(Tar_Pos,DM_Info.Angle);
	Motor_Speed_Set(Speed_Tar);
	if(Motor_Offset_MotoFunc !=NULL) _output +=(*Motor_Offset_MotoFunc)(Tar_Pos);
	
	Set_DMMotor_Output(_output);
}


 /**
  * @概述	位置环电机实例化的构造函数，初始化电机ID。
  * @参数1	电机ID
  */
IFR_GyroControl_DMMotor::IFR_GyroControl_DMMotor(uint8_t Motor_id):IFR_Speed_DMMotor(Motor_id)
{
	Angle_Kd = 0;
}
 /**
  * @概述	角度环电机实例化的构造函数，初始化电机ID和重力补偿参数。
  * @参数1	重力补偿函数指针
  * @参数2	电机ID
  */
IFR_GyroControl_DMMotor::IFR_GyroControl_DMMotor(float (*Offset_MotoFunc)(float Motor_Tar),uint8_t Motor_id):IFR_Speed_DMMotor(Motor_id)
{
	Motor_Offset_MotoFunc = Offset_MotoFunc;
	Angle_Kd=0;
}

 /**
  * @概述	位置环电机实例化的构造函数，初始化电机ID。
  * @参数1	电机ID
  */
IFR_GyroControl_DMMotor::IFR_GyroControl_DMMotor(uint8_t Motor_id, float T_Max):IFR_Speed_DMMotor(Motor_id, T_Max)
{
	Angle_Kd = 0;
}
 /**
  * @概述	角度环电机实例化的构造函数，初始化电机ID和重力补偿参数。
  * @参数1	重力补偿函数指针
  * @参数2	电机ID
  */
IFR_GyroControl_DMMotor::IFR_GyroControl_DMMotor(float (*Offset_MotoFunc)(float Motor_Tar),uint8_t Motor_id, float T_Max):IFR_Speed_DMMotor(Motor_id, T_Max)
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
void IFR_GyroControl_DMMotor::IMU_Data_Get(float imu_speed,float imu_angle)
{
	IMU_Speed = imu_speed;
	IMU_Angle = imu_angle;
}
 /**
  * @概述	电机角度设置。
  * @参数1	电机目标角度设置
  * @返回值 void
  */
void IFR_GyroControl_DMMotor::Motor_Angle_Set(float Angle_Tar)
{
	Tar_Angle = Angle_Tar;
	float Speed_Tar = Angle_PID.Positional_PID(Tar_Angle,IMU_Angle) + IMU_Speed * Angle_Kd;
	_output = Speed_PID.Positional_PID(Speed_Tar,IMU_Speed);
	if(Motor_Offset_MotoFunc !=NULL) _output +=(*Motor_Offset_MotoFunc)(DM_Info.Angle);
	
	Set_DMMotor_Output(_output);
}


inline float IFR_DM_Motor::uint_to_float(int x_int, float x_max, int bits){
	return ((float)x_int)*(x_max*2.0f)/((float)((1<<bits)-1)) - x_max;
}

inline int IFR_DM_Motor::float_to_uint(float x, float x_max, int bits){
	return (int) ((x+x_max)*((float)((1<<bits)-1))/(x_max*2.0f));
}

inline float IFR_DM_Motor::uint_to_float(int x_int, float x_min, float x_max, int bits){
	return ((float)x_int)*(x_max - x_min)/((float)((1<<bits)-1)) + x_min;
}

inline int IFR_DM_Motor::float_to_uint(float x, float x_min, float x_max, int bits){
	return (int) ((x-x_min)*((float)((1<<bits)-1))/(x_max - x_min));
}


