/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2024, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 		: ROBSTRIDE 01.c
  * Version			: v1.1
  * Author			: PanJiajun ZhaoZiLin
  * Date				: 2024-07-31
  * Description	:	灵足电机库
	*
  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "string.h"
#include "ifr_fdcan.h"
uint16_t Index_List[] = {0X7005, 0X7006, 0X700A, 0X700B, 0X7010, 0X7011, 0X7014, 0X7016, 0X7017, 0X7018, 0x7019, 0x701A, 0x701B, 0x701C, 0x701D, 0x701E, 0x701F, 0x7020};

/*******************************************************************************
 * @功能     		: float型转uint16_t型浮点数
 * @参数1       : 需要转换的值
 * @参数2       : x的最小值
 * @参数3       : x的最大值
 * @参数4       : 需要转换的进制数
 * @返回值 			: 十进制的uint16_t型浮点数
 * @概述  			: 不需要你用
 *******************************************************************************/
int float_to_uint(float x, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	if (x > x_max)
		x = x_max;
	else if (x < x_min)
		x = x_min;
	return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
/*******************************************************************************
 * @功能     		 : uint16_t型转float型浮点数
 * @参数1        : 需要转换的值
 * @参数2        : x的最小值
 * @参数3        : x的最大值
 * @参数4        : 需要转换的进制数
 * @返回值 			 : 十进制的float型浮点数
 * @概述  			 : 不需要你用
 *******************************************************************************/
float uint16_to_float(uint16_t x, float x_min, float x_max, int bits)
{
	uint32_t span = (1 << bits) - 1;
	float offset = x_max - x_min;
	return offset * x / span + x_min;
}
/*******************************************************************************
 * @功能     		: Byte型转float型浮点数
 * @参数1        : 需要转换的值
 * @返回值 			: 十进制的float型浮点数
 * @概述  				: 不需要你用
 *******************************************************************************/
float Byte_to_float(uint8_t *bytedata)
{
	uint32_t data = bytedata[7] << 24 | bytedata[6] << 16 | bytedata[5] << 8 | bytedata[4];
	float data_float = *(float *)(&data);
	return data_float;
}

/*******************************************************************************
 * @功能     		: 接收处理函数		（通信类型2）在CAN接收处理函数自动调用
 * @参数1       : 接收到的数据
 * @参数2       : 接收到的CAN_ID_ExtId(拓展帧ID)
 * @返回值 			: None
 * @概述  			: 不需要你用，他在CAN接收回调自己调用
 *******************************************************************************/
void IFR_LZ_Motor::LZ_Motor_Analysis(uint8_t *LZ_Data, uint32_t ID_ExtId)
{
	if ((uint8_t)((ID_ExtId & 0xFF00) >> 8) == CAN_ID)
	{
		if (int((ID_ExtId & 0x3F000000) >> 24) == 2) // 判断是否为通信类型2
		{
			LZ_Pos_Info.Angle = uint16_to_float(LZ_Data[0] << 8 | LZ_Data[1], P_MIN, P_MAX, 16);
			LZ_Pos_Info.Speed = uint16_to_float(LZ_Data[2] << 8 | LZ_Data[3], V_MIN, V_MAX, 16);
			LZ_Pos_Info.Torque = uint16_to_float(LZ_Data[4] << 8 | LZ_Data[5], T_MIN, T_MAX, 16);
			LZ_Pos_Info.Temp = (LZ_Data[6] << 8 | LZ_Data[7]) / 10.0f;
			LZ_Pos_Info.Error_Code = int((ID_ExtId & 0x3F0000) >> 16);
			LZ_Pos_Info.Pattern = uint8_t((ID_ExtId & 0xC00000) >> 22);
		}
		else if (int((ID_ExtId & 0x3F000000) >> 24) == 17) // 判断是否为通信类型17
		{
			for (int i = 0; i <= 18; i++)
			{
				if ((LZ_Data[1] << 8 | LZ_Data[0]) == Index_List[i])
					switch (i)
					{
					case 0:
						LZ_Message_From_17.run_mode = uint8_t(LZ_Data[4]);
						break;
					case 1:
						LZ_Message_From_17.iq_ref = Byte_to_float(LZ_Data);
						break;
					case 2:
						LZ_Message_From_17.spd_ref = Byte_to_float(LZ_Data);
						break;
					case 3:
						LZ_Message_From_17.limit_torque = Byte_to_float(LZ_Data);
						break;
					case 4:
						LZ_Message_From_17.cur_kp = Byte_to_float(LZ_Data);
						break;
					case 5:
						LZ_Message_From_17.cur_ki = Byte_to_float(LZ_Data);
						break;
					case 6:
						LZ_Message_From_17.curfilt_gain = Byte_to_float(LZ_Data);
						break;
					case 7:
						LZ_Message_From_17.loc_ref = Byte_to_float(LZ_Data);
						break;
					case 8:
						LZ_Message_From_17.limit_spd = Byte_to_float(LZ_Data);
						break;
					case 9:
						LZ_Message_From_17.limit_cur = Byte_to_float(LZ_Data);
						break;
					case 10:
						LZ_Message_From_17.mechPos = Byte_to_float(LZ_Data);
						break;
					case 11:
						LZ_Message_From_17.iqf = Byte_to_float(LZ_Data);
						break;
					case 12:
						LZ_Message_From_17.mechVel = Byte_to_float(LZ_Data);
						break;
					case 13:
						LZ_Message_From_17.VBUS = Byte_to_float(LZ_Data);
						break;
					case 14:
						LZ_Message_From_17.rotation = (*LZ_Data);
						break;
					case 15:
						LZ_Message_From_17.loc_kp = Byte_to_float(LZ_Data);
						break;
					case 16:
						LZ_Message_From_17.spd_kp = Byte_to_float(LZ_Data);
						break;
					case 17:
						LZ_Message_From_17.spd_ki = Byte_to_float(LZ_Data);
						break;
					}
			}
		}
	}
}

//以下为通信方式
/*******************************************************************************
 * @功能     		: 灵足电机获取设备ID和MCU（通信类型0）
 * @参数        : None
 * @返回值 			: void
 * @概述  			: None
 *******************************************************************************/
void IFR_LZ_Motor::Get_LZ_Motor_ID()
{
	uint8_t txdata[8] = {0};	   		// 发送数据
	FDCAN_TxHeaderTypeDef TxMessage;
	TxMessage.IdType = FDCAN_EXTENDED_ID;								//拓展帧，29 位 ID
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;						//数据帧
	TxMessage.DataLength = FDCAN_DLC_BYTES_8;						//8 字节数据长度
	TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		//活动错误状态
	TxMessage.BitRateSwitch = FDCAN_BRS_OFF;						//关闭位速率切换
	TxMessage.FDFormat = FDCAN_CLASSIC_CAN;							//经典 CAN 格式
	TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	//不记录发送事件
	TxMessage.MessageMarker = 0;	//标记此消息，在 TX Event FIFO 中区分不同消息
	TxMessage.Identifier = Communication_Type_Get_ID << 24 | CAN_MASTER_ID << 8 | CAN_ID;
	FDCAN_Pointer[Can_Motor]->FDCAN_Transmit(&TxMessage, txdata);
}
/*******************************************************************************
 * @功能     		: 灵足电机运控模式  （通信类型1）
 * @参数1       : 力矩（-17Nm~17Nm）
 * @参数2       : 目标角度(-12.5~12.5)
 * @参数3       : 目标角速度(-44rad/s~44rad/s)
 * @参数4       : Kp(0.0~500.0)
 * @参数5       : Kp(0.0~5.0)
 * @返回值 			: void
 * @概述  			: 数据转换后高字节在前，低字节在后
 *******************************************************************************/
void IFR_LZ_Motor::LZ_Motor_move_control(float Torque, float Angle, float Speed, float Kp, float Kd)
{
	uint8_t txdata[8] = {0};	   // 发送数据
	Motor_Set.set_Torque = Torque;
	Motor_Set.set_angle = Angle;
	Motor_Set.set_speed = Speed;
	Motor_Set.set_Kp = Kp;
	Motor_Set.set_Kd = Kd;
	if (LZ_Pos_Info.Pattern == 2 && Motor_Set.set_motor_mode != move_control_mode)
	{
		Motor_Set.set_motor_mode = move_control_mode;
		Set_LZ_Motor_parameter(0X7005, Motor_Set.set_motor_mode, 'P'); // 设置电机模式
	}
	FDCAN_TxHeaderTypeDef TxMessage;
	TxMessage.IdType = FDCAN_EXTENDED_ID;								//拓展帧，29 位 ID
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;						//数据帧
	TxMessage.DataLength = FDCAN_DLC_BYTES_8;						//8 字节数据长度
	TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		//活动错误状态
	TxMessage.BitRateSwitch = FDCAN_BRS_OFF;						//关闭位速率切换
	TxMessage.FDFormat = FDCAN_CLASSIC_CAN;							//经典 CAN 格式
	TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	//不记录发送事件
	TxMessage.MessageMarker = 0;	//标记此消息，在 TX Event FIFO 中区分不同消息
	TxMessage.Identifier = Communication_Type_MotionControl << 24 | float_to_uint(Motor_Set.set_Torque, T_MIN, T_MAX, 16) << 8 | CAN_ID;
	txdata[0] = float_to_uint(Motor_Set.set_angle, P_MIN, P_MAX, 16) >> 8;
	txdata[1] = float_to_uint(Motor_Set.set_angle, P_MIN, P_MAX, 16);
	txdata[2] = float_to_uint(Motor_Set.set_speed, V_MIN, V_MAX, 16) >> 8;
	txdata[3] = float_to_uint(Motor_Set.set_speed, V_MIN, V_MAX, 16);
	txdata[4] = float_to_uint(Motor_Set.set_Kp, KP_MIN, KP_MAX, 16) >> 8;
	txdata[5] = float_to_uint(Motor_Set.set_Kp, KP_MIN, KP_MAX, 16);
	txdata[6] = float_to_uint(Motor_Set.set_Kd, KD_MIN, KD_MAX, 16) >> 8;
	txdata[7] = float_to_uint(Motor_Set.set_Kd, KD_MIN, KD_MAX, 16);
	FDCAN_Pointer[Can_Motor] -> FDCAN_Transmit(&TxMessage, txdata);
}

/*******************************************************************************
 * @功能     		: 灵足电机电流模式  
 * @参数1       : 目标电流(-23~23A)
 * @返回值 			: void
 * @概述  			: 数据转换后高字节在前，低字节在后
 *******************************************************************************/
void IFR_LZ_Motor::LZ_Motor_Elect_control(float Elect)
{		
	Motor_Set.set_current = Elect;
	output = Motor_Set.set_current;
	if (LZ_Pos_Info.Pattern == 2 && Motor_Set.set_motor_mode != Elect_control_mode)
	{
		Set_LZ_Motor_parameter(0X7005, Elect_control_mode, 'P'); // 设置电机模式
		Motor_Set.set_motor_mode = Elect_control_mode;
	}
	Set_LZ_Motor_parameter(0x7006, output, 'V');
}

/*******************************************************************************
 * @功能     		: 灵足电机速度模式  
 * @参数1       : 电流限制(0~23A)
 * @参数2       : 目标速度(-30rad/s~30rad/s)
 * @返回值 			: void
 * @概述  			: 数据转换后高字节在前，低字节在后
 *******************************************************************************/
void IFR_LZ_Motor::LZ_Motor_Speed_control(float limit_cur, float Tar_Speed)
{
	Motor_Set.set_speed = Tar_Speed;
	Motor_Set.set_limit_cur = limit_cur;
	if (LZ_Pos_Info.Pattern == 2)
	{
		Set_LZ_Motor_parameter(0X7005, Speed_control_mode, 'P'); // 设置电机模式
		Motor_Set.set_motor_mode = Speed_control_mode;
	}
	//	Enable_Motor();
	Set_LZ_Motor_parameter(0X7018, Motor_Set.set_limit_cur, 'V');
	Set_LZ_Motor_parameter(0X700A, Motor_Set.set_speed, 'V');
}

/*******************************************************************************
 * @功能     		: 灵足电机零点模式  
 * @返回值 			: void
 * @概述  			: 数据转换后高字节在前，低字节在后
 *******************************************************************************/
void IFR_LZ_Motor::LZ_Motor_Zero_Pos_control()
{
	if (LZ_Pos_Info.Pattern == 2)
	{
		Motor_Set.set_motor_mode = Set_Zero_mode;
		Set_LZ_Motor_parameter(0X7005, Set_Zero_mode, 'P'); // 设置电机模式
	}
	//	Enable_Motor();
}

/*******************************************************************************
 * @功能     		: 灵足电机使能 （通信类型3）
 * @参数        : None
 * @返回值 			: void
 * @概述  			: 想要使用灵足电机必须得先使能电机
 *******************************************************************************/
void IFR_LZ_Motor::Enable_Motor()
{
	uint8_t txdata[8] = {0};	   // 发送数据
	FDCAN_TxHeaderTypeDef TxMessage;
	TxMessage.IdType = FDCAN_EXTENDED_ID;								//拓展帧，29 位 ID
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;						//数据帧
	TxMessage.DataLength = FDCAN_DLC_BYTES_8;						//8 字节数据长度
	TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		//活动错误状态
	TxMessage.BitRateSwitch = FDCAN_BRS_OFF;						//关闭位速率切换
	TxMessage.FDFormat = FDCAN_CLASSIC_CAN;							//经典 CAN 格式
	TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	//不记录发送事件
	TxMessage.MessageMarker = 0;	//标记此消息，在 TX Event FIFO 中区分不同消息
	TxMessage.Identifier = Communication_Type_MotorEnable << 24 | CAN_MASTER_ID << 8 | CAN_ID;

	FDCAN_Pointer[Can_Motor]->FDCAN_Transmit(&TxMessage, txdata);
	
}

/*******************************************************************************
 * @功能     		: 灵足电机失能 （通信类型4）
 * @参数        : 是否清除错误位（0不清除 1清除）
 * @返回值 			: void
 * @概述  			: None
 *******************************************************************************/
void IFR_LZ_Motor::Disable_Motor(uint8_t clear_error)
{
	uint8_t txdata[8] = {0};	   // 发送数据
	FDCAN_TxHeaderTypeDef TxMessage;
	TxMessage.IdType = FDCAN_EXTENDED_ID;								//拓展帧，29 位 ID
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;						//数据帧
	TxMessage.DataLength = FDCAN_DLC_BYTES_8;						//8 字节数据长度
	TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		//活动错误状态
	TxMessage.BitRateSwitch = FDCAN_BRS_OFF;						//关闭位速率切换
	TxMessage.FDFormat = FDCAN_CLASSIC_CAN;							//经典 CAN 格式
	TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	//不记录发送事件
	TxMessage.MessageMarker = 0;	//标记此消息，在 TX Event FIFO 中区分不同消息
	TxMessage.Identifier = Communication_Type_MotorStop << 24 | CAN_MASTER_ID << 8 | CAN_ID;

	FDCAN_Pointer[Can_Motor]->FDCAN_Transmit(&TxMessage, txdata);
}

/*******************************************************************************
 * @功能     		: 灵足电机设置电机机械零位 （通信类型6）
 * @参数        : 无
 * @返回值 			: void
 * @概述  			: 先失能是为了防止电机疯转到下一个你的目标位置
 *******************************************************************************/
void IFR_LZ_Motor::Set_ZeroPos()
{
	Disable_Motor(0);			   // 失能电机
	uint8_t txdata[8] = {0};	   // 发送数据
	FDCAN_TxHeaderTypeDef TxMessage;
	TxMessage.IdType = FDCAN_EXTENDED_ID;								//拓展帧，29 位 ID
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;						//数据帧
	TxMessage.DataLength = FDCAN_DLC_BYTES_8;						//8 字节数据长度
	TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		//活动错误状态
	TxMessage.BitRateSwitch = FDCAN_BRS_OFF;						//关闭位速率切换
	TxMessage.FDFormat = FDCAN_CLASSIC_CAN;							//经典 CAN 格式
	TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	//不记录发送事件
	TxMessage.MessageMarker = 0;	//标记此消息，在 TX Event FIFO 中区分不同消息
	TxMessage.Identifier = Communication_Type_SetPosZero << 24 | CAN_MASTER_ID << 8 | CAN_ID;
	txdata[0] = 1;
	FDCAN_Pointer[Can_Motor]->FDCAN_Transmit(&TxMessage, txdata);
	Enable_Motor();

}

/*******************************************************************************
 * @功能     		: 灵足电机设置CAN_ID （通信类型7）
 * @参数        : 修改后（预设）CANID
 * @返回值 			: void
 * @概述  			: None
 *******************************************************************************/
void IFR_LZ_Motor::Set_CAN_ID(uint8_t Set_CAN_ID)
{
	Disable_Motor(0);
	uint8_t txdata[8] = {0};	   // 发送数据
	FDCAN_TxHeaderTypeDef TxMessage;
	TxMessage.IdType = FDCAN_EXTENDED_ID;								//拓展帧，29 位 ID
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;						//数据帧
	TxMessage.DataLength = FDCAN_DLC_BYTES_8;						//8 字节数据长度
	TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		//活动错误状态
	TxMessage.BitRateSwitch = FDCAN_BRS_OFF;						//关闭位速率切换
	TxMessage.FDFormat = FDCAN_CLASSIC_CAN;							//经典 CAN 格式
	TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	//不记录发送事件
	TxMessage.MessageMarker = 0;	//标记此消息，在 TX Event FIFO 中区分不同消息
	TxMessage.Identifier = Communication_Type_Can_ID << 24 | Set_CAN_ID << 16 | CAN_MASTER_ID << 8 | CAN_ID;
	FDCAN_Pointer[Can_Motor]->FDCAN_Transmit(&TxMessage, txdata);
}

/*******************************************************************************
 * @功能     		: 灵足电机单个参数读取 （通信类型17）
 * @参数        : 参数地址
 * @返回值 			: void
 * @概述  			: None
 *******************************************************************************/
void IFR_LZ_Motor::Get_LZ_Motor_parameter(uint16_t Index)
{
	uint8_t txdata[8] = {0};	   // 发送数据
	FDCAN_TxHeaderTypeDef TxMessage;
	TxMessage.IdType = FDCAN_EXTENDED_ID;								//拓展帧，29 位 ID
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;						//数据帧
	TxMessage.DataLength = FDCAN_DLC_BYTES_8;						//8 字节数据长度
	TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		//活动错误状态
	TxMessage.BitRateSwitch = FDCAN_BRS_OFF;						//关闭位速率切换
	TxMessage.FDFormat = FDCAN_CLASSIC_CAN;							//经典 CAN 格式
	TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	//不记录发送事件
	TxMessage.MessageMarker = 0;	//标记此消息，在 TX Event FIFO 中区分不同消息
	TxMessage.Identifier = Communication_Type_GetSingleParameter << 24 | CAN_MASTER_ID << 8 | CAN_ID;
	FDCAN_Pointer[Can_Motor]->FDCAN_Transmit(&TxMessage, txdata);
}

/*******************************************************************************
 * @功能     		: 灵足电机写入参数 （通信类型18）
 * @参数1       : 参数地址
 * @参数2       : 参数数值
 * @参数3       : 选择是传入控制模式 还是其他参数 （Set_parameter = P设置控制模式 Set_Value = V设置参数数据）
 * @返回值 			: void
 * @概述  			: 最好别动奥！！
 *******************************************************************************/
void IFR_LZ_Motor::Set_LZ_Motor_parameter(uint16_t Index, float Value, char Value_mode)
{
	uint8_t txdata[8] = {0};	   // 发送数据
	FDCAN_TxHeaderTypeDef TxMessage;
	TxMessage.IdType = FDCAN_EXTENDED_ID;								//拓展帧，29 位 ID
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;						//数据帧
	TxMessage.DataLength = FDCAN_DLC_BYTES_8;						//8 字节数据长度
	TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		//活动错误状态
	TxMessage.BitRateSwitch = FDCAN_BRS_OFF;						//关闭位速率切换
	TxMessage.FDFormat = FDCAN_CLASSIC_CAN;							//经典 CAN 格式
	TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	//不记录发送事件
	TxMessage.MessageMarker = 0;	//标记此消息，在 TX Event FIFO 中区分不同消息
	TxMessage.Identifier = Communication_Type_SetSingleParameter << 24 | CAN_MASTER_ID << 8 | CAN_ID;

	//	txdata[0] = Index;
	//	txdata[1] = Index>>8;
	//	txdata[2] = 0x00;
	//	txdata[3] = 0x00;
	//	txdata[4] = Value;
	//	txdata[5] = 0x00;
	//	txdata[6] = 0x00;
	//	txdata[7] = 0x00;
	memcpy(&txdata[0], &Index, 2);
	if (Value_mode == 'P') // 选择电机模式
	{
		Motor_Set.set_motor_mode = int(Value);
		memcpy(&txdata[4], &Motor_Set.set_motor_mode, 1);
	}
	else if (Value_mode == 'V') // 写入数据
	{
		memcpy(&txdata[4], &Value, 4);
	}
	FDCAN_Pointer[Can_Motor]->FDCAN_Transmit(&TxMessage, txdata);
}

/*******************************************************************************
 * @功能     		: 灵足电机自行配制速度环PID
 * @参数1       : 目标速度
 * @返回值 			: void
 * @概述  			: None
 *******************************************************************************/
void IFR_Speed_LZ_Motor::Motor_Speed_Set(float Speed_Tar)
{
	Motor_Set.set_speed = Speed_Tar;
	output = Speed_PID.Positional_PID(Speed_Tar, LZ_Pos_Info.Speed);
	LZ_Motor_Elect_control(output);
}

/*******************************************************************************
 * @功能     		: 灵足电机自行配制位置环PID
 * @参数1       : 目标位置
 * @返回值 			: void
 * @概述  			: None
 *******************************************************************************/
void IFR_Pos_LZ_Motor::Motor_Pos_Set(float Pos_Tar)
{
	Motor_Set.set_angle = Pos_Tar;
	float Speed_Tar = Pos_PID.Positional_PID(Motor_Set.set_angle, LZ_Pos_Info.Angle);
	output = Speed_PID.Positional_PID(Speed_Tar, LZ_Pos_Info.Speed);
	if (Motor_Offset_MotoFunc != NULL)
		output += (*Motor_Offset_MotoFunc)(LZ_Pos_Info.Angle);
	LZ_Motor_Elect_control(output);
}

/*******************************************************************************
 * @功能     		: 灵足电机自行配制位置电机的速度环PID
 * @参数1       : 目标位置
 * @返回值 			: void
 * @概述  			: None
 *******************************************************************************/
void IFR_Pos_LZ_Motor::Motor_Speed_Set(int16_t Speed_Tar)
{
	Motor_Set.set_speed = Speed_Tar;
	output = Speed_PID.Positional_PID(Speed_Tar, LZ_Pos_Info.Speed);
	LZ_Motor_Elect_control(output);
}

/*******************************************************************************
* @功能     		: CyberGear电机角度环角度数据更新。
* @参数1        : 陀螺仪角速度信息
* @参数2        : 陀螺仪角度信息
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
void IFR_GyroControl_LZ_Motor::IMU_Data_Get(float imu_speed,float imu_angle)
{
	IMU_Speed = imu_speed;
	IMU_Angle = imu_angle;
}
/*******************************************************************************
* @功能     		: CyberGear陀螺仪控制的电机角度设置。
* @参数         : 电机目标角度设置
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
void IFR_GyroControl_LZ_Motor::Motor_Angle_Set(float Angle)
{
	Motor_Set.set_angle = Angle;
	float Speed_Tar = Angle_PID.Positional_PID(Motor_Set.set_angle, IMU_Angle);
	Motor_Set.set_current = Speed_PID.Positional_PID(Speed_Tar, IMU_Speed);
	if(Motor_Offset_MotoFunc !=NULL) Motor_Set.set_current += (*Motor_Offset_MotoFunc)(LZ_Pos_Info.Angle);
	output = Motor_Set.set_current;	
	LZ_Motor_Elect_control(Motor_Set.set_current);	
}
/*******************************************************************************
 * @功能     		: 灵足电机自行控制实例化的构造函数
 * @参数        : CAN ID
 * @返回值 			: void
 * @概述  			: 初始化电机ID。
 *******************************************************************************/
IFR_Speed_LZ_Motor::IFR_Speed_LZ_Motor(uint8_t CAN_Id) : IFR_LZ_Motor(CAN_Id)
{
	CAN_ID = CAN_Id;
	CAN_MASTER_ID = 0x1F;
	Motor_Set.set_motor_mode = move_control_mode;
}
IFR_Pos_LZ_Motor::IFR_Pos_LZ_Motor(uint8_t CAN_Id) : IFR_LZ_Motor(CAN_Id)
{
	CAN_ID = CAN_Id;
	CAN_MASTER_ID = 0x1F;
	Motor_Set.set_motor_mode = move_control_mode;
}
IFR_GyroControl_LZ_Motor::IFR_GyroControl_LZ_Motor(uint8_t CAN_Id) : IFR_LZ_Motor(CAN_Id)
{
	CAN_ID = CAN_Id;
	CAN_MASTER_ID = 0x1F;
	Motor_Set.set_motor_mode = move_control_mode;
}
IFR_Pos_LZ_Motor::IFR_Pos_LZ_Motor(float (*Offset_MotoFunc)(float Motor_Tar), uint8_t CAN_Id) : IFR_LZ_Motor(Offset_MotoFunc, CAN_Id){}

IFR_GyroControl_LZ_Motor::IFR_GyroControl_LZ_Motor(float (*Offset_MotoFunc)(float Motor_Tar), uint8_t CAN_Id) : IFR_LZ_Motor(Offset_MotoFunc, CAN_Id){}

/*******************************************************************************
 * @功能     		: 灵足电机实例化的构造函数
 * @参数        : CAN ID
 * @返回值 			: void
 * @概述  			: 初始化电机ID。
 *******************************************************************************/
IFR_LZ_Motor::IFR_LZ_Motor(uint8_t CAN_Id)
{
	CAN_ID = CAN_Id;
	CAN_MASTER_ID = 0x1F;
	Motor_Set.set_motor_mode = move_control_mode;
}
IFR_LZ_Motor::IFR_LZ_Motor(float (*Offset_MotoFunc)(float Motor_Tar), uint8_t CAN_Id)
{
	CAN_ID = CAN_Id;
	CAN_MASTER_ID = 0x1F;
	Motor_Set.set_motor_mode = move_control_mode;
	Motor_Offset_MotoFunc = Offset_MotoFunc;
}
