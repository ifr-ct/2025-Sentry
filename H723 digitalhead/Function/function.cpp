#include "math.h"
#include "function.h"
IFR_FDCAN_ClassDef CAN_1;
IFR_FDCAN_ClassDef CAN_2;
IFR_FDCAN_ClassDef CAN_3;
IFR_TIM_ClassDef TIM_7;
//IFR_UDBRx_ClassDef USART_1;
IFR_UDBRx_ClassDef UART_5;//串口接收遥控器键鼠信息
//IFR_UDBRx_ClassDef USART_2;//串口接收裁判系统
SPI_InitTypeDef SPI_2;
IFR_Speed_Motor Motor_shoot_zhao(0x201);
IFR_Speed_Motor Motor_shoot_xing(0x202);
IFR_Speed_Motor Motor_shoot_jie(0x203);
IFR_Pos_Motor Motor_gongdan(0x204);
IFR_GyroControl_Motor Motor_yaw(0x01);//陀螺仪角度,fdcan1
//IFR_Speed_Motor Can_user_define(0x20A);//fdcan2
IFR_GyroControl_Motor Motor_little_yaw(0x209);
IFR_GyroControl_Motor Motor_pitch(0x02);//陀螺仪角度,fdcan3
IFR_PID Wheel_Pos_Follow;//
extern uint8_t computer[50];
Leida leida;
Robot robot;
Tx_data tx_data;
//IMU_Info
#define PAI 3.1415926
uint16_t tar_x_0 = 0;
uint16_t tar_y_0 = 0;
float Change = 0;
float Wheel_Tar;
uint8_t flag = 1;
uint8_t shoot_ang = 1;
uint8_t shoot_on = 0;
uint8_t gongdan_on = 0;
void USART_handle(Robot *robot0)//遥控器底盘目标值算法
{
	if(DJI_Remote.Chx_Left <= 1040 && DJI_Remote.Chx_Left >= 1008)
	{
		DJI_Remote.Chx_Left = 1024;
	}
	if(DJI_Remote.Chy_Left <= 1040 && DJI_Remote.Chy_Left >= 1008)
	{
		DJI_Remote.Chy_Left = 1024;
	}
	if(DJI_Remote.Chx_Right <= 1040 && DJI_Remote.Chx_Right >= 1008)
	{
		DJI_Remote.Chx_Right = 1024;
	}
	if(DJI_Remote.Chy_Right <= 1040 && DJI_Remote.Chy_Right >= 1008)
	{
		DJI_Remote.Chy_Right = 1024;
	}
//	if(DJI_Remote.Switch_Left == 3 &&DJI_Remote.Switch_Right == 1)
//	{
		robot0 ->tar_x = 1000.0f * (DJI_Remote.Chx_Left - 1024) / 660.0f;
		robot0 ->tar_y = 1000.0f * (DJI_Remote.Chy_Left - 1024) / 660.0f;
//	}
//	if(DJI_Remote.Switch_Left == 3 &&DJI_Remote.Switch_Right == 3)
//	{
//		robot0 ->tar_x = 100.0f * (leida.move_x - 1024) / 660.0f;
//		robot0 ->tar_y = 100.0f * (leida.move_y - 1024) / 660.0f;
//	}
}

void Judge_8192(float *Motor_tar_angle, uint16_t *Motor_now_angle)//处理角度跳变
{
	if(*Motor_tar_angle - *Motor_now_angle >= 4096)
	{
		*Motor_tar_angle -= 8192;
	}
	else if(*Motor_tar_angle - *Motor_now_angle < -4096)
	{
		*Motor_tar_angle += 8192;	
	}
}

float HandleHalfTurnJump(float Tar, float Now , float Circle_angle)
{
  if (Tar <= Now - (Circle_angle / 2))
	{
     Tar = Tar + Circle_angle; 
	}// 向前校正位置
  if (Tar >= Now + (Circle_angle / 2))
	{
     Tar = Now - Circle_angle; 
	} // 向后校正位置
	return Tar;
}

void leida_handle(uint8_t *pData, uint8_t len)
{
	if(pData == NULL)
	{
		return;
	}
	leida.head = 0x5a; 
	if(pData[0] == leida.head)
	{
		leida.move_y = ((uint16_t)pData[1] | (uint16_t)pData[2] << 8) & 0x7FF;
		leida.move_x = ((uint16_t)pData[3] | (uint16_t)pData[4] << 8) & 0x7FF;
		leida.gun_yaw = ((int16_t)pData[6] | (int16_t)pData[5] << 8) & 0x7FF;
		leida.gun_pitch = ((int16_t)pData[8] | (int16_t)pData[7] << 8) & 0x7FF;
		leida.is_scan = ((uint8_t)pData[9]) & 0x01;
		leida.gyro = ((uint8_t)pData[9] >> 1) & 0x01;
		leida.shoot = ((uint8_t)pData[9] >> 2) & 0x01;
		leida.reserve = ((uint8_t)pData[9] >> 3) & 0x1F;
		leida.crc = (uint8_t)pData[10] & 0x7FF;
	}
	else
	{
		return;
	}
}

//void Target_yuntai_init()//云台目标值初始化
//{
//	if(flag == 1)
//	{
//		robot.Motor_yaw = IMU_Info.Angle.Yaw;//Motor_yaw.Pos_Info.Angle;
//		robot.Motor_pitch = IMU_Info.Angle.Pitch;//Motor_yaw.Pos_Info.Angle;//Motor_pitch.Pos_Info.Angle;
//		flag = 0;
//	}
//}

//void Yuntai_control()//云台控制（遥控器）
//{
//	robot.Motor_yaw += (1024 - DJI_Remote.Chx_Right) * 0.0004;//遥控器控制云台偏航目标值
//	robot.Motor_pitch += (1024 - DJI_Remote.Chy_Right) * 0.00095;//遥控器控制云台俯仰目标值
//	if (robot.Motor_pitch <= -37)//限位
//	{
//		robot.Motor_pitch = -37;
//	}
//	else if (robot.Motor_pitch >= 10)
//	{
//		robot.Motor_pitch = 10;
//	}
//	robot.Motor_yaw = HandleHalfTurnJump(robot.Motor_yaw, IMU_Info.Angle.Yaw, 360.0f);
//	Motor_yaw.IMU_Data_Get(IMU_Info.Gyro.Yaw, IMU_Info.Angle.Yaw);
//	Motor_pitch.IMU_Data_Get(-IMU_Info.Gyro.Pitch, -IMU_Info.Angle.Pitch);
//}

void Fashe_control_remot()//发射控制
{
	robot.Motor_zhao = 2000;
	robot.Motor_xing = 2000;
	robot.Motor_jie = 2000;
	if(shoot_ang == 1)
	{
		robot.Motor_gongdan_ang = Motor_gongdan.Pos_Info.Abs_Angle;
		shoot_ang = 0;
	}
	//robot.Motor_gongdan_speed = -2000;//拨盘电机2006减速比36：1，拨盘齿轮和齿轮之间还有一个比例，2006末端一圈可以打出5.1724发
}

void Transmit_to_slave_data(uint16_t tar_turn)
{
//	Tx_can_data[6] = (uint8_t)(tar_turn >> 8);  // 高字节
//	Tx_can_data[7] = (uint8_t)(tar_turn & 0xFF);  // 低字节
}

void Tar_motor_analysis()
{
//	Target_yuntai_init();
//	Yuntai_control();
	if(DJI_Remote.Switch_Left == 3 && DJI_Remote.Switch_Right == 1)//普通全向（关摩擦）
	{
		robot.tar_w = 0;//10.0f * (1024 - DJI_Remote.Chz_Left);		
		Transmit_to_slave_data(robot.tar_w);
		shoot_on = 0;
		gongdan_on = 0;
	}
	if(DJI_Remote.Switch_Left == 3 && DJI_Remote.Switch_Right == 3)//小陀螺（关摩擦）
	{	
		robot.tar_w = 0;//5000.0f;
		Transmit_to_slave_data(robot.tar_w);
		shoot_on = 0;
		gongdan_on = 0;
	}
	if(DJI_Remote.Switch_Left == 3 && DJI_Remote.Switch_Right == 2)//小陀螺
	{
		robot.tar_w = 0;//5000.0f;
		Transmit_to_slave_data(robot.tar_w);
		Fashe_control_remot();	
		shoot_on = 1;
		gongdan_on = 1;
	}
	if(DJI_Remote.Switch_Left == 2 && DJI_Remote.Switch_Right == 1)//滑轮控拨盘
	{
		robot.Motor_gongdan_speed = 10.0f * (1024 - DJI_Remote.Chz_Left);	
		shoot_on = 1;
		gongdan_on = 1;
	}
}

int shoot_tar = 0;
int gongdan_tar = 0;
void Motor_set()
{
	Motor_yaw.Motor_Angle_Set(robot.Motor_yaw);
	Motor_pitch.Motor_Angle_Set(robot.Motor_pitch);
//	if(shoot_on == 1)
//	{
		robot.Motor_zhao = shoot_tar;
		robot.Motor_xing = shoot_tar;
		robot.Motor_jie = shoot_tar;
		Motor_shoot_zhao.Motor_Speed_Set(robot.Motor_zhao);
		Motor_shoot_xing.Motor_Speed_Set(robot.Motor_xing);
		Motor_shoot_jie.Motor_Speed_Set(robot.Motor_jie);
//	}
//	else if(shoot_on == 0)
//	{
//		Motor_shoot_zhao.Set_DJIMotor_Output(0);
//		Motor_shoot_xing.Set_DJIMotor_Output(0);
//		Motor_shoot_jie.Set_DJIMotor_Output(0);
//	}
//	if(gongdan_on == 1)
//	{
		robot.Motor_gongdan_speed = gongdan_tar;
		Motor_gongdan.Motor_Speed_Set(robot.Motor_gongdan_speed);
//	}
//	else if(gongdan_on == 0)
//	{
//		Motor_gongdan.Set_DJIMotor_Output(0);
//	}
}

void Set_all_0()
{
	Motor_yaw.Set_DJIMotor_Output(0);
	Motor_pitch.Set_DJIMotor_Output(0);
	Motor_shoot_zhao.Set_DJIMotor_Output(0);
	Motor_shoot_xing.Set_DJIMotor_Output(0);
	Motor_shoot_jie.Set_DJIMotor_Output(0);
	Motor_gongdan.Set_DJIMotor_Output(0);
	Motor_little_yaw.Set_DJIMotor_Output(0);
}

FDCAN_TxHeaderTypeDef Tx;//实例化一个fdcan句柄
void Transmit_under()
{
	Tx.IdType = FDCAN_STANDARD_ID;								//标准帧，11 位 ID
	Tx.TxFrameType = FDCAN_DATA_FRAME;						//数据帧
	Tx.DataLength = FDCAN_DLC_BYTES_8;						//8 字节数据长度
	Tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		//活动错误状态
	Tx.BitRateSwitch = FDCAN_BRS_OFF;						//关闭位速率切换
	Tx.FDFormat = FDCAN_CLASSIC_CAN;							//经典 CAN 格式
	Tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	//不记录发送事件
	Tx.MessageMarker = 0;	//标记此消息，在 TX Event FIFO 中区分不同消息
	Tx.Identifier = 0x2ff;
//	CAN_2.FDCAN_Transmit(&Tx, Tx_can_data);
}

void TIM7_Function()
{
	leida.move_x = 1024;
	leida.move_y = 1024;	
	//leida_handle(computer, sizeof(computer));//解算雷达传回来的信息	
	USART_handle(&robot);
	if(DJI_Remote.Switch_Left == 1)
	{
		shoot_on = 0;
		gongdan_on = 0;
//		Set_all_0();
		Motor_set();
	}
	else if(DJI_Remote.Switch_Left == 3)
	{	
		Tar_motor_analysis();
		Motor_set();
	}
	else if(DJI_Remote.Switch_Left == 2)
	{
		Tar_motor_analysis();
		Motor_set();
	}
	Transmit_to_slave_data(robot.tar_w);
	CAN_1.FDCAN_TransmitForMotor();
	//CAN_2.FDCAN_TransmitForMotor();
	//CAN_3.FDCAN_TransmitForMotor();
	//Transmit_under();
}

void ALL_init()
{
	UART_5.UDB_Recevice_Init(&huart5, IFR_DJI_Remote_Analysis);
	CAN_1.FDCAN_Init(&hfdcan1, &Motor_shoot_zhao);
	CAN_1.FDCAN_Init(&hfdcan1, &Motor_shoot_xing);
	CAN_1.FDCAN_Init(&hfdcan1, &Motor_shoot_jie);
	CAN_1.FDCAN_Init(&hfdcan1, &Motor_gongdan);	
//	CAN_1.FDCAN_Init(&hfdcan1, &Motor_yaw);	
	CAN_2.FDCAN_Init(&hfdcan2, &Motor_little_yaw);			
//	//CAN_2.FDCAN_Init(&hfdcan2, &Can_user_define);
//	CAN_3.FDCAN_Init(&hfdcan3, &Motor_pitch);	
	Motor_yaw.Speed_PID.PID_Init(30, 0, 4, 16000, 8000, 16000, 0);//yaw速度环输出电流
	Motor_yaw.Angle_PID.PID_Init(60, 0, 25, 20000, 3000, 10000, 0);//yaw位置环输出速度目标
	Motor_pitch.Speed_PID.PID_Init(30, 0, 10, 16000, 8000, 16000, 0);//yaw速度环输出电流
	Motor_pitch.Angle_PID.PID_Init(40, 0, 20, 20000, 3000, 10000, 0);//yaw位置环输出速度目标
	Motor_gongdan.Pos_PID.PID_Init(2.8, 0.007, 8, 10000, 5000, 20000, 0);
	Motor_gongdan.Speed_PID.PID_Init(20, 0, 2, 12000, 8000, 8000, 0);
	Motor_shoot_zhao.Speed_PID.PID_Init(15, 0.0001, 1, 12000, 1000, 10000, 0);
	Motor_shoot_xing.Speed_PID.PID_Init(15, 0.0001, 1, 12000, 1000, 10000, 0);	
	Motor_shoot_jie.Speed_PID.PID_Init(10, 0.0001, 1, 12000, 1000, 10000, 0);
	Motor_little_yaw.Speed_PID.PID_Init(30, 0, 10, 16000, 8000, 16000, 0);
	Motor_little_yaw.Angle_PID.PID_Init(60, 0, 25, 20000, 3000, 10000, 0);
	//Wheel_Pos_Follow.PID_Init(8, 0.0005, 0, 4000, 4000, 8000, 100);//底盘跟随
//	TIM_7.TIM_ITStart(&htim7, TIM7_Function);
}
