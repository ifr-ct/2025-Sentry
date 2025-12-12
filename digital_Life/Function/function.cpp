#include "math.h"
#include "function.h"
#define CHASIS           0x00U
#define YUNTAI           0x01U
#define CHASIS_OR_YUNTAI CHASIS
#define PAI 3.1415926f
#define Wheel_tar 9.70518684f//3.42085838f//初始化正方向yaw角度值电池冲前
#define P_MIN -12.57f
#define P_MAX 12.57f
#define V_MIN -44.0f
#define V_MAX 44.0f
#define T_MIN -17.0f
#define T_MAX 17.0f
IFR_CAN_ClassDef CAN_1;//实例化CAN1
IFR_CAN_ClassDef CAN_2;//实例化CAN2
IFR_TIM_ClassDef TIM_7;//实例化TIM7
IFR_UDBRx_ClassDef USART_3;//串口接收遥控器键鼠信息
IFR_UDBRx_ClassDef USART_1;//串口接收裁判系统
IFR_Speed_Motor Motor_LF(0x201);//轮向
IFR_Speed_Motor Motor_RF(0x202);
IFR_Speed_Motor Motor_RB(0x203);
IFR_Speed_Motor Motor_LB(0x204);
IFR_Pos_Motor Motor_direction_LF(0x205);//舵向
IFR_Pos_Motor Motor_direction_RF(0x206);
IFR_Pos_Motor Motor_direction_RB(0x207);
IFR_Pos_Motor Motor_direction_LB(0x208);
//上下通信自定义标准帧帧ID(0X007)
Robot robot;//机器人电机速度等目标值结构体
Vehicle vehicle;//舵轮解算结构体
Rx_can_data rx_can_data;//自定义帧信息结构体
Judge_DataTypedef Judge_data;//裁判系统结构体
Yaw yaw_info;//yaw信息结构体

float angle_LF_w = 0;//自旋时解算角度转换坐标之后360°制的目标值
float angle_RF_w = 0;
float angle_RB_w = 0;
float angle_LB_w = 0;
float tar_x_0;//坐标系转换后的遥控器目标x方向
float tar_y_0;//坐标系转换后的遥控器目标y方向
double Change = 0;
uint32_t connect = 0;//遥控器断连标志位
float copies = 0;//斜坡函数运算的次数
float last_top = 0;
float ramp_function(float top, float time_ms)//斜坡函数
{
	float tar_out = 0;	
	if(top == 0)
	{
		copies = 0;
		return 0;
	}
	if(last_top == top)//跟上一次目标值一样
	{
		copies++;//逐渐增加
		if(copies < time_ms)
		{
			tar_out = copies / time_ms * top;//目标速度递增
		}
		else
		{
			tar_out = top;
		}
	}
	else
	{
		if(last_top < top)//速度目标值增加
		{
			copies++;//逐渐增加
			if(copies < time_ms)
			{
				tar_out = copies / time_ms * top;//目标速度递增
			}
			else
			{
				tar_out = top;
			}
		}
		else
		{
			tar_out = top * (copies / time_ms);
		}
	}
	last_top = top;	
	return tar_out;
}

float Angle_limit(float angle_max, float angle_min, float angle_now)
{
	if(angle_now >= angle_max)
	{
		angle_now -= angle_max;
	}
	else if(angle_now < angle_min)
	{
		angle_now += angle_max;
	}
	return angle_now;
}

void Wheel_analytic_function(float tar_x, float tar_y, float tar_w)//轮子目标值解算
{
	float Lx0;
	float Lx1;
	float Ly0;
	float Ly1;

	if(tar_w <= 5)//近似为没有自转
	{
		tar_w = 0.00000000001f;
	}
	vehicle.L = 0.474f;
	vehicle.D = 0.474f;
	vehicle.Angle = atan2(tar_y, tar_x);//3点钟方向逆时针，0-pai，-pai-0
	vehicle.V = sqrt(tar_x * tar_x + tar_y * tar_y);		
	vehicle.R = vehicle.V / tar_w;
	if(vehicle.Angle < 0)
	{
		vehicle.Angle += 2 * PAI;//3点钟方向逆时针0-2pai
	}
	Lx0 = vehicle.L / 2 + vehicle.R * cos(vehicle.Angle);
	Lx1 = Lx0 - vehicle.L;
	Ly0 = vehicle.D / 2 + vehicle.R * sin(vehicle.Angle);
	Ly1 = vehicle.D - Ly0;
	angle_LF_w = atan2(Ly0, Lx0);//(1,1)
	angle_RF_w = atan2(Ly0, Lx1);//(-1,1)
	angle_RB_w = atan2(-Ly1, Lx1);//(-1,-1)
	angle_LB_w = atan2(-Ly1, Lx0);//(1,-1)
	if(angle_LF_w < 0)
	{
		angle_LF_w += 2 * PAI;//3点钟方向逆时针0-2pai
	}
	if(angle_RF_w < 0)
	{
		angle_RF_w += 2 * PAI;//3点钟方向逆时针0-2pai
	}
	if(angle_RB_w < 0)
	{
		angle_RB_w += 2 * PAI;//3点钟方向逆时针0-2pai
	}	if(angle_LB_w < 0)
	{
		angle_LB_w += 2 * PAI;//3点钟方向逆时针0-2pai
	}
	robot.Motor_dir[LF][DIRECTION] = angle_RB_w;//设置舵向电机角度目标值360度制
	robot.Motor_dir[RF][DIRECTION] = angle_RF_w;
	robot.Motor_dir[RB][DIRECTION] = angle_LF_w;;
	robot.Motor_dir[LB][DIRECTION] = angle_LB_w;	
	vehicle.R1 = sqrt(Lx0 * Lx0 + Ly0 * Ly0);
	vehicle.R2 = sqrt(Lx1 * Lx1 + Ly0 * Ly0);
	vehicle.R3 = sqrt(Lx1 * Lx1 + Ly1 * Ly1);
	vehicle.R4 = sqrt(Lx0 * Lx0 + Ly1 * Ly1);
	vehicle.V1 = vehicle.R1 * tar_w;
	vehicle.V2 = vehicle.R2 * tar_w;
	vehicle.V3 = vehicle.R3 * tar_w;
	vehicle.V4 = vehicle.R4 * tar_w;
	robot.Motor[LF][SPEED] = vehicle.V1;
	robot.Motor[RF][SPEED] = vehicle.V2;
	robot.Motor[RB][SPEED] = vehicle.V3;
	robot.Motor[LB][SPEED] = vehicle.V4;	
}

void Motor_tar_optimize_change(float tar_now_min, float *Angle_motr_tar, float *Speed_motor_tar)//舵向目标角度最短路径，轮向速度转换
{
	if(tar_now_min >= 2048)
	{
		*Speed_motor_tar *= -1;		
		*Angle_motr_tar -= 4096;
	}
	else if(tar_now_min < -2048)
	{			
		*Speed_motor_tar *= -1;	
		*Angle_motr_tar += 4096;
	}
}

void Calculate_excursion()//计算x,y的偏移
{
	Change = yaw_info.Angle - Wheel_tar;//计算角度误差并转化为弧度制
	tar_x_0 = cos(Change) * robot.tar_x + robot.tar_y * sin(Change);//x轴的偏移，tar_x的改变
	tar_y_0 = cos(Change) * robot.tar_y - robot.tar_x * sin(Change);//y轴的偏移，tar_y的改变 		
	robot.tar_x = tar_x_0;
	robot.tar_y = tar_y_0;
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

void RXCAN_Analysis(uint8_t *Data, uint32_t stdid)
{
	if(stdid == 0x007)//自定义数据帧解码
	{
		//memcpy(rxdata_209, Data, 8);//复制遥控器数据
		Judge_data.buffer_energy = ((uint16_t)Data[0] | (uint16_t)Data[1] << 8) & 0xFFFF;
		rx_can_data.Buffer_energy = ((uint16_t)Data[0] | (uint16_t)Data[1] << 8) & 0xFFFF;
		rx_can_data.Left_x = (((int16_t)Data[2] >> 6) | ((int16_t)Data[3] << 2) | ((int16_t)Data[4] << 10)) & 0x07FF;
		rx_can_data.Left_y = (((int16_t)Data[4] >> 1) | ((int16_t)Data[5]<<7)) & 0x07FF;
		rx_can_data.Modle_s1 = ((Data[5] >> 4) & 0x000C) >> 2;
		rx_can_data.Modle_s2 = ((Data[5] >> 4) & 0x0003);
		rx_can_data.W = ((Data[6] & 0x3F) << 8) | Data[7];
		rx_can_data.home = (Data[6] & 0x40) >> 6;
		rx_can_data.old = Data[6] >> 7;//旧遥控器为1
		connect = 0;
	}
}
		
void RXCAN_Analysis_LZ(uint8_t *Data, uint32_t exdid)
{
	if((uint8_t)((exdid & 0xFF00) >> 8) == 1)//确认canid
	{
		if (int((exdid & 0x3F000000) >> 24) == 2) // 判断是否为通信类型2
		//获取yaw电机角度
		{
			yaw_info.Angle = uint16_to_float(Data[0] << 8 | Data[1], P_MIN, P_MAX, 16);
			yaw_info.Speed = uint16_to_float(Data[2] << 8 | Data[3], V_MIN, V_MAX, 16);
			yaw_info.Torque = uint16_to_float(Data[4] << 8 | Data[5], T_MIN, T_MAX, 16);
			yaw_info.Temperature = (Data[6] << 8 | Data[7]) / 10.0f;
		}
	}
}

void Duolun_angle(uint16_t zero_angle, float *Tar_angle_8192, uint16_t Now_angle, float *Tar_speed)//Tar_angle为8192制
{
	float tar_error = 0;
	*Tar_angle_8192 += zero_angle;
	if(*Tar_angle_8192 > 8191)
	{
		*Tar_angle_8192 -= 8192;
	}
	tar_error = *Tar_angle_8192 - Now_angle;
	if(tar_error >= 4096)//最短
	{
		tar_error -= 8192;
		*Tar_angle_8192 -= 8192;
	}
	else if(tar_error <= -4096)
	{
		tar_error += 8192;
		*Tar_angle_8192 += 8192; 
	}
	Motor_tar_optimize_change(tar_error, Tar_angle_8192, Tar_speed);//电机角度值改变
}

float Wm = 2.5f;//2.5
float XY = 18000.f;//18000
void Tar_analysis()//设置电机目标
{
	if(rx_can_data.Left_x <= 1040 && rx_can_data.Left_x >= 1008)
	{
		rx_can_data.Left_x = 1024;
	}
	if(rx_can_data.Left_y <= 1040 && rx_can_data.Left_y >= 1008)
	{
		rx_can_data.Left_y = 1024;
	}
	robot.tar_x = XY * (rx_can_data.Left_x - 1024) / 660.0f;//rx_can_data.Left_x;8800
	robot.tar_y = XY * (rx_can_data.Left_y - 1024) / 660.0f;//rx_can_data.Left_y;8800
	robot.tar_w = Wm * rx_can_data.W;//3.2f
	if(robot.tar_w > 5)
	{
		robot.tar_x = robot.tar_x + 54;
		robot.tar_y = robot.tar_y + 44;//85
	}
	Calculate_excursion();//坐标系变换
	Wheel_analytic_function(robot.tar_x, robot.tar_y, robot.tar_w);
	robot.Motor_dir[LF][DIRECTION] = robot.Motor_dir[LF][DIRECTION] / 2.f / PAI * 8192.f;
	robot.Motor_dir[RF][DIRECTION] = robot.Motor_dir[RF][DIRECTION] / 2.f / PAI * 8192.f;
	robot.Motor_dir[RB][DIRECTION] = robot.Motor_dir[RB][DIRECTION] / 2.f / PAI * 8192.f;
	robot.Motor_dir[LB][DIRECTION] = robot.Motor_dir[LB][DIRECTION] / 2.f / PAI * 8192.f;
	Duolun_angle(4489, &robot.Motor_dir[LF][DIRECTION], Motor_direction_LF.Pos_Info.Angle, &robot.Motor[LF][SPEED]);//4423//2441
	Duolun_angle(6225, &robot.Motor_dir[RF][DIRECTION], Motor_direction_RF.Pos_Info.Angle, &robot.Motor[RF][SPEED]);//2602//4657
	Duolun_angle(7254, &robot.Motor_dir[RB][DIRECTION], Motor_direction_RB.Pos_Info.Angle, &robot.Motor[RB][SPEED]);//7208//5206
	Duolun_angle(5721, &robot.Motor_dir[LB][DIRECTION], Motor_direction_LB.Pos_Info.Angle, &robot.Motor[LB][SPEED]);//5714//7769
}

void Set_all_0()
{
	Motor_LF.Set_DJIMotor_Output(0);
	Motor_RF.Set_DJIMotor_Output(0);
	Motor_RB.Set_DJIMotor_Output(0);
	Motor_LB.Set_DJIMotor_Output(0);
	Motor_direction_LF.Set_DJIMotor_Output(0);
	Motor_direction_RF.Set_DJIMotor_Output(0);
	Motor_direction_RB.Set_DJIMotor_Output(0);
	Motor_direction_LB.Set_DJIMotor_Output(0);
}

float limit_dipan;
void Power_control(Robot *robot0, uint16_t buffer_energy)//功率限制
{
	if(buffer_energy <= 60)
	{
		limit_dipan = buffer_energy / 60.0f;
	}
	else
	{
		limit_dipan = 1;
	}
	if (buffer_energy <= 0)
	{
		Motor_LF.Set_DJIMotor_Output(0);
		Motor_RF.Set_DJIMotor_Output(0);
		Motor_RB.Set_DJIMotor_Output(0);
		Motor_LB.Set_DJIMotor_Output(0);
	}
	Motor_LF.Set_DJIMotor_Output(Motor_LF.Get_DJIMotor_Output() * limit_dipan);
	Motor_RF.Set_DJIMotor_Output(Motor_RF.Get_DJIMotor_Output() * limit_dipan);
	Motor_RB.Set_DJIMotor_Output(Motor_RB.Get_DJIMotor_Output() * limit_dipan);
	Motor_LB.Set_DJIMotor_Output(Motor_LB.Get_DJIMotor_Output() * limit_dipan);
}

void Motor_set()
{
//	robot.Motor[LF][SPEED] = ramp_function(robot.Motor[LF][SPEED], 1000.f);
//	robot.Motor[RF][SPEED] = ramp_function(robot.Motor[RF][SPEED], 1000.f);
//	robot.Motor[RB][SPEED] = ramp_function(robot.Motor[RB][SPEED], 1000.f);
//	robot.Motor[LB][SPEED] = ramp_function(robot.Motor[LB][SPEED], 1000.f);
	if(rx_can_data.Left_x == 1024 && rx_can_data.Left_y == 1024 && rx_can_data.W == 0)//不转向不移动时舵向轮子为当前值
	{
		robot.Motor_dir[LF][DIRECTION] = Motor_direction_LF.Pos_Info.Angle;
		robot.Motor_dir[RF][DIRECTION] = Motor_direction_RF.Pos_Info.Angle;
		robot.Motor_dir[RB][DIRECTION] = Motor_direction_RB.Pos_Info.Angle;
		robot.Motor_dir[LB][DIRECTION] = Motor_direction_LB.Pos_Info.Angle;
	}
	Motor_direction_LF.Motor_Pos_Set(robot.Motor_dir[LF][DIRECTION]);
	Motor_direction_RF.Motor_Pos_Set(robot.Motor_dir[RF][DIRECTION]);
	Motor_direction_RB.Motor_Pos_Set(robot.Motor_dir[RB][DIRECTION]);
	Motor_direction_LB.Motor_Pos_Set(robot.Motor_dir[LB][DIRECTION]);
	Motor_LF.Motor_Speed_Set(robot.Motor[LF][SPEED]);//设置轮向电机速度值	
	Motor_RF.Motor_Speed_Set(robot.Motor[RF][SPEED]);
	Motor_RB.Motor_Speed_Set(-robot.Motor[RB][SPEED]);//跟对零点时轮子的方位有关，此坐标系下零点后轮速度时反的
	Motor_LB.Motor_Speed_Set(-robot.Motor[LB][SPEED]);
	//Power_control(&robot, Judge_data.buffer_energy);
}
uint8_t time_s = 0;
uint16_t time_ms = 0;
void TIM7_Function()
{
	connect ++;
	if((rx_can_data.Modle_s1 == 1 && rx_can_data.Modle_s2 == 1 && rx_can_data.old == 1) || (rx_can_data.old == 0 && rx_can_data.home == 1))
	{
		Set_all_0();
	}
	else// if(rx_can_data.Modle_s1 == 1 && rx_can_data.Modle_s2 == 1)
	{
		Tar_analysis();
		Motor_set();
	}
	if(connect >= 150)
	{
		Set_all_0();
	}
	CAN_1.CAN_TransmitForMotor();
	CAN_2.CAN_TransmitForMotor();
}

void InitializeRxCanData(Rx_can_data *data)
{
    data->Modle_s1 = 1;         // S1 初始化为 1
    data->Modle_s2 = 1;         // S2 初始化为 1
    data->Left_x = 1024;        // Left_x 初始化为 1024
    data->Left_y = 1024;        // Left_y 初始化为 1024
    data->Right_x = 1024;       // Right_x 初始化为 1024
    data->Right_y = 1024;       // Right_y 初始化为 1024
		data->Buffer_energy = 0;    // Buffer_energy 初始化为 0
    data->W = 0;                // W 初始化为 0
}

void ALL_init()
{
	InitializeRxCanData(&rx_can_data);//初始化自定义帧
	CAN_2.CAN_Analysis_Function_std = RXCAN_Analysis;//标准自定解析
	CAN_2.CAN_Analysis_Function_exd = RXCAN_Analysis_LZ;//拓展自定解析
	CAN_2.CAN_Init(&hcan2, &Motor_direction_LF);
	CAN_1.CAN_Init(&hcan1, &Motor_direction_RF);
	CAN_2.CAN_Init(&hcan2, &Motor_direction_RB);
	CAN_2.CAN_Init(&hcan2, &Motor_direction_LB);
	CAN_1.CAN_Init(&hcan1, &Motor_LF);
	CAN_1.CAN_Init(&hcan1, &Motor_RF);
	CAN_1.CAN_Init(&hcan1, &Motor_RB);
	CAN_1.CAN_Init(&hcan1, &Motor_LB);
	//CAN_2.CAN_Init(&hcan2, &Can_user_define);
	Motor_direction_LF.Speed_PID.PID_Init(10, 0.01, 0, 18000, 8000, 10000, 0);//1号舵向6215速度环
	Motor_direction_LF.Pos_PID.PID_Init(15, 0.01, 0.5, 12000, 4000, 10000, 0);
	Motor_direction_RF.Speed_PID.PID_Init(10, 0.01, 0, 18000, 8000, 10000, 0);//2号舵向6215速度环
	Motor_direction_RF.Pos_PID.PID_Init(15, 0.01, 0, 12000, 4000, 10000, 0);	
	Motor_direction_LB.Speed_PID.PID_Init(10, 0, 1, 18000, 8000, 10000, 0);//3号舵向6215速度环
	Motor_direction_LB.Pos_PID.PID_Init(15, 0.01, 0.8, 12000, 4000, 10000, 0);
	Motor_direction_RB.Speed_PID.PID_Init(10, 0.01, 0, 18000, 8000, 10000, 0);//4号舵向6215速度环
	Motor_direction_RB.Pos_PID.PID_Init(15, 0.01, 3, 12000, 4000, 10000, 0);
	Motor_LF.Speed_PID.PID_Init(12, 0.08, 1, 10000, 700, 12000, 0);//1号轮向3508
	Motor_RF.Speed_PID.PID_Init(12, 0.08, 1, 10000, 700, 12000, 0);//2号轮向3508
	Motor_RB.Speed_PID.PID_Init(12, 0.09, 1, 10000, 700, 12000, 0);//3号轮向3508
	Motor_LB.Speed_PID.PID_Init(12, 0.08, 1, 10000, 700, 12000, 0);//4号轮向3508
	TIM_7.TIM_ITStart(&htim7, TIM7_Function);
}
