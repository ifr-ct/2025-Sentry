#include "math.h"
#include "function.h"
#include "usbd_cdc_if.h"
#include "tf.h"
#include "forecast.h"
IFR_FDCAN_ClassDef CAN_1;
IFR_FDCAN_ClassDef CAN_2;
IFR_FDCAN_ClassDef CAN_3;
IFR_TIM_ClassDef TIM_7;
IFR_TIM_ClassDef TIM_6;
IFR_UDBRx_ClassDef USART_1;//串口接收裁判系统
IFR_UDBRx_ClassDef RS485;//485接收外置陀螺仪
IFR_UDBRx_ClassDef UART_5;//串口接收遥控器键鼠信息
IFR_UDBRx_ClassDef USART_10;//串口接收新遥控器
SPI_InitTypeDef SPI_2;//陀螺仪
IFR_Speed_Motor null(9);
IFR_Speed_Motor Motor_shoot_zhao(0x201);//fdcan1
IFR_Speed_Motor Motor_shoot_xing(0x202);//fdcan1
IFR_Speed_Motor Motor_shoot_jie(0x203);//fdcan1
IFR_Pos_Motor Motor_gongdan(0x204);//fdcan1
IFR_GyroControl_LZ_Motor Motor_yaw(turn, 1);//陀螺仪角度,fdcan2
IFR_GyroControl_LZ_Motor Motor_little_yaw(3);//陀螺仪角度,fdcan3
//IFR_Pos_LZ_Motor Motor_pitch(killG, 2);//陀螺仪角度,fdcan3
IFR_GyroControl_LZ_Motor Motor_pitch(killG, 2);//陀螺仪角度,fdcan3
//IFR_Speed_Motor Can_user_define(0x20A);//上下通信帧,fdcan2
Remote_RxDataTypedef Remote_RxData;//遥控器数据
//New_DJI_Remote;新遥控器数据
extern IMU_DM_TypeDef IMU_DM_Info;//达妙陀螺仪数据
Sentry_tx_cmd sentry_tx_cmd;//哨兵复活帧
AntiGyro ag;
extern uint8_t computer[50];//自瞄信息
extern Leida leida;//回传雷达自瞄值
extern float leida_count_time;
extern float camera_yaw;
extern float camera_pitch;
extern float leida_yaw;
extern float leida_pitch;
extern float leida_yaw_v;
extern float leida_pitch_v;
extern float leida_distance;//m
Robot robot;//电机目标值
#define PAI 3.1415926f
#define SINGLE_SHOOT_POS 36864  //转一圈所需要的角度
#define LITTLE_yaw 5.72214508f//小yaw位于大yaw中央对应小yaw位置
#define Pitch_zero 5.37314143f//5.36346817f//陀螺仪零点对应pitch角度
float pitch_angle_err = 0;//pitch位置对应陀螺仪零点位置的误差，用作自瞄所需角度转换
uint8_t set = 0;//模式标志位
uint8_t flag = 1;//云台初始化标志位
uint8_t shoot_on = 0;//摩擦轮标志位
uint8_t gongdan_on = 0;//拨盘标志位
uint8_t test_auto = 0;//扫描测试模式标志位
uint32_t lose_time = 0;//遥控器断连时间
uint32_t new_lose_time = 0;//新遥控器断连时间
uint8_t remote_lose = 0;//遥控器是否断连
uint8_t new_remote_lose = 0;//新遥控器是否断连
uint8_t leida_update = 0;//小电脑数据标志位
uint32_t leida_losetime = 0;//小电脑丢失时间
uint32_t time_ms;//程序运行总ms数
float ms_time = 0;//比赛计时器ms
float ss_time = 0;//比赛计时器s
uint8_t state_home = 0;//当前状态
uint8_t state_button = 0;
uint8_t state_fn = 0;
uint8_t state_4 = 0;
uint8_t sure_home = 1;//确定按下
uint8_t sure_button = 0;
uint8_t sure_fn = 0;
uint8_t sure_4 = 0;
float Output_temp[50] = {0};//重力补偿数据列表
uint8_t old = 0;//用作自定义帧新旧遥控器哪个的
float copies = 0;//斜坡函数运算的次数
uint8_t leida_data[30];//小电脑回传原始数据
uint8_t modle_err = 0;//重要模块离线标志位
uint8_t size_8 = 0;//裁判系统帧头长度
uint8_t size_16 = 0;//裁判系统数据帧长度
int8_t k = 0;//循环列表索引
uint16_t fashe_speed = 6100;
float imu_yaw_past[31] = {0};//历史30ms陀螺仪yaw
float imu_pitch_past[31] = {0};//历史30ms陀螺仪pitch
float imu_roll_past[31] = {0};
float imu_yaw_w_past[31] = {0};
float imu_pitch_w_past[31] = {0};
int8_t t_delay = 22;//相机触发采集到下位机接到上位机的时间，用于取相机触发采集那一帧的陀螺仪
TF_Tree tf_tree = TF_Tree::makeSentry();//构造创建一个TF_Tree对象
TF_Vec3 raw_p(0,0,0),target_p(0,0,0);
TF_Vec3 imu_r(0,0,0),imu_a(0,0,0),imu_v(0,0,0),imu_v_a(0,0,0),raw_r(0,0,0),raw_a(0,0,0),target_r(0,0,0),vel_r(0,0,0),zero(0,0,0);
TF_Vec3 target_r_angle(0,0,0),vel_r_angle(0,0,0),target_vel(0,0,0),target_vel_angle(0,0,0),
imu_r_vel(0,0,0),imu_angle_vel(0,0,0),targetv_r(0,0,0),targetv_a(0,0,0),raw_targetv_r(0,0,0),raw_targetv_a(0,0,0),target_v_xianxing(0,0,0);

TF_Vec3 lst_target_r(0,0,0);

double pred_delay = 0;
extern uint8_t update_imu;

#ifndef M_PI
# define M_PI		3.14159265358979323846f	/* pi */
#endif
#define ANG2RAD(a) ((a)*M_PI/180.f)
#define RAD2ANG(r) ((r)*180.f/M_PI)
void Set_all_0()
{
	Motor_yaw.LZ_Motor_Torque_control(0);
	Motor_pitch.LZ_Motor_Torque_control(0);
	Motor_little_yaw.LZ_Motor_Torque_control(0);	
	Motor_shoot_zhao.Set_DJIMotor_Output(0);
	Motor_shoot_xing.Set_DJIMotor_Output(0);
	Motor_shoot_jie.Set_DJIMotor_Output(0);
	Motor_gongdan.Set_DJIMotor_Output(0);
}

void USART_handle(Robot *robot0)//遥控器底盘目标值算法
{
	if(new_remote_lose == 1)//如果新遥控器确认断连
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
		robot0 ->tar_x = 1000.0f * (DJI_Remote.Chx_Left - 1024) / 660.0f;
		robot0 ->tar_y = 1000.0f * (DJI_Remote.Chy_Left - 1024) / 660.0f;
	}
	else
	{
		robot0 ->tar_x = 1000.0f * (New_DJI_Remote.Chx_Left - 1024) / 660.0f;
		robot0 ->tar_y = 1000.0f * (New_DJI_Remote.Chy_Left - 1024) / 660.0f;		
	}
//	for	(int i = -24 ; i < 16; i++)
//	{
//		if (Output_temp[i + 24] == 0 && i <= IMU_DM_Info.Angle.roll + 0.1f && i >= IMU_DM_Info.Angle.roll - 0.1f)
//			Output_temp[i + 24] = Motor_pitch.Get_LZ_Motor_Output();
//	}
}

float USART_lose()
{
	if(new_remote_lose == 0)
	{
		old = 0;
	}
	if (New_DJI_Remote.Updata == 0)//新遥控器判断是否断连
	{
		new_lose_time++;
		if (new_lose_time > 180)//17*5=85 丢5包数据
		{
			new_remote_lose = 1;
			if(new_remote_lose == 1)//新遥控器断连看旧遥控器
			{
				if (DJI_Remote.Updata == 0)//旧遥控器判断是否断连
				{
					lose_time++;
					if (lose_time > 180)//17*5=85 丢5包数据
					{
						if(DJI_Remote.Switch_Left == 1 && DJI_Remote.Switch_Right == 1)//全停模式断连只给底下发就够了，避免两次set_all_0把can发炸
						{
							//强制停止
							DJI_Remote.Switch_Left = 1;
							DJI_Remote.Switch_Right = 1;
							Tx_can_data[4] = 0x01;
							Tx_can_data[5] = 0x58;
							remote_lose = 1;//新遥控器确认断连
						}
						else
						{
							//强制停止
							DJI_Remote.Switch_Left = 1;
							DJI_Remote.Switch_Right = 1;
							Tx_can_data[4] = 0x01;
							Tx_can_data[5] = 0x58;
							Set_all_0();
							remote_lose = 1;//新遥控器确认断连
						}
					}
				}
				else
				{
					lose_time = 0;
					remote_lose = 0;
					old = 0x80;
				}
			}
		}
	}
	else
	{
		new_remote_lose = 0;
		new_lose_time = 0;
		old = 0;
	}
	DJI_Remote.Updata = 0;
	New_DJI_Remote.Updata = 0;	
	return 0.f;
}

float killG(float Motor_tar)
{
	return -5.91771542e-04f * IMU_DM_Info.Angle.roll * IMU_DM_Info.Angle.roll + 4.97485960e-04f * IMU_DM_Info.Angle.roll + 1.1675446e+00f;//- 0.00013 * Motor_shoot_zhao.Pos_Info.Speed
}

float turn(float now_turn)
{
	if(my_buff.remaining_energy == 0 && robot.tar_w != 0)
	{
		return 0.2f;
	}
	else
	{
		if(robot.tar_w == 5000)
		{
			return 2.5f;
		}
		else if(robot.tar_w == 7500)
		{
			return 2.8f;
		}
		else if(robot.tar_w == 4000)
		{
			return 2.0f;
		}
		else
		{
			return 0.f;
		}
	}
}

float Judge_HalfTurnJump(float Motor_tar_angle, float Motor_now_angle, float Circle_angle)//处理角度跳变
{
	if(Motor_tar_angle - Motor_now_angle >= Circle_angle / 2)
	{  
		Motor_tar_angle -= Circle_angle;
	}
	else if(Motor_tar_angle - Motor_now_angle <= -Circle_angle / 2)
	{
		Motor_tar_angle += Circle_angle;	
	}
	return Motor_tar_angle;
}

float ramp_function(float top, float time_ms)//斜坡函数
{
	float tar_out = 0;
	if(top == 0)
	{
		copies = 0;
		return 0;
	}
	else
	{
		copies++;	
		tar_out = copies / time_ms * top;
		if(copies >= time_ms)
		{
			return top;
		}
		else
		{
			return tar_out;
		}
	}
}

uint8_t Key_detection(uint64_t key, uint8_t *state, uint8_t sure)
{
	if(key == 1)
	{
		*state = 1;//确认按下
	}
	if(*state == 1 && key == 0 && sure == 0)
	{
		*state = 0;//确认松开
		sure = 1;//切换模式
	}
	else if(*state == 1 && key == 0 && sure == 1)
	{
		*state = 0;//确认松开
		sure = 0;//切换模式
	}
	return sure;
}

uint8_t Ammo_Booster_Plug_Flag = 0;//开始反转标志
uint32_t Ammo_Booster_HighOut_StartTime = 0;//当时间为0时，表示没有记录
int32_t temp_tar_feed_pos;
float food_num = 0;
void gongdan_judge()
{
	if (fabs((float)Motor_gongdan.Pos_Info.Electric) > 6000 && abs((float)Motor_gongdan.Pos_Info.Speed) < 80 && Ammo_Booster_Plug_Flag == 0)//超过堵转界限，记录时间
	{
		if (Ammo_Booster_HighOut_StartTime == 0)//记录高输出开始时间
		{
			Ammo_Booster_HighOut_StartTime = HAL_GetTick();		
		}
		else if (HAL_GetTick() - Ammo_Booster_HighOut_StartTime > 250)//堵转超过250ms
		{
			Ammo_Booster_Plug_Flag = 1;
			temp_tar_feed_pos = Motor_gongdan.Pos_Info.Abs_Angle - (SINGLE_SHOOT_POS / 2);//回转半颗的距离
		}
	}
	else//清空堵转状态
	{
		Ammo_Booster_HighOut_StartTime = 0;
	}
	if (Ammo_Booster_Plug_Flag)//开始反转
	{
		if (abs((float)(temp_tar_feed_pos - Motor_gongdan.Pos_Info.Abs_Angle)) < 5000)//位置误差超过1000
		{
			Motor_gongdan.Motor_AbsPos_Set(Motor_gongdan.Pos_Info.Abs_Angle);
			return;//退出函数
		}
		else//反转完成
		{
			Ammo_Booster_Plug_Flag = 0;
			Ammo_Booster_HighOut_StartTime = 0;
			while (food_num*SINGLE_SHOOT_POS - Motor_gongdan.Pos_Info.Abs_Angle > SINGLE_SHOOT_POS) food_num--;
		}
	}
	if(Ammo_Booster_Plug_Flag == 0)	Motor_gongdan.Motor_AbsPos_Set(SINGLE_SHOOT_POS*food_num);
}

void Heat_control()//热量限制
{
	if(shoot_on == 1 && gongdan_on == 1)
	{
		if (Judge_data.shooter_17mm_1_barrel_heat >= Judge_data.shooter_barrel_heat_limit - 60)
		{
			robot.Motor_gongdan_speed = 0;
		}
		else
		{
			shoot_on = 1;
			gongdan_on = 1;
		}
		if(gongdan_on == 1)
		{
			//gongdan_judge();
		}
	}
}

uint16_t fashe_modle_judge = 0;//做延时发射用
void Fashe_control_remot()//发射控制
{
	if(DJI_Remote.Switch_Left == 2 && DJI_Remote.Switch_Right == 2)//雷达模式
	{
		if(shoot_on == 1)
		{
			robot.Motor_zhao = fashe_speed;
			robot.Motor_xing = fashe_speed;
			robot.Motor_jie = fashe_speed;			
		}
		else
		{
			robot.Motor_zhao = 0;
			robot.Motor_xing = 0;
			robot.Motor_jie = 0;
		}
		if(gongdan_on == 1)
		{
			robot.Motor_gongdan_speed = 6800;
		}
		else
		{
			robot.Motor_gongdan_speed = 0;
		}
	}
	else//其他模式
	{
		if(shoot_on == 1)
		{
			fashe_modle_judge ++;
			if(fashe_modle_judge > 1000)
			{
				robot.Motor_zhao = fashe_speed;//6680
				robot.Motor_xing = fashe_speed;
				robot.Motor_jie = fashe_speed;	
			}
		}
		if(shoot_on == 0)
		{
			robot.Motor_zhao = 0;
			robot.Motor_xing = 0;
			robot.Motor_jie = 0;	
			fashe_modle_judge = 0;	
		}
		if(gongdan_on == 1)
		{
			if(DJI_Remote.Chz_Left != 1024)
			{
				robot.Motor_gongdan_speed = 6800;
			}
			else
			{
				robot.Motor_gongdan_speed = 0;
			}
		}
		if(gongdan_on == 0)
		{
			robot.Motor_gongdan_speed = 0;	
		}
	}
	Heat_control();
}

void New_Fashe_control_remot()//发射控制
{	
	if(New_DJI_Remote.mode_sw == 2)//雷达自瞄模式模式
	{
		shoot_on = 1;//摩擦轮常开
	}
	if(New_DJI_Remote.mode_sw != 2 )//非雷达自瞄模式
	{
		if(sure_fn == 1)//按键决定是否开摩擦
		{
			shoot_on = 1;
		}
		else
		{
			shoot_on = 0;
		}
		if(shoot_on == 1)
		{
			if(New_DJI_Remote.wheel != 1024)
			{
				gongdan_on = 1;
			}
			else
			{
				gongdan_on = 0;
			}
		}
		else
		{
			gongdan_on = 0;
		}
	}
	if(gongdan_on == 1)
	{
		robot.Motor_gongdan_speed = 6800;		
	}
	else
	{
		robot.Motor_gongdan_speed = 0;	
	}
	if(shoot_on == 1)
	{
		robot.Motor_zhao = fashe_speed;//6680
		robot.Motor_xing = fashe_speed;
		robot.Motor_jie = fashe_speed;	
	}
	else
	{
		robot.Motor_zhao = 0;
		robot.Motor_xing = 0;
		robot.Motor_jie = 0;	
	}
	Heat_control();
}

float aotoshoot_yaw_angle = 0;//小yaw角度和自瞄差
float aotoshoot_pitch_angle = 0;//pitch角度和自瞄差
uint16_t last_HP = 0;//上帧血量
uint16_t now_HP = 0; //当前血量
float w = 0.95f;//sin函数的w
uint32_t time = 0;//sin函数的t
uint32_t time_w = 0;//弹丸受击检测增速时间
uint8_t judge_w = 0;//弹丸受击标志位
uint16_t trigger_and_transmit_time;//相机传输，小电脑传输时间和相机触发到采集结束时间
uint16_t all_time;//相机外触发采集帧到接到小电脑信息总时间
float yaw_before[101] = {0};
float yaw_after[101] = {0};
float pitch_before[101] = {0};
float pitch_after[101] = {0};
int16_t yaw_v[101] = {0};
int16_t pitch_v[101] = {0};
uint8_t jj = 0;
int8_t k_delay = 0;
float cp,sp,cy,sy;
float x_max,x_min,y_max,y_min,z_max,z_min;
float yaw_error;
float Y,P,R,CO_Y,CO_P;
float target_r_yaw[31] = {0};
float target_r_pitch[31] = {0};
uint32_t TIM[31] = {0};
float t_err = 0;
int8_t y = 0;//target_r_yaw，pitch索引
int8_t last_y = 0;//上次目标绝对位置的索引，用于平滑速度
int8_t llast_y = 0;//上一帧目标绝对位置的索引，用于计算加速度
float target_yaw_v = 0;
float last_target_yaw_v = 0;
float last_target_pitch_v = 0;
float target_pitch_v = 0;
uint32_t calculate_time[31];
uint8_t p = 0;
uint8_t pitch = 0;
uint8_t yaw = 0;
uint32_t lose_target = 0;
float last_accel_yaw = 0.0f;               // 上一帧平滑后的yaw加速度
float last_accel_pitch = 0.0f;             // 上一帧平滑后的pitch加速度
float current_accel_yaw = 0.0f;
float current_accel_pitch = 0.0f;
extern float err_yaw;
extern float err_pitch;
void AutoShoot()//自瞄瞄准 
{
	shoot_on = 1;//雷达自瞄模式直接开启摩擦轮
	if(test_auto == 1)//如果是测试模式
	{
		if(leida.has_target == 0)//如果没有目标
		{
			leida.is_scan = 1;
		}
		else//有目标
		{
			leida.is_scan = 0;
		}		
	}
	if(leida.has_target == 1 && leida_distance > 3.8f)//远距离不打弹
	{		
		leida.is_scan = 1;
	}
	if(leida.is_scan == 1)//默认模式，没找到目标进入扫描模式
	{
		time++;
		if(time >= 600)
		{
			robot.Motor_little_yaw += 0.13f;//0.17
			robot.Motor_little_yaw = Judge_HalfTurnJump(robot.Motor_little_yaw, IMU_DM_Info.Angle.yaw, 360.0f);				
			robot.Motor_pitch = 5.29f + 0.17f * sin((float)(w * time) / 57.298f);//fabs
		}
		else
		{
			flag = 1;
			Target_yuntai_init();
		}
		if(test_auto == 0)
		{
			leida.shoot = 0;//扫描模式不打弹保险
			gongdan_on = 0;
			robot.Motor_gongdan_speed = 0;
		}
		robot.Motor_yaw = LITTLE_yaw;
	}
	if(leida.is_scan == 0)//自瞄认为找到目标，锁自瞄值
	{
		time = 0;
		k_delay = k - t_delay;
		if (k_delay < 0) 
		{
			k_delay += 30;
		}
		imu_r = TF_Vec3(ANG2RAD(imu_yaw_past[k_delay]), ANG2RAD(imu_pitch_past[k_delay]), ANG2RAD(imu_roll_past[k_delay]));//imu弧度制
		imu_a = imu_r.radian2angle();
		imu_v = TF_Vec3(imu_yaw_w_past[k_delay], imu_pitch_w_past[k_delay], leida_distance);//陀螺仪速度弧度制
		imu_v_a = imu_v.radian2angle();//陀螺仪速度角度制
		raw_r = TF_Vec3(ANG2RAD(-camera_yaw),ANG2RAD(-camera_pitch),leida_distance);//目标相对相机位置弧度制，y-p-d
		raw_a = raw_r.radian2angle();//换成角度制
		//raw_p = TF_Vec3(leida_distance*cp*cy, leida_distance*cp*sy, leida_distance*sp);//目标相对相机坐标（右手系）
		target_r = TF_Vec3(imu_r.x+raw_r.x, imu_r.y+raw_r.y, raw_r.z);//车体坐标下目标位置弧度制y-p-d
		target_r_angle = target_r.radian2angle();//换成角度制
		if(leida_update == 1)
		{
			if(leida.has_target == 0)
			{
				robot.Motor_little_yaw = target_r_angle.x;//导航时转头但不自瞄
				robot.Motor_pitch = Pitch_zero;//target_r.y;
				lose_target++;
				if(lose_target >= 100)
				{
					target_yaw_v = 0;
					target_pitch_v = 0;
				}
			}
			else if(leida.has_target == 1)
			{
				ag.update(target_r.x,target_r.y,target_r.z,time_ms*1e-3,&targetv_r.x,&targetv_r.y);
				lose_target = 0;
				last_y = ((y - 4) + 30) % 30;
				llast_y = ((y - 1) + 30) % 30;
				target_r_yaw[y] = target_r.x;
				target_r_pitch[y] = target_r.y;
				TIM[y] = time_ms;
				t_err = (float)(TIM[y] - TIM[last_y]) / 1000.f;
				target_yaw_v = (target_r_yaw[y] - target_r_yaw[last_y]) / t_err;//目标相对相机水平速度弧度制
				target_pitch_v = (target_r_pitch[y] - target_r_pitch[last_y]) / t_err;//目标相对相机垂直速度弧度制
				y = (y + 1) % 30;
//				targetv_r = TF_Vec3(target_yaw_v, target_pitch_v, 0);//TF_Vec3(vel_r.x - imu_r_vel.x, vel_r.y - imu_r_vel.y, 0);
				target_v_xianxing = TF_Vec3(target_yaw_v, target_pitch_v, 0);//TF_Vec3(vel_r.x - imu_r_vel.x, vel_r.y - imu_r_vel.y, 0);
				targetv_a = targetv_r.radian2angle();//target_vel.radian2angle();//换成角度制
//				if(leida.is_gyro == 0)//目标缓慢移动线性预测
//				{
					pred_delay = Motor_foreast((t_delay+2)/1e3, target_r, targetv_r, &robot.Motor_little_yaw, &robot.Motor_pitch);
					//ag.predLimit(&robot.Motor_little_yaw, &robot.Motor_pitch);
					//pred_delay = Motor_foreast((t_delay+2)/1e3, target_r, target_v_xianxing, &robot.Motor_little_yaw, &robot.Motor_pitch);
//				}
//				else//目标转速过快不预测
//				{
//					pred_delay = Motor_foreast((t_delay+2)/1e3, target_r, zero, &robot.Motor_little_yaw, &robot.Motor_pitch);
//				}
//				else
//				{
//					if(leida_distance > 3.f)//远距离小陀螺
//					{
//						pred_delay = Motor_foreast((t_delay+2)/1e3, target_r, zero, &robot.Motor_little_yaw, &robot.Motor_pitch);//不预测只跟踪
//					}
//					else//近距离小陀螺
//					{
//						ag.predLimit(&robot.Motor_little_yaw, &robot.Motor_pitch);
//					}
					err_calculate(leida_distance, &err_yaw, &err_pitch);
					robot.Motor_little_yaw =  robot.Motor_little_yaw * (180.f/3.14159265f) - err_yaw;
					robot.Motor_pitch += 5.37314143f;//err_pitch;
				}
			}
		}
		if(leida.has_target == 0)//没目标不打弹
		{
			leida.shoot = 0;
		}
		if(leida.shoot == 1)//自瞄认为可以发射
		{
			aotoshoot_yaw_angle = fabs(IMU_DM_Info.Angle.yaw - robot.Motor_little_yaw);//自瞄目标和当前差值
			aotoshoot_pitch_angle = fabs(Motor_pitch.LZ_Pos_Info.Angle - robot.Motor_pitch);//自瞄目标和当前差值
			gongdan_on = 1;//上膛！开火！！
		}
	if(leida.gyro == 1)//默认模式，小电脑让自转
	{
		if(leida.move_x != 1024 || leida.move_y != 1024)
		{
			robot.tar_w = 4000;
		}
		else
		{
			if(my_robot_status.robot_id == 107)		//蓝
			{
				now_HP = my_game_robot_HP_t.blue_7_robot_HP;
				if((now_HP < last_HP) && judge_w == 0 && my_robot_hurt.HP_deduction_reason == 0 && my_robot_hurt.armor_id != 0)
				{
					judge_w = 1;
				}
				if(judge_w == 1)
				{
					time_w++;
				}
				if(time_w > 5000)
				{
					judge_w = 0;
					time_w = 0;
				}
				last_HP = my_game_robot_HP_t.blue_7_robot_HP;
				if(judge_w == 1)
				{
					robot.tar_w = 8500;//受击加速
				}
				else
				{
					robot.tar_w = 5000;//正常速度
				}
			}
			else if(my_robot_status.robot_id == 7)//红
			{
				now_HP = my_game_robot_HP_t.red_7_robot_HP;
				if((now_HP < last_HP) && judge_w == 0 && my_robot_hurt.HP_deduction_reason == 0 && my_robot_hurt.armor_id != 0 && my_ext_referee_warning.level != 1 && my_ext_referee_warning.level != 2)
				{
					judge_w = 1;
				}
				if(judge_w == 1)
				{
					time_w++;
				}
				if(time_w > 5000)
				{
					judge_w = 0;
					time_w = 0;
				}
				last_HP = my_game_robot_HP_t.red_7_robot_HP;
			}
			if(judge_w == 1)
			{
				robot.tar_w = 8500;
			}
			else
			{
				robot.tar_w = 5000;
			}		
		}
	}
	else if(leida.gyro == 0)//小电脑不让自转
	{
		robot.tar_w = 0;
	}
}

void InitializeRxCanData()
{
  Tx_can_data[0] = 0x00;            
	Tx_can_data[1] = 0x04;
	Tx_can_data[2] = 0x20;
	Tx_can_data[3] = 0x00;
	Tx_can_data[4] = 0x01;
	Tx_can_data[5] = 0x58;
	Tx_can_data[6] = 0x00;
	Tx_can_data[7] = 0x00;
}

void Transmit_to_slave_data(uint16_t tar_turn)
{
	if((DJI_Remote.Switch_Left == 2 && DJI_Remote.Switch_Right == 2) || (New_DJI_Remote.mode_sw == 2 && New_DJI_Remote.go_home == 0))
	{
    leida.move_x &= 0x07FF;
    leida.move_y &= 0x07FF;
		Tx_can_data[0] = (uint8_t)Judge_data.buffer_energy;
		Tx_can_data[1] = (uint8_t)(Judge_data.buffer_energy >> 8);
    Tx_can_data[2] = ((uint8_t)leida.move_x << 6) | (Tx_can_data[2] & 0x3F);//ch2的1-2位
    Tx_can_data[3] = (uint8_t)((leida.move_x >> 2) & 0xFF); //ch2的3-10位
    Tx_can_data[4] = (uint8_t)((leida.move_x >> 10) & 0x1) | ((((uint8_t)(leida.move_y & 0x7F)) << 1) & 0xFE);//ch3的1-7位,ch2的11位
    Tx_can_data[5] = ((uint8_t)((leida.move_y >> 7)) & 0xF) | (Tx_can_data[5] & 0xF0); //ch3的8-11位
		Tx_can_data[6] = (uint8_t)(((tar_turn >> 8) & 0x7F) | old);  //自转高字节 + 新旧遥控器
		Tx_can_data[7] = (uint8_t)(tar_turn & 0xFF);  // 低字节
	}
	else
	{
		if(new_remote_lose == 1 && remote_lose == 0)
		{
			Tx_can_data[0] = (uint8_t)Judge_data.buffer_energy;
			Tx_can_data[1] = (uint8_t)(Judge_data.buffer_energy >> 8);
			Tx_can_data[6] = (uint8_t)(((tar_turn >> 8) & 0x3F) | old | sure_home << 6);  //自转高字节 + 新旧遥控器 + 是否全停
			Tx_can_data[7] = (uint8_t)(tar_turn & 0xFF);  // 低字节
		}
		if(new_remote_lose == 0)
		{
			Tx_can_data[0] = (uint8_t)Judge_data.buffer_energy;
			Tx_can_data[1] = (uint8_t)(Judge_data.buffer_energy >> 8);
			Tx_can_data[2] = ((uint8_t)New_DJI_Remote.Chx_Left << 6) | (Tx_can_data[2] & 0x3F);//ch2的1-2位
			Tx_can_data[3] = (uint8_t)((New_DJI_Remote.Chx_Left >> 2) & 0xFF); //ch2的3-10位
			Tx_can_data[4] = (uint8_t)((New_DJI_Remote.Chx_Left >> 10) & 0x1) | ((((uint8_t)(New_DJI_Remote.Chy_Left & 0x7F)) << 1) & 0xFE);//ch3的1-7位,ch2的11位
			Tx_can_data[5] = ((uint8_t)((New_DJI_Remote.Chy_Left >> 7)) & 0xF) | (Tx_can_data[5] & 0xF0); //ch3的8-11位
			Tx_can_data[6] = (uint8_t)(((tar_turn >> 8) & 0x3F) | old | sure_home << 6);  //自转高字节 + 新旧遥控器 + 是否全停
			Tx_can_data[7] = (uint8_t)(tar_turn & 0xFF);  //低字节	
		}
	}
}

void Transmit_under()
{
	FDCAN_TxHeaderTypeDef Tx;//实例化一个fdcan句柄
	Tx.IdType = FDCAN_STANDARD_ID;								//标准帧，11 位 ID
	Tx.TxFrameType = FDCAN_DATA_FRAME;						//数据帧
	Tx.DataLength = FDCAN_DLC_BYTES_8;						//8 字节数据长度
	Tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		//活动错误状态
	Tx.BitRateSwitch = FDCAN_BRS_OFF;						//关闭位速率切换
	Tx.FDFormat = FDCAN_CLASSIC_CAN;							//经典 CAN 格式
	Tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	//不记录发送事件
	Tx.MessageMarker = 1;	//标记此消息，在 TX Event FIFO 中区分不同消息
	Tx.Identifier = 0x007;
	CAN_2.FDCAN_Transmit(&Tx, Tx_can_data);
}

void Sentry_relive()
{
	sentry_tx_cmd.Frame_Header.SOF = 0xA5;
	sentry_tx_cmd.Frame_Header.Data_Length = 10;
	sentry_tx_cmd.Frame_Header.Seq = ss_time;//包序号
	sentry_tx_cmd.Frame_Header.CRC8 = IFR_Get_CRC8_Check((uint8_t*)(&sentry_tx_cmd.Frame_Header), sizeof(frame_header) - 1 , 0xff);//写入帧头CRC8校验码
	size_8 = sizeof(frame_header) - 1;
	size_16 = sizeof(sentry_tx_cmd) - 2;
	sentry_tx_cmd.CmdId = 0x0301;	//	通信通道id 
	sentry_tx_cmd.Sub_content = 0x0120;
	if(my_robot_status.robot_id == 107)		//蓝
	{
		sentry_tx_cmd.Send_id = 107;
	}
	else if(my_robot_status.robot_id == 7)//红
	{
		sentry_tx_cmd.Send_id = 7;
	}
	else
	{
		sentry_tx_cmd.Send_id = 7;			//如果从裁判系统中读不到数据，直接默认为红方哨兵
	}
	sentry_tx_cmd.receive_id = 0x8080;
	if(my_robot_hurt.HP_deduction_reason == 1)//重要模块离线扣血
	{
		modle_err = 1;
	}
//	if(modle_err == 1)
//	{
//		sentry_tx_cmd.sentry_cmd = 0x0000;
//	}
//	else
//	{
		sentry_tx_cmd.sentry_cmd = 0x0001;		
//	}		
	sentry_tx_cmd.crc16 = IFR_Get_CRC16_Check((uint8_t*)(&sentry_tx_cmd), sizeof(sentry_tx_cmd) - 2, 0xffff);//sizeof(sentry_tx_cmd),
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&sentry_tx_cmd, 19);
}

void Target_yuntai_init()//云台目标值初始化
{
	if(flag == 1)
	{
		robot.Motor_yaw = LITTLE_yaw;
		robot.Motor_pitch = Motor_pitch.LZ_Pos_Info.Angle;
		robot.Motor_little_yaw = IMU_DM_Info.Angle.yaw;
		flag = 0;
	}
}

void Yuntai_control_Little_Yaw()//控制小yaw（遥控器）
{
	Target_yuntai_init();
	if(new_remote_lose == 1 && remote_lose == 0)
	{
		robot.Motor_little_yaw -= (DJI_Remote.Chx_Right - 1024) * 0.00045f;
		robot.Motor_pitch += (DJI_Remote.Chy_Right - 1024) * 0.000008f;//遥控器控制云台俯仰目标值
		robot.Motor_yaw = LITTLE_yaw;
		robot.Motor_little_yaw = Judge_HalfTurnJump(robot.Motor_little_yaw, IMU_DM_Info.Angle.yaw, 360.0f);	
	}
	else
	{
		robot.Motor_little_yaw -= (New_DJI_Remote.Chx_Right - 1024) * 0.00037f;
		robot.Motor_pitch += (New_DJI_Remote.Chy_Right - 1024) * 0.0000055f;//遥控器控制云台俯仰目标值		
		robot.Motor_yaw = LITTLE_yaw;
		robot.Motor_little_yaw = Judge_HalfTurnJump(robot.Motor_little_yaw, IMU_DM_Info.Angle.yaw, 360.0f);	
	}
}

void Tar_motor_analysis()
{
	if(set == 5 || set == 7) 
	{
		if(k - t_delay < 0)
		{
			Motor_little_yaw.IMU_Data_Get(IMU_DM_Info.Gyro.z, imu_yaw_past[k + 30 - t_delay]);//获取同时刻自瞄算出的目标时的位置
			Motor_pitch.IMU_Data_Get(Motor_pitch.LZ_Pos_Info.Speed, imu_pitch_past[k + 30 - t_delay] / 57.29577951418011f + 5.37314143f);	 
		}	
		else
		{
			Motor_little_yaw.IMU_Data_Get(IMU_DM_Info.Gyro.z, imu_yaw_past[k - t_delay]);//获取同时刻自瞄算出的目标时的位置
			Motor_pitch.IMU_Data_Get(Motor_pitch.LZ_Pos_Info.Speed, imu_pitch_past[k - t_delay] / 57.29577951418011f + 5.37314143f);	
		}
	}
	else
	{
		Motor_little_yaw.IMU_Data_Get(IMU_DM_Info.Gyro.z, IMU_DM_Info.Angle.yaw);//获取当前值		
		Motor_pitch.IMU_Data_Get(Motor_pitch.LZ_Pos_Info.Speed, Motor_pitch.LZ_Pos_Info.Angle);	
	}
	Motor_yaw.IMU_Data_Get(Motor_yaw.LZ_Pos_Info.Speed, Motor_little_yaw.LZ_Pos_Info.Angle);//-IMU_Info.Gyro.Yaw
	if(DJI_Remote.Switch_Left == 3 && DJI_Remote.Switch_Right == 1)//小yaw + 全向移动
	{
		if(set != 1)//第一次进入此模式
		{
			flag = 1;//进行云台初始化
		}		
		set = 1;//进入该模式标志位
		robot.tar_w = 0;
		Yuntai_control_Little_Yaw();
		shoot_on = 0;
		gongdan_on = 0;
	}
	if(DJI_Remote.Switch_Left == 3 && DJI_Remote.Switch_Right == 3)//云台小yaw + 小陀螺
	{
		if(set != 2)//第一次进入此模式
		{
			flag = 1;//进行云台初始化
		}		
		set = 2;
		robot.tar_w = 5000;
		Yuntai_control_Little_Yaw();
		shoot_on = 0;
		gongdan_on = 0;
	}
	if(DJI_Remote.Switch_Left == 3 && DJI_Remote.Switch_Right == 2)//底盘跟随
	{
		set = 3;
	}
	if(DJI_Remote.Switch_Left == 2 && DJI_Remote.Switch_Right == 1)//摩擦轮 + 滑轮拨盘 + 小yaw + 全向移动
	{
		if(set != 4)//第一次进入此模式
		{
			flag = 1;//进行云台初始化
		}		
		set = 4;
		robot.tar_w = 0;
		Yuntai_control_Little_Yaw();
		shoot_on = 1;
		if(DJI_Remote.Chz_Left != 1024)
		{			
			gongdan_on = 1;
		}
		else
		{
			gongdan_on = 0;
		}
	}	
	if(DJI_Remote.Switch_Left == 2 && DJI_Remote.Switch_Right == 3)//自瞄模式
	{
		test_auto = 1;
		if(set != 5)//第一次进入此模式
		{
			flag = 1;//进行云台初始化
		}		
		set = 5;
		Target_yuntai_init();//首次进入初始化
		if(DJI_Remote.Chx_Right != 1024 || DJI_Remote.Chy_Right != 1024)
		{
			if(set != 6)//第一次进入此模式
			{
				flag = 1;//进行云台初始化
			}		
			set = 6;				
			Yuntai_control_Little_Yaw();
		}
		else
		{
			AutoShoot();//摩擦轮开，如果锁中开拨盘，没人扫描，有人自锁，小陀螺根据回传值
		}
		robot.tar_w = 0;//aotoshoot中是否自转取决于雷达回传值，而在测试扫描模式不需要自转		
		if(leida.shoot == 1 && DJI_Remote.Chz_Left != 1024)
		{
			gongdan_on = 1;
		}
		else
		{
			gongdan_on = 0;
		}
	}
	if(DJI_Remote.Switch_Left == 2 && DJI_Remote.Switch_Right == 2)//雷达控制模式
	{
		test_auto = 0;
		if(set != 7)//第一次进入此模式
		{
			flag = 1;//进行云台初始化
		}		
		set = 7;	
		Target_yuntai_init();//首次进入初始化
		AutoShoot();//摩擦轮开，如果锁中自动开拨盘，没人扫描，有人自锁，小陀螺	根据回传值	
		//leida.gyro = 1;//雷达模式强行小陀螺，用于测试
	}	
	if (robot.Motor_pitch <= 4.95f)//限位
	{
		robot.Motor_pitch = 4.95f;
	}
	else if (robot.Motor_pitch >= 5.59f)
	{
		robot.Motor_pitch = 5.59f;
	}	
	Transmit_to_slave_data(robot.tar_w);
	Fashe_control_remot();
}

float danfa_abs = 0;//拨盘绝对角度值控制
void Motor_set()
{
	Motor_yaw.Motor_Angle_Set(robot.Motor_yaw);		
	Motor_little_yaw.Motor_Angle_Set(robot.Motor_little_yaw);
	Motor_pitch.Motor_Angle_Set(robot.Motor_pitch);
	robot.Motor_zhao = ramp_function(robot.Motor_zhao, 2000.f);
	robot.Motor_xing = ramp_function(robot.Motor_xing, 2000.f);
	robot.Motor_jie = ramp_function(robot.Motor_jie, 2000.f);
	if(robot.Motor_zhao == 0 || robot.Motor_xing == 0 || robot.Motor_jie == 0)
	{
		Motor_shoot_zhao.Motor_Speed_Set(robot.Motor_zhao);
		Motor_shoot_xing.Motor_Speed_Set(robot.Motor_xing);
		Motor_shoot_jie.Motor_Speed_Set(robot.Motor_jie);	
	}
	else
	{
		Motor_shoot_zhao.Motor_Speed_Set(robot.Motor_zhao);
		Motor_shoot_xing.Motor_Speed_Set(robot.Motor_xing);
		Motor_shoot_jie.Motor_Speed_Set(robot.Motor_jie);
	}
	if(robot.Motor_gongdan_speed == 0)
	{
		Motor_gongdan.Motor_Speed_Set(robot.Motor_gongdan_speed);
	}
	else
	{
		Motor_gongdan.Motor_Speed_Set(robot.Motor_gongdan_speed);
//		danfa_abs += 18000;
//		Motor_gongdan.Motor_AbsPos_Set(danfa_abs);
	}
}

void New_Modle_Control()//新遥控器按键检测
{
	sure_home = Key_detection(New_DJI_Remote.go_home, &state_home, sure_home);//home按键检测控制是否全关
	if(set == 5 || set == 7) 
	{
		if(k - t_delay < 0)
		{
			Motor_little_yaw.IMU_Data_Get(IMU_DM_Info.Gyro.z, imu_yaw_past[k + 30 - t_delay]);//获取同时刻自瞄算出的目标时的位置
			Motor_pitch.IMU_Data_Get(Motor_pitch.LZ_Pos_Info.Speed, imu_pitch_past[k + 30 - t_delay] / 57.29577951418011f + 5.37314143f);	 
		}	
		else
		{
			Motor_little_yaw.IMU_Data_Get(IMU_DM_Info.Gyro.z, imu_yaw_past[k - t_delay]);//获取同时刻自瞄算出的目标时的位置
			Motor_pitch.IMU_Data_Get(Motor_pitch.LZ_Pos_Info.Speed, imu_pitch_past[k - t_delay] / 57.29577951418011f + 5.37314143f);	
		}
	}
	else
	{
		Motor_little_yaw.IMU_Data_Get(IMU_DM_Info.Gyro.z, IMU_DM_Info.Angle.yaw);//获取当前值		
		Motor_pitch.IMU_Data_Get(Motor_pitch.LZ_Pos_Info.Speed, Motor_pitch.LZ_Pos_Info.Angle);	
	}
	if(sure_home == 1)//全关模式
	{
		sure_fn = 0;
		sure_button = 0;
		Set_all_0();
		robot.tar_w = 0;
		set = 0;//表示模式切换过
		Transmit_to_slave_data(robot.tar_w);		
	}
	else
	{
		Motor_yaw.IMU_Data_Get(Motor_yaw.LZ_Pos_Info.Speed, Motor_little_yaw.LZ_Pos_Info.Angle);
		sure_button = Key_detection(New_DJI_Remote.button, &state_button, sure_button);//button按键控制是否扫描模式
		sure_fn = Key_detection(New_DJI_Remote.fn, &state_fn, sure_fn);//fn按键检测控制是否开摩擦
		if(sure_button == 1 && New_DJI_Remote.mode_sw != 2)//仅有不是雷达模式才能扫描模式
		{
			if(set != 5)//第一次进入此模式
			{
				flag = 1;//进行云台初始化
			}
			set = 5;	
			Target_yuntai_init();//首次进入初始化
			if(New_DJI_Remote.mode_sw == 0)
			{
				leida.gyro = 0;
			}
			else if(New_DJI_Remote.mode_sw == 1)
			{
				leida.gyro = 1;//aotoshoot的计算通过leida
			}			
			if(New_DJI_Remote.Chx_Right != 1024 || New_DJI_Remote.Chy_Right != 1024)//右摇杆控
			{
				if(set != 6)//第一次进入此模式
				{
					flag = 1;//进行云台初始化
				}		
				set = 6;				
				Yuntai_control_Little_Yaw();
			}
			else//扫描模式
			{
				test_auto = 1;
				AutoShoot();//摩擦轮开，如果锁中开拨盘，没人扫描，有人自锁，小陀螺根据回传值
			}
		}
		else//非扫描模式等
		{
			if(New_DJI_Remote.mode_sw == 0)//底盘云台
			{
				if(set != 1)//第一次进入此模式
				{
					flag = 1;//进行云台初始化
				}		
				set = 1;//进入该模式标志位
				robot.tar_w = 0;
				Yuntai_control_Little_Yaw();
				shoot_on = 0;
				gongdan_on = 0;
			}
			if(New_DJI_Remote.mode_sw == 1)//小陀螺云台
			{
				if(set != 2)//第一次进入此模式
				{
					flag = 1;//进行云台初始化
				}		
				set = 2;
				robot.tar_w = 7000;
				Yuntai_control_Little_Yaw();
				shoot_on = 0;
				gongdan_on = 0;
			}
			if(New_DJI_Remote.mode_sw == 2)//雷达自瞄
			{
				test_auto = 0;
				if(set != 7)//第一次进入此模式
				{
					flag = 1;//进行云台初始化
				}		
				set = 7;	
				Target_yuntai_init();//首次进入初始化
				//leida.gyro = 1;//强行小陀螺
				AutoShoot();//开摩擦轮，如果锁中开拨盘，没人扫描，有人自锁，小陀螺根据回传值
				//robot.tar_w = 0;没有这句是否小陀螺取决于小电脑
			}
			if (robot.Motor_pitch <= 4.95f)//限位
			{
				robot.Motor_pitch = 4.95f;
			}
			else if (robot.Motor_pitch >= 5.59f)
			{
				robot.Motor_pitch = 5.59f;
			}	
		}
		Transmit_to_slave_data(robot.tar_w);//发给底板
		New_Fashe_control_remot();//发射机构控制
		Motor_set();//所有电机赋值
	}
}


uint8_t change = 0;
void fashe_turn(uint16_t *speed)
{
	if(my_shoot_data.initial_speed > 24.2f)
	{
		*speed -= 80;
		change = 1;
	}
//	else if(my_shoot_data.initial_speed < 23.4f && my_shoot_data.initial_speed != 0)
//	{
//		*speed += 30;
//		change = 1;
//	}
}

//小电脑测试
uint8_t iiii = 0;
uint32_t pc_time;
uint32_t start_tim = 0;
uint8_t j = 0;
uint32_t Camera_time[51] = {0};
uint8_t last_gun_yaw;
uint16_t change_time = 0;

void TIM7_Function()
{	
	time_ms++;//程序运行时间计数器
	if(change == 0)//没改变转速
	{
		change_time = 0;
		fashe_turn(&fashe_speed);//判断射速
	}
	if(change == 1)
	{
		change_time++;
	}
	if(change_time > 2000)//改变目标值后超过5s
	{
		change = 0;
		change_time = 0;
	}
	pitch_angle_err = Motor_pitch.LZ_Pos_Info.Angle - Pitch_zero;
	Auto_TransmitTask((int32_t)(robot.Motor_little_yaw * 10000.f), (int32_t)(robot.Motor_pitch * 10000.f));
	leida.move_x = 1024;
	leida.move_y = 1024;//初始化雷达回传值
	leida_update = 0;//进行小电脑回传的判断
	leida_handle(computer, sizeof(computer));//解算雷达传回来的信息
	USART_handle(&robot);	
	Target_yuntai_init();
	imu_yaw_past[k] = IMU_DM_Info.Angle.yaw;
	imu_pitch_past[k] = (Motor_pitch.LZ_Pos_Info.Angle - Pitch_zero) * 57.2957795147f;//IMU_DM_Info.Angle.roll;
	imu_roll_past[k] = IMU_DM_Info.Angle.pitch;
	imu_yaw_w_past[k] = IMU_DM_Info.Gyro.z;
	imu_pitch_w_past[k] = Motor_pitch.LZ_Pos_Info.Speed;//IMU_DM_Info.Gyro.x;	
	k++;
	if(k > 30)
	{
		k = 0;
	}	
	/**********/
//	leida.gyro = 1;//强制不自转
//  leida.is_scan = 0;//强制不扫描

//	if(my_game_status.game_progress < 4)//比赛没开始不扫描，此段代码比赛时要启用
//	{
//		leida.is_scan = 0;
//		//leida.gyro = 0;
//	}
	if(my_game_status.game_progress != 4)
	{
		ms_time = 0;
		ss_time = 0;
	}
	if(my_game_status.game_progress == 4)//比赛开始
	{
		ms_time++;
	}
	if(ms_time >= 1000)
	{
		ms_time = 0;
		ss_time ++;//比赛开始时间（s）
		Sentry_relive();//比赛开始哨兵才会申请复活
	}
	if(ss_time < 12)//比赛开始小于超过14秒
	{
		leida.gyro = 0;
	}
 
	else
	{
		if(leida_update == 0)//比赛开始后没接到小电脑
		{
			leida_losetime++;
			if(leida_losetime > 5000)//5秒未接到一直小陀螺扫描
			{
				leida.gyro = 1;
				leida.is_scan = 1;
			}
		}
	}
	if((my_rfid_status_t.rfid_status & 0x04000000) == 0x04000000)//堡垒只有防御增益，还是应该小陀螺
	{
		leida.gyro = 0;
	}
	if(ss_time > 30)//保险
	{
		leida.gyro = 1;
	}
	new_remote_lose = 1;//比赛中要有，在比赛中会强行连新遥控器
	if(new_remote_lose == 0)//接到新遥控器使用新遥控器控
	{
//		if(my_game_status.game_progress != 4 && New_DJI_Remote.mode_sw == 2)//雷达模式且比赛未开始，比赛这句要加上
//		{
//			Set_all_0();
//			robot.tar_w = 0;
//			leida.is_scan = 0;//比赛没开始不进入扫描
//			InitializeRxCanData();	
//		}
		New_Modle_Control();
	}
	else//新遥控器断连用旧遥控器
	{
		if(my_game_status.game_progress != 4 && DJI_Remote.Switch_Left == 2 && DJI_Remote.Switch_Right == 2)//雷达模式且比赛未开始，比赛这句要加上
		{
			Set_all_0();
			robot.tar_w = 0;
			leida.is_scan = 0;
			InitializeRxCanData();	
		}
		if(DJI_Remote.Switch_Left == 1)// && DJI_Remote.Switch_Right == 1)//全关模式
		{
			Set_all_0();
			robot.tar_w = 0;
			set = 0;//表示模式切换过
			Transmit_to_slave_data(robot.tar_w);
		}
		else
		{
			Tar_motor_analysis();
			Motor_set();
		}	
		if(ss_time < 25 && (DJI_Remote.Switch_Left == 2 && DJI_Remote.Switch_Right == 2))
		{
			robot.Motor_gongdan_speed = 0;
		}
		if(leida.is_scan == 1)
		{
			robot.Motor_gongdan_speed = 0;
		}
		if(robot.Motor_gongdan_speed == 0)
		{
			Motor_gongdan.Motor_Speed_Set(robot.Motor_gongdan_speed);
		}
	}
	CAN_1.FDCAN_TransmitForMotor();
	CAN_2.FDCAN_Robstride_Motor_All_Enable();
	CAN_3.FDCAN_Robstride_Motor_All_Enable();
	USART_lose();	
	Transmit_under();	
}
uint8_t o;
void ALL_init()
{
	HAL_Delay(300);
	InitializeRxCanData();
	USART_1.UDB_Recevice_Init(&huart1, JudgeData_analysis);
	UART_5.UDB_Recevice_Init(&huart5, IFR_DJI_Remote_Analysis);	
	//USART_10.UDB_Recevice_Init(&huart10, NewDJIRemote_Analysis);//新遥控器接收解算，比赛中直接不接
	RS485.UDB_Recevice_Init(&huart2, IFR_IMU_DM_Analysis);
	CAN_1.FDCAN_Init(&hfdcan1, &Motor_shoot_zhao);
	CAN_1.FDCAN_Init(&hfdcan1, &Motor_shoot_xing);
	CAN_1.FDCAN_Init(&hfdcan1, &Motor_shoot_jie);
	CAN_1.FDCAN_Init(&hfdcan1, &Motor_gongdan);
	CAN_2.FDCAN_Init(&hfdcan2, &Motor_yaw);
	CAN_3.FDCAN_Init(&hfdcan3, &Motor_pitch);
	CAN_3.FDCAN_Init(&hfdcan3, &Motor_little_yaw);
	//国赛版本
//	Motor_yaw.Speed_PID.PID_Init(0.015, 0, 4, 4.5, 8000, 16000, 0);//yaw速度环输出电流4
//	Motor_yaw.Angle_PID.PID_Init(1030, 0.04, 0.2, 400, 3000, 800, 0);//yaw位置环输出速度目标	
//	Motor_little_yaw.Speed_PID.PID_Init(2.0, 0, 10, 4, 8000, 10000, 0);//4
//	Motor_little_yaw.Angle_PID.PID_Init(0.3, 0.001, 0, 5.5, 3000, 3, 0);//0.35
	//后测试版本
	Motor_yaw.Speed_PID.PID_Init(0.01, 0, 2, 4.5, 8000, 16000, 0);//yaw速度环输出电流4
	Motor_yaw.Angle_PID.PID_Init(900, 0.01, 0, 400, 3000, 800, 0);//yaw位置环输出速度目标	
	Motor_little_yaw.Speed_PID.PID_Init(1.3, 0, 6, 4, 8000, 10000, 0);//4
	Motor_little_yaw.Angle_PID.PID_Init(0.15, 0.001, 0, 5.5, 3000, 3, 0);//0.35
	Motor_pitch.Speed_PID.PID_Init(8, 0, 4, 3.5, 8000, 16000, 0);//pitch速度环输出电流(16, 0, 6, 5, 8000, 16000, 0)3.5
	Motor_pitch.Angle_PID.PID_Init(10, 0, 0, 100, 10000, 1000, 0);//pitch位置环输出速度目标(6, 0.001, 0, 100, 10000, 1000, 0)15
	Motor_gongdan.Pos_PID.PID_Init(8.0, 0.007, 0.5, 6800, 5000, 20000, 0);//6800
	Motor_gongdan.Speed_PID.PID_Init(8, 0, 0.3, 12000, 7200, 8000, 0);//9500
	Motor_shoot_zhao.Speed_PID.PID_Init(18.f , 0.f, 0.5 , 0000 , 300 , 3000 , 0);//8000
	Motor_shoot_xing.Speed_PID.PID_Init(18.f , 0.f, 0.5 , 0000 , 300 , 3000 , 0);	
	Motor_shoot_jie.Speed_PID.PID_Init(18.f , 0.f, 0.5 , 0000 , 300 , 3000 , 0);
	TIM_7.TIM_ITStart(&htim7, TIM7_Function);
}
