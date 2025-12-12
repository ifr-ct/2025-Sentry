#include "sense_analysis.h"
#include "usbd_cdc_if.h"
IMU_TxDataTypedef IFR_Sense_Send_Data;
IMU_DATA imu_data;
Leida leida;//回传雷达自瞄值
int16_t Debug_move_x_Filter;
float camera_yaw = 0;//目标相对相机水平角度
float camera_pitch = 0;//目标相对相机垂直角度角度
//float leida_yaw;//雷达目标水平角度
//float leida_pitch;//雷达目标垂直角度
float leida_distance = 0;//小电脑回传目标距离
//float camera_yaw_v = 0;//目标相对相机水平角度
//float camera_pitch_v = 0;//目标相对相机垂直角度
float leida_count_time = 0;//小电脑运算时间ms
extern uint32_t time_ms;//用作记录当前程序运行时间
uint32_t last_time_ms;//用做记录上次小电脑回传程序运行时间
uint8_t last_reserve;//存放上次保留位用作判断小电脑是否回传
//uint32_t pc_time[51];//小电脑回传时间，单位ms
uint8_t ii;//计算小电脑回传时间的索引
extern uint8_t leida_update;//小电脑数据标志位
extern uint32_t leida_losetime;//小电脑丢失时间
extern TF_Vec3 imu_a, imu_v_a, raw_a, target_r_angle, targetv_a;
extern AntiGyro ag;
void leida_handle(uint8_t *pData, uint8_t len)//处理小电脑信息
{
	if(pData == NULL)
	{
		return;
	}
	leida.head = 0x5a; 
	uint8_t start_offset = 0;
	while (pData[start_offset] != 0x5a) 			//查找帧头
	{
		start_offset++;
		if (start_offset + sizeof(leida) > len) return;		//避免数组越界，即查看在帧头之后是否还有7个字节
	}
	uint8_t crc8 = pData[sizeof(leida) - 1 + start_offset];
	if (IFR_Get_CRC8_Check(pData + start_offset, sizeof(leida) - 1, 0xFF) == crc8)
	{
		leida = *((Leida*)(pData + start_offset));
		camera_yaw = leida.camera_yaw / 10000.0f * 57.29578f;//目标相对于相机位置的水平相对角度
		camera_pitch = leida.camera_pitch / 10000.0f * 57.29578f;//目标相对于相机位置的垂直相对角度
//		leida_yaw = leida.leida_yaw / 10000.0f * 57.29578f;//雷达要求的水平角度
//		leida_pitch = leida.leida_pitch / 10000.0f * 57.29578f;//雷达要求的垂直角度		
		leida_distance = leida.distance / 1000.0f;//目标相对于相机位置的距离
//		camera_yaw_v = leida.yaw_v / 10000.0f;//目标相对于相机的水平速度
//		camera_pitch_v = leida.pitch_v / 10000.0f;//目标相对于相机的垂直速度		
		leida_count_time = leida.count_time / 1000;//小电脑的运算时间
		if(leida.reserve != last_reserve)
		{
			leida_update = 1;			
			leida_losetime = 0;
			if(ii <= 50)
			{
//				pc_time[ii] = time_ms - last_time_ms;//记录小电脑回传时间
				last_time_ms = time_ms;
				ii++;
			}
		}
		last_reserve = leida.reserve;
	}
	else
	{
		return;
	}
}

uint8_t pTxData[80];//裁判系统数据发送缓存区
uint8_t red_or_blue = 1;//红蓝方
uint8_t buffer = 0;//计数标志用作遍历发送
extern float pitch_angle_err;//pitch位置对应陀螺仪零点位置的误差，用作自瞄所需角度转换
void Auto_TransmitTask(int32_t little_yaw, int32_t pitch)
{
	static uint8_t i = 0;
	const void* pJudjeData[9] = {&my_game_status, &my_game_robot_HP_t, &my_event_data, &my_robot_status, &my_buff, &my_robot_hurt, &my_shoot_data, &my_projectile_allowance_t, &my_sentry_info_t};//, &my_map_data_t};
  const uint8_t pJudgeDataLength[9] = {sizeof(game_status_t), sizeof(game_robot_HP_t), sizeof(event_data_t), sizeof(robot_status_t),
																				sizeof(buff_t), sizeof(hurt_data_t), sizeof(shoot_data_t), sizeof(projectile_allowance_t), sizeof(sentry_info_t)};//, sizeof(map_data_t)};
	if(my_robot_status.robot_id == 107)		//red_or_blue   0:蓝     1：红		
	{
		red_or_blue = 0;
	}
	else if(my_robot_status.robot_id == 7)
	{
		red_or_blue = 1;
	}
	else
	{
		red_or_blue = 1;			//如果从裁判系统中读不到数据，直接默认为红方哨兵
	}
	if(buffer % 4 == 2)
	{
		Sense_send_data(IMU_DM_Info, red_or_blue, 1, 0, pitch_angle_err, little_yaw, pitch);//陀螺仪，红方，预测
	}
	else if (buffer % 4 == 0)
	{
		if (((frame_header*)(pJudjeData[i%9]))->SOF == 0xA5)
		{
			__disable_irq(); // 替代FreeRTOS临界区
			memcpy(pTxData, pJudjeData[i%9], pJudgeDataLength[i%9]);
			//CDC_Transmit_HS((uint8_t*)pTxData, pJudgeDataLength[i%9]);
			__enable_irq();  
			((frame_header*)(pJudjeData[i%9]))->SOF = 0;				
		}
		i++;
	}
	buffer ++;
}
void Sense_send_data(IMU_DM_TypeDef IMU_DM_Info,uint8_t is_red ,uint8_t use_forecast, uint8_t imu_type, float pitch_err, int32_t little_yaw, int32_t pitch)//自瞄通讯数据发送
{
	imu_data.head = 0x5C;                                    
	imu_data.yaw  = IMU_DM_Info.Angle.yaw * 100;
	imu_data.pitch = -57.29578f * pitch_err * 100;
	imu_data.roll = -IMU_Info.Angle.Roll * 100;
	imu_data.is_red = is_red;
	imu_data.use_forecast = use_forecast;
	imu_data.imu_type = 0;
	imu_data.aim_type = 1;
	imu_data.need_flash = 0;
	imu_data.crc = IFR_Get_CRC8_Check((unsigned char *)(&imu_data), sizeof(IMU_DATA) - 1, 0xFF);
	CDC_Transmit_HS((uint8_t*)(&imu_data), sizeof(IMU_DATA));
//	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)(&IFR_Sense_Send_Data), sizeof(IFR_Sense_Send_Data));
}

//extern Antigyro ag;
//void Sense_send_data(IMU_DM_TypeDef IMU_DM_Info,uint8_t is_red ,uint8_t use_forecast, uint8_t imu_type, float pitch_err, int32_t little_yaw, int32_t pitch)//自瞄通讯数据发送
//{
//	IFR_Sense_Send_Data.head = 0x5C;                                    
//	IFR_Sense_Send_Data.Yaw_info  = IMU_DM_Info.Angle.yaw * 100;
//	IFR_Sense_Send_Data.Pitch_info = -57.29578f * pitch_err * 100;
//	IFR_Sense_Send_Data.Roll_info = -IMU_Info.Angle.Roll * 100;
//	IFR_Sense_Send_Data.is_red = is_red;
//	IFR_Sense_Send_Data.use_forecast = use_forecast;
//	IFR_Sense_Send_Data.imu_type = 0;
////	IFR_Sense_Send_Data.imu_a_yaw = imu_a.x * 10000.f;
////	IFR_Sense_Send_Data.raw_a_yaw = raw_a.x * 10000.f;
////	IFR_Sense_Send_Data.target_a_yaw = target_r_angle.x * 10000.f;
////	IFR_Sense_Send_Data.imu_av_yaw = imu_v_a.x * 10000.f;
////	IFR_Sense_Send_Data.target_av_yaw = targetv_a.x * 10000.f;
////	IFR_Sense_Send_Data.Motor_little_yaw = little_yaw;
////	IFR_Sense_Send_Data.imu_a_pitch = imu_a.y * 10000.f;
////	IFR_Sense_Send_Data.raw_a_pitch = raw_a.y * 10000.f;
////	IFR_Sense_Send_Data.target_a_pitch = target_r_angle.y * 10000.f;
////	IFR_Sense_Send_Data.imu_av_pitch = imu_v_a.y * 10000.f;
////	IFR_Sense_Send_Data.target_av_pitch = targetv_a.y * 10000.f;
////	IFR_Sense_Send_Data.Motor_pitch = pitch;
////	IFR_Sense_Send_Data.big_max_yaw = ag.big_max_yaw * 10000.f;
////	IFR_Sense_Send_Data.big_min_yaw = ag.big_min_yaw * 10000.f;
////	IFR_Sense_Send_Data.max_yaw = ag.max_yaw * 10000.f;
////	IFR_Sense_Send_Data.min_yaw = ag.min_yaw * 10000.f;
////	IFR_Sense_Send_Data.lst_max_yaw = ag.lst_max_yaw * 10000.f;
////	IFR_Sense_Send_Data.lst_min_yaw = ag.lst_min_yaw * 10000.f;
//	IFR_Sense_Send_Data.CRC8 = IFR_Get_CRC8_Check((unsigned char *)(&IFR_Sense_Send_Data), sizeof(IFR_Sense_Send_Data) - 1, 0xFF);
//	CDC_Transmit_HS((uint8_t*)(&IFR_Sense_Send_Data), sizeof(IFR_Sense_Send_Data));
////	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)(&IFR_Sense_Send_Data), sizeof(IFR_Sense_Send_Data));
//}


