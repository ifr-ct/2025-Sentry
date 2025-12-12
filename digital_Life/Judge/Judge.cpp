#include "math.h"
#include "judge.h"


#include <string.h> 



//ext_power_heat_data_t									heat_data_judge 
game_status_t 					my_game_status;
game_result_t           my_game_result_t;
game_robot_HP_t					my_game_robot_HP_t;
event_data_t 						my_event_data;
referee_warning_t				my_ext_referee_warning;
dart_info_t 						my_dart_info_t;
robot_status_t					my_robot_status;
power_heat_data_t 			my_power_heat_data = {0,0,0,60,0,0,0};
robot_pos_t 						my_robot_pos_t;
buff_t 									my_buff;
hurt_data_t 						my_robot_hurt;
shoot_data_t 						my_shoot_data;
projectile_allowance_t  my_projectile_allowance_t;
rfid_status_t           my_rfid_status_t;
dart_client_cmd_t       my_dart_client_cmd_t;
ground_robot_position_t my_ground_robot_position_t;
radar_mark_data_t       my_radar_mark_data_t;
sentry_info_t           my_sentry_info_t;
radar_info_t            my_radar_info_t;
robot_interaction_data_t my_robot_interaction_data_t;
interaction_layer_delete_t my_interaction_layer_delete_t;
interaction_figure_t    my_interaction_figure_t;
interaction_figure_2_t  my_interaction_figure_2_t;
interaction_figure_3_t  my_interaction_figure_3_t;
interaction_figure_4_t  my_interaction_figure_4_t;
ext_client_custom_character_t my_ext_client_custom_character_t;
sentry_cmd_t 						my_sentry_cmd_t;
radar_cmd_t							my_radar_cmd_t;

map_command_t 				my_robot_command;
map_robot_data_t      my_map_robot_data_t;
map_data_t            my_map_data_t;
custom_info_t         my_custom_info_t;
custom_robot_data_t   my_custom_robot_data_t;
robot_custom_data_t   my_robot_custom_data_t;
remote_control_t    	my_remote_control_t;
custom_client_data_t  my_custom_client_data_t;



Judge_DataTypedef Judge_data;
uint8_t Judge_Data_TF = 0;	
interaction_figure Line_Data[3];
//Char_DataTypedef  Char_Data[5];
interaction_figure Double_Data;
Graph_DataTypedef_5 Line_init;

//const unsigned char CRC8_INIT = 0xff; 
//const uint16_t CRC16_INIT = 0xffff; 
/*******************************************************************************
* @功能     		: 裁判系统数据处理函数
* @参数         : None
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
uint32_t count_tem_1 = 0;
uint32_t count_tem_2 = 0;
uint32_t count_tem_3 = 0;
void JudgeData_analysis(uint8_t *pata, uint8_t len)
{
	uint16_t data_length 	= 0	;
	uint16_t CRC16 				= 0	;
	uint16_t Cmd_ID 			= 0	;
	int i = 0;
	while(pata[i] != 0xA5)
	{
		i++;
	}
	while(i < len - 5)
	{
		if(IFR_Get_CRC8_Check(&pata[i], 4, CRC8_INIT) == pata[i + 4])
		{
			count_tem_1++;
			data_length = (uint16_t)((pata[i + 2] << 8)| pata[i + 1]);
			if (data_length + 7 > len - i) return;
			Cmd_ID = (uint16_t)((pata[i + 6]<<8) | pata[i + 5]);
			CRC16 = (uint16_t)((pata[i + 5 + 2 + data_length + 1]<<8) | pata[i + 5 + 2 + data_length]);
			if(IFR_Get_CRC16_Check(&pata[i], 5 + 2 + data_length, CRC16_INIT) == CRC16)
			{
				count_tem_2++;
				Judge_Data_TF = 1;
				switch(Cmd_ID)
				{
					//***
					case GAME_State				: memcpy(&my_game_status,					&pata[i],	data_length+7+2); break;// = 0x0001,	//比赛状态数据  memcpymemcpy        11 Byte   1HZ发送
					//
					
					case Competition_result_data				: memcpy(&my_game_result_t,					&pata[i+7],	data_length); break;// = 0x0002,//比赛结果数据          1 Byte   比赛结束触发发送
					
					//***
					case Robot_HP_data				: memcpy(&my_game_robot_HP_t,					&pata[i],	data_length+7+2); break;// = 0x0003,//机器人血量数据，         32Byte   固定以3Hz频率发送 
					case Event_data				: memcpy(&my_event_data,					&pata[i],	data_length+7+2); break;// = 0x0101,	//场地事件数据     		4 Byte   事件改变后发送 
					//
					
					case Referee_warningID				: memcpy(&my_ext_referee_warning,					&pata[i+7],	data_length); break;//	= 0x0104,	//裁判警告信息					3  Byte		己方判罚/判负时触发发送,其余时间以1Hz频率发送
					case Dart_launch_related_data	: memcpy(&my_dart_info_t,					&pata[i+7],	data_length); break;//	= 0x0105,	//飞镖发射相关数据		3  Byte		固定以1Hz频率发送
					
					//***
					case RobotStateId   : memcpy(&my_robot_status,					&pata[i],	data_length+7+2);break;//		= 	0x0201,	//机器人性能体系数据  	13 Byte   10Hz 
					//
					
					case RobotHeatDataId: memcpy(&my_power_heat_data,					&pata[i+7],	data_length);break; //  = 	0x0202,	//实时功率热量数据  		16 Byte   10HZ
					case RobotPosId : memcpy(&my_robot_pos_t,					&pata[i+7],	data_length);break;//      	= 	0x0203,	//机器人位置数据				16 Byte		1Hz
					
					//***
					case BuffMuskId	: memcpy(&my_buff,					&pata[i],	data_length+7+2);break;//			= 	0x0204,	//机器人增益和底盘能量数据				7  Byte		固定以3Hz频率发送 
					case Damage_status  : memcpy(&my_robot_hurt,					&pata[i],	data_length+7+2);break;// 	=   0x0206,	//伤害状态数据			    1  Byte		伤害发生后发送 
					case Real_shooting   : memcpy(&my_shoot_data,					&pata[i],	data_length+7+2);break;//	=   0x0207,	//实时射击数据			    7  Byte	  弹丸发射后发送 
					case Allowable_firing  : memcpy(&my_projectile_allowance_t,					&pata[i],	data_length+7+2);break;//=   0x0208,	//允许发弹量			      6  Byte	  固定以10Hz频率发送 
					//
					case RFID_status   : memcpy(&my_rfid_status_t,					&pata[i+7],	data_length);break;//	  =   0x0209,	//机器人RFID模块状态		4  Byte	  固定以3Hz频率发送  
					case Dart_command   : memcpy(&my_dart_client_cmd_t,					&pata[i+7],	data_length);break;//	  =   0x020A,	//飞镖选手端指令数据		6  Byte	  固定以3Hz频率发送  
					case robot_position  : memcpy(&my_ground_robot_position_t,					&pata[i+7],	data_length);break;// 	=   0x020B,	//地面机器人位置数据		40 Byte		固定以1Hz频率发送  
					case Radar_marking   : memcpy(&my_radar_mark_data_t,					&pata[i+7],	data_length);break;//	=   0x020C,	//雷达标记进度数据		  1  Byte		固定以1Hz频率发送  
					case Sentinel_auto_decision  : memcpy(&my_sentry_info_t,					&pata[i+7],	data_length);break;//=   0x020D,	//哨兵自主决策信息同步	6 Byte		固定以1Hz频率发送  
					case Radar_auto_decision   : memcpy(&my_radar_info_t,					&pata[i+7],	data_length);break;//	=   0x020E,	//雷达自主决策信息同步	1 Byte		固定以1Hz频率发送  

//						case Custom_controller : memcpy(&my_interaction_layer_delete_t,					&pata[i+7],	data_length);//=   0x0302,	//自定义控制器与机器人交互数据   30 Byte   发送方触发发送，频率上限为30Hz
//						case Mini_map_interaction : memcpy(&my_game_status,					&pata[i+7],	data_length);//=   0x0303,	//选手端下发数据		 15  Byte		选手端触发发送   
					case mouse_control   : memcpy(&my_remote_control_t,					&pata[i+7],	data_length);break;//  =   0x0304,	//键鼠遥控数据		 12  Byte		固定30Hz频率发送   
					case Receives_data  : memcpy(&my_map_robot_data_t,					&pata[i+7],	data_length);break;// =   0x0305,	//选手端小地图可接收机器人数据		 24  Byte		频率上限为5Hz    
//						case Custom_controller_interaction  : memcpy(&my_custom_client_data_t,					&pata[i+7],	data_length);// =   0x0306,	//操作手可使用自定义控制器模拟键鼠操作选手端		 8  Byte		发送方触发发送，频率上限为30Hz    
					case Receives_sentinel_data  : memcpy(&my_map_data_t,					&pata[i+7],	data_length);break;// =   0x0307,	//选手端小地图接收哨兵数据 		 103  Byte		频率上限为1Hz     
					case Receives_robot_data  : memcpy(&my_custom_info_t,					&pata[i+7],	data_length);break;// =   0x0308,	//选手端小地图接收机器人数据		 34  Byte		频率上限为3Hz    
					case Custom_controller_receives  : memcpy(&my_custom_info_t,					&pata[i+7],	data_length);break;// =   0x0309,	//自定义控制器接收机器人数据		 30  Byte		频率上限为10Hz    
					default:break;
				}
				i = i + 5 + 2 + data_length + 1 + 1;
			}
			else i += 5;
		}
		else i +=	5;
		
		if(i < len - 5)
		{
			while(pata[i] != 0xA5)
			{
				i++;
				if(i >= len - 5) break;
			}
		}
	}
	need_data_analysis();
}
/*******************************************************************************
* @功能     		: 所需的裁判系统数据处理函数
* @参数         :  
* @本机器人ID 
* @机器人等级 
* @机器人当前血量 
* @机器人血量上限 
* @机器人射击热量每秒冷却值 
* @机器人射击热量上限 
* @机器人底盘功率上限  
* @缓冲能量（单位：J） 
* @第1个17mm发射机构的射击热量 
* @第2个17mm发射机构的射击热量 
* @42mm发射机构的射击热量 
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
void need_data_analysis()
{
	Judge_data.robot_id = my_robot_status.robot_id;
	Judge_data.robot_level = my_robot_status.robot_level;
	Judge_data.current_HP = my_robot_status.current_HP;
	Judge_data.maximum_HP = my_robot_status.maximum_HP;
	Judge_data.shooter_barrel_cooling_value = my_robot_status.shooter_barrel_cooling_value;
	Judge_data.shooter_barrel_heat_limit = my_robot_status.shooter_barrel_heat_limit;
	Judge_data.chassis_power_limit = my_robot_status.chassis_power_limit;

	Judge_data.buffer_energy = my_power_heat_data.buffer_energy;
	Judge_data.shooter_17mm_1_barrel_heat = my_power_heat_data.shooter_17mm_1_barrel_heat;
	Judge_data.shooter_17mm_2_barrel_heat = my_power_heat_data.shooter_17mm_2_barrel_heat;
	Judge_data.shooter_42mm_barrel_heat = my_power_heat_data.shooter_42mm_barrel_heat;
}

unsigned char Tx_Buffer[200];

//屏幕分辨率1920x1080

uint8_t tx_seq = 0;

///*******************************************************************************
//* @功能     					: UI绘制线
//* @参数ShowData       : 绘制数据结构体
//* @参数Draw_Mode      : 绘制模式 UI_Graph_ADD 增加图形	UI_Graph_Change 修改图形	UI_Graph_Del 删除图形
//* @参数Graph_Name[4]  : 图形名（char型数组）
//* @参数Graph_Layer    : 图层数（0-9）
//* @参数Line_Width     : 线宽，建议字体大小与线宽比例为 10：1
//* @参数Line_Color     : 颜色 0：红/蓝（己方颜色）	1：黄色	2：绿色	3：橙色	4：紫红色	5：粉色	6：青色	7：黑色	8：白色
//* @参数Start_x        : 起点x坐标
//* @参数Start_y        : 起点y坐标
//* @参数End_x       		: 终点x坐标
//* @参数End_y       		: 终点y坐标
//* @返回值 						: void
//* @概述  							: 此函数用来在UI上绘制线
//*******************************************************************************/
//void UI_Draw_Line(interaction_figure* ShowData, uint16_t Draw_Mode,char Graph_Name[4], uint32_t Graph_Layer, uint32_t Line_Width, uint32_t Line_Color, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y)
//{	
//	ShowData->txFrameHeader.SOF = 0xA5;
//	ShowData->txFrameHeader.DataLength = 21;//sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_graphic_single_t);
//	ShowData->txFrameHeader.Seq = tx_seq;
//	
//	Append_CRC8_Check_Sum((uint8_t* )ShowData, sizeof(xFrameHeader));//写入帧头CRC8校验码
//	ShowData->CmdID = 0x0301;	//	通信通道id 0301
//	ShowData->dataFrameHeader.data_cmd_id = 0x0101;//绘制一是0x0101
//	ShowData->dataFrameHeader.sender_ID = Judge_data.robot_id;//发送者ID
//	ShowData->dataFrameHeader.receiver_ID = Judge_data.robot_id + 256;
//	ShowData->clientData.grapic_data_struct.graphic_name[0] = Graph_Name[2];
//	ShowData->clientData.grapic_data_struct.graphic_name[1] = Graph_Name[1];
//	ShowData->clientData.grapic_data_struct.graphic_name[2] = Graph_Name[0];
//	ShowData->clientData.grapic_data_struct.operate_tpye = Draw_Mode;
//	ShowData->clientData.grapic_data_struct.graphic_tpye = 0;					//0代表直线
//	ShowData->clientData.grapic_data_struct.layer = Graph_Layer;			
//	ShowData->clientData.grapic_data_struct.width = Line_Width;
//	ShowData->clientData.grapic_data_struct.color = Line_Color;
//	ShowData->clientData.grapic_data_struct.start_x = Start_x;//SCREEN_LENGTH/3 + 10;
//	ShowData->clientData.grapic_data_struct.start_y = Start_y;//10;
//	ShowData->clientData.grapic_data_struct.details_d = End_x;//SCREEN_LENGTH/3 + 200;
//	ShowData->clientData.grapic_data_struct.details_e = End_y;//SCREEN_WIDTH/2 - 120;
//	
//	Append_CRC16_Check_Sum((uint8_t* )ShowData,sizeof(*ShowData));//写入数据段CRC16校验码
//	if(tx_seq == 0xff) tx_seq = 0;
//	else tx_seq++;

//	//send_ui();
//}

//void UI_Draw_Line_5(Graph_DataTypedef_5* ShowData, uint16_t Draw_Mode, uint32_t Graph_Layer, uint32_t Line_Width, uint32_t Line_Color, uint32_t*Start_x, uint32_t* Start_y, uint32_t* End_x, uint32_t* End_y)
//{	
//	char* Graph_Name_1 = (char*)"001";
//	char* Graph_Name_2 = (char*)"002";
//	char* Graph_Name_3 = (char*)"003";
//	char* Graph_Name_4 = (char*)"004";
//	char* Graph_Name_5 = (char*)"005";
//	ShowData->txFrameHeader.SOF = 0xA5;
//	ShowData->txFrameHeader.DataLength = 6+15*5;//sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_graphic_single_t);
//	ShowData->txFrameHeader.Seq = tx_seq;
//	
//	Append_CRC8_Check_Sum((uint8_t* )ShowData, sizeof(xFrameHeader));//写入帧头CRC8校验码
//	ShowData->CmdID = 0x0301;	//	通信通道id 0301
//	ShowData->dataFrameHeader.data_cmd_id = UI_Data_ID_Draw5;//绘制五个图形是0x0101
//	ShowData->dataFrameHeader.sender_ID = Judge_data.robot_id;//发送者ID
//	ShowData->dataFrameHeader.receiver_ID = Judge_data.robot_id + 256;
//	ShowData->clientData.grapic_data_struct[0].graphic_name[0] = Graph_Name_1[2];
//	ShowData->clientData.grapic_data_struct[0].graphic_name[1] = Graph_Name_1[1];
//	ShowData->clientData.grapic_data_struct[0].graphic_name[2] = Graph_Name_1[0];
//	ShowData->clientData.grapic_data_struct[0].operate_tpye = Draw_Mode;
//	ShowData->clientData.grapic_data_struct[0].graphic_tpye = 0;					//0代表直线
//	ShowData->clientData.grapic_data_struct[0].layer = Graph_Layer;			
//	ShowData->clientData.grapic_data_struct[0].width = Line_Width;
//	ShowData->clientData.grapic_data_struct[0].color = Line_Color;
//	ShowData->clientData.grapic_data_struct[0].start_x = Start_x[0];//SCREEN_LENGTH/3 + 10;
//	ShowData->clientData.grapic_data_struct[0].start_y = Start_y[0];//10;
//	ShowData->clientData.grapic_data_struct[0].details_d = End_x[0];//SCREEN_LENGTH/3 + 200;
//	ShowData->clientData.grapic_data_struct[0].details_e = End_y[0];//SCREEN_WIDTH/2 - 120;
//	
//	ShowData->clientData.grapic_data_struct[1].graphic_name[0] = Graph_Name_2[2];
//	ShowData->clientData.grapic_data_struct[1].graphic_name[1] = Graph_Name_2[1];
//	ShowData->clientData.grapic_data_struct[1].graphic_name[2] = Graph_Name_2[0];
//	ShowData->clientData.grapic_data_struct[1].operate_tpye = Draw_Mode;
//	ShowData->clientData.grapic_data_struct[1].graphic_tpye = 0;					//0代表直线
//	ShowData->clientData.grapic_data_struct[1].layer = Graph_Layer;			
//	ShowData->clientData.grapic_data_struct[1].width = Line_Width;
//	ShowData->clientData.grapic_data_struct[1].color = Line_Color;
//	ShowData->clientData.grapic_data_struct[1].start_x = Start_x[1];//SCREEN_LENGTH/3 + 11;
//	ShowData->clientData.grapic_data_struct[1].start_y = Start_y[1];//11;
//	ShowData->clientData.grapic_data_struct[1].details_d = End_x[1];//SCREEN_LENGTH/3 + 211;
//	ShowData->clientData.grapic_data_struct[1].details_e = End_y[1];//SCREEN_WIDTH/2 - 120;	
//	
//	ShowData->clientData.grapic_data_struct[2].graphic_name[0] = Graph_Name_3[2];
//	ShowData->clientData.grapic_data_struct[2].graphic_name[1] = Graph_Name_3[1];
//	ShowData->clientData.grapic_data_struct[2].graphic_name[2] = Graph_Name_3[0];
//	ShowData->clientData.grapic_data_struct[2].operate_tpye = Draw_Mode;
//	ShowData->clientData.grapic_data_struct[2].graphic_tpye = 0;					//0代表直线
//	ShowData->clientData.grapic_data_struct[2].layer = Graph_Layer;			
//	ShowData->clientData.grapic_data_struct[2].width = Line_Width;
//	ShowData->clientData.grapic_data_struct[2].color = Line_Color;
//	ShowData->clientData.grapic_data_struct[2].start_x = Start_x[2];//SCREEN_LENGTH/3 + 10;
//	ShowData->clientData.grapic_data_struct[2].start_y = Start_y[2];//10;
//	ShowData->clientData.grapic_data_struct[2].details_d = End_x[2];//SCREEN_LENGTH/3 + 200;
//	ShowData->clientData.grapic_data_struct[2].details_e = End_y[2];//SCREEN_WIDTH/2 - 120;		
//	
//	ShowData->clientData.grapic_data_struct[3].graphic_name[0] = Graph_Name_4[2];
//	ShowData->clientData.grapic_data_struct[3].graphic_name[1] = Graph_Name_4[1];
//	ShowData->clientData.grapic_data_struct[3].graphic_name[2] = Graph_Name_4[0];
//	ShowData->clientData.grapic_data_struct[3].operate_tpye = Draw_Mode;
//	ShowData->clientData.grapic_data_struct[3].graphic_tpye = 0;					//0代表直线
//	ShowData->clientData.grapic_data_struct[3].layer = Graph_Layer;			
//	ShowData->clientData.grapic_data_struct[3].width = Line_Width;
//	ShowData->clientData.grapic_data_struct[3].color = Line_Color;
//	ShowData->clientData.grapic_data_struct[3].start_x = Start_x[3];//SCREEN_LENGTH/3 + 10;
//	ShowData->clientData.grapic_data_struct[3].start_y = Start_y[3];//10;
//	ShowData->clientData.grapic_data_struct[3].details_d = End_x[3];//SCREEN_LENGTH/3 + 200;
//	ShowData->clientData.grapic_data_struct[3].details_e = End_y[3];//SCREEN_WIDTH/2 - 120;		
//	
//	ShowData->clientData.grapic_data_struct[4].graphic_name[0] = Graph_Name_5[2];
//	ShowData->clientData.grapic_data_struct[4].graphic_name[1] = Graph_Name_5[1];
//	ShowData->clientData.grapic_data_struct[4].graphic_name[2] = Graph_Name_5[0];
//	ShowData->clientData.grapic_data_struct[4].operate_tpye = Draw_Mode;
//	ShowData->clientData.grapic_data_struct[4].graphic_tpye = 0;					//0代表直线
//	ShowData->clientData.grapic_data_struct[4].layer = Graph_Layer;			
//	ShowData->clientData.grapic_data_struct[4].width = Line_Width;
//	ShowData->clientData.grapic_data_struct[4].color = Line_Color;
//	ShowData->clientData.grapic_data_struct[4].start_x = Start_x[4];//SCREEN_LENGTH/3 + 10;
//	ShowData->clientData.grapic_data_struct[4].start_y = Start_y[4];//10;
//	ShowData->clientData.grapic_data_struct[4].details_d = End_x[4];//SCREEN_LENGTH/3 + 200;
//	ShowData->clientData.grapic_data_struct[4].details_e = End_y[4];//SCREEN_WIDTH/2 - 120;	
//		
//	Append_CRC16_Check_Sum((uint8_t* )ShowData,sizeof(*ShowData));//写入数据段CRC16校验码
//	if(tx_seq == 0xff) tx_seq = 0;
//	else tx_seq++;

//	//send_ui();
//}
//void UI_Draw_Char(interaction_figure* ShowData, uint16_t Draw_Mood, char Graph_Name[3], uint32_t Graph_Layer, uint32_t Line_Width,uint32_t Char_Color, uint32_t Char_Size,uint32_t Char_Length, char Show_Str[30], uint32_t Start_x, uint32_t Start_y)
//{	
//	ShowData->txFrameHeader.SOF = 0xA5;
//	ShowData->txFrameHeader.DataLength = 51;//sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_graphic_single_t);
//	ShowData->txFrameHeader.Seq = tx_seq;
//	
//	Append_CRC8_Check_Sum((uint8_t* )ShowData, sizeof(xFrameHeader));//写入帧头CRC8校验码
//	ShowData->CmdID = 0x0301;	//	通信通道id 0301
//	ShowData->dataFrameHeader.data_cmd_id = UI_Data_ID_DrawChar;//如果是绘制字符是0x0110
//	ShowData->dataFrameHeader.sender_ID = Judge_data.robot_id;//发送者
//	ShowData->dataFrameHeader.receiver_ID = Judge_data.robot_id + 256;
//	//图形名
//	ShowData->clientData.grapic_data_struct.graphic_name[0] = Graph_Name[2];
//	ShowData->clientData.grapic_data_struct.graphic_name[1] = Graph_Name[1];
//	ShowData->clientData.grapic_data_struct.graphic_name[2] = Graph_Name[0];
//	//具体字符设置
//	ShowData->clientData.grapic_data_struct.operate_tpye = Draw_Mood;
//	ShowData->clientData.grapic_data_struct.graphic_tpye = 7 ;//draw char
//	ShowData->clientData.grapic_data_struct.layer        = Graph_Layer;
//	ShowData->clientData.grapic_data_struct.color        = Char_Color ;
//	
//	ShowData->clientData.grapic_data_struct.details_a = Char_Size;		//字体大小
//	ShowData->clientData.grapic_data_struct.details_b 	= Char_Length;//字符长度
//	ShowData->clientData.grapic_data_struct.width			  = Line_Width; //线条宽度
//	ShowData->clientData.grapic_data_struct.details_c			= 0;				//字体大小
//	ShowData->clientData.grapic_data_struct.start_x		 = Start_x;
//	ShowData->clientData.grapic_data_struct.start_y		 = Start_y;
//	
//	memset(ShowData->clientData.data, 0, 30);
//	int i;
//	for (i = 0; i < 29 && Show_Str[i] != '\0'; i += 1)
//  {
//		ShowData->clientData.data[i] = Show_Str[i];
//		
//  }
//	ShowData->clientData.data[i] = '\0';
// 
//	Append_CRC16_Check_Sum((uint8_t* )ShowData,sizeof(*ShowData));//写入数据段CRC16校验码
//	if(tx_seq == 0xff) tx_seq = 0;
//	else tx_seq++;
//}
///*******************************************************************************
//* @功能     					: UI绘制浮点数
//* @参数ShowData       : 绘制数据结构体
//* @参数Draw_Mode      : 绘制模式 UI_Graph_ADD 增加图形	UI_Graph_Change 修改图形	UI_Graph_Del 删除图形
//* @参数Graph_Name[4]  : 图形名（char型数组）
//* @参数Graph_Layer    : 图层数（0-9）
//* @参数Num_Width      : 线宽，建议字体大小与线宽比例为 10：1
//* @参数Num_Color      : 颜色 0：红/蓝（己方颜色）	1：黄色	2：绿色	3：橙色	4：紫红色	5：粉色	6：青色	7：黑色	8：白色
//* @参数Num_Size    		: 字体大小
//* @参数Num		    		: 传入的数字（仅会保留两位小数）
//* @参数Start_x        : 起点x坐标
//* @参数Start_y        : 起点y坐标
//* @返回值 						: void
//* @概述  							: 此函数用来在UI上绘制浮点数（仅会保留两位小数）   
//*******************************************************************************/
//void UI_Draw_Double(interaction_figure* ShowData, uint16_t Draw_Mood, char Graph_Name[3], uint32_t Graph_Layer,uint32_t Num_Width, uint32_t Num_Color, uint32_t Num_Size, float Num, uint32_t Start_x, uint32_t Start_y)
//{
//	ShowData->txFrameHeader.SOF = 0xA5;
//	ShowData->txFrameHeader.DataLength = 21;//sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_graphic_single_t);
//	ShowData->txFrameHeader.Seq = tx_seq;
//	
//	Append_CRC8_Check_Sum((uint8_t* )ShowData, sizeof(xFrameHeader));//写入帧头CRC8校验码
//	ShowData->CmdID = 0x0301;	//	通信通道id 0301
//	ShowData->dataFrameHeader.data_cmd_id = UI_Data_ID_Draw1;//如果是绘制字符是0x0110
//	ShowData->dataFrameHeader.sender_ID = Judge_data.robot_id;//发送者
//	ShowData->dataFrameHeader.receiver_ID = Judge_data.robot_id + 256;
//	//图形名
//	ShowData->clientData.grapic_data_struct.graphic_name[0] = Graph_Name[2];
//	ShowData->clientData.grapic_data_struct.graphic_name[1] = Graph_Name[1];
//	ShowData->clientData.grapic_data_struct.graphic_name[2] = Graph_Name[0];
//	//具体字符设置
//	ShowData->clientData.grapic_data_struct.operate_tpye = Draw_Mood;
//	ShowData->clientData.grapic_data_struct.graphic_tpye = 5 ;			//浮点数模式
//	ShowData->clientData.grapic_data_struct.layer = Graph_Layer;
//	ShowData->clientData.grapic_data_struct.width	= Num_Width; 			//线条宽度	
//	ShowData->clientData.grapic_data_struct.color = Num_Color ;
//	ShowData->clientData.grapic_data_struct.details_a = Num_Size;		//字体大小
//	
//	int num = (int)(Num * 100);
//	ShowData->clientData.grapic_data_struct.details_c = num>>22;
//	ShowData->clientData.grapic_data_struct.details_d = (num>>11)&0x3FF800;
//	ShowData->clientData.grapic_data_struct.details_e = num&0x7ff;
////	memcpy(&(ShowData->clientData.grapic_data_struct) + 11, &num, 4);
////	//ShowData->clientData.grapic_data_struct.details_c = num;
//	ShowData->clientData.grapic_data_struct.start_x	= Start_x;
//	ShowData->clientData.grapic_data_struct.start_y	= Start_y;
//	
//}
//void Send_UIdata(uint8_t* ShowData)
//{
//		uint16_t DataLength = (uint16_t)ShowData[1] | ((uint16_t)ShowData[2] << 8);
////		HAL_UART_Transmit(&huart6, ShowData, DataLength + 5 + 4, 200);									//暂时先不用，等裁判系统
//}

///*******************************************************************************
//* @功能     		: UI设置初始化函数
//* @参数         : None
//* @返回值 			: void
//* @概述  				: 在最开始的时候写入一些固定的图形
//*******************************************************************************/
//uint8_t count = 0;
//void UI_Init()
//{
//	if (count > 100)
//		return;
//	count++;
//	
//	uint32_t start_x[] = {650, 1280, 792, 860, 860};
//	uint32_t start_y[] = {10,  10, 307, 560, 550};
//	uint32_t end_x[] = {840, 1090, 1137, 1060, 1060};
//	uint32_t end_y[] = {420, 420, 307, 560, 550};
//	UI_Draw_Line_5(&Line_init  , UI_Graph_ADD, 1, 3, UI_Color_Yellow, start_x, start_y, end_x, end_y);
////	UI_Draw_Line(Line_Data  	 , UI_Graph_ADD, (char*)"001", 1, 4, UI_Color_Yellow, SCREEN_LENGTH/3 + 10, 10, SCREEN_LENGTH/3 + 200, SCREEN_WIDTH/2 - 120);
////	UI_Draw_Line(Line_Data + 1 , UI_Graph_ADD, (char*)"002", 1, 4, UI_Color_Yellow, SCREEN_LENGTH - SCREEN_LENGTH/3, 10,SCREEN_LENGTH - SCREEN_LENGTH/3 - 190, SCREEN_WIDTH/2 - 120);
////	UI_Draw_Line(Line_Data + 2 , UI_Graph_ADD, (char*)"003", 1, 4, UI_Color_Yellow, 792, 307, 1137, 307);
//	UI_Draw_Char(Char_Data	   , UI_Graph_ADD, (char*)"006", 1, 2, UI_Color_Yellow, 30, 8, (char*)"Tuoluo:\0", SCREEN_LENGTH/10 + 50, SCREEN_WIDTH/2 + 100);
//	UI_Draw_Char(Char_Data + 1 , UI_Graph_ADD, (char*)"007", 1, 2, UI_Color_Yellow, 30, 7, (char*)"Pitch:\0" , SCREEN_LENGTH/10 + 50, SCREEN_WIDTH/2 + 150);
//	UI_Draw_Char(Char_Data + 2 , UI_Graph_ADD, (char*)"008", 2, 2, UI_Color_Yellow, 30, 2, (char*)"F\0", SCREEN_LENGTH/10 + 80, SCREEN_WIDTH/2 + 100);
//	UI_Draw_Char(Char_Data + 3 , UI_Graph_ADD, (char*)"009", 1, 2, UI_Color_Yellow, 30, 8, (char*)"Zimiao:\0", SCREEN_LENGTH/10 + 50, SCREEN_WIDTH/2 + 200);
//	UI_Draw_Char(Char_Data + 4 , UI_Graph_ADD, (char*)"010", 2, 2, UI_Color_Yellow, 30, 2, (char*)"F\0", SCREEN_LENGTH/10 + 80, SCREEN_WIDTH/2 + 200);
//	UI_Draw_Double(&Double_Data, UI_Graph_ADD, (char*)"011", 2, 1, UI_Color_Yellow, 30, 15 , SCREEN_LENGTH/2, SCREEN_WIDTH/2);
//	
//	switch(count % 9)
//	{
//		case 1:
//			Send_UIdata((uint8_t*)&Line_init);break;
//		case 2:
//			Send_UIdata((uint8_t*)(Char_Data));break;
//		case 3:
//			Send_UIdata((uint8_t*)(Char_Data + 1));break;
//		case 4:
//			Send_UIdata((uint8_t*)(Char_Data + 2));break;
//		case 6:
//			Send_UIdata((uint8_t*)(Char_Data + 3));break;
//		case 7:
//			Send_UIdata((uint8_t*)(Char_Data + 4));break;
//		case 8:
//			Send_UIdata((uint8_t*)(&Double_Data));break;
//	}	
//}

///*******************************************************************************
//* @功能     		: UI绘制函数
//* @参数1        : 是否在小陀螺
//* @返回值 			: void
//* @概述  				: None
//*******************************************************************************/
//uint8_t count_num = 0;
//void UI_draw(uint8_t IS_tuoluo, uint8_t IS_zimiao)
//{
//	count_num++;
//	if (IS_tuoluo == 2)
//		UI_Draw_Char(Char_Data + 2 , UI_Graph_Change, (char*)"008", 2, 2, UI_Color_Yellow, 30, 2, (char*)"T\0", SCREEN_LENGTH/10 + 260, SCREEN_WIDTH/2 + 100);
//	else 
//		UI_Draw_Char(Char_Data + 2 , UI_Graph_Change, (char*)"008", 2, 2, UI_Color_Yellow, 30, 2, (char*)"F\0", SCREEN_LENGTH/10 + 260, SCREEN_WIDTH/2 + 100);
//	if (IS_zimiao == 1)
//		UI_Draw_Char(Char_Data + 4 , UI_Graph_Change, (char*)"010", 2, 2, UI_Color_Yellow, 30, 2, (char*)"T\0", SCREEN_LENGTH/10 + 80, SCREEN_WIDTH/2 + 200);
//	else
//		UI_Draw_Char(Char_Data + 4 , UI_Graph_Change, (char*)"010", 2, 2, UI_Color_Yellow, 30, 2, (char*)"F\0", SCREEN_LENGTH/10 + 80, SCREEN_WIDTH/2 + 200);
//	UI_Draw_Double(&Double_Data, UI_Graph_Change, (char*)"011", 2, 1, UI_Color_Yellow, 15, 15 , SCREEN_LENGTH/2, SCREEN_WIDTH/2);	
//	switch (count_num % 3)
//	{
//		case 0:
//			Send_UIdata((uint8_t*)(Char_Data + 2));break;
//		case 1:
//			Send_UIdata((uint8_t*)(&Double_Data));break;
//		case 2:
//			Send_UIdata((uint8_t*)(Char_Data + 4));count_num = 0;;break;
//	}
//}


float fTotalCurrentLimit;//电流分配,平地模式下分配是均匀的
float WARNING_REMAIN_POWER = 60;
float fChasCurrentLimit = 36000;//限制4个轮子的速度总和

/**
  * @brief  数据是否可用
  * @param  void
  * @retval  TRUE可用   FALSE不可用
  * @attention  在裁判读取函数中实时改变返回值
  */
uint8_t JUDGE_sGetDataState(void)
{
	return Judge_Data_TF;
}

/**
  * @brief  读取剩余焦耳能量
  * @param  void
  * @retval 剩余缓冲焦耳能量(最大60)
  * @attention  
  */
uint16_t JUDGE_fGetRemainEnergy(void)
{
	return (my_power_heat_data.buffer_energy);
}


