

#ifndef __judge_H__
#define __judge_H__
#include "ifr_crc.h"
#include "usart.h"


//#define ABS(x)     (((x)>0)?(x):-(x))


//红方机器人ID 1英雄 2工程 3步兵 4步兵 5步兵 7哨兵
//蓝方机器人ID 1英雄 2工程 3步兵 4步兵 5步兵 7哨兵

/****************************开始标志*********************/
#define UI_SOF 0xA5
/****************************CMD_ID数据********************/
#define UI_CMD_Robo_Exchange 0x0301    
/****************************内容ID数据********************/
#define UI_Data_ID_Del 0x100 
#define UI_Data_ID_Draw1 0x101
#define UI_Data_ID_Draw2 0x102
#define UI_Data_ID_Draw5 0x103
#define UI_Data_ID_Draw7 0x104
#define UI_Data_ID_DrawChar 0x110

/****************************红方机器人ID********************/
#define UI_Data_RobotID_RHero 1         
#define UI_Data_RobotID_REngineer 2
#define UI_Data_RobotID_RStandard1 3
#define UI_Data_RobotID_RStandard2 4
#define UI_Data_RobotID_RStandard3 5

/****************************蓝方机器人ID********************/
#define UI_Data_RobotID_BHero 101
#define UI_Data_RobotID_BEngineer 102
#define UI_Data_RobotID_BStandard1 103
#define UI_Data_RobotID_BStandard2 104
#define UI_Data_RobotID_BStandard3 105

/**************************红方操作手ID************************/
#define UI_Data_CilentID_RHero 0x0101
#define UI_Data_CilentID_REngineer 0x0102
#define UI_Data_CilentID_RStandard1 0x0103
#define UI_Data_CilentID_RStandard2 0x0104
#define UI_Data_CilentID_RStandard3 0x0105

/***************************蓝方操作手ID***********************/
#define UI_Data_CilentID_BHero 0x0165
#define UI_Data_CilentID_BEngineer 0x0166
#define UI_Data_CilentID_BStandard1 0x0167
#define UI_Data_CilentID_BStandard2 0x0168
#define UI_Data_CilentID_BStandard3 0x0169


/***************************图形配置参数__字内容ID********************/
#define UI_Coverage_Dellete 0x0100 //选手端删除图层
#define UI_Draw_One 0x0101 				 //选手端绘制一个图形
#define UI_Draw_Two 0x0102 				 //选手端绘制两个图形
#define UI_Draw_three 0x0103  		 //选手端绘制五个图形
#define UI_Draw_four 0x0104 			 //选手端绘制七个图形
#define UI_Draw_five 0x0110 			 //选手端绘制字符图形

/***************************图形配置参数__图形操作********************/
#define UI_Graph_ADD 1		//增加图形
#define UI_Graph_Change 2	//修改图形
#define UI_Graph_Del 3		//删除图形



/***************************图形配置参数__图形颜色********************/
#define UI_Color_Main 0         //红/蓝（己方颜色）
#define UI_Color_Yellow 1				//黄
#define UI_Color_Green 2				//绿
#define UI_Color_Orange 3				//橙
#define UI_Color_Purplish_red 4 //紫红
#define UI_Color_Pink 5					//粉
#define UI_Color_Cyan 6         //青
#define UI_Color_Black 7				//黑
#define UI_Color_White 8				//白

/***************************图形配置参数__分辨率*********************/
#define SCREEN_WIDTH 1080
#define SCREEN_LENGTH 1920
/**********************************END******************************/
typedef enum
{
	GAME_State = 0x0001,	//比赛状态数据  memcpymemcpy        11 Byte   1HZ发送
	Competition_result_data = 0x0002,//比赛结果数据          1 Byte   比赛结束触发发送
	Robot_HP_data = 0x0003,//机器人血量数据，         32Byte   固定以3Hz频率发送 

	Event_data = 0x0101,	//场地事件数据     		4 Byte   事件改变后发送 

	Referee_warningID	= 0x0104,	//裁判警告信息					3  Byte		己方判罚/判负时触发发送,其余时间以1Hz频率发送
	Dart_launch_related_data	= 0x0105,	//飞镖发射相关数据		3  Byte		固定以1Hz频率发送

	RobotStateId			= 	0x0201,	//机器人性能体系数据  	13 Byte   10Hz 
	RobotHeatDataId   = 	0x0202,	//实时功率热量数据  		16 Byte   10HZ
	RobotPosId       	= 	0x0203,	//机器人位置数据				16 Byte		1Hz
	BuffMuskId				= 	0x0204,	//机器人增益和底盘能量数据				7  Byte		固定以3Hz频率发送 

	Damage_status   	=   0x0206,	//伤害状态数据			    1  Byte		伤害发生后发送 
	Real_shooting   	=   0x0207,	//实时射击数据			    7  Byte	  弹丸发射后发送 
	Allowable_firing  =   0x0208,	//允许发弹量			      6  Byte	  固定以10Hz频率发送 
	RFID_status   	  =   0x0209,	//机器人RFID模块状态		4  Byte	  固定以3Hz频率发送  
	Dart_command   	  =   0x020A,	//飞镖选手端指令数据		6  Byte	  固定以3Hz频率发送  
	robot_position   	=   0x020B,	//地面机器人位置数据		40 Byte		固定以1Hz频率发送  
	Radar_marking   	=   0x020C,	//雷达标记进度数据		  1  Byte		固定以1Hz频率发送  
	Sentinel_auto_decision  =   0x020D,	//哨兵自主决策信息同步	6 Byte		固定以1Hz频率发送  
	Radar_auto_decision   	=   0x020E,	//雷达自主决策信息同步	1 Byte		固定以1Hz频率发送  

	Robot_interaction =   0x0301,	//机器人交互数据        127  Byte  发送方触发发送，频率上限为30Hz  
	Custom_controller =   0x0302,	//自定义控制器与机器人交互数据   30 Byte   发送方触发发送，频率上限为30Hz
	Mini_map_interaction =   0x0303,	//选手端小地图交互数据		 15  Byte		选手端触发发送   
	mouse_control     =   0x0304,	//键鼠遥控数据		 12  Byte		固定30Hz频率发送   
	Receives_data   =   0x0305,	//选手端小地图可接收机器人数据。 		 24  Byte		频率上限为5Hz    
	Custom_controller_interaction   =   0x0306,	//自定义控制器与选手端交互数据		 8  Byte		发送方触发发送，频率上限为30Hz    
	Receives_sentinel_data   =   0x0307,	//选手端小地图接收哨兵数据		 103  Byte		频率上限为1Hz     
	Receives_robot_data   =   0x0308,	//选手端小地图接收机器人数据		 34  Byte		频率上限为3Hz    
	Custom_controller_receives   =   0x0309,	//自定义控制器接收机器人数据		 30  Byte		频率上限为10Hz    

}CmdIDType;

 /* 自定义帧头 */
typedef __packed struct
{
	uint8_t  SOF;
	uint16_t Data_Length;
	uint8_t  Seq;
	uint8_t  CRC8;
} frame_header;

/*******************************************************************************
 * @概述	比赛类型与比赛阶段
 * @大小
 * @命令码ID：0x0001
bit 0-3：比赛类型 
1：RoboMaster 机甲大师超级对抗赛 
2：RoboMaster 机甲大师高校单项赛 
3：ICRA RoboMaster高校人工智能挑战赛 
4：RoboMaster机甲大师高校联盟赛3V3对抗  
5：RoboMaster 机甲大师高校联盟赛步兵对抗 
bit 4-7：当前比赛阶段 
0：未开始比赛 
1：准备阶段 
2：十五秒裁判系统自检阶段 
3：五秒倒计时 

4：比赛中 
5：比赛结算中 
 * @当前阶段剩余时间，单位：秒 
 * @ UNIX时间，当机器人正确连接到裁判系统的NTP服务器后生效 
 *******************************************************************************/
typedef __packed struct 
{ 
	frame_header Frame_Header;//根据感知解包协议的自定义帧头
	uint16_t CmdId;//裁判系统发区分每一个包
	
  uint8_t game_type : 4; 
  uint8_t game_progress : 4; 
  uint16_t stage_remain_time; 
  uint64_t SyncTimeStamp; 
	
	uint16_t crc16;
}game_status_t; 

/*******************************************************************************
 * @概述	胜利信息
 * @大小  1bit
 * @命令码ID：0x0002
	0：平局 
	1：红方胜利 
	2：蓝方胜利
 *******************************************************************************/
typedef __packed struct 
{ 
   uint8_t winner; 
}game_result_t;

/*******************************************************************************
 * @概述	血量
 * @大小  32bits
 * @命令码ID：0x0003
 * @红1英雄机器人血量。若该机器人未上场或者被罚下，则血量为0 
 * @红2工程机器人血量 
 * @红3步兵机器人血量 
 * @红4步兵机器人血量 
 * @保留位 
 * @红7哨兵机器人血量 
 * @红方前哨站血量 
 * @红方基地血量 
 * @蓝1英雄机器人血量 
 * @蓝2工程机器人血量 
 * @蓝3步兵机器人血量 
 * @蓝4步兵机器人血量 
 * @保留位 
 * @蓝7哨兵机器人血量 
 * @蓝方前哨站血量 
 * @蓝方基地血量 
 *******************************************************************************/
typedef __packed struct 
{ 
	frame_header Frame_Header;//根据感知解包协议的自定义帧头
	uint16_t CmdId;//裁判系统发区分每一个包
	
  uint16_t red_1_robot_HP; 
  uint16_t red_2_robot_HP; 
  uint16_t red_3_robot_HP; 
  uint16_t red_4_robot_HP; 
  uint16_t reserved; 
  uint16_t red_7_robot_HP; 
  uint16_t red_outpost_HP; 
  uint16_t red_base_HP; 
  uint16_t blue_1_robot_HP; 
  uint16_t blue_2_robot_HP; 
  uint16_t blue_3_robot_HP; 
  uint16_t blue_4_robot_HP; 
  uint16_t reserved1; 
  uint16_t blue_7_robot_HP; 
  uint16_t blue_outpost_HP; 
  uint16_t blue_base_HP; 
	
	uint16_t crc16;	
}game_robot_HP_t; 

/*******************************************************************************
 * @概述	地图信息
 * @大小  32bits
 * @命令码ID：0x0101
0：未占领/未激活  
1：已占领/已激活 
 * @bit 0-2： 
 * @bit 0：己方与兑换区不重叠的补给区占领状态，1为已占领 
 * @bit 1：己方与兑换区重叠的补给区占领状态，1为已占领 
 * @bit 2：己方补给区的占领状态，1为已占领（仅 RMUL 适用） 
 * @bit 3-5：己方能量机关状态 
 * @bit 3：己方小能量机关的激活状态，1为已激活 
 * @bit 4：己方大能量机关的激活状态，1为已激活 
 * @bit 5-6：己方中央高地的占领状态，1为被己方占领，2为被对方占领 
 * @bit 7-8：己方梯形高地的占领状态，1为已占领 
 * @bit 9-17：对方飞镖最后一次击中己方前哨站或基地的时间（0-420，开局默认为0） 
 * @bit 18-20：对方飞镖最后一次击中己方前哨站或基地的具体目标，开局默认为0，1为击中前哨站，2为击中基地固定目标，3为击中基地随机固定目标，4为击中基地随机移动目标 
 * @bit 21-22：中心增益点的占领状态，0为未被占领，1为被己方占领，2为被对方占领，3为被双方占领。（仅RMUL适用）
 * @bit 23-24：乙方堡垒增益点的占领状态，0为未被占领，1为被己方占领，2为被对方占领，3为被双方占领
 * @bit 25-31：保留位 
 *******************************************************************************/
typedef __packed struct 
{ 
	frame_header Frame_Header;//根据感知解包协议的自定义帧头
	uint16_t CmdId;//裁判系统发区分每一个包
	
  uint32_t event_data; 
	
	uint16_t crc16;
}event_data_t; 

/*******************************************************************************
 * @概述	判罚
 * @大小  24bits
 * @命令码ID：0x0104
 * @己方最后一次受到判罚的等级： 
	1：双方黄牌 
	2：黄牌 
	3：红牌 
	4：判负  
 * @己方最后一次受到判罚的违规机器人ID。（如红1机器人ID为1，蓝1机器人ID为101） 
 * @判负和双方黄牌时，该值为0 
 * @己方最后一次受到判罚的违规机器人对应判罚等级的违规次数。（开局默认为0。） 
 *******************************************************************************/
typedef __packed struct 
{ 
  uint8_t level; 
  uint8_t offending_robot_id; 
  uint8_t count; 
}referee_warning_t; 

/*******************************************************************************
 * @概述	飞镖信息
 * @大小  24bits
 * @命令码ID：0x0105
 * @己方飞镖发射剩余时间，单位：秒 
	bit 0-2：  
 * @最近一次己方飞镖击中的目标，开局默认为0，1为击中前哨站，2为击中基地固定目标，3为击中基地随机固定目标，4为击中基地随机移动目标 
	bit 3-5： 
 * @对方最近被击中的目标累计被击中计次数，开局默认为0，至多为4 
	bit 6-7： 
 * @飞镖此时选定的击打目标，开局默认或未选定/选定前哨站时为0，选中基地固定目标为1，选中基地随机固定目标为2，选中基地随机移动目标为3 
	bit 8-15：保留
 *******************************************************************************/
typedef __packed struct 
{ 
  uint8_t dart_remaining_time; 
  uint16_t dart_info; 
}dart_info_t; 

/*******************************************************************************
 * @概述	机器人状态
 * @大小  104bits
 * @命令码ID：0x0201
 * @本机器人ID 
 * @机器人等级 
 * @机器人当前血量 
 * @机器人血量上限 
 * @机器人射击热量每秒冷却值 
 * @机器人射击热量上限 
 * @机器人底盘功率上限  
 * @电源管理模块的输出情况： 
	bit 0：gimbal口输出，0为无输出，1为 24V输出 
	bit 1：chassis口输出，0为无输出，1为24V输出 
	bit 2：shooter口输出，0为无输出，1为24V输出 
 *******************************************************************************/
typedef __packed struct 
{ 
	frame_header Frame_Header;//根据感知解包协议的自定义帧头
	uint16_t CmdId;//裁判系统发区分每一个包
	
  uint8_t robot_id; 
  uint8_t robot_level; 
  uint16_t current_HP;  
  uint16_t maximum_HP; 
  uint16_t shooter_barrel_cooling_value; 
  uint16_t shooter_barrel_heat_limit; 
  uint16_t chassis_power_limit;  
  uint8_t power_management_gimbal_output : 1; 
  uint8_t power_management_chassis_output : 1;  
  uint8_t power_management_shooter_output : 1; 
	
	uint16_t crc16;
}robot_status_t;  

/*******************************************************************************
 * @概述	机器人热量状态
 * @大小  128bits
 * @命令码ID：0x0202
 * @保留位 
 * @保留位 
 * @保留位 
 * @缓冲能量（单位：J） 
 * @第1个17mm发射机构的射击热量 
 * @第2个17mm发射机构的射击热量 
 * @42mm发射机构的射击热量 
 *******************************************************************************/
typedef __packed struct 
{ 
//	frame_header Frame_Header;//根据感知解包协议的自定义帧头
//	uint16_t CmdId;//裁判系统发区分每一个包
	
  uint16_t reserved; 
  uint16_t reserved1; 
  float reserved2; 
  uint16_t buffer_energy; 
  uint16_t shooter_17mm_1_barrel_heat; 
  uint16_t shooter_17mm_2_barrel_heat; 
  uint16_t shooter_42mm_barrel_heat; 
	
//	uint16_t crc16;
}power_heat_data_t; 

/*******************************************************************************
 * @概述	本机器人位置坐标
 * @大小  12bits
 * @命令码ID：0x0203
 * @本机器人位置x坐标，单位：m 
 * @本机器人位置y坐标，单位：m 
 * @本机器人测速模块的朝向，单位：度。正北为0度
 *******************************************************************************/
typedef __packed struct 
{ 
  float x; 
  float y; 
  float angle; 
}robot_pos_t; 

/*******************************************************************************
 * @概述	机器人不同增益
 * @大小  24bits
 * @命令码ID：0x0204
 * @机器人回血增益（百分比，值为10表示每秒恢复血量上限的10%） 
 * @机器人射击热量冷却倍率（直接值，值为5表示5倍冷却） 
 * @机器人防御增益（百分比，值为50表示50%防御增益） 
 * @机器人负防御增益（百分比，值为30表示-30%防御增益） 
 * @机器人攻击增益（百分比，值为50表示50%攻击增益） 
	bit 0-4：机器人剩余能量值反馈，以16进制标识机器人剩余能量值比例，仅在机器人剩余能量小于50%时反馈，其余默认反馈0x32。 
	bit 0：在剩余能量≥50%时为1，其余情况为0 
	bit 1：在剩余能量≥30%时为1，其余情况为0 
	bit 2：在剩余能量≥15%时为1，其余情况为0 
	bit 3：在剩余能量≥5%时为1，其余情况为0Bit4：在剩余能量≥1%
时为1，其余情况为0 
 *******************************************************************************/
typedef __packed struct 
{ 
	frame_header Frame_Header;//根据感知解包协议的自定义帧头
	uint16_t CmdId;//裁判系统发区分每一个包
	
  uint8_t recovery_buff;  
  uint8_t cooling_buff;  
  uint8_t defence_buff;  
  uint8_t vulnerability_buff; 
  uint16_t attack_buff; 
  uint8_t remaining_energy; 
	
	uint16_t crc16;
}buff_t; 

/*******************************************************************************
 * @概述	扣血原因
 * @大小  16bits
 * @命令码ID：0x0206
 * @bit 0-3：当扣血原因为装甲模块被弹丸攻击、受撞击、离线或测速模块离线时，该4 bit组成的数值为装甲模块或测速模块的ID编号；当其他原因导致扣血时，该数值为0 
 * @bit 4-7：血量变化类型 
	0：装甲模块被弹丸攻击导致扣血 
	1：装甲模块或超级电容管理模块离线导致扣血
	5：装甲模块受到撞击导致扣血 
 *******************************************************************************/
typedef __packed struct 
{ 
	frame_header Frame_Header;//根据感知解包协议的自定义帧头
	uint16_t CmdId;//裁判系统发区分每一个包
	
  uint8_t armor_id : 4; 
  uint8_t HP_deduction_reason : 4; 
	
	uint16_t crc16;
}hurt_data_t; 

/*******************************************************************************
 * @概述	弹丸参数
 * @大小  56bits
 * @命令码ID：0x0207
 * @弹丸类型： 
	1：17mm弹丸 
	2：42mm弹丸 
 * @发射机构ID： 
	1：第1个17mm发射机构  
	2：第2个17mm发射机构 
	3：42mm发射机构 
 * @弹丸射速（单位：Hz） 
 * @弹丸初速度（单位：m/s） 
 *******************************************************************************/
typedef __packed struct 
{ 
	frame_header Frame_Header;//根据感知解包协议的自定义帧头
	uint16_t CmdId;//裁判系统发区分每一个包
	
  uint8_t bullet_type;  
  uint8_t shooter_number; 
  uint8_t launching_frequency;  
  float initial_speed;  
	
	uint16_t crc16;	
}shoot_data_t; 

/*******************************************************************************
 * @概述	允许发弹量
 * @大小  48bits：
 * @命令码ID：0x0208 
 * @17mm弹丸允许发弹量 
 * @42mm弹丸允许发弹量 
 * @堡垒增益点提供的储备17mm弹丸允许发弹量
 * @剩余金币数量 
 *******************************************************************************/
typedef __packed struct 
{ 
	frame_header Frame_Header;//根据感知解包协议的自定义帧头
	uint16_t CmdId;//裁判系统发区分每一个包
	
  uint16_t projectile_allowance_17mm; 
  uint16_t projectile_allowance_42mm;  
  uint16_t remaining_gold_coin;
	uint16_t projectile_allowance_fortress;
	
	uint16_t crc16;
}projectile_allowance_t; 

/*******************************************************************************
 * @概述	增益点
 * @大小  32bits： 
 * @命令码ID：0x0209
	bit位值为1/0的含义：是否已检测到该增益点RFID卡 
	bit 0：己方基地增益点 
	bit 1：己方中央高地增益点 
	bit 2：对方中央高地增益点 
	bit 3：己方梯形高地增益点 
	bit 4：对方梯形高地增益点 
	bit 5：己方地形跨越增益点（飞坡）（靠近己方一侧飞坡前） 
	bit 6：己方地形跨越增益点（飞坡）（靠近己方一侧飞坡后） 
	bit 7：对方地形跨越增益点（飞坡）（靠近对方一侧飞坡前） 
	bit 8：对方地形跨越增益点（飞坡）（靠近对方一侧飞坡后） 
	bit 9：己方地形跨越增益点（中央高地下方） 
	bit 10：己方地形跨越增益点（中央高地上方） 
	bit 11：对方地形跨越增益点（中央高地下方） 
	bit 12：对方地形跨越增益点（中央高地上方） 
	bit 13：己方地形跨越增益点（公路下方） 
	bit 14：己方地形跨越增益点（公路上方） 
	bit15：对方地形跨越增益点（公路下方） 
	bit16：对方地形跨越增益点（公路上方） 
	bit 17：己方堡垒增益点 
	bit 18：己方前哨站增益点 
	bit 19：己方与兑换区不重叠的补给区/RMUL补给区 
	bit 20：己方与兑换区重叠的补给区 
	bit 21：己方大资源岛增益点 
	bit 22：对方大资源岛增益点 
	bit 23：中心增益点（仅 RMUL 适用） 
	bit 24：对方堡垒增益点
	bit 25-31：保留 
注：所有RFID卡仅在赛内生效。在赛外，即使检测到对应的RFID卡，对
应值也为0。 
 *******************************************************************************/
typedef __packed struct 
{ 
  uint32_t rfid_status; 
}rfid_status_t; 

/*******************************************************************************
 * @概述	飞镖发射站的状态
 * @大小  48bits： 
 * @命令码ID：0x020A
 * @当前飞镖发射站的状态： 
	1：关闭 
	2：正在开启或者关闭中 
	0：已经开启 
 * @保留位 
 * @切换击打目标时的比赛剩余时间，单位：秒，无/未切换动作，默认为0。 
 
 * @最后一次操作手确定发射指令时的比赛剩余时间，单位：秒，初始值为0。 
 *******************************************************************************/
typedef __packed struct 
{ 
  uint8_t dart_launch_opening_status;  
  uint8_t reserved;  
  uint16_t target_change_time;  
  uint16_t latest_launch_cmd_time; 
}dart_client_cmd_t; 

/*******************************************************************************
 * @概述	己方机器人位置坐标
 * @大小  40bits： 
 * @命令码ID：0x020B
 * @己方英雄机器人位置x轴坐标，单位：m 
 * @己方英雄机器人位置y轴坐标，单位：m 
 * @己方工程机器人位置x轴坐标，单位：m 
 * @己方工程机器人位置y轴坐标，单位：m 
 * @己方3号步兵机器人位置x轴坐标，单位：m 
 * @己方3号步兵机器人位置y轴坐标，单位：m 
 * @己方4号步兵机器人位置x轴坐标，单位：m 
 * @己方4号步兵机器人位置y轴坐标，单位：m 
 * @保留位 
 * @保留位  
 *******************************************************************************/
typedef __packed struct 
{ 
  float hero_x;  
  float hero_y;  
  float engineer_x;  
  float engineer_y;  
  float standard_3_x;  
  float standard_3_y;  
  float standard_4_x;  
  float standard_4_y;  
  float reserved;  
  float reserved1; 
}ground_robot_position_t; 
/*******************************************************************************
 * @概述	对方机器人易伤情况
 * @大小  8bits： 
 * @命令码ID：0x020C
	bit 0：对方1号英雄机器人易伤情况 
	bit 1：对方2号工程机器人易伤情况 
	bit 2：对方3号步兵机器人易伤情况 
	bit 3：对方4号步兵机器人易伤情况 
	bit 4：对方哨兵机器人易伤情况 
备注：在对应机器人被标记进度≥100时发送1，被标记进度<100时发送0。
 *******************************************************************************/
typedef __packed struct 
{ 
  uint8_t mark_progress;  
}radar_mark_data_t; 
/*******************************************************************************
 * @概述	哨兵机器人信息
 * @大小  48bits： 
 * @命令码ID：0x020D
	bit 0-10：除远程兑换外，哨兵机器人成功兑换的允许发弹量，开局为 0，在哨兵机器人成功兑换一定允许发弹量后，该值将变为哨兵机器人成功兑换的允许发弹量值。 
	bit 11-14：哨兵机器人成功远程兑换允许发弹量的次数，开局为 0，在哨兵机器人成功远程兑换允许发弹量后，该值将变为哨兵机器人成功远程兑换允许发弹量的次数。 
	bit 15-18：哨兵机器人成功远程兑换血量的次数，开局为 0，在哨兵机器人成功远程兑换血量后，该值将变为哨兵机器人成功远程兑换血量的次数。 
	bit 19：哨兵机器人当前是否可以确认免费复活，可以确认免费复活时值为1，否则为0。 
	bit 20：哨兵机器人当前是否可以兑换立即复活，可以兑换立即复活时值为1，否则为0。 
	bit 21-30：哨兵机器人当前若兑换立即复活需要花费的金币数。 
	bit 31：保留。 

	bit 0：哨兵当前是否处于脱战状态，处于脱战状态时为1，否则为0。 
	bit 1-11：队伍17mm允许发弹量的剩余可兑换数。 
	bit 12-15：保留。
 *******************************************************************************/
typedef __packed struct 
{  
	frame_header Frame_Header;//根据感知解包协议的自定义帧头
	uint16_t CmdId;//裁判系统发区分每一个包
	
	uint32_t sentry_info; 
  uint16_t sentry_info_2; 
	
	uint16_t crc_16;
} sentry_info_t; 

/*******************************************************************************
 * @概述	雷达触发双倍易伤
 * @大小  48bits： 
 * @命令码ID：0x020E
	bit 0-1：雷达是否拥有触发双倍易伤的机会，开局为0，数值为雷达拥有触发双倍易伤的机会，至多为2 
	bit 2：对方是否正在被触发双倍易伤 
	0：对方未被触发双倍易伤 
	1：对方正在被触发双倍易伤 
	bit 3-7：保留
 *******************************************************************************/
typedef __packed struct 
{ 
  uint8_t radar_info; 
} radar_info_t; 

/*******************************************************************************
 * @概述	内容信息
 * @大小  48bits： 
 * @命令码ID：0x0301
 * @子内容ID 需为开放的子内容ID 
 * @发送者ID 需与自身ID匹配，ID编号详见附录 
 * @接收者ID 
 * @仅限己方通信 
 * @需为规则允许的多机通讯接收者 
 * @若接收者为选手端，则仅可发送至发送者对应的选手端 
 * @ID编号详见附录 
 * @内容数据段 x最大为112 

子内容ID 			内容数据段长度 					功能说明 
0x0200~0x02FF 		x≤112 					机器人之间通信 
0x0100 							2 						选手端删除图层 
0x0101 							15 						选手端绘制一个图形 
0x0102 							30 						选手端绘制两个图形 
0x0103 							75 						选手端绘制五个图形 
0x0104 							105						选手端绘制七个图形 
0x0110 							45 						选手端绘制字符图形 
0x0120 							4 						哨兵自主决策指令 
0x0121 							1 						雷达自主决策指令
 *******************************************************************************/
typedef __packed struct 
{ 
  uint16_t data_cmd_id; 
  uint16_t sender_id; 
  uint16_t receiver_id; 
  uint8_t user_data[112]; 
}robot_interaction_data_t; 

/*******************************************************************************
 * @概述	图层操作
 * @大小  16bits： 
 * @子内容ID：0x0100
 * @删除操作 
	0：空操作 
	1：删除图层 
	2：删除所有 
* @图层数 图层数：0~9 
 *******************************************************************************/
typedef __packed struct 
{ 
	uint8_t delete_type; 
	uint8_t layer; 
}interaction_layer_delete_t; 

/*******************************************************************************
 * @概述	图层操作
 * @大小  48bits： 
 * @子内容ID：0x0101
 * @图形名 在图形删除、修改等操作中，作为索引 
 * @图形配置1 
	bit 0-2：图形操作 
	0：空操作 
	1：增加 
	2：修改 
	3：删除 
	bit 3-5：图形类型 
	0：直线 
	1：矩形 
	2：正圆 
	3：椭圆 
	4：圆弧 
	5：浮点数 
	6：整型数 
	7：字符 
	bit 6-9：图层数（0~9） 
	bit 10-13：颜色 
	0：红/蓝（己方颜色） 
	1：黄色 
	2：绿色 
	3：橙色 
	4：紫红色 
	5：粉色 
	6：青色 
	7：黑色 
	8：白色 
	bit 14-31：根据绘制的图形不同，含义不同，详见“表 2-24 图形细节参数说明” 
 * @图形配置2 
	bit 0-9：线宽，建议字体大小与线宽比例为10：1 
	bit 10-20：起点/圆心x坐标 
	bit 21-31：起点/圆心y坐标 
 * @图形配置3 根据绘制的图形不同，含义不同，详见“表 2-24 图形细节参数说明”自己去看吧

 * @角度值含义为：0°指12点钟方向，顺时针绘制； 
 * @屏幕位置：（0,0）为屏幕左下角（1920，1080）为屏幕右上角； 
 * @浮点数：整型数均为32位，对于浮点数，实际显示的值为输入的值/1000，如在details_c、details_d、details_e对应的字节输入1234，选手端实际显示的值将为1.234。 
 * @即使发送的数值超过对应数据类型的限制，图形仍有可能显示，但此时不保证显示的效果。 
 *******************************************************************************/
typedef __packed struct 
{  
uint8_t figure_name[3];  
uint32_t operate_tpye:3;  
uint32_t figure_tpye:3;  
uint32_t layer:4;  
uint32_t color:4;  
uint32_t details_a:9; 
uint32_t details_b:9; 
uint32_t width:10;  
uint32_t start_x:11;  
uint32_t start_y:11;  
uint32_t details_c:10;  
uint32_t details_d:11;  
uint32_t details_e:11;  
}interaction_figure_t; 

/*******************************************************************************
 * @概述	图层操作
 * @大小  48bits： 
 * @子内容ID：0x0102
 * @图形1 与0x0101的数据段相同 
 * @图形2 与0x0101的数据段相同  
 *******************************************************************************/
typedef __packed struct 
{ 
  interaction_figure_t interaction_figure[2]; 
}interaction_figure_2_t; 

/*******************************************************************************
 * @概述	图层操作
 * @大小  48bits：
 * @子内容ID：0x0103 
 * @图形1 与0x0101的数据段相同 
 * @图形2 与0x0101的数据段相同 
 * @图形3 与0x0101的数据段相同 
 * @图形4 与0x0101的数据段相同 
 * @图形5 与0x0101的数据段相同  
 *******************************************************************************/
typedef __packed struct 
{ 
	interaction_figure_t interaction_figure[5]; 
}interaction_figure_3_t; 

/*******************************************************************************
 * @概述	图层操作
 * @大小  48bits： 
 * @子内容ID：0x0104
 * @图形1 与0x0101的数据段相同 
 * @图形2 与0x0101的数据段相同 
 * @图形3 与0x0101的数据段相同 
 * @图形4 与0x0101的数据段相同 
 * @图形5 与0x0101的数据段相同 
 * @图形6 与0x0101的数据段相同 
 * @图形7 与0x0101的数据段相同  
 *******************************************************************************/
typedef __packed struct 
{ 
	interaction_figure_t interaction_figure[7]; 
}interaction_figure_4_t; 

/*******************************************************************************
 * @概述	发送内容检验
 * @大小  48bits： 
 * @子内容ID：0x0110 
 * @发送者的ID 需要校验发送者的ID正确性 
 * @接收者的ID 
	需要校验接收者的ID正确性，仅支持发送机器人对应的选手端 
6 15 字符配置 详见图形数据介绍 
21 30 字符 -   
 *******************************************************************************/
typedef __packed struct 
{ 
	interaction_figure_t  grapic_data_struct; //
	uint8_t data[30]; 
} ext_client_custom_character_t; 

/*******************************************************************************
 * @概述	哨兵自主决策相关
 * @大小  48bits： 
 * @哨兵自主决策指令：0x0120 
 * @bit 0：哨兵机器人是否确认复活 
	0表示哨兵机器人确认不复活，即使此时哨兵的复活读条已经完成 
	1表示哨兵机器人确认复活，若复活读条完成将立即复活 
 * @bit 1：哨兵机器人是否确认兑换立即复活 
	0表示哨兵机器人确认不兑换立即复活； 
	1表示哨兵机器人确认兑换立即复活，若此时哨兵机器人符合兑换立即复活的规则要求，则会立即消耗金币兑换立即复活 
 * @bit 2-12：哨兵将要兑换的发弹量值，开局为0，修改此值后，哨兵在补血点即可兑换允许发弹量。
 * @此值的变化需要单调递增，否则视为不合法。 
 * @示例：此值开局仅能为0，此后哨兵可将其从0修改至X，则消耗X金币成功兑换X允许发弹量。此后哨兵可将其从X修改至X+Y，以此类推。 
 * @bit 13-16：哨兵远程兑换发弹量的请求次数，开局为0，修改
 * @此值即可请求远程兑换发弹量。 
 * @此值的变化需要单调递增且每次仅能增加1，否则视为不合法。 
 * @示例：此值开局仅能为0，此后哨兵可将其从0修改至1，则消耗金币远程兑换允许发弹量。此后哨兵可将其从1修改至2，以此类推。 
 * @bit 17-20：哨兵远程兑换血量的请求次数，开局为0，修改此值即可请求远程兑换血量。 

 * @此值的变化需要单调递增且每次仅能增加1，否则视为不合法。 
 * @示例：此值开局仅能为0，此后哨兵可将其从0修改至1，则消耗金币远程兑换血量。此后哨兵可将其从1修改至2，以此类推。 
 * @在哨兵发送该子命令时，服务器将按照从相对低位到相对高位的原则依次处理这些指令，直至全部成功或不能处理为止。 
 * @示例：若队伍金币数为0，此时哨兵战亡，“是否确认复活”的值为1，“是否确认兑换立即复活”的值为1，“确认兑换的允许发弹量值”为100。（假定之前哨兵未兑换过允许发弹量）由于此时队伍金币数不足以使哨兵兑换立即复活，则服务器将会忽视后续指令，等待哨兵发送的下一组指令。 
 * @bit 21-31：保留 
 *******************************************************************************/
typedef __packed struct 
{ 
 uint32_t sentry_cmd;  
} sentry_cmd_t; 

/*******************************************************************************
 * @概述	雷达是否确认触发双倍易伤 
 * @大小  48bits： 
 * @雷达自主决策指令：0x0121
 * @开局为0，修改此值即可请求触发双倍易伤，若此时雷达拥有触发双倍易伤的机会，则可触发。 
 * @此值的变化需要单调递增且每次仅能增加1，否则视为不合法。 
 * @示例：此值开局仅能为0，此后雷达可将其从0修改至1，若雷达拥有触发双倍易伤的机会，则触发双倍易伤。此后雷达可将其从1修改至2，以此类推。 
 * @若雷达请求双倍易伤时，双倍易伤正在生效，则第二次双倍易伤将在第一次双倍易伤结束后生效。
 *******************************************************************************/
typedef __packed struct 
{ 
	uint8_t radar_cmd; 
} radar_cmd_t; 

/*******************************************************************************
 * @概述	选手端下发数据 
 * @大小  48bits： 
 * @命令码ID：0x0303 
 * @目标位置x轴坐标，单位m 当发送目标机器人ID时，该值为0 
 * @目标位置y轴坐标，单位m 当发送目标机器人ID时，该值为0 
 * @云台手按下的键盘按键通用键值 无按键按下，则为0 
 * @对方机器人ID 当发送坐标数据时，该值为0 
 * @信息来源ID 信息来源的ID，ID对应关系详见附录 
 *******************************************************************************/
typedef __packed struct 
{ 
	float target_position_x; 
	float target_position_y; 
	uint8_t cmd_keyboard; 
	uint8_t target_robot_id; 
	uint16_t cmd_source; 
}map_command_t; 

/*******************************************************************************
 * @概述	选手端接收数据  
 * @大小  48bits： 
 * @命令码ID：0x0305 
 * @英雄机器人x位置坐标，单位：cm 
 * @当x、y超出边界时显示在对应边缘处，当x、y均为0时，视为未发送此机器人坐标。 
 * @英雄机器人y位置坐标，单位：cm 
 * @工程机器人x位置坐标，单位：cm 
 * @工程机器人y位置坐标，单位：cm 
 * @3号步兵机器人x位置坐标，单位：cm 
 * @3号步兵机器人y位置坐标，单位：cm 
 * @4号步兵机器人x位置坐标，单位：cm 
 * @4号步兵机器人y位置坐标，单位：cm 
 * @5号步兵机器人x位置坐标，单位：cm 
 * @5号步兵机器人y位置坐标，单位：cm 
 * @哨兵机器人x位置坐标，单位：cm 
 * @哨兵机器人y位置坐标，单位：cm 
 *******************************************************************************/
typedef __packed struct 
{  
uint16_t hero_position_x; 
  uint16_t hero_position_y; 
  uint16_t engineer_position_x; 
  uint16_t engineer_position_y; 
  uint16_t infantry_3_position_x; 
  uint16_t infantry_3_position_y; 
  uint16_t infantry_4_position_x; 
  uint16_t infantry_4_position_y; 
  uint16_t infantry_5_position_x; 
  uint16_t infantry_5_position_y; 
  uint16_t sentry_position_x; 
  uint16_t sentry_position_y; 
} map_robot_data_t; 

/*******************************************************************************
 * @概述	哨兵行为  
 * @大小  48bits： 
 * @命令码ID：0x0307 
 * @1：到目标点攻击 
		2：到目标点防守 
		3：移动到目标点 - 
 * @路径起点x轴坐标，单位：dm 小地图左下角为坐标原点，水平向右为X轴正方向，竖直向上为Y轴正方向。显示位置将按照场地尺寸与小地图尺寸等比缩放，超出边界的位置将在边界处显示 3 2 路径起点y轴坐标，单位：dm 
 * @路径点x轴增量数组，单位：dm 增量相较于上一个点位进行计算，共49个新点位，X与Y轴增量对应组成点位 
 * @路径点y轴增量数组，单位：dm 
 * @发送者ID 需与自身ID匹配，ID编号详见附录 
 *******************************************************************************/
typedef __packed struct 
{ 
	uint8_t intention; 
	uint16_t start_position_x; 
	uint16_t start_position_y; 
	int8_t delta_x[49]; 
	int8_t delta_y[49]; 
	uint16_t sender_id; 
}map_data_t; 

/*******************************************************************************
 * @概述	ID  
 * @大小  48bits： 
 * @命令码ID：0x0308 
 * @发送者的ID  需要校验发送者的ID正确性 
 * @接收者的ID  需要校验接收者的ID正确性，仅支持发送己方选手端 
 * @字符以utf-16 格式编码发送，支持显示中文。编码发送时请注意数据的大小端问题  
 *******************************************************************************/
typedef __packed struct 
{  
	uint16_t sender_id; 
	uint16_t receiver_id; 
	uint8_t user_data[30]; 
} custom_info_t; 

/*******************************************************************************
 * @概述	自定义控制器发送数据  
 * @大小 	30bits： 
 * @命令码ID：0x0302
 * @自定义数据 
 * @操作手可使用自定义控制器通过图传链路向对应的机器人发送数据。 
 *******************************************************************************/
typedef __packed struct 
{ 
	//uint8_t data[x]; 
}custom_robot_data_t; 

/*******************************************************************************
 * @概述	自定义控制器发送数据
 * @大小 	30bits： 
 * @命令码ID：0x0309 
 * @自定义数据  
 * @机器人可通过图传链路向对应的操作手选手端连接的自定义控制器发送数据（RMUL暂不适用）。
 *******************************************************************************/
typedef __packed struct 
{ 
	//uint8_t data[x]; 
}robot_custom_data_t; 

/*******************************************************************************
 * @概述	键鼠遥控数据 
 * @大小 	30bits： 
 * @命令码ID：0x0304 
 * @鼠标x轴移动速度，负值标识向左移动 
 * @鼠标y轴移动速度，负值标识向下移动 
 * @鼠标滚轮移动速度，负值标识向后滚动 
 * @鼠标左键是否按下：0为未按下；1为按下 
 * @鼠标右键是否按下：0为未按下，1为按下 
 * @键盘按键信息，每个bit对应一个按键，0为未按下，1为按下： 
	bit 0：W键 
	bit 1：S键 
	bit 2：A键 
	bit 3：D键 
	bit 4：Shift键 
	bit 5：Ctrl键 
	bit 6：Q键 
	bit 7：E键 
	bit 8：R键 
	bit 9：F键 
	bit 10：G键 
	bit 11：Z键 
	bit 12：X键 
	bit 13：C键 
	bit 14：V键 
	bit 15：B键 
 * @保留位 
 *******************************************************************************/
typedef __packed struct 
{ 
	int16_t mouse_x; 
	int16_t mouse_y; 
	int16_t mouse_z; 
	int8_t left_button_down; 
	int8_t right_button_down; 
	uint16_t keyboard_value; 
	uint16_t reserved; 
}remote_control_t; 

/*******************************************************************************
 * @概述	操作手可使用自定义控制器模拟键鼠操作选手端。
 * @大小 	30bits： 
 * @命令码ID：0x0306 
 * @键盘键值： 
	bit 0-7：按键1键值 
	bit 8-15：按键2键值 
 * @仅响应选手端开放的按键 
* @使用通用键值，支持2键无冲，键值顺序变更不会改变按下状态，若无新的按键信息，将保持上一帧数据的按下状态 
	bit 0-11：鼠标X轴像素位置 
	bit 12-15：鼠标左键状态 
 * @位置信息使用绝对像素点值（赛事客户端使用的分辨率为1920×1080，屏幕左上角为（0，0）） 
 * @鼠标按键状态1为按下，其他值为未按下，仅在出现鼠标图标后响应该信息，若无新的鼠标信息，选手端将保持上一帧数据的鼠标信息，当鼠标图标消失后该数据不再保持 
	bit 0-11：鼠标Y轴像素位置 
	bit 12-15：鼠标右键状态 
 * @保留位
 
 
一次鼠标移动点击需要先发送鼠标未按下及指定位置的数据帧，再发送保持该位置时按下鼠标
的数据帧，最后发送保持该位置时鼠标未按下的数据帧 
 *******************************************************************************/
typedef __packed struct 
{ 
uint16_t key_value; 
  uint16_t x_position:12; 
  uint16_t mouse_left:4; 
  uint16_t y_position:12; 
uint16_t mouse_right:4; 
uint16_t reserved; 
}custom_client_data_t; 


typedef __packed struct //客户端绘制1个图形 0x0101
{
 interaction_figure_t grapic_data_struct;
} ext_client_custom_graphic_single_t;

 
//typedef __packed struct
//{
//	frame_header   					txFrameHeader;									//帧头
//	uint16_t		 						CmdID;													//命令码
//	game_status_t   game_status;//数据段头结构
//	//ext_client_custom_graphic_five_t  					clientData;//数据段
//	//ext_client_custom_character_t clientData;								//客户端字符信息
//	ext_client_custom_graphic_single_t clientData;
//	uint16_t		 						FrameTail;											//帧尾
//}ext_SendClientData_t;

//typedef __packed struct 
//{
//	frame_header   					txFrameHeader;									//帧头
//	uint16_t		 						CmdID;													//命令码
//			dataFrameHeader;//数据段头结构
//	ext_client_custom_character_t clientData;								//客户端字符信息
//	uint16_t		 						FrameTail;											//帧尾
//}ext_SendClientData_Char_t;

//typedef __packed struct
//{
//	frame_header   					txFrameHeader;									//帧头
//	uint16_t		 						CmdID;													//命令码
//	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
//	//ext_client_custom_graphic_five_t  					clientData;//数据段
//	//ext_client_custom_character_t clientData;								//客户端字符信息
//	ext_client_custom_graphic_five_t clientData;
//	uint16_t		 						FrameTail;											//帧尾
//}ext_SendClientData_5_t;

 //你该看的
typedef __packed struct
{
  uint8_t robot_id; 
  uint8_t robot_level; 
  uint16_t current_HP;  
  uint16_t maximum_HP; 
  uint16_t shooter_barrel_cooling_value; 
  uint16_t shooter_barrel_heat_limit; 
  uint16_t chassis_power_limit;  
  uint16_t buffer_energy; 
  uint16_t shooter_17mm_1_barrel_heat; 
  uint16_t shooter_17mm_2_barrel_heat; 
  uint16_t shooter_42mm_barrel_heat; 

}Judge_DataTypedef;

typedef interaction_figure_t 			interaction_figure;
typedef interaction_figure_2_t		interaction_figure_2;
typedef interaction_figure_3_t		interaction_figure_3;
typedef interaction_figure_4_t		interaction_figure_4;
//typedef ext_SendClientData_Char_t Char_DataTypedef;
typedef interaction_figure_t Graph_DataTypedef_5;
#ifdef __cplusplus
 extern "C" {
#endif
void UI_Init();
void UI_draw(uint8_t IS_tuoluo, uint8_t IS_zimiao);
//void UI_draw();
void JudgeData_analysis(uint8_t *pata, uint8_t len);
void need_data_analysis();
uint16_t Get_CRC16_Check(uint8_t *pchMessage,uint8_t dwLength);
uint8_t Get_CRC8_Check(uint8_t *pchMessage,uint16_t dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength) ;
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);
	 
void UI_Draw_Line(interaction_figure* ShowData, uint16_t Draw_Mood,uint8_t Graph_Name[3], uint32_t Graph_Layer,uint32_t Line_Width,
	uint32_t Line_Color, uint32_t Start_x, uint32_t Start_y,uint32_t End_x,uint32_t End_y);

void UI_Draw_Char(interaction_figure* ShowData, uint16_t Draw_Mood, char Graph_Name[3], uint32_t Graph_Layer,uint32_t Line_Width,
	uint32_t Char_Color, uint32_t Char_Size,uint32_t Char_Length, char Show_Str[30], uint32_t Start_x, uint32_t Start_y);
void UI_Draw_Double(interaction_figure* ShowData, uint16_t Draw_Mood, char Graph_Name[3], uint32_t Graph_Layer,uint32_t Num_Width, uint32_t Num_Color, uint32_t Num_Size, float Num, uint32_t Start_x, uint32_t Start_y);
void Send_UIdata(uint8_t* ShowData);
uint8_t JUDGE_sGetDataState(void);
uint16_t JUDGE_fGetRemainEnergy(void);
void Chassis_Power_Limit(void);
unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8) ;

//extern power_heat_data_t 			my_power_heat_data;
//extern remote_control_t 			my_remote_control_t;
extern Judge_DataTypedef Judge_data;
extern game_status_t 					my_game_status;
extern game_result_t           my_game_result_t;
extern game_robot_HP_t					my_game_robot_HP_t;
extern event_data_t 						my_event_data;
extern referee_warning_t				my_ext_referee_warning;
extern dart_info_t 						my_dart_info_t;
extern robot_status_t					my_robot_status;
extern power_heat_data_t 			my_power_heat_data;// = {0,0,0,60,0,0,0};
extern robot_pos_t 						my_robot_pos_t;
extern buff_t 									my_buff;
extern hurt_data_t 						my_robot_hurt;
extern shoot_data_t 						my_shoot_data;
extern projectile_allowance_t  my_projectile_allowance_t;
extern rfid_status_t           my_rfid_status_t;
extern dart_client_cmd_t       my_dart_client_cmd_t;
extern ground_robot_position_t my_ground_robot_position_t;
extern radar_mark_data_t       my_radar_mark_data_t;
extern sentry_info_t           my_sentry_info_t;
extern radar_info_t            my_radar_info_t;
extern robot_interaction_data_t my_robot_interaction_data_t;
extern interaction_layer_delete_t my_interaction_layer_delete_t;
extern interaction_figure_t    my_interaction_figure_t;
extern interaction_figure_2_t  my_interaction_figure_2_t;
extern interaction_figure_3_t  my_interaction_figure_3_t;
extern interaction_figure_4_t  my_interaction_figure_4_t;
extern ext_client_custom_character_t my_ext_client_custom_character_t;
extern sentry_cmd_t 						my_sentry_cmd_t;
extern radar_cmd_t							my_radar_cmd_t;

extern map_command_t 				my_robot_command;
extern map_robot_data_t      my_map_robot_data_t;
extern map_data_t            my_map_data_t;
extern custom_info_t         my_custom_info_t;
extern custom_robot_data_t   my_custom_robot_data_t;
extern robot_custom_data_t   my_robot_custom_data_t;
extern remote_control_t    	my_remote_control_t;
extern custom_client_data_t  my_custom_client_data_t;
#ifdef __cplusplus
 }
#endif
 
#endif
