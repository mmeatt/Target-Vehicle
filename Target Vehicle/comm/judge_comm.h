#ifndef _JUDGE_COMM_H_
#define _JUDGE_COMM_H_

#include "stdint.h"


#define 	JUDGESEND_PERIOD 	10
#define		JUDGESEND_TIME		20

#define    LEN_HEADER    5        //帧头长度(字节)
#define    LEN_CMDID     2        //命令码长度
#define    LEN_TAIL      2	      //帧尾CRC16

/* 协议固定起始字节：0xA5 */
#define   JUDGE_FRAME_HEADER   (0xA5)
/* 机器人阵营 */
typedef enum
{
    BLUE,
    RED
} color_e;

/* 整段数据偏移量 */
typedef enum
{
    FRAME_HEADER = 0,
    CMD_ID       = 5,
    DATA         = 7,
} JudgeFrameOffset;

/* 帧头数据偏移量 */
typedef enum
{
    SOF         = 0, //起始位
    DATA_LENGTH = 1, //帧内数据长度,根据这个来获取数据长度
    SEQ         = 3, //包序号
    CRC8        = 4, //CRC8
} FrameHeaderOffset;

/* 命令码ID说明 */
typedef enum
{
    ID_game_state		=		0x0001,		//比赛状态				1Hz
    ID_game_result		=		0x0002,		//比赛结果				结束时发送
    ID_robot_HP		    =		0x0003,		//机器人血量			1Hz
    ID_darts_status		=		0x0004,		//飞镖发射状态		飞镖发射后发送

    ID_event_data		=		0x0101,		//场地事件数据		事件改变后发送
    ID_supply_action	=		0x0102,		//补给站动作标识	动作发生后发送
    ID_judge_warning	=		0x0104,		//裁判警告数据		警告发生后发送
    ID_Dart_Info 		=    	0x0105,		//飞镖发射信息	1Hz

    ID_robot_status		=		0x0201,		//机器人状态数据	10Hz
    ID_robot_power		=		0x0202,		//实时功率热量		50Hz
    ID_robot_position   =		0x0203,		//机器人位置数据	10Hz
    ID_robot_buff		=		0x0204,		//机器人增益数据	增益状态改变后发送
    ID_AerialRobotEnergy =       0x0205, 	//无人机能量			10Hz,只发送空中
    ID_robot_hurt		=		0x0206,		//伤害状态数据		伤害发生后发送
    ID_shoot_data		=		0x0207,		//实时射击数据		子弹发射后发送
    ID_bullet_remaining =       0x0208,   //弹丸剩余发射数	1Hz，空中/哨兵
    ID_RFID_status		=		0x0209,		//机器人RFID状态	1Hz
    ID_dart_client      =       0x020A,		//飞镖机器人客户端指令数据 10Hz

    ID_robot_interact	=		0x0301,		//机器人间交互数据	发送方触发发送,上限10Hz
    ID_client_map		=		0x0303,		//客户端小地图交互数据	触发发送
} cmd_ID;

/* 数据段长度 */
typedef enum
{
    LEN_game_state			=		11,
    LEN_game_result			=		1,
    LEN_robot_HP			=		32,
    LEN_darts_status		=		4,

    LEN_event_data			=		4,
    LEN_supply_action		=		4,
    LEN_judge_warning		=		3,
    LEN_darts_info			=       3,

    LEN_robot_status		=		13,
    LEN_robot_power			=		16,
    LEN_robot_position 	    =		16,
    LEN_robot_buff			=       6,
    LEN_AerialRobotEnergy   =       2,
    LEN_robot_hurt			=		1,
    LEN_shoot_data			=		7,
    LEN_bullet_remaining    =		6,
    LEN_RFID_status			=		4,
    LEN_dart_client			=       6,

    LEN_robot_interract		=		20,		//机器人间交互数据段(自定义,不超过113)
} cmd_LEN;


/******************************以下为数据结构体的详细定义******************************/

/* 帧头 */
typedef __packed struct
{
    uint8_t  SOF;					//起始字节
    uint16_t DataLength;	//数据长度
    uint8_t  Seq;					//包序号
    uint8_t  CRC8;				//crc8校验
} frame_header;

/* 比赛状态数据：0x0001。发送频率：1Hz，发送范围：所有机器人。*/
typedef __packed struct
{
    uint8_t	 game_type : 4;
    uint8_t  game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} ext_game_state_t;

/* 比赛结果数据：0x0002。发送频率：比赛结束后发送，发送范围：所有机器人。*/
typedef __packed struct
{
    uint8_t winner;
} ext_game_result_t;

/* 机器人血量数据：0x0003。发送频率：1Hz，发送范围：所有机器人。*/
typedef __packed struct
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;	//前哨站
    uint16_t red_base_HP; 		//基地

    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;


/* 场地事件数据：0x0101。发送频率：1Hz 周期发送，发送范围：己方机器人。*/
typedef __packed struct
{
    uint32_t event_type;
    /*
    bit 0-2：
    bit0 己方补给站 1号补血点占领状态 1为已占领；
    bit1 己方补给站 2号补血点占领状态 1为已占领
    bit2 己方补给站 3号补血点占领状态 1为已占领；
    bit 3-5：己方能量机关状态：
    bit 3 为打击点占领状态，1为占领；
    bit 4 为小能量机关激活状态，1为已激活；
    bit 5 为大能量机关激活状态，1为已激活；

    bit 6-7：己方环形高地的占领状态，1 为被己方占领，2 为被对方占领
    bit 8-9：己方梯形高地的占领状态，1 为被己方占领，2 为被对方占领
    bit 10-11：己方梯形高地的占领状态，1 为被己方占领，2 为被对方占领
    bit 12-18：己方基地虚拟护盾的剩余值百分比（四舍五入，保留整数）
    bit 19-27：飞镖最后一次击中己方前哨站或基地的时间（0-420，开局默认为0）
    bit 28-29：飞镖最后一次击中己方前哨站或基地的具体目标，开局默认为 0，
    			1 为击中前哨站，2 为击中基地固定目标，3 为击中基地随机目标
    bit 30-31：中心增益点的占领情况，0 为未被占领，1 为被己方占领，2 为被
    			对方占领，3 为被双方占领。（仅 RMUL 适用）
    */
} ext_event_data_t;

/* 补给站动作标识：0x0102。发送频率：动作触发后发送，发送范围：己方机器人。*/
typedef __packed struct
{
    uint8_t supply_projectile_id; 		//补给站口 ID：1 ，2
    uint8_t supply_robot_id; 					//补弹机器人 ID：0 为当前无机器人补弹，1 为红方英雄机器人补弹，2 为红方工程机器人补弹，3/4/5 为红方步兵机器人补弹，蓝方为101/102/103/104/105
    uint8_t supply_projectile_step; 	//出弹口开闭状态：0 为关闭，1 为子弹准备中，2 为子弹下落
    uint8_t supply_projectile_num;		//补弹数量：50 ，100 ，150 ，200
} ext_supply_projectile_action_t;

/* 裁判警告信息：cmd_id (0x0104)。发送频率：警告发生后发送，发送范围：己方机器人。*/
typedef __packed struct
{
	uint8_t level; //己方最后一次受到判罚的等级： 1：双方黄牌 2：黄牌 3：红牌 4：判负
	uint8_t offending_robot_id; 	//己方最后一次受到判罚的违规机器人 ID。（如红 1 机器人 ID 为 1，蓝1 机器人 ID 为 101）
	uint8_t count;	//己方最后一次受到判罚的违规机器人对应判罚等级的违规次数。（开局默认为 0。）
} ext_referee_warning_t;

/* 飞镖发射口倒计时：cmd_id (0x0105)。发送频率：1Hz 周期发送，发送范围：己方机器人*/
typedef __packed struct
{
    uint8_t dart_remaining_time; //己方飞镖发射剩余时间，单位：秒 
	uint16_t dart_info;	//bit 0-1： 最近一次己方飞镖击中的目标，开局默认为 0，1 为击中前哨站，2 为击中基地固定目标，3 为击中基地随机目标 
						//bit 2-4： 对方最近被击中的目标累计被击中计数，开局默认为 0，至多为 4 bit 5-6：飞镖此时选定的击打目标，开局默认或未选定/选定前哨站时为 0，
						//选中基地固定目标为 1，选中基地随机目标为 2 bit 7-15：保留
}dart_info_t;
/* 比赛机器人状态：0x0201。发送频率：10Hz，发送范围：单一机器人。*/
typedef __packed struct
{
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
} ext_game_robot_status_t;


/* 实时功率热量数据：0x0202。发送频率：50Hz，发送范围：单一机器人。*/
typedef __packed struct
{
    uint16_t chassis_volt; 					//单位毫伏
    uint16_t chassis_current; 			//单位毫安
    float chassis_power;						//单位瓦
    uint16_t chassis_power_buffer;	//单位焦耳，备注：飞坡根据规则增加至250J
    uint16_t shooter_id1_17mm_cooling_heat; 		//1号17mm 枪口热量
    uint16_t shooter_id2_17mm_cooling_heat;			//2号17mm枪口热量
    uint16_t shooter_id1_42mm_cooling_heat;			//42mm 枪口热量
} ext_power_heat_data_t;

/* 机器人位置：0x0203。发送频率：10Hz，发送范围：单一机器人。*/
typedef __packed struct
{
    float x;		//本机器人位置 x 坐标,单位米
    float y;		//本机器人位置 y 坐标,单位米
    float angle;	//本机器人测速模块的朝向
} ext_game_robot_pos_t;

/* 机器人增益：0x0204。发送频率：1Hz 周期发送，发送范围：单一机器人。*/
typedef __packed struct
{
 uint8_t recovery_buff; //机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%）
 uint8_t cooling_buff; //机器人枪口冷却倍率（直接值，值为 5 表示 5 倍冷却） 
 uint8_t defence_buff; //机器人防御增益（百分比，值为 50 表示 50%防御增益） 
 uint8_t vulnerability_buff; //机器人负防御增益（百分比，值为 30 表示-30%防御增益）
 uint16_t attack_buff;	//机器人攻击增益（百分比，值为 50 表示 50%攻击增益）
} ext_buff_t;

/* 空中机器人能量状态：0x0205。发送频率：10Hz，发送范围：单一机器人。*/
typedef __packed struct
{
 uint8_t airforce_status; 	//空中机器人状态（0 为正在冷却，1 为冷却完毕，2 为正在空中支援）
 uint8_t time_remain;	//此状态的剩余时间（单位为：秒，向下取整，即冷却时间剩余 1.9 秒时，此值为 1） 
						//若冷却时间为 0，但未呼叫空中支援，则该值为 0
} ext_aerial_robot_energy_t;

/* 伤害状态：0x0206。发送频率：伤害发生后发送，发送范围：单一机器人。*/
typedef __packed struct
{
    uint8_t armor_id : 4;
    uint8_t hurt_type : 4;
    /*
    bit 0-3：当血量变化类型为装甲伤害，代表装甲 ID，其中数值为 0-4 号代表机器人的五个装甲片，其他血量变化类型，该变量数值为 0。
    bit 4-7：血量变化类型:
             0x0 装甲伤害扣血；
             0x1 模块掉线扣血；
             0x2 超射速扣血；
             0x3 超枪口热量扣血；
             0x4 超底盘功率扣血；
             0x5 装甲撞击扣血
    */
} ext_robot_hurt_t;

/* 实时射击信息：0x0207。发送频率：射击后发送，发送范围：单一机器人。*/
typedef __packed struct
{
    uint8_t bullet_type;		//子弹类型: 1：17mm 弹丸 2：42mm 弹丸
    uint8_t shooter_number;			//发射机构ID： 1:1号17mm发射机构	2:2号17mm发射机构		3：42mm 发射机构
    uint8_t launching_frequency;		//子弹射频 单位 Hz
    float initial_speed;			//子弹射速 单位 m/s
} ext_shoot_data_t;

/* 子弹剩余发射数：0x0208。发送频率：10Hz 周期发送，所有机器人，*/

typedef __packed struct
{
    uint16_t bullet_remaining_num_17mm;
    /*
    17mm子弹剩余发射数目
    步兵机器人/英雄机器人:全队步兵与英雄剩余可发射17mm弹丸总量(联盟赛)
    空中机器人/哨兵机器人:该机器人剩余可发射17mm弹丸总量(联盟赛/对抗赛)
    */
    uint16_t bullet_remaining_num_42mm;	//42mm子弹剩余发射数目
    uint16_t coin_remaining_num; 				//剩余金币数量
} ext_bullet_remaining_t;

/* 机器人 RFID 状态：0x0209。发送频率：1Hz，发送范围：单一机器人。*/
typedef __packed struct
{
    uint32_t rfid_status;
	/*
	bit 位值为 1/0 的含义：是否已检测到该增益点 RFID 卡 
	bit 0：己方基地增益点 
	bit 1：己方环形高地增益点 
	bit 2：对方环形高地增益点 
	bit 3：己方 R3/B3 梯形高地增益点 
	bit 4：对方 R3/B3 梯形高地增益点 
	bit 5：己方 R4/B4 梯形高地增益点 
	bit 6：对方 R4/B4 梯形高地增益点 
	bit 7：己方能量机关激活点 
	bit 8：己方飞坡增益点（靠近己方一侧飞坡前） 
	bit 9：己方飞坡增益点（靠近己方一侧飞坡后） 
	bit 10：对方飞坡增益点（靠近对方一侧飞坡前） 
	bit 11：对方飞坡增益点（靠近对方一侧飞坡后） 
	bit 12：己方前哨站增益点 
	bit 13：己方补血点（检测到任一均视为激活） 
	bit 14：己方哨兵巡逻区  
	bit 15：对方哨兵巡逻区 
	bit 16：己方大资源岛增益点 
	bit 17：对方大资源岛增益点 
	bit 18：己方兑换区 
	bit 19：中心增益点（仅 RMUL 适用） 
	bit 20-31：保留 
	
	注：基地增益点、高地增益点、飞坡增益点、前哨站增益点、资源岛增益点、
	补血点、兑换区、中心增益点（仅适用于 RMUL）和哨兵巡逻区的 RFID 卡
	仅在赛内生效。在赛外，即使检测到对应的 RFID 卡，对应值也为 0。
	*/
} ext_rfid_status_t;

/* 飞镖机器人客户端指令数据：0x020A。发送频率：10Hz，发送范围：单一机器人。*/
typedef __packed struct
{
	uint8_t dart_launch_opening_status; 	//当前飞镖发射站的状态：1：关闭 2：正在开启或者关闭中 0：已经开启
	uint8_t reserved; 				//保留位
	uint16_t target_change_time; 	//切换击打目标时的比赛剩余时间，单位：秒，无/未切换动作，默认为 0
	uint16_t latest_launch_cmd_time;	//最后一次操作手确定发射指令时的比赛剩余时间，单位：秒，初始值为 0
} ext_dart_client_cmd_t;
/*	地面机器人位置数据: 0x020B 固定以1Hz 频率发送 	*/
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
 float standard_5_x; 
 float standard_5_y; 
}ground_robot_position_t;

/*	雷达标记进度数据: 0x020C 固定以 1Hz频率发送 	*/
typedef __packed struct 
{ 
 uint8_t mark_hero_progress; 
 uint8_t mark_engineer_progress; 
 uint8_t mark_standard_3_progress; 
 uint8_t mark_standard_4_progress; 
 uint8_t mark_standard_5_progress; 
 uint8_t mark_sentry_progress; 
}radar_mark_data_t;

/*	哨兵自主决策信息同步: 0x020D 固定以1Hz 频率发送	*/
typedef __packed struct 
{ 
 uint32_t sentry_info; 
} sentry_info_t;

/*	雷达自主决策信息同步: 0x020E 固定以1Hz 频率发送	*/
typedef __packed struct 
{ 
 uint8_t radar_info; 
} radar_info_t;
/* 机器人间交互数据：0x0200~0x02ff。发送范围：单一机器人。*/
typedef __packed struct
{
    uint8_t data[LEN_robot_interract + 6];
} robot_interactive_receivedata_t;//机器人间交互数据

/**************************************************************************/
/*
	交互数据接收信息：0x0301。
	包括一个统一的数据段头结构，
	包含了内容 ID，发送者以及接受者的 ID 和内容数据段，
	整个交互数据的包总共长最大为 128 个字节，
	减去 frame_header,cmd_id,frame_tail 以及数据段头结构的 6 个字节，
	故而发送的内容数据段最大为 113。
	每个机器人交互数据与自定义控制器数据上下行合计带宽不超过5000 Byte。
	上下行发送频率分别不超过30Hz。

	机器人 ID：
	1,英雄(红)；
	2,工程(红)；
	3/4/5,步兵(红)；
	6,空中(红)；
	7,哨兵(红)；
	9,雷达站(红);
	101,英雄(蓝)；
	102,工程(蓝)；
	103/104/105,步兵(蓝)；
	106,空中(蓝)；
	107,哨兵(蓝)
	109,雷达站(蓝)。
	110：蓝方前哨站 
	111：蓝方基地 
	客户端 ID：
	0x0101 为英雄操作手客户端(红) ；
	0x0102 ，工程操作手客户端 (红)；
	0x0103/0x0104/0x0105，步兵操作手客户端(红)；
	0x0106，空中操作手客户端(红)；
	0x016A，空中操作手客户端(蓝)。
	0x0165，英雄操作手客户端(蓝)；
	0x0166，工程操作手客户端(蓝)；
	0x0167/0x0168/0x0169，步兵操作手客户端(蓝)；
	0x8080：裁判系统服务器（用于哨兵和雷达自主决策指令）
*/
/* 自定义数据发送总长度 */



extern ext_game_robot_status_t      Game_Robot_Status;
extern ext_power_heat_data_t	    Power_Heat_Data;
extern ext_game_state_t			Game_State;

/* 读取裁判系统反馈信息 */
int judge_data_handler(uint8_t *ReadFromUsart);

/* UI绘制任务 */
void judge_send_task(void const *argu);

/* 机器人阵营/ID判断 */
int  determine_red_blue(void);
void determine_ID(void);





#endif
