#ifndef _JUDGE_COMM_H_
#define _JUDGE_COMM_H_

#include "stdint.h"


#define 	JUDGESEND_PERIOD 	10
#define		JUDGESEND_TIME		20

#define    LEN_HEADER    5        //֡ͷ����(�ֽ�)
#define    LEN_CMDID     2        //�����볤��
#define    LEN_TAIL      2	      //֡βCRC16

/* Э��̶���ʼ�ֽڣ�0xA5 */
#define   JUDGE_FRAME_HEADER   (0xA5)
/* ��������Ӫ */
typedef enum
{
    BLUE,
    RED
} color_e;

/* ��������ƫ���� */
typedef enum
{
    FRAME_HEADER = 0,
    CMD_ID       = 5,
    DATA         = 7,
} JudgeFrameOffset;

/* ֡ͷ����ƫ���� */
typedef enum
{
    SOF         = 0, //��ʼλ
    DATA_LENGTH = 1, //֡�����ݳ���,�����������ȡ���ݳ���
    SEQ         = 3, //�����
    CRC8        = 4, //CRC8
} FrameHeaderOffset;

/* ������ID˵�� */
typedef enum
{
    ID_game_state		=		0x0001,		//����״̬				1Hz
    ID_game_result		=		0x0002,		//�������				����ʱ����
    ID_robot_HP		    =		0x0003,		//������Ѫ��			1Hz
    ID_darts_status		=		0x0004,		//���ڷ���״̬		���ڷ������

    ID_event_data		=		0x0101,		//�����¼�����		�¼��ı����
    ID_supply_action	=		0x0102,		//����վ������ʶ	������������
    ID_judge_warning	=		0x0104,		//���о�������		���淢������
    ID_Dart_Info 		=    	0x0105,		//���ڷ�����Ϣ	1Hz

    ID_robot_status		=		0x0201,		//������״̬����	10Hz
    ID_robot_power		=		0x0202,		//ʵʱ��������		50Hz
    ID_robot_position   =		0x0203,		//������λ������	10Hz
    ID_robot_buff		=		0x0204,		//��������������	����״̬�ı����
    ID_AerialRobotEnergy =       0x0205, 	//���˻�����			10Hz,ֻ���Ϳ���
    ID_robot_hurt		=		0x0206,		//�˺�״̬����		�˺���������
    ID_shoot_data		=		0x0207,		//ʵʱ�������		�ӵ��������
    ID_bullet_remaining =       0x0208,   //����ʣ�෢����	1Hz������/�ڱ�
    ID_RFID_status		=		0x0209,		//������RFID״̬	1Hz
    ID_dart_client      =       0x020A,		//���ڻ����˿ͻ���ָ������ 10Hz

    ID_robot_interact	=		0x0301,		//�����˼佻������	���ͷ���������,����10Hz
    ID_client_map		=		0x0303,		//�ͻ���С��ͼ��������	��������
} cmd_ID;

/* ���ݶγ��� */
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

    LEN_robot_interract		=		20,		//�����˼佻�����ݶ�(�Զ���,������113)
} cmd_LEN;


/******************************����Ϊ���ݽṹ�����ϸ����******************************/

/* ֡ͷ */
typedef __packed struct
{
    uint8_t  SOF;					//��ʼ�ֽ�
    uint16_t DataLength;	//���ݳ���
    uint8_t  Seq;					//�����
    uint8_t  CRC8;				//crc8У��
} frame_header;

/* ����״̬���ݣ�0x0001������Ƶ�ʣ�1Hz�����ͷ�Χ�����л����ˡ�*/
typedef __packed struct
{
    uint8_t	 game_type : 4;
    uint8_t  game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} ext_game_state_t;

/* ����������ݣ�0x0002������Ƶ�ʣ������������ͣ����ͷ�Χ�����л����ˡ�*/
typedef __packed struct
{
    uint8_t winner;
} ext_game_result_t;

/* ������Ѫ�����ݣ�0x0003������Ƶ�ʣ�1Hz�����ͷ�Χ�����л����ˡ�*/
typedef __packed struct
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;	//ǰ��վ
    uint16_t red_base_HP; 		//����

    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;


/* �����¼����ݣ�0x0101������Ƶ�ʣ�1Hz ���ڷ��ͣ����ͷ�Χ�����������ˡ�*/
typedef __packed struct
{
    uint32_t event_type;
    /*
    bit 0-2��
    bit0 ��������վ 1�Ų�Ѫ��ռ��״̬ 1Ϊ��ռ�죻
    bit1 ��������վ 2�Ų�Ѫ��ռ��״̬ 1Ϊ��ռ��
    bit2 ��������վ 3�Ų�Ѫ��ռ��״̬ 1Ϊ��ռ�죻
    bit 3-5��������������״̬��
    bit 3 Ϊ�����ռ��״̬��1Ϊռ�죻
    bit 4 ΪС�������ؼ���״̬��1Ϊ�Ѽ��
    bit 5 Ϊ���������ؼ���״̬��1Ϊ�Ѽ��

    bit 6-7���������θߵص�ռ��״̬��1 Ϊ������ռ�죬2 Ϊ���Է�ռ��
    bit 8-9���������θߵص�ռ��״̬��1 Ϊ������ռ�죬2 Ϊ���Է�ռ��
    bit 10-11���������θߵص�ռ��״̬��1 Ϊ������ռ�죬2 Ϊ���Է�ռ��
    bit 12-18�������������⻤�ܵ�ʣ��ֵ�ٷֱȣ��������룬����������
    bit 19-27���������һ�λ��м���ǰ��վ����ص�ʱ�䣨0-420������Ĭ��Ϊ0��
    bit 28-29���������һ�λ��м���ǰ��վ����صľ���Ŀ�꣬����Ĭ��Ϊ 0��
    			1 Ϊ����ǰ��վ��2 Ϊ���л��ع̶�Ŀ�꣬3 Ϊ���л������Ŀ��
    bit 30-31������������ռ�������0 Ϊδ��ռ�죬1 Ϊ������ռ�죬2 Ϊ��
    			�Է�ռ�죬3 Ϊ��˫��ռ�졣���� RMUL ���ã�
    */
} ext_event_data_t;

/* ����վ������ʶ��0x0102������Ƶ�ʣ������������ͣ����ͷ�Χ�����������ˡ�*/
typedef __packed struct
{
    uint8_t supply_projectile_id; 		//����վ�� ID��1 ��2
    uint8_t supply_robot_id; 					//���������� ID��0 Ϊ��ǰ�޻����˲�����1 Ϊ�췽Ӣ�ۻ����˲�����2 Ϊ�췽���̻����˲�����3/4/5 Ϊ�췽���������˲���������Ϊ101/102/103/104/105
    uint8_t supply_projectile_step; 	//�����ڿ���״̬��0 Ϊ�رգ�1 Ϊ�ӵ�׼���У�2 Ϊ�ӵ�����
    uint8_t supply_projectile_num;		//����������50 ��100 ��150 ��200
} ext_supply_projectile_action_t;

/* ���о�����Ϣ��cmd_id (0x0104)������Ƶ�ʣ����淢�����ͣ����ͷ�Χ�����������ˡ�*/
typedef __packed struct
{
	uint8_t level; //�������һ���ܵ��з��ĵȼ��� 1��˫������ 2������ 3������ 4���и�
	uint8_t offending_robot_id; 	//�������һ���ܵ��з���Υ������� ID������� 1 ������ ID Ϊ 1����1 ������ ID Ϊ 101��
	uint8_t count;	//�������һ���ܵ��з���Υ������˶�Ӧ�з��ȼ���Υ�������������Ĭ��Ϊ 0����
} ext_referee_warning_t;

/* ���ڷ���ڵ���ʱ��cmd_id (0x0105)������Ƶ�ʣ�1Hz ���ڷ��ͣ����ͷ�Χ������������*/
typedef __packed struct
{
    uint8_t dart_remaining_time; //�������ڷ���ʣ��ʱ�䣬��λ���� 
	uint16_t dart_info;	//bit 0-1�� ���һ�μ������ڻ��е�Ŀ�꣬����Ĭ��Ϊ 0��1 Ϊ����ǰ��վ��2 Ϊ���л��ع̶�Ŀ�꣬3 Ϊ���л������Ŀ�� 
						//bit 2-4�� �Է���������е�Ŀ���ۼƱ����м���������Ĭ��Ϊ 0������Ϊ 4 bit 5-6�����ڴ�ʱѡ���Ļ���Ŀ�꣬����Ĭ�ϻ�δѡ��/ѡ��ǰ��վʱΪ 0��
						//ѡ�л��ع̶�Ŀ��Ϊ 1��ѡ�л������Ŀ��Ϊ 2 bit 7-15������
}dart_info_t;
/* ����������״̬��0x0201������Ƶ�ʣ�10Hz�����ͷ�Χ����һ�����ˡ�*/
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


/* ʵʱ�����������ݣ�0x0202������Ƶ�ʣ�50Hz�����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{
    uint16_t chassis_volt; 					//��λ����
    uint16_t chassis_current; 			//��λ����
    float chassis_power;						//��λ��
    uint16_t chassis_power_buffer;	//��λ��������ע�����¸��ݹ���������250J
    uint16_t shooter_id1_17mm_cooling_heat; 		//1��17mm ǹ������
    uint16_t shooter_id2_17mm_cooling_heat;			//2��17mmǹ������
    uint16_t shooter_id1_42mm_cooling_heat;			//42mm ǹ������
} ext_power_heat_data_t;

/* ������λ�ã�0x0203������Ƶ�ʣ�10Hz�����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{
    float x;		//��������λ�� x ����,��λ��
    float y;		//��������λ�� y ����,��λ��
    float angle;	//�������˲���ģ��ĳ���
} ext_game_robot_pos_t;

/* ���������棺0x0204������Ƶ�ʣ�1Hz ���ڷ��ͣ����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{
 uint8_t recovery_buff; //�����˻�Ѫ���棨�ٷֱȣ�ֵΪ 10 ��ʾÿ��ָ�Ѫ�����޵� 10%��
 uint8_t cooling_buff; //������ǹ����ȴ���ʣ�ֱ��ֵ��ֵΪ 5 ��ʾ 5 ����ȴ�� 
 uint8_t defence_buff; //�����˷������棨�ٷֱȣ�ֵΪ 50 ��ʾ 50%�������棩 
 uint8_t vulnerability_buff; //�����˸��������棨�ٷֱȣ�ֵΪ 30 ��ʾ-30%�������棩
 uint16_t attack_buff;	//�����˹������棨�ٷֱȣ�ֵΪ 50 ��ʾ 50%�������棩
} ext_buff_t;

/* ���л���������״̬��0x0205������Ƶ�ʣ�10Hz�����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{
 uint8_t airforce_status; 	//���л�����״̬��0 Ϊ������ȴ��1 Ϊ��ȴ��ϣ�2 Ϊ���ڿ���֧Ԯ��
 uint8_t time_remain;	//��״̬��ʣ��ʱ�䣨��λΪ���룬����ȡ��������ȴʱ��ʣ�� 1.9 ��ʱ����ֵΪ 1�� 
						//����ȴʱ��Ϊ 0����δ���п���֧Ԯ�����ֵΪ 0
} ext_aerial_robot_energy_t;

/* �˺�״̬��0x0206������Ƶ�ʣ��˺��������ͣ����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{
    uint8_t armor_id : 4;
    uint8_t hurt_type : 4;
    /*
    bit 0-3����Ѫ���仯����Ϊװ���˺�������װ�� ID��������ֵΪ 0-4 �Ŵ�������˵����װ��Ƭ������Ѫ���仯���ͣ��ñ�����ֵΪ 0��
    bit 4-7��Ѫ���仯����:
             0x0 װ���˺���Ѫ��
             0x1 ģ����߿�Ѫ��
             0x2 �����ٿ�Ѫ��
             0x3 ��ǹ��������Ѫ��
             0x4 �����̹��ʿ�Ѫ��
             0x5 װ��ײ����Ѫ
    */
} ext_robot_hurt_t;

/* ʵʱ�����Ϣ��0x0207������Ƶ�ʣ�������ͣ����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{
    uint8_t bullet_type;		//�ӵ�����: 1��17mm ���� 2��42mm ����
    uint8_t shooter_number;			//�������ID�� 1:1��17mm�������	2:2��17mm�������		3��42mm �������
    uint8_t launching_frequency;		//�ӵ���Ƶ ��λ Hz
    float initial_speed;			//�ӵ����� ��λ m/s
} ext_shoot_data_t;

/* �ӵ�ʣ�෢������0x0208������Ƶ�ʣ�10Hz ���ڷ��ͣ����л����ˣ�*/

typedef __packed struct
{
    uint16_t bullet_remaining_num_17mm;
    /*
    17mm�ӵ�ʣ�෢����Ŀ
    ����������/Ӣ�ۻ�����:ȫ�Ӳ�����Ӣ��ʣ��ɷ���17mm��������(������)
    ���л�����/�ڱ�������:�û�����ʣ��ɷ���17mm��������(������/�Կ���)
    */
    uint16_t bullet_remaining_num_42mm;	//42mm�ӵ�ʣ�෢����Ŀ
    uint16_t coin_remaining_num; 				//ʣ��������
} ext_bullet_remaining_t;

/* ������ RFID ״̬��0x0209������Ƶ�ʣ�1Hz�����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{
    uint32_t rfid_status;
	/*
	bit λֵΪ 1/0 �ĺ��壺�Ƿ��Ѽ�⵽������� RFID �� 
	bit 0��������������� 
	bit 1���������θߵ������ 
	bit 2���Է����θߵ������ 
	bit 3������ R3/B3 ���θߵ������ 
	bit 4���Է� R3/B3 ���θߵ������ 
	bit 5������ R4/B4 ���θߵ������ 
	bit 6���Է� R4/B4 ���θߵ������ 
	bit 7�������������ؼ���� 
	bit 8��������������㣨��������һ�����ǰ�� 
	bit 9��������������㣨��������һ����º� 
	bit 10���Է���������㣨�����Է�һ�����ǰ�� 
	bit 11���Է���������㣨�����Է�һ����º� 
	bit 12������ǰ��վ����� 
	bit 13��������Ѫ�㣨��⵽��һ����Ϊ��� 
	bit 14�������ڱ�Ѳ����  
	bit 15���Է��ڱ�Ѳ���� 
	bit 16����������Դ������� 
	bit 17���Է�����Դ������� 
	bit 18�������һ��� 
	bit 19����������㣨�� RMUL ���ã� 
	bit 20-31������ 
	
	ע����������㡢�ߵ�����㡢��������㡢ǰ��վ����㡢��Դ������㡢
	��Ѫ�㡢�һ�������������㣨�������� RMUL�����ڱ�Ѳ������ RFID ��
	����������Ч�������⣬��ʹ��⵽��Ӧ�� RFID ������ӦֵҲΪ 0��
	*/
} ext_rfid_status_t;

/* ���ڻ����˿ͻ���ָ�����ݣ�0x020A������Ƶ�ʣ�10Hz�����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{
	uint8_t dart_launch_opening_status; 	//��ǰ���ڷ���վ��״̬��1���ر� 2�����ڿ������߹ر��� 0���Ѿ�����
	uint8_t reserved; 				//����λ
	uint16_t target_change_time; 	//�л�����Ŀ��ʱ�ı���ʣ��ʱ�䣬��λ���룬��/δ�л�������Ĭ��Ϊ 0
	uint16_t latest_launch_cmd_time;	//���һ�β�����ȷ������ָ��ʱ�ı���ʣ��ʱ�䣬��λ���룬��ʼֵΪ 0
} ext_dart_client_cmd_t;
/*	���������λ������: 0x020B �̶���1Hz Ƶ�ʷ��� 	*/
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

/*	�״��ǽ�������: 0x020C �̶��� 1HzƵ�ʷ��� 	*/
typedef __packed struct 
{ 
 uint8_t mark_hero_progress; 
 uint8_t mark_engineer_progress; 
 uint8_t mark_standard_3_progress; 
 uint8_t mark_standard_4_progress; 
 uint8_t mark_standard_5_progress; 
 uint8_t mark_sentry_progress; 
}radar_mark_data_t;

/*	�ڱ�����������Ϣͬ��: 0x020D �̶���1Hz Ƶ�ʷ���	*/
typedef __packed struct 
{ 
 uint32_t sentry_info; 
} sentry_info_t;

/*	�״�����������Ϣͬ��: 0x020E �̶���1Hz Ƶ�ʷ���	*/
typedef __packed struct 
{ 
 uint8_t radar_info; 
} radar_info_t;
/* �����˼佻�����ݣ�0x0200~0x02ff�����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{
    uint8_t data[LEN_robot_interract + 6];
} robot_interactive_receivedata_t;//�����˼佻������

/**************************************************************************/
/*
	�������ݽ�����Ϣ��0x0301��
	����һ��ͳһ�����ݶ�ͷ�ṹ��
	���������� ID���������Լ������ߵ� ID ���������ݶΣ�
	�����������ݵİ��ܹ������Ϊ 128 ���ֽڣ�
	��ȥ frame_header,cmd_id,frame_tail �Լ����ݶ�ͷ�ṹ�� 6 ���ֽڣ�
	�ʶ����͵��������ݶ����Ϊ 113��
	ÿ�������˽����������Զ�����������������кϼƴ�������5000 Byte��
	�����з���Ƶ�ʷֱ𲻳���30Hz��

	������ ID��
	1,Ӣ��(��)��
	2,����(��)��
	3/4/5,����(��)��
	6,����(��)��
	7,�ڱ�(��)��
	9,�״�վ(��);
	101,Ӣ��(��)��
	102,����(��)��
	103/104/105,����(��)��
	106,����(��)��
	107,�ڱ�(��)
	109,�״�վ(��)��
	110������ǰ��վ 
	111���������� 
	�ͻ��� ID��
	0x0101 ΪӢ�۲����ֿͻ���(��) ��
	0x0102 �����̲����ֿͻ��� (��)��
	0x0103/0x0104/0x0105�����������ֿͻ���(��)��
	0x0106�����в����ֿͻ���(��)��
	0x016A�����в����ֿͻ���(��)��
	0x0165��Ӣ�۲����ֿͻ���(��)��
	0x0166�����̲����ֿͻ���(��)��
	0x0167/0x0168/0x0169�����������ֿͻ���(��)��
	0x8080������ϵͳ�������������ڱ����״���������ָ�
*/
/* �Զ������ݷ����ܳ��� */



extern ext_game_robot_status_t      Game_Robot_Status;
extern ext_power_heat_data_t	    Power_Heat_Data;
extern ext_game_state_t			Game_State;

/* ��ȡ����ϵͳ������Ϣ */
int judge_data_handler(uint8_t *ReadFromUsart);

/* UI�������� */
void judge_send_task(void const *argu);

/* ��������Ӫ/ID�ж� */
int  determine_red_blue(void);
void determine_ID(void);





#endif
