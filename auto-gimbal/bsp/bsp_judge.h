#ifndef __BSP_JUDGE_H__
#define __BSP_JUDGE_H__

#include "stdint.h"
#include "crc.h"

#define    LEN_HEADER    5        //֡ͷ����(�ֽ�)
#define    LEN_CMDID     2        //�����볤��
#define    LEN_TAIL      2	      //֡βCRC16

/* Э��̶���ʼ�ֽڣ�0xA5 */
#define   JUDGE_FRAME_HEADER   (0xA5)
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
    ID_game_state		  =		0x0001,		//����״̬				1Hz
    ID_game_result		=		0x0002,		//�������				����ʱ����
    ID_robot_HP				=		0x0003,		//������Ѫ��			1Hz
    ID_darts_status		=		0x0004,		//���ڷ���״̬		���ڷ������

    ID_event_data			=		0x0101,		//�����¼�����		�¼��ı����
    ID_supply_action	=		0x0102,		//����վ������ʶ	������������
    ID_judge_warning	=		0x0104,		//���о�������		���淢������
    ID_dart_remaining_time =0x0105,	//���ڷ��䵹��ʱ	1Hz

    ID_robot_status		=		0x0201,		//������״̬����	10Hz
    ID_robot_power		=		0x0202,		//ʵʱ��������		50Hz
    ID_robot_position =		0x0203,		//������λ������	10Hz
    ID_robot_buff			=		0x0204,		//��������������	����״̬�ı����
    ID_AerialRobotEnergy = 0x0205, 	//���˻�����			10Hz,ֻ���Ϳ���
    ID_robot_hurt			=		0x0206,		//�˺�״̬����		�˺���������
    ID_shoot_data			=		0x0207,		//ʵʱ�������		�ӵ��������
    ID_bullet_remaining = 0x0208,   //����ʣ�෢����	1Hz������/�ڱ�
    ID_RFID_status		=		0x0209,		//������RFID״̬	1Hz
    ID_dart_client    =   0x020A,		//���ڻ����˿ͻ���ָ������ 10Hz

    ID_interact				=		0x0301,		//�����˼佻������	���ͷ���������,����10Hz
    ID_client_map		  =		0x0303,		//�ͻ���С��ͼ��������	��������
		ID_map_output		  =		0x0307,		//������̨����Ϣ
} cmd_ID;

/* ���ݶγ��� */
typedef enum
{
    LEN_game_state			=		11,
    LEN_game_result			=		1,
    LEN_robot_HP				=		32,
    LEN_darts_status		=		3,

    LEN_event_data			=		4,
    LEN_supply_action		=		4,
    LEN_judge_warning		=		2,
    LEN_darts_remaining_tim = 1,

    LEN_robot_status		=		27,
    LEN_robot_power			=		16,
    LEN_robot_position 	=		16,
    LEN_robot_buff			=   1,
    LEN_AerialRobotEnergy = 2,
    LEN_robot_hurt			=		1,
    LEN_shoot_data			=		7,
    LEN_bullet_remaining=		6,
    LEN_RFID_status			=		4,
    LEN_dart_client			=   12,

    LEN_interact				=		20,		//�����˼佻�����ݶ�(�Զ���,������113)
		LEN_Map				=		103,		
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
} ext_game_status_t;

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

/* ���ڷ���״̬��0x0004������Ƶ�ʣ����ڷ�����ͣ����ͷ�Χ�����л����ˡ�*/
typedef __packed struct
{
    uint8_t  dart_belong;
    uint16_t stage_remaining_time;
} ext_dart_status_t;

/* �����¼����ݣ�0x0101������Ƶ�ʣ�1Hz ���ڷ��ͣ����ͷ�Χ�����������ˡ�*/
typedef __packed struct
{
    uint32_t event_type;
} ext_event_data_t;

/* ����վ������ʶ��0x0102������Ƶ�ʣ������������ͣ����ͷ�Χ�����������ˡ�*/
typedef __packed struct
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

/* ���о�����Ϣ��cmd_id (0x0104)������Ƶ�ʣ����淢�����ͣ����ͷ�Χ�����������ˡ�*/
typedef __packed struct
{
    uint8_t level;
    uint8_t foul_robot_id;
} ext_referee_warning_t;

/* ���ڷ���ڵ���ʱ��cmd_id (0x0105)������Ƶ�ʣ�1Hz ���ڷ��ͣ����ͷ�Χ������������*/
typedef __packed struct
{
    uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

/* ����������״̬��0x0201������Ƶ�ʣ�10Hz�����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{
    uint8_t	 robot_id;
    uint8_t	 robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;

    uint16_t shooter_id1_17mm_cooling_rate;
    uint16_t shooter_id1_17mm_cooling_limit;
    uint16_t shooter_id1_17mm_speed_limit;
    uint16_t shooter_id2_17mm_cooling_rate;
    uint16_t shooter_id2_17mm_cooling_limit;
    uint16_t shooter_id2_17mm_speed_limit;
    uint16_t shooter_id1_42mm_cooling_rate;
    uint16_t shooter_id1_42mm_cooling_limit;
    uint16_t shooter_id1_42mm_speed_limit;

    uint16_t chassis_power_limit;

    uint8_t	 mains_power_gimbal_output : 1;
    uint8_t	 mains_power_chassis_output : 1;
    uint8_t	 mains_power_shooter_output : 1;
} ext_game_robot_status_t;

/* ʵʱ�����������ݣ�0x0202������Ƶ�ʣ�50Hz�����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float		 chassis_power;
    uint16_t chassis_power_buffer;
    uint16_t shooter_id1_17mm_cooling_heat;
    uint16_t shooter_id2_17mm_cooling_heat;
    uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

/* ������λ�ã�0x0203������Ƶ�ʣ�10Hz�����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{
    float x;
    float y;
    float z;
    float yaw;
} ext_game_robot_pos_t;

/* ���������棺0x0204������Ƶ�ʣ�1Hz ���ڷ��ͣ����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{
    uint8_t power_rune_buff;
} ext_buff_t;

/* ���л���������״̬��0x0205������Ƶ�ʣ�10Hz�����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{
    uint8_t	attack_time;
} ext_aerial_robot_energy_t;

/* �˺�״̬��0x0206������Ƶ�ʣ��˺��������ͣ����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{
    uint8_t armor_id  : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;

/* ʵʱ�����Ϣ��0x0207������Ƶ�ʣ�������ͣ����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{
    uint8_t bullet_type;
    uint8_t shooter_id;
    uint8_t bullet_freq;
    float 	bullet_speed;
} ext_shoot_data_t;

/* �ӵ�ʣ�෢������0x0208������Ƶ�ʣ�10Hz�����ͷ�Χ�����л����ˡ�*/
typedef __packed struct
{
    uint16_t bullet_remaining_num_17mm;
    uint16_t bullet_remaining_num_42mm;
    uint16_t coin_remaining_num;
} ext_bullet_remaining_t;

/* ������RFID״̬��0x0209������Ƶ�ʣ�1Hz�����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{
    uint32_t rfid_status;
} ext_rfid_status_t;

/* ���ڻ����˿ͻ���ָ�����ݣ�0x020A������Ƶ�ʣ�10Hz�����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{
    uint8_t  dart_launch_opening_status;
    uint8_t  dart_attack_target;
    uint16_t target_change_time;
    uint8_t  first_dart_speed;
    uint8_t  second_dart_speed;
    uint8_t  third_dart_speed;
    uint8_t  fourth_dart_speed;
    uint16_t last_dart_launch_time;
    uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;


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
	�ͻ��� ID��
	0x0101 ΪӢ�۲����ֿͻ���(��) ��
	0x0102 �����̲����ֿͻ��� (��)��
	0x0103/0x0104/0x0105�����������ֿͻ���(��)��
	0x0106�����в����ֿͻ���(��)��
	0x0165��Ӣ�۲����ֿͻ���(��)��
	0x0166�����̲����ֿͻ���(��)��
	0x0167/0x0168/0x0169�����������ֿͻ���(��)��
	0x016A�����в����ֿͻ���(��)��
*/
/* �Զ������ݷ����ܳ��� */
typedef enum
{
    DRAW_IMAGINE1=30,
    DRAW_IMAGINE2=45,
    DRAW_IMAGINE5=90,
    DRAW_IMAGINE7=120,
    PRINTF_CODE	=60,
    JUDGE_DELETE=17,
    ROBOT_COMM=(15+LEN_interact),
} Judgedatalength;

/* ���ݶε�����ID */
typedef enum
{
    CLIENT_INTERACTIVE_CMD_ID_TEST    = 0x0200,  //�������ݣ�����0x0200~0x02ffѡȡ������ID�����ɲ������Զ���
    CLIENT_DELETE_GRAPH_CMD_ID     		= 0x0100,  //�ͻ���ɾ��ͼ��
    CLIENT_DRAW_1_GRAPHS_CMD_ID     	= 0x0101,  //�ͻ��˻���1��ͼ��
    CLIENT_DRAW_2_GRAPHS_CMD_ID    		= 0x0102,  //�ͻ��˻���2��ͼ��
    CLIENT_DRAW_5_GRAPHS_CMD_ID    		= 0x0103,  //�ͻ��˻���5��ͼ��
    CLIENT_DRAW_7_GRAPHS_CMD_ID    		= 0x0104,  //�ͻ��˻���7��ͼ��
    CLIENT_WRITE_STRINGS_CMD_ID    		= 0x0110,  //�ͻ��˻����ַ���ͼ��
} client_data_cmd_e;

/* �������ݽ�����Ϣ֡ͷ */
typedef __packed struct
{
    client_data_cmd_e data_cmd_id;
    uint16_t 					send_ID;
    uint16_t 					receiver_ID;
} ext_judgesend_custom_header_t;

/* ͼ��ɾ������ */
typedef enum
{
    type_delete_nop   = 0,
    type_delete_layer = 1,
    type_delete_all   = 2,
} type_graphic_delete_e;

/* �ͻ���ɾ��ͼ�� */
typedef __packed struct
{
    type_graphic_delete_e operate_tpye;
    uint8_t layer;
} ext_SendClientDelete_t;

/* ͼ������ */
typedef __packed struct
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye:3;
    uint32_t graphic_tpye:3;
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t start_angle:9;
    uint32_t end_angle:9;
    uint32_t width:10;
    uint32_t start_x:11;
    uint32_t start_y:11;
    uint32_t radius:10;
    uint32_t end_x:11;
    uint32_t end_y:11;
} graphic_data_struct_t;

/* �ͻ��˻����ַ� */
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct;
    uint8_t data[30];
} ext_client_custom_character_t;

/* �����˼佻�����ݣ�0x0200~0x02ff */
typedef __packed struct
{
    uint8_t data[LEN_interact];
} robot_interactive_data_t;

/*
	С��ͼ������Ϣ����һ��ͳһ�����ݶ�ͷ�ṹ��
	С��ͼ������Ϣ��ʶ��0x0303������Ƶ�ʣ�����ʱ���͡�
*/
typedef __packed struct
{
    float target_position_x;		//Ŀ��xλ�����꣬��λm		������Ŀ�������IDʱ������Ϊ0
    float target_position_y;		//Ŀ��yλ�����꣬��λm 		������Ŀ�������IDʱ������Ϊ0
    float target_position_z;		//Ŀ��zλ�����꣬��λm 		������Ŀ�������IDʱ������Ϊ0
    uint8_t commd_keyboard;			//����ָ��ʱ����̨�ְ��µļ�����Ϣ 		�ް���������Ϊ0
    uint16_t target_robot_ID;		//Ҫ���õ�Ŀ�������ID 		������λ����Ϣʱ������Ϊ0
} ext_robot_command_t;
//0x0307 �ڱ�ģʽ�Լ�·��С��ͼ��ʾ
typedef __packed struct 
{ 
 uint8_t intention; 
 uint16_t start_position_x; 
 uint16_t start_position_y; 
 int8_t delta_x[49]; 
 int8_t delta_y[49]; 
 }map_sentry_data_t; //103���ֽ�
/* ���ݶ� */
typedef __packed struct
{
    ext_SendClientDelete_t 					delete_data;		//(2)
    graphic_data_struct_t						draw1_data;			//�ͻ��˻���1��ͼ��(15)
    graphic_data_struct_t						draw2_data[2];	//�ͻ��˻���2��ͼ��(30)
    graphic_data_struct_t						draw5_data[5];	//�ͻ��˻���5��ͼ��(75)
    graphic_data_struct_t						draw7_data[7];	//�ͻ��˻���7��ͼ��(105)
    ext_client_custom_character_t		code_data;			//�ͻ��˻����ַ�(45)
    robot_interactive_data_t				comm_senddata;	//�����˼佻������(LEN_comm_data)
    ext_robot_command_t							Client_map;			//С��ͼ������Ϣ(15)
} ext_clientdata_t;

/* �ϴ��ͻ��� */
typedef __packed struct
{
    frame_header   									txFrameHeader;//֡ͷ5
    cmd_ID		 											CmdID;//������2
    ext_judgesend_custom_header_t		FrameHeader;//���ݶ�ͷ�ṹ6
    ext_clientdata_t  							clientdata;//���ݶ�
    uint16_t		 										FrameTail;//֡β2
} ext_SendClient_t;

typedef __packed struct
{		
		uint16_t data_cmd_id;
		uint16_t sender_ID;
		uint16_t receiver_ID;
    float blue_x;
		float blue_y;
		float blue_confiden;
		float red_x;
		float red_y;
		float red_confiden;
} radar_data;
typedef __packed struct
{		
		uint16_t data_cmd_id;
		uint16_t sender_ID;
		uint16_t receiver_ID;
    uint16_t position_control;
		uint16_t tuoluo_control;
		uint16_t shoot_control;
		uint16_t other_control;
	
} sentry_control;
typedef struct 
{
    /* data */
ext_game_status_t					Game_State_data;
ext_game_result_t					Game_Result_data;
ext_game_robot_HP_t                 RobotHP_data;
ext_dart_status_t					Dart_Status_data;
ext_event_data_t					Event_Data_data;
ext_supply_projectile_action_t		Supply_Projectile_Action_data;
ext_referee_warning_t				Referee_Warning_data;
ext_dart_remaining_time_t			Dart_Remaining_Time_data;
ext_game_robot_status_t				Game_Robot_Status_data;
ext_power_heat_data_t				Power_Heat_Data_data;
ext_game_robot_pos_t				Robot_Position_data;
ext_buff_t							Buff_data;
ext_aerial_robot_energy_t			Aerial_Robot_Energy_data;
ext_robot_hurt_t					Robot_Hurt_data;
ext_shoot_data_t					Shoot_Data_data;
ext_bullet_remaining_t				Bullet_Remaining_data;
ext_rfid_status_t					RFID_Status_data;
ext_dart_client_cmd_t				Dart_Client_data;
	radar_data 									Radar_Data;
	ext_robot_command_t  				Robot_command_Data;
}robot_judge_msg_t;



/* �������ݽ�����Ϣ֡ͷ */
typedef __packed struct 
{ 
	client_data_cmd_e data_cmd_id;    
	uint16_t 					send_ID;    
	uint16_t 					receiver_ID; 
} judgesend_custom_header_t;
typedef __packed struct
{
	frame_header   							txFrameHeader;//֡ͷ5
	cmd_ID		 									CmdID;//������2;
	judgesend_custom_header_t		data_header;//���ݶ�֡ͷ 6
	//�˴�Ϊ�������ݶ�
	uint16_t		 								FrameTail;//֡β 2
}judgesend_frame_t;
typedef enum 
{
	BLUE,
	RED
}color_e;
typedef __packed struct
{
	color_e	 color;				//��Ӫ
	uint16_t robot_ID;		//����id
	uint16_t client_ID;		//��Ӧ�ͻ���id
	uint16_t self_HP;			//����Ѫ��
}self_inf_t;
extern robot_judge_msg_t robot_judge_msg;
extern ext_game_robot_status_t Game_Robot_Status;
extern ext_power_heat_data_t	 Power_Heat_Data;
extern ext_game_status_t			 Game_State;
extern map_sentry_data_t 					Sentry_map;
extern ext_robot_command_t        	Robot_command;//��̨��ָ������
extern ext_shoot_data_t					Shoot_Data;
/* ��ȡ����ϵͳ������Ϣ */
int judge_data_handler(uint8_t *ReadFromUsart);

/* UI�������� */
void judge_send_task(void const *argu);
/* ��������Ӫ/ID�ж� */
int  determine_red_blue(void);
void determine_ID(void);
void judge_get_basedata(void);
void ui_sendmessage(void);
/* UI��ͼ�ײ㺯�� */
void data_pack_imagine(Judgedatalength data_length, uint8_t* data_locate, client_data_cmd_e date_type);
void data_pack_code(ext_client_custom_character_t code_date, uint8_t* CODE);
void data_pack_delete(type_graphic_delete_e type, uint8_t layer);
void data_pack_comm_data(uint16_t Robot_Target_ID, client_data_cmd_e CLIENT_INTERACTIVE_CMD_ID, uint8_t* comm_date);
void data_pack_map_position(float target_position_x, float target_position_y, float target_position_z, uint8_t commd_keyboard, uint16_t target_robot_ID);

/* UI����ͼ�λ��� */
void ui_init(void);
void ui_data_pack(void);
void judge_send_supercap(uint8_t supercap_percent);
void judge_send_shoot_mode(uint8_t up, uint8_t down);
void judge_send_code_display(uint8_t gimbal, uint8_t chassis);
void judge_send_light_display(uint8_t light);
void robot_judge_msg_copy(void);
#endif

