#include <stdbool.h>
#include "stm32f10x_conf.h"
#include "user_conf.h"
#include "io_buf_ctrl.h"
#include "key_led.h"
#include "key_led_table.h"

u8 g_u8KeyTable[KEY_Y_CNT][KEY_X_CNT] = 
{
	{
		_Key_Make_Live,
		_Key_Make_MutliRecord,
		_Key_Ctrl_Machine_Switch2,
		_Key_Ctrl_Machine_Switch4,
		_Key_Ctrl_Machine_Down,
		_Key_Cam_Ctrl_Wide,
		_Key_Cam_Ctrl_Present2,
		_Key_Cam_Ctrl_Present4,

	},	/* 1 */
	{
		_Key_Make_Record,
		_Key_Make_FullScreen,
		_Key_Ctrl_Machine_Switch1,
		_Key_Ctrl_Machine_Switch3,
		_Key_Ctrl_Machine_Up,
		_Key_Cam_Ctrl_Tele,
		_Key_Cam_Ctrl_Present1,
		_Key_Cam_Ctrl_Present3,
	},	/* 2 */
	{
		_Key_PVW_1,
		_Key_PVW_2,
		_Key_PVW_3,
		_Key_PVW_4,
		_Key_PVW_5,
		_Key_PVW_6,
		_Key_PVW_7,
		_Key_PVW_8,
	},	/* 3 */
	{
		_Key_PGM_1,
		_Key_PGM_2,
		_Key_PGM_3,
		_Key_PGM_4,
		_Key_PGM_5,
		_Key_PGM_6,
		_Key_PGM_7,
		_Key_PGM_8,
	},	/* 4 */
	{
		_Key_PVW_9,
		_Key_PVW_10,
		_Key_PVW_11,
		_Key_PVW_12,
		0,
		_Key_Rock_Btn,
	},	/* 5 */
	{
		_Key_PGM_9,
		_Key_PGM_10,
		_Key_PGM_11,
		_Key_PGM_12,
		_Key_MiniRock_Btn,
	
	}, /* 6 */
	{
		_Key_Effect_2,
		_Key_Effect_4,
		_Key_Effect_6,		
		_Key_Effect_8,		
		_Key_Overlay_2,
		_Key_Overlay_4,
		_Key_SlowMove_Machine_Choose2,
		_Key_SlowMove_Machine_Choose4,		
	
	}, /* 7 */
	{
		_Key_Effect_1,
		_Key_Effect_3,
		_Key_Effect_5,		
		_Key_Effect_7,		
		_Key_Overlay_1,
		_Key_Overlay_3,
		_Key_SlowMove_Machine_Choose1,
		_Key_SlowMove_Machine_Choose3,		
	
	}, /* 8 */
	{
		_Key_SlowMove_Record_Stop,
		_Key_SlowMove_Make_Stop,
		_Key_SlowMove_Replay_Stop,
		_Key_Switch_Speed,
		_Key_AI_Present1,
		_Key_AI_Present2,
		_Key_AI_Present3,
		_Key_AI_Present4,
		
	}, /* 9 */
	{
		_Key_SlowMove_Record_Start,
		_Key_SlowMove_Make_Start,
		_Key_SlowMove_Replay_Start,
		_Key_AI_Ctrl_BuildPic,
		_Key_AI_Walk_Stop,
		_Key_AI_TP_Charge,
		_Key_AI_Fun1,
		_Key_AI_Fun2,
	}, /* 10 */
};

/* dp, g, f, e, d, c, b, a */
const u8 g_u8LED7Code[] = 
{
	0x3F,		// 0
	0x06,		// 1
	0x5B,		// 2
	0x4F,		// 3
	0x66,		// 4
	0x6D,		// 5
	0x7D,		// 6
	0x07,		// 7
	0x7F,		// 8
	0x6F,		// 9
	0x77,		// A
	0x7C,		// B
	0x39,		// C
	0x5E,		// D
	0x79,		// E
	0x71,		// F
	0x40,		// -
};
 

const u16 g_u16CamAddrLoc[CAM_ADDR_MAX] = 
{
	0,
};

