#ifndef _KEY_LED_TABLE_H_
#define _KEY_LED_TABLE_H_
#include "stm32f10x_conf.h"
#include "user_conf.h"

#define LOC(x, y) 		((((x - 1) & 0xFF) << 8) | ((y - 1) & 0xFF))  	/* 高8 位X 的位置，低8 位Y 的位置 */
#define GET_X(loc)		((loc >> 8) & 0xFF)
#define GET_Y(loc)		(loc & 0xFF)
#define GET_XY(loc) 	GET_X(loc), GET_Y(loc)

extern u8 g_u8KeyTable[KEY_Y_CNT][KEY_X_CNT];
enum 
{
	_Key_Make_Record = 1,
	_Key_Make_Live,
	_Key_Make_FullScreen,
	_Key_Make_MutliRecord,
	
	_Key_Ctrl_Machine_Switch1,
	_Key_Ctrl_Machine_Switch2,
	_Key_Ctrl_Machine_Switch3,
	_Key_Ctrl_Machine_Switch4,

	_Key_Ctrl_Machine_Up,
	_Key_Ctrl_Machine_Down,

	_Key_SlowMove_Machine_Choose1,
	_Key_SlowMove_Machine_Choose2,
	_Key_SlowMove_Machine_Choose3,
	_Key_SlowMove_Machine_Choose4,

	_Key_SlowMove_Record_Start,
	_Key_SlowMove_Record_Stop,
	_Key_SlowMove_Make_Start,
	_Key_SlowMove_Make_Stop,
	_Key_SlowMove_Replay_Start,
	_Key_SlowMove_Replay_Stop,
	
	
	_Key_AI_Present1,
	_Key_AI_Present2,
	_Key_AI_Present3,
	_Key_AI_Present4,
	
	_Key_AI_Ctrl_BuildPic,
	_Key_AI_Walk_Stop,
	_Key_AI_TP_Charge,
	_Key_AI_Fun1,
	_Key_AI_Fun2,
	
	_Key_PGM_1,				
	_Key_PGM_2,
	_Key_PGM_3,
	_Key_PGM_4,
	_Key_PGM_5,
	_Key_PGM_6,
	_Key_PGM_7,
	_Key_PGM_8,
	_Key_PGM_9,
	_Key_PGM_10,
	_Key_PGM_11,
	_Key_PGM_12,		/* 24 */


	_Key_PVW_1,
	_Key_PVW_2,
	_Key_PVW_3,
	_Key_PVW_4,
	_Key_PVW_5,
	_Key_PVW_6,
	_Key_PVW_7,
	_Key_PVW_8,
	_Key_PVW_9,
	_Key_PVW_10,
	_Key_PVW_11,
	_Key_PVW_12,		/* 36 */	

	_Key_Cam_Ctrl_Tele,
	_Key_Cam_Ctrl_Wide,	

	_Key_Cam_Ctrl_Present1,
	_Key_Cam_Ctrl_Present2,
	_Key_Cam_Ctrl_Present3,
	_Key_Cam_Ctrl_Present4,


	_Key_Effect_1,
	_Key_Effect_2,
	_Key_Effect_3,
	_Key_Effect_4,
	_Key_Effect_5,
	_Key_Effect_6,		
	_Key_Effect_7,		
	_Key_Effect_8,		
	
	_Key_Overlay_1,
	_Key_Overlay_2,
	_Key_Overlay_3,
	_Key_Overlay_4,
	
	
	_Key_Switch_Speed,
	_Key_Rock_Btn,
	_Key_MiniRock_Btn,
	
	_Key_Ctrl_Reserved,

};


enum 
{
	
	_Led_Make_Live = LOC(1, 1),
	_Led_Make_MutliRecord = LOC(2, 1),
	_Led_Ctrl_Machine_Switch2 = LOC(3, 1),
	_Led_Ctrl_Machine_Switch4 = LOC(4, 1),
	_Led_Ctrl_Machine_Down = LOC(5, 1),
	_Led_Cam_Ctrl_Wide = LOC(6, 1),
	_Led_Cam_Ctrl_Present2 = LOC(7, 1),
	_Led_Cam_Ctrl_Present4 = LOC(8, 1),

	/* 1 */

	_Led_Make_Record = LOC(1, 2),
	_Led_Make_FullScreen = LOC(2, 2),
	_Led_Ctrl_Machine_Switch1 = LOC(3, 2),
	_Led_Ctrl_Machine_Switch3 = LOC(4, 2),
	_Led_Ctrl_Machine_Up = LOC(5, 2),
	_Led_Cam_Ctrl_Tele = LOC(6, 2),
	_Led_Cam_Ctrl_Present1 = LOC(7, 2),
	_Led_Cam_Ctrl_Present3 = LOC(8, 2),
	/* 2 */

	_Led_PVW_1 = LOC(1, 3),
	_Led_PVW_2 = LOC(2, 3),
	_Led_PVW_3 = LOC(3, 3),
	_Led_PVW_4 = LOC(4, 3),
	_Led_PVW_5 = LOC(5, 3),
	_Led_PVW_6 = LOC(6, 3),
	_Led_PVW_7 = LOC(7, 3),
	_Led_PVW_8 = LOC(8, 3),
	/* 3 */

	_Led_PGM_1 = LOC(1, 4),
	_Led_PGM_2 = LOC(2, 4),
	_Led_PGM_3 = LOC(3, 4),
	_Led_PGM_4 = LOC(4, 4),
	_Led_PGM_5 = LOC(5, 4),
	_Led_PGM_6 = LOC(6, 4),
	_Led_PGM_7 = LOC(7, 4),
	_Led_PGM_8 = LOC(8, 4),
	/* 4 */

	_Led_PVW_9 = LOC(1, 5),
	_Led_PVW_10 = LOC(2, 5),
	_Led_PVW_11 = LOC(3, 5),
	_Led_PVW_12 = LOC(4, 5),
	//5
	_Led_Rock_Btn = LOC(6, 5),
	/* 5 */

	_Led_PGM_9 = LOC(1, 6),
	_Led_PGM_10 = LOC(2, 6),
	_Led_PGM_11 = LOC(3, 6),
	_Led_PGM_12 = LOC(4, 6),
	_Led_MiniRock_Btn = LOC(5, 6),

	/* 6 */

	_Led_Effect_2 = LOC(1, 7),
	_Led_Effect_4 = LOC(2, 7),
	_Led_Effect_6 = LOC(3, 7),		
	_Led_Effect_8 = LOC(4, 7),		
	_Led_Overlay_2 = LOC(5, 7),
	_Led_Overlay_4 = LOC(6, 7),
	_Led_SlowMove_Machine_Choose2 = LOC(7, 7),
	_Led_SlowMove_Machine_Choose4 = LOC(8, 7),		

	/* 7 */

	_Led_Effect_1 = LOC(1, 8),
	_Led_Effect_3 = LOC(2, 8),
	_Led_Effect_5 = LOC(3, 8),		
	_Led_Effect_7 = LOC(4, 8),		
	_Led_Overlay_1 = LOC(5, 8),
	_Led_Overlay_3 = LOC(6, 8),
	_Led_SlowMove_Machine_Choose1 = LOC(7, 8),
	_Led_SlowMove_Machine_Choose3 = LOC(8, 8),		

	/* 8 */

	_Led_SlowMove_Record_Stop = LOC(1, 9),
	_Led_SlowMove_Make_Stop = LOC(2, 9),
	_Led_SlowMove_Replay_Stop = LOC(3, 9),
	_Led_Switch_Speed = LOC(4, 9),
	_Led_AI_Present1 = LOC(5, 9),
	_Led_AI_Present2 = LOC(6, 9),
	_Led_AI_Present3 = LOC(7, 9),
	_Led_AI_Present4 = LOC(8, 9),

	/* 9 */

	_Led_SlowMove_Record_Start = LOC(1, 10),
	_Led_SlowMove_Make_Start = LOC(2, 10),
	_Led_SlowMove_Replay_Start = LOC(3, 10),
	_Led_AI_Ctrl_BuildPic = LOC(4, 10),
	_Led_AI_Walk_Stop = LOC(5, 10),
	_Led_AI_TP_Charge = LOC(6, 10),
	_Led_AI_Fun1 = LOC(7, 10),
	_Led_AI_Fun2 = LOC(8, 10),
	/* 10 */	
};

#endif

