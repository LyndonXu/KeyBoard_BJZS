/******************(C) copyright 天津市XXXXX有限公司 *************************
* All Rights Reserved
* 文件名：code_swtich.c
* 摘要: 键盘以及LED刷新程序
* 版本：0.0.1
* 作者：许龙杰
* 日期：2013年01月25日
*******************************************************************************/

#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include "stm32f10x_conf.h"
#include "user_conf.h"
#include "user_api.h"
#include "io_buf_ctrl.h"
#include "app_port.h"

#include "key_led.h"
#include "adc_ctrl.h"
#include "code_switch.h"
#include "key_led_ctrl.h"

#if 0
#define CODE_SWITCH_MAX						2
#define CODE_SWITCH1_MAX_VALUE				30
#define CODE_SWITCH2_MAX_VALUE				30
#define CODE_SWITCH3_MAX_VALUE				64
#define CODE_SWITCH4_MAX_VALUE				64

#define CODE_SWITCH1_REVERSE	1
#define CODE_SWITCH2_REVERSE	0
#define CODE_SWITCH3_REVERSE	1
#define CODE_SWITCH4_REVERSE	1

#define CODE_SWITCH1_PIN_A				GPIO_Pin_14
#define CODE_SWITCH1_PIN_A_PORT			GPIOC
#define CODE_SWITCH1_PIN_B				GPIO_Pin_15
#define CODE_SWITCH1_PIN_B_PORT			GPIOC


#define CODE_SWITCH1_INT_SRC			GPIO_PinSource14
#define CODE_SWITCH1_INT_SRC_PORT		GPIO_PortSourceGPIOC
#define CODE_SWITCH1_INT_LINE			EXTI_Line14
#define CODE_SWITCH1_INT_CHANNEL		EXTI15_10_IRQn

#define CODE_SWITCH2_PIN_A				GPIO_Pin_12
#define CODE_SWITCH2_PIN_A_PORT			GPIOC
#define CODE_SWITCH2_PIN_B				GPIO_Pin_13
#define CODE_SWITCH2_PIN_B_PORT			GPIOC


#define CODE_SWITCH2_INT_SRC			GPIO_PinSource13
#define CODE_SWITCH2_INT_SRC_PORT		GPIO_PortSourceGPIOC
#define CODE_SWITCH2_INT_LINE			EXTI_Line13
#define CODE_SWITCH2_INT_CHANNEL		EXTI15_10_IRQn


#endif


#define CODE_SWITCH_MAX						2
#define CODE_SWITCH1_MAX_VALUE				30
#define CODE_SWITCH2_MAX_VALUE				30
#define CODE_SWITCH3_MAX_VALUE				64
#define CODE_SWITCH4_MAX_VALUE				64

#define CODE_SWITCH1_MAX_CW_VALUE			0x3F
#define CODE_SWITCH1_MAX_CCW_VALUE			0x3F
#define CODE_SWITCH2_MAX_CW_VALUE			0x3F
#define CODE_SWITCH2_MAX_CCW_VALUE			0x3F


#define CODE_SWITCH1_REVERSE	1
#define CODE_SWITCH2_REVERSE	1
#define CODE_SWITCH3_REVERSE	1
#define CODE_SWITCH4_REVERSE	1


#define CANCLE_TIME			(1)

#define CODE_SWITCH1_PIN_A				GPIO_Pin_14
#define CODE_SWITCH1_PIN_A_PORT			GPIOC
#define CODE_SWITCH1_PIN_B				GPIO_Pin_15
#define CODE_SWITCH1_PIN_B_PORT			GPIOC


#define CODE_SWITCH1_INT_SRC			GPIO_PinSource14
#define CODE_SWITCH1_INT_SRC_PORT		GPIO_PortSourceGPIOC
#define CODE_SWITCH1_INT_LINE			EXTI_Line14
#define CODE_SWITCH1_INT_CHANNEL		EXTI15_10_IRQn

#define CODE_SWITCH2_PIN_A				GPIO_Pin_12
#define CODE_SWITCH2_PIN_A_PORT			GPIOC
#define CODE_SWITCH2_PIN_B				GPIO_Pin_13
#define CODE_SWITCH2_PIN_B_PORT			GPIOC


#define CODE_SWITCH2_INT_SRC			GPIO_PinSource12
#define CODE_SWITCH2_INT_SRC_PORT		GPIO_PortSourceGPIOC
#define CODE_SWITCH2_INT_LINE			EXTI_Line12
#define CODE_SWITCH2_INT_CHANNEL		EXTI15_10_IRQn

typedef struct _tagStCodeSwitchPin
{
	StPinSource stPinA;
	StPinSource stPinB;
}StCodeSwitchPin;

static const StCodeSwitchPin s_c_stCodeSwitchPin[CODE_SWITCH_MAX] = 
{
	{{CODE_SWITCH1_PIN_A_PORT, CODE_SWITCH1_PIN_A}, {CODE_SWITCH1_PIN_B_PORT, CODE_SWITCH1_PIN_B}},
	{{CODE_SWITCH2_PIN_A_PORT, CODE_SWITCH2_PIN_A}, {CODE_SWITCH2_PIN_B_PORT, CODE_SWITCH2_PIN_B}},
};

/* ms count, 0 for no change, 1 for get interrupt, 2 and others for ms after interrupt  */
static uint8_t s_u8SwitchMode[CODE_SWITCH_MAX] = {0};	
#define s_u8SwitchMode1		s_u8SwitchMode[0]
#define s_u8SwitchMode2		s_u8SwitchMode[1]
#define s_u8SwitchMode3		s_u8SwitchMode[2]
#define s_u8SwitchMode4		s_u8SwitchMode[3]


static StCodeSwitchState 	s_stCodeSwitchState[CODE_SWITCH_MAX];
#define s_stCodeSwitch1		s_stCodeSwitchState[0]
#define s_stCodeSwitch2		s_stCodeSwitchState[1]
#define s_stCodeSwitch3		s_stCodeSwitchState[2]
#define s_stCodeSwitch4		s_stCodeSwitchState[3]

const u16 c_u16CodeSwitchMaxValue[CODE_SWITCH_MAX] = 
{
	CODE_SWITCH1_MAX_VALUE,
	CODE_SWITCH2_MAX_VALUE,
};

const u16 c_u16CodeSwitchMaxCWValue[CODE_SWITCH_MAX] = 
{
	CODE_SWITCH1_MAX_CW_VALUE,
	CODE_SWITCH2_MAX_CW_VALUE,
};

const u16 c_u16CodeSwitchMaxCCWValue[CODE_SWITCH_MAX] = 
{
	CODE_SWITCH1_MAX_CCW_VALUE,
	CODE_SWITCH2_MAX_CCW_VALUE,
};

const bool c_bCodeSwitchDirection[CODE_SWITCH_MAX] = 
{
	CODE_SWITCH1_REVERSE, 
	CODE_SWITCH2_REVERSE,
};

const bool c_bCodeSwtichLastValueShouldBeEven[CODE_SWITCH_MAX] = 
{
	false,
	false,
};


static void CodeSwitchPinInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed =   GPIO_Speed_2MHz;
	
	/* switch1 */
	GPIO_InitStructure.GPIO_Pin = CODE_SWITCH1_PIN_A;
	GPIO_Init(CODE_SWITCH1_PIN_A_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = CODE_SWITCH1_PIN_B;
	GPIO_Init(CODE_SWITCH1_PIN_B_PORT, &GPIO_InitStructure);


	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	
	/* Connect EXTI for switch 1 */
	GPIO_EXTILineConfig(CODE_SWITCH1_INT_SRC_PORT, CODE_SWITCH1_INT_SRC);

	EXTI_InitStructure.EXTI_Line = CODE_SWITCH1_INT_LINE;
	EXTI_Init(&EXTI_InitStructure);


	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	/* Enable and set switch 1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = CODE_SWITCH1_INT_CHANNEL;
	NVIC_Init(&NVIC_InitStructure);


	/* switch2 */
	GPIO_InitStructure.GPIO_Pin = CODE_SWITCH2_PIN_A;
	GPIO_Init(CODE_SWITCH2_PIN_A_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = CODE_SWITCH2_PIN_B;
	GPIO_Init(CODE_SWITCH2_PIN_B_PORT, &GPIO_InitStructure);


	/* Connect EXTI for switch 1 */
	GPIO_EXTILineConfig(CODE_SWITCH2_INT_SRC_PORT, CODE_SWITCH2_INT_SRC);

	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_Line = CODE_SWITCH2_INT_LINE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set switch 1 Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = CODE_SWITCH2_INT_CHANNEL;
	NVIC_Init(&NVIC_InitStructure); */

}

void ChangeCodeSwitch(u8 u8Index, bool boIsCW)
{
	if (u8Index >= CODE_SWITCH_MAX)
	{
		return;
	}
	if (boIsCW)
	{
		s_stCodeSwitchState[u8Index].u16Cnt++;
		s_stCodeSwitchState[u8Index].u16CWCnt++;
	}
	else
	{
		s_stCodeSwitchState[u8Index].u16Cnt--;
		s_stCodeSwitchState[u8Index].u16CCWCnt++;
	}
	s_stCodeSwitchState[u8Index].u16Dir = boIsCW;
	
	if (s_stCodeSwitchState[u8Index].u16Cnt > c_u16CodeSwitchMaxValue[u8Index])
	{
		if (boIsCW)
		{
			s_stCodeSwitchState[u8Index].u16Cnt = 0;
		}
		else
		{
			s_stCodeSwitchState[u8Index].u16Cnt = c_u16CodeSwitchMaxValue[u8Index];
		}
	}

	if (s_stCodeSwitchState[u8Index].u16CWCnt > c_u16CodeSwitchMaxCWValue[u8Index])
	{
		s_stCodeSwitchState[u8Index].u16CWCnt = 0;
	}
		
	if (s_stCodeSwitchState[u8Index].u16CCWCnt > c_u16CodeSwitchMaxCCWValue[u8Index])
	{
		s_stCodeSwitchState[u8Index].u16CCWCnt = 0;
	}
	
}

void FlushCodeSwitch(u8 u8Index)
{
	bool boIsCW;
	u16 u16PinA, u16PinB;

	if (u8Index >= CODE_SWITCH_MAX)
	{
		return;
	}
	
	u16PinB = s_c_stCodeSwitchPin[u8Index].stPinB.pPort->IDR;
	u16PinA = s_c_stCodeSwitchPin[u8Index].stPinA.pPort->IDR;

	u16PinA &= s_c_stCodeSwitchPin[u8Index].stPinA.u16Pin;
	u16PinB &= s_c_stCodeSwitchPin[u8Index].stPinB.u16Pin;

	if (u16PinA == 0)
	{
		if (u16PinB == 0)
		{
			boIsCW = true;
		}
		else
		{
			boIsCW = false;
		}	
	}
	else
	{
		if (u16PinB == 0)
		{
			boIsCW = false;
		}
		else
		{
			boIsCW = true;
		}	
	}
	if (c_bCodeSwitchDirection[u8Index])
	{
		boIsCW = !boIsCW;
	}
	ChangeCodeSwitch(u8Index, boIsCW);
	
	{
		u32 u32Time = s_stCodeSwitchState[u8Index].u32TriggerTime;
		s_stCodeSwitchState[u8Index].u32TriggerTime = g_u32SysTickCnt;
		u32Time  = SysTimeDiff(u32Time, g_u32SysTickCnt);
		if (u32Time > 250)
		{
			s_stCodeSwitchState[u8Index].u16Speed = 1;
		}
		else
		{
			if (u32Time != 0)
			{
				s_stCodeSwitchState[u8Index].u16Speed = 500 / u32Time;				
			}
			if (s_stCodeSwitchState[u8Index].u16Speed > 80)
			{
				s_stCodeSwitchState[u8Index].u16Speed = 80;
			}
		}
		
	}
	
}

/* codeswitch 1 & 2 */
void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(CODE_SWITCH1_INT_LINE) != RESET)
	{	
		USE_CRITICAL();
		ENTER_CRITICAL();
		s_u8SwitchMode1 = 1;
		EXIT_CRITICAL();
		/* Clear the  pending bit */
		EXTI_ClearITPendingBit(CODE_SWITCH1_INT_LINE);
	}
	
	if(EXTI_GetITStatus(CODE_SWITCH2_INT_LINE) != RESET)
	{	
		USE_CRITICAL();
		ENTER_CRITICAL();
		s_u8SwitchMode2 = 1;
		EXIT_CRITICAL();
		/* Clear the  pending bit */
		EXTI_ClearITPendingBit(CODE_SWITCH2_INT_LINE);
	}
}

void CodeSwitchInit(void)
{
	u32 i;
	CodeSwitchPinInit();

	for (i = 0; i < CODE_SWITCH_MAX; i++)
	{
		s_stCodeSwitchState[i].u16Index = i;
		s_stCodeSwitchState[i].u16Cnt = 0;
		s_stCodeSwitchState[i].u16OldCnt = 0;		
		s_stCodeSwitchState[i].u16CWCnt = 0;		
		s_stCodeSwitchState[i].u16CCWCnt = 0;		
	}
}



static bool CodeSwitchGetValueInner(StCodeSwitchState *pState)
{
	if (pState->u16Cnt != pState->u16OldCnt)
	{
		pState->u16OldCnt = pState->u16Cnt;
		return true;
	}
	return false;
}

u16 CodeSwitchPlus(u16 u16Index)
{
	USE_CRITICAL();
	u16 u16Cnt;
	if (u16Index > CODE_SWITCH_MAX)
	{
		return ~0;
	}

	u16Cnt = s_stCodeSwitchState[u16Index].u16Cnt + 1;

	ENTER_CRITICAL();
	if (u16Cnt > c_u16CodeSwitchMaxValue[u16Index])
	{
		u16Cnt = 0;
	}
	s_stCodeSwitchState[u16Index].u16Cnt = s_stCodeSwitchState[u16Index].u16OldCnt = u16Cnt;
	EXIT_CRITICAL();
	return u16Cnt;
}


u16 CodeSwitchGetValue(u16 u16Index)
{
	return s_stCodeSwitchState[u16Index].u16OldCnt;
}

u16 CodeSwitchSetValue(u16 u16Index, u16 u16Value)
{
	USE_CRITICAL();
	if (u16Index > CODE_SWITCH_MAX)
	{
		return ~0;
	}

	if (u16Value > c_u16CodeSwitchMaxValue[u16Index])
	{
		u16Value = c_u16CodeSwitchMaxValue[u16Index];
	}

	ENTER_CRITICAL();
	s_stCodeSwitchState[u16Index].u16Cnt = s_stCodeSwitchState[u16Index].u16OldCnt = u16Value;
	EXIT_CRITICAL();
	return u16Value;
	
}

void CodeSwitchFlush(void)
{
	{
		/* 0 for no change, 1 for should read pin, 2 for get even value */
		u8 u8SwitchMode[CODE_SWITCH_MAX] = {0};
		u8 i;
		USE_CRITICAL();
		ENTER_CRITICAL();
		
		
		/* analyze CodeSwitch status */
		for (i = 0; i < CODE_SWITCH_MAX; i++)
		{
			if (s_u8SwitchMode[i] != 0)
			{
				s_u8SwitchMode[i]++;
			}
			if (s_u8SwitchMode[i] == 3)
			{
				u8SwitchMode[i] = 1;
				if (!c_bCodeSwtichLastValueShouldBeEven[i])
				{
					s_u8SwitchMode[i] = 0;
				}
			}
			else if (s_u8SwitchMode[i] == (3 + 25))
			{
				s_u8SwitchMode[i] = 0;
				u8SwitchMode[i] = 2;				
			}
		}
		
		EXIT_CRITICAL();
		
		for (i = 0; i < CODE_SWITCH_MAX; i++)
		{
			if (u8SwitchMode[i] == 1)
			{
				FlushCodeSwitch(i);
			}
			else if (u8SwitchMode[i] == 2)
			{
				ChangeCodeSwitch(i, s_stCodeSwitchState[i].u16Dir);
				s_stCodeSwitchState[i].u16Cnt &= 0xFFFE;
				s_stCodeSwitchState[i].u16OldCnt = s_stCodeSwitchState[i].u16Cnt - 1;
			}
		}
	}
	
	{
		u32 i;
		for (i = 0; i < CODE_SWITCH_MAX; i++)
		{
			if (CodeSwitchGetValueInner(s_stCodeSwitchState + i))
			{
				StKeyMixIn stKey;
				stKey.emKeyType = _Key_CodeSwitch;
				memcpy(&(stKey.unKeyMixIn.stCodeSwitchState), s_stCodeSwitchState + i, 
						sizeof(StCodeSwitchState));
				KeyBufWrite(&stKey);

			}
		}	
	}

}

