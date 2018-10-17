/******************(C) copyright 天津市XXXXX有限公司 *************************
* All Rights Reserved
* 文件名：protocol.c
* 摘要: 协议控制程序
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
	
#include "buzzer.h"
	
#include "user_init.h"
#include "user_api.h"

#include "key_led_table.h"

#include "protocol.h"

#include "message.h"
#include "message_2.h"
#include "message_3.h"
#include "message_usb.h"
#include "flash_ctrl.h"
#include "extern_io_ctrl.h"

#include "common.h"

#include "usb_desc.h"

#define APP_VERSOIN_YY		17
#define APP_VERSOIN_MM		11
#define APP_VERSOIN_DD		07
#define APP_VERSOIN_V		01

#define APP_VERSOIN	"YA_BJZS_KEY_181017"

u8 g_u8CamAddr = 0;
bool g_boUsingUART2 = true;
bool g_boIsBroadcastPlay = false;
bool g_boIsDDRPlay = false;
bool g_boIsRockAus = false;
bool g_boIsRockEnable = true;

EmProtocol g_emProtocol = DEFAULT_MAIN_PROTOCOL;
u8 g_u8MIDIChannel = DEFAULT_MIDI_CHANNEL;

const u16 g_u16CamLoc[CAM_ADDR_MAX] = 
{
	0,
};


int32_t CycleMsgInit(StCycleBuf *pCycleBuf, void *pBuf, uint32_t u32Length)
{
	if ((pCycleBuf == NULL) || (pBuf == NULL) || (u32Length == 0))
	{
		return -1;
	}
	memset(pCycleBuf, 0, sizeof(StCycleBuf));
	pCycleBuf->pBuf = pBuf;
	pCycleBuf->u32TotalLength = u32Length;
	
	return 0;
}

void *CycleGetOneMsg(StCycleBuf *pCycleBuf, const char *pData, 
	uint32_t u32DataLength, uint32_t *pLength, int32_t *pProtocolType, int32_t *pErr)
{
	char *pBuf = NULL;
	int32_t s32Err = 0;
	if ((pCycleBuf == NULL) || (pLength == NULL))
	{
		s32Err = -1;
		goto end;
	}
	if (((pCycleBuf->u32TotalLength - pCycleBuf->u32Using) < u32DataLength)
		/*|| (u32DataLength == 0)*/)
	{
		PRINT_MFC("data too long\n");
		s32Err = -1;
	}
	if (u32DataLength != 0)
	{
		if (pData == NULL)
		{
			s32Err = -1;
			goto end;
		}
		else	/* copy data */
		{
			uint32_t u32Tmp = pCycleBuf->u32Write + u32DataLength;
			if (u32Tmp > pCycleBuf->u32TotalLength)
			{
				uint32_t u32CopyLength = pCycleBuf->u32TotalLength - pCycleBuf->u32Write;
				memcpy(pCycleBuf->pBuf + pCycleBuf->u32Write, pData, u32CopyLength);
				memcpy(pCycleBuf->pBuf, pData + u32CopyLength, u32DataLength - u32CopyLength);
				pCycleBuf->u32Write = u32DataLength - u32CopyLength;
			}
			else
			{
				memcpy(pCycleBuf->pBuf + pCycleBuf->u32Write, pData, u32DataLength);
				pCycleBuf->u32Write += u32DataLength;
			}
			pCycleBuf->u32Using += u32DataLength;

		}
	}

	do
	{
		uint32_t i;
		bool boIsBreak = false;

		for (i = 0; i < pCycleBuf->u32Using; i++)
		{
			uint32_t u32ReadIndex = i + pCycleBuf->u32Read;
			char c8FirstByte;
			u32ReadIndex %= pCycleBuf->u32TotalLength;
			c8FirstByte = pCycleBuf->pBuf[u32ReadIndex];
			if (c8FirstByte == ((char)0xAA))
			{
				#define YNA_NORMAL_CMD		0
				#define YNA_VARIABLE_CMD	1 /*big than PROTOCOL_YNA_DECODE_LENGTH */
				uint32_t u32MSB = 0;
				uint32_t u32LSB = 0;
				int32_t s32RemainLength = pCycleBuf->u32Using - i;
				
				/* check whether it's a variable length command */
				if (s32RemainLength >= PROTOCOL_YNA_DECODE_LENGTH - 1)
				{
					if (pCycleBuf->u32Flag != YNA_NORMAL_CMD)
					{
						u32MSB = ((pCycleBuf->u32Flag >> 8) & 0xFF);
						u32LSB = ((pCycleBuf->u32Flag >> 0) & 0xFF);
					}
					else
					{
						uint32_t u32Start = i + pCycleBuf->u32Read;
						char *pTmp = pCycleBuf->pBuf;
						if ((pTmp[(u32Start + _YNA_Mix) % pCycleBuf->u32TotalLength] == 0x04)
							&& (pTmp[(u32Start + _YNA_Cmd) % pCycleBuf->u32TotalLength] == 0x00))
						{
							u32MSB = pTmp[(u32Start + _YNA_Data2) % pCycleBuf->u32TotalLength];
							u32LSB = pTmp[(u32Start + _YNA_Data3) % pCycleBuf->u32TotalLength];
							if (s32RemainLength >= PROTOCOL_YNA_DECODE_LENGTH)
							{
								uint32_t u32Start = i + pCycleBuf->u32Read;
								uint32_t u32End = PROTOCOL_YNA_DECODE_LENGTH - 1 + i + pCycleBuf->u32Read;
								char c8CheckSum = 0;
								uint32_t j;
								char c8Tmp;
								for (j = u32Start; j < u32End; j++)
								{
									c8CheckSum ^= pCycleBuf->pBuf[j % pCycleBuf->u32TotalLength];
								}
								c8Tmp = pCycleBuf->pBuf[u32End % pCycleBuf->u32TotalLength];
								if (c8CheckSum != c8Tmp) /* wrong message */
								{
									PRINT_MFC("get a wrong command: %d\n", u32MSB);
									pCycleBuf->u32Using = (pCycleBuf->u32Using - (i + 1));
									pCycleBuf->u32Read += (i + 1);
									pCycleBuf->u32Read %= pCycleBuf->u32TotalLength;
									break;
								}
								
								u32MSB &= 0xFF;
								u32LSB &= 0xFF;
								
								pCycleBuf->u32Flag = ((u32MSB << 8) + u32LSB);
							}
						}
					}
				}
				u32MSB &= 0xFF;
				u32LSB &= 0xFF;
				u32MSB = (u32MSB << 8) + u32LSB;
				u32MSB += PROTOCOL_YNA_DECODE_LENGTH;
				PRINT_MFC("the data length is: %d\n", u32MSB);
				if (u32MSB > (pCycleBuf->u32TotalLength / 2))	/* maybe the message is wrong */
				{
					pCycleBuf->u32Using = (pCycleBuf->u32Using - (i + 1));
					pCycleBuf->u32Read += (i + 1);
					pCycleBuf->u32Read %= pCycleBuf->u32TotalLength;
					pCycleBuf->u32Flag = 0;
				}
				else if (((int32_t)(u32MSB)) <= s32RemainLength) /* good, I may got a message */
				{
					if (u32MSB == PROTOCOL_YNA_DECODE_LENGTH)
					{
						char c8CheckSum = 0, *pBufTmp, c8Tmp;
						uint32_t j, u32Start, u32End;
						uint32_t u32CmdLength = u32MSB;
						pBuf = (char *)malloc(u32CmdLength);
						if (pBuf == NULL)
						{
							s32Err = -1; /* big problem */
							goto end;
						}
						pBufTmp = pBuf;
						u32Start = i + pCycleBuf->u32Read;

						u32End = u32MSB - 1 + i + pCycleBuf->u32Read;
						PRINT_MFC("start: %d, end: %d\n", u32Start, u32End);
						for (j = u32Start; j < u32End; j++)
						{
							c8Tmp = pCycleBuf->pBuf[j % pCycleBuf->u32TotalLength];
							c8CheckSum ^= c8Tmp;
							*pBufTmp++ = c8Tmp;
						}
						c8Tmp = pCycleBuf->pBuf[u32End % pCycleBuf->u32TotalLength];
						if (c8CheckSum == c8Tmp) /* good message */
						{
							boIsBreak = true;
							*pBufTmp = c8Tmp;

							pCycleBuf->u32Using = (pCycleBuf->u32Using - (i + u32CmdLength));
							pCycleBuf->u32Read = i + pCycleBuf->u32Read + u32CmdLength;
							pCycleBuf->u32Read %= pCycleBuf->u32TotalLength;
							PRINT_MFC("get a command: %d\n", u32MSB);
							PRINT_MFC("u32Using: %d, u32Read: %d, u32Write: %d\n", pCycleBuf->u32Using, pCycleBuf->u32Read, pCycleBuf->u32Write);
							*pLength = u32CmdLength;
							if (pProtocolType != NULL)
							{
								*pProtocolType = _Protocol_YNA;
							}
						}
						else
						{
							free(pBuf);
							pBuf = NULL;
							PRINT_MFC("get a wrong command: %d\n", u32MSB);
							pCycleBuf->u32Using = (pCycleBuf->u32Using - (i + 1));
							pCycleBuf->u32Read += (i + 1);
							pCycleBuf->u32Read %= pCycleBuf->u32TotalLength;
							pCycleBuf->u32Flag = 0;
						}
					}
					else /* variable length */
					{
						uint32_t u32Start, u32End;
						uint32_t u32CmdLength = u32MSB;
						uint16_t u16CRCModBus;
						uint16_t u16CRCBuf;
						pBuf = (char *)malloc(u32CmdLength);
						if (pBuf == NULL)
						{
							s32Err = -1; /* big problem */
							goto end;
						}
						u32Start = (i + pCycleBuf->u32Read) % pCycleBuf->u32TotalLength;
						u32End = (u32MSB + i + pCycleBuf->u32Read) % pCycleBuf->u32TotalLength;
						PRINT_MFC("start: %d, end: %d\n", u32Start, u32End);
						if (u32End > u32Start)
						{
							memcpy(pBuf, pCycleBuf->pBuf + u32Start, u32MSB);
						}
						else
						{
							uint32_t u32FirstCopy = pCycleBuf->u32TotalLength - u32Start;
							memcpy(pBuf, pCycleBuf->pBuf + u32Start, u32FirstCopy);
							memcpy(pBuf + u32FirstCopy, pCycleBuf->pBuf, u32MSB - u32FirstCopy);
						}

						pCycleBuf->u32Flag = YNA_NORMAL_CMD;
						
						/* we need not check the head's check sum,
						 * just check the CRC16-MODBUS
						 */
						u16CRCModBus = CRC16((const uint8_t *)pBuf + PROTOCOL_YNA_DECODE_LENGTH, 
							u32MSB - PROTOCOL_YNA_DECODE_LENGTH - 2);
						u16CRCBuf = 0;

						LittleAndBigEndianTransfer((char *)(&u16CRCBuf), pBuf + u32MSB - 2, 2);
						if (u16CRCBuf == u16CRCModBus) /* good message */
						{
							boIsBreak = true;

							pCycleBuf->u32Using = (pCycleBuf->u32Using - (i + u32CmdLength));
							pCycleBuf->u32Read = i + pCycleBuf->u32Read + u32CmdLength;
							pCycleBuf->u32Read %= pCycleBuf->u32TotalLength;
							PRINT_MFC("get a command: %d\n", u32MSB);
							PRINT_MFC("u32Using: %d, u32Read: %d, u32Write: %d\n", pCycleBuf->u32Using, pCycleBuf->u32Read, pCycleBuf->u32Write);
							*pLength = u32CmdLength;
							if (pProtocolType != NULL)
							{
								*pProtocolType = _Protocol_YNA;
							}
						}
						else
						{
							free(pBuf);
							pBuf = NULL;
							PRINT_MFC("get a wrong command: %d\n", u32MSB);
							pCycleBuf->u32Using = (pCycleBuf->u32Using - (i + 1));
							pCycleBuf->u32Read += (i + 1);
							pCycleBuf->u32Read %= pCycleBuf->u32TotalLength;
						}
					}
				}
				else	/* message not enough long */
				{
					pCycleBuf->u32Using = (pCycleBuf->u32Using - i);
					pCycleBuf->u32Read += i;
					pCycleBuf->u32Read %= pCycleBuf->u32TotalLength;
					boIsBreak = true;
				}
				break;
			}
			else if(c8FirstByte == ((char)0xFA))
			{
				int32_t s32RemainLength = pCycleBuf->u32Using - i;
				if (s32RemainLength >= PROTOCOL_RQ_LENGTH)
				{
					volatile char c8CheckSum = 0, *pBufTmp, c8Tmp;
					volatile uint32_t j, u32Start, u32End;
					volatile uint32_t u32CmdLength = PROTOCOL_RQ_LENGTH;
					pBuf = (char *)malloc(u32CmdLength);
					if (pBuf == NULL)
					{
						s32Err = -1; /* big problem */
						goto end;
					}
					pBufTmp = pBuf;
					u32Start = i + pCycleBuf->u32Read;

					u32End = PROTOCOL_RQ_LENGTH - 1 + i + pCycleBuf->u32Read;
					PRINT_MFC("start: %d, end: %d\n", u32Start, u32End);
					for (j = u32Start; j < u32End; j++)
					{
						c8Tmp = pCycleBuf->pBuf[j % pCycleBuf->u32TotalLength];
						*pBufTmp++ = c8Tmp;
					}
					
					for (j = 2; j < PROTOCOL_RQ_LENGTH - 1; j++)
					{
						c8CheckSum += pBuf[j];
					}
					
					
					c8Tmp = pCycleBuf->pBuf[u32End % pCycleBuf->u32TotalLength];
					if (c8CheckSum == c8Tmp) /* good message */
					{
						boIsBreak = true;
						*pBufTmp = c8Tmp;

						pCycleBuf->u32Using = (pCycleBuf->u32Using - (i + u32CmdLength));
						pCycleBuf->u32Read = i + pCycleBuf->u32Read + u32CmdLength;
						pCycleBuf->u32Read %= pCycleBuf->u32TotalLength;
						PRINT_MFC("get a command: %d\n", u32MSB);
						PRINT_MFC("u32Using: %d, u32Read: %d, u32Write: %d\n", pCycleBuf->u32Using, pCycleBuf->u32Read, pCycleBuf->u32Write);
						*pLength = u32CmdLength;
						if (pProtocolType != NULL)
						{
							*pProtocolType = _Protocol_RQ;
						}
					}
					else
					{
						free(pBuf);
						pBuf = NULL;
						PRINT_MFC("get a wrong command: %d\n", u32MSB);
						pCycleBuf->u32Using = (pCycleBuf->u32Using - (i + 1));
						pCycleBuf->u32Read += (i + 1);
						pCycleBuf->u32Read %= pCycleBuf->u32TotalLength;
					}
				}
				else	/* message not enough long */
				{
					pCycleBuf->u32Using = (pCycleBuf->u32Using - i);
					pCycleBuf->u32Read += i;
					pCycleBuf->u32Read %= pCycleBuf->u32TotalLength;
					boIsBreak = true;
				}
				break;
			}
			
			else if((c8FirstByte & 0xF0) == ((char)0x80))
			{
				int32_t s32RemainLength = pCycleBuf->u32Using - i;
				if (s32RemainLength >= PROTOCOL_VISCA_MIN_LENGTH)
				{
					u32 j;
					u32 u32Start = i + pCycleBuf->u32Read;
					u32 u32End = pCycleBuf->u32Using + pCycleBuf->u32Read;
					char c8Tmp = 0;
					for (j = u32Start + PROTOCOL_VISCA_MIN_LENGTH - 1; j < u32End; j++)
					{
						c8Tmp = pCycleBuf->pBuf[j % pCycleBuf->u32TotalLength];
						if (c8Tmp == (char)0xFF)
						{
							u32End = j;
							break;
						}
					}
					if (c8Tmp == (char)0xFF)
					{
						/* wrong message */
						if ((u32End - u32Start + 1) > PROTOCOL_VISCA_MAX_LENGTH)
						{
							pBuf = NULL;
							PRINT_MFC("get a wrong command: %d\n", u32MSB);
							pCycleBuf->u32Using = (pCycleBuf->u32Using - (i + 1));
							pCycleBuf->u32Read += (i + 1);
							pCycleBuf->u32Read %= pCycleBuf->u32TotalLength;							
						}
						else
						{
							char *pBufTmp;
							uint32_t u32CmdLength = u32End - u32Start + 1;
							pBuf = (char *)malloc(u32CmdLength);
							if (pBuf == NULL)
							{
								s32Err = -1; /* big problem */
								goto end;
							}	
							pBufTmp = pBuf;
							boIsBreak = true;
							
							PRINT_MFC("start: %d, end: %d\n", u32Start, u32End);
							for (j = u32Start; j <= u32End; j++)
							{
								*pBufTmp++ = pCycleBuf->pBuf[j % pCycleBuf->u32TotalLength];
							}
							
							pCycleBuf->u32Using = (pCycleBuf->u32Using - (i + u32CmdLength));
							pCycleBuf->u32Read = i + pCycleBuf->u32Read + u32CmdLength;
							pCycleBuf->u32Read %= pCycleBuf->u32TotalLength;
							PRINT_MFC("get a command: %d\n", u32MSB);
							PRINT_MFC("u32Using: %d, u32Read: %d, u32Write: %d\n", pCycleBuf->u32Using, pCycleBuf->u32Read, pCycleBuf->u32Write);
							*pLength = u32CmdLength;
							if (pProtocolType != NULL)
							{
								*pProtocolType = _Protocol_VISCA;
							}

						}
					}
					else
					{
						/* wrong message */
						if ((u32End - u32Start) >= PROTOCOL_VISCA_MAX_LENGTH)
						{
							pBuf = NULL;
							PRINT_MFC("get a wrong command: %d\n", u32MSB);
							pCycleBuf->u32Using = (pCycleBuf->u32Using - (i + 1));
							pCycleBuf->u32Read += (i + 1);
							pCycleBuf->u32Read %= pCycleBuf->u32TotalLength;							
						}
						else
						{
							pCycleBuf->u32Using = (pCycleBuf->u32Using - i);
							pCycleBuf->u32Read += i;
							pCycleBuf->u32Read %= pCycleBuf->u32TotalLength;
							boIsBreak = true;						
						}
					}
				}
				else	/* message not enough long */
				{
					pCycleBuf->u32Using = (pCycleBuf->u32Using - i);
					pCycleBuf->u32Read += i;
					pCycleBuf->u32Read %= pCycleBuf->u32TotalLength;
					boIsBreak = true;
				}
				break;
			}
			else if(c8FirstByte == ((char)0xA5))
			{
				int32_t s32RemainLength = pCycleBuf->u32Using - i;
				if (s32RemainLength >= PROTOCOL_SB_LENGTH)
				{
					volatile char c8CheckSum = 0, *pBufTmp, c8Tmp;
					volatile uint32_t j, u32Start, u32End;
					volatile uint32_t u32CmdLength = PROTOCOL_SB_LENGTH;
					pBuf = (char *)malloc(u32CmdLength);
					if (pBuf == NULL)
					{
						s32Err = -1; /* big problem */
						goto end;
					}
					pBufTmp = pBuf;
					u32Start = i + pCycleBuf->u32Read;

					u32End = PROTOCOL_SB_LENGTH - 1 + i + pCycleBuf->u32Read;
					PRINT_MFC("start: %d, end: %d\n", u32Start, u32End);
					for (j = u32Start; j < u32End; j++)
					{
						c8Tmp = pCycleBuf->pBuf[j % pCycleBuf->u32TotalLength];
						*pBufTmp++ = c8Tmp;
					}
					
					for (j = 1; j < PROTOCOL_SB_LENGTH - 1; j++)
					{
						c8CheckSum += pBuf[j];
					}
					
					
					c8Tmp = pCycleBuf->pBuf[u32End % pCycleBuf->u32TotalLength];
					if (c8CheckSum == c8Tmp) /* good message */
					{
						boIsBreak = true;
						*pBufTmp = c8Tmp;

						pCycleBuf->u32Using = (pCycleBuf->u32Using - (i + u32CmdLength));
						pCycleBuf->u32Read = i + pCycleBuf->u32Read + u32CmdLength;
						pCycleBuf->u32Read %= pCycleBuf->u32TotalLength;
						PRINT_MFC("get a command: %d\n", u32MSB);
						PRINT_MFC("u32Using: %d, u32Read: %d, u32Write: %d\n", pCycleBuf->u32Using, pCycleBuf->u32Read, pCycleBuf->u32Write);
						*pLength = u32CmdLength;
						if (pProtocolType != NULL)
						{
							*pProtocolType = _Protocol_SB;
						}
					}
					else
					{
						free(pBuf);
						pBuf = NULL;
						PRINT_MFC("get a wrong command: %d\n", u32MSB);
						pCycleBuf->u32Using = (pCycleBuf->u32Using - (i + 1));
						pCycleBuf->u32Read += (i + 1);
						pCycleBuf->u32Read %= pCycleBuf->u32TotalLength;
					}
				}
				else	/* message not enough long */
				{
					pCycleBuf->u32Using = (pCycleBuf->u32Using - i);
					pCycleBuf->u32Read += i;
					pCycleBuf->u32Read %= pCycleBuf->u32TotalLength;
					boIsBreak = true;
				}
				break;
			}
		}
		if ((i == pCycleBuf->u32Using) && (!boIsBreak))
		{
			PRINT_MFC("cannot find AA, i = %d\n", pCycleBuf->u32Using);
			pCycleBuf->u32Using = 0;
			pCycleBuf->u32Read += i;
			pCycleBuf->u32Read %= pCycleBuf->u32TotalLength;
			pCycleBuf->u32Flag = 0;
		}

		if (boIsBreak)
		{
			break;
		}
	} while (((int32_t)pCycleBuf->u32Using) > 0);

	//if (pCycleBuf->u32Write + u32DataLength)

end:
	if (pErr != NULL)
	{
		*pErr = s32Err;
	}
	return pBuf;
}
void *YNAMakeAnArrayVarialbleCmd(uint16_t u16Cmd, void *pData,
	uint32_t u32Count, uint32_t u32Length, uint32_t *pCmdLength)
{
	uint32_t u32CmdLength;
	uint32_t u32DataLength;
	uint32_t u32Tmp;
	uint8_t *pCmd = NULL;
	uint8_t *pVarialbleCmd;
	if (pData == NULL)
	{
		return NULL;
	}
	
	u32DataLength = u32Count * u32Length;
	
	/*  */
	u32CmdLength = PROTOCOL_YNA_DECODE_LENGTH + 6 + u32DataLength + 2;
	pCmd = malloc(u32CmdLength);
	if (pCmd == NULL)
	{
		return NULL;
	}
	memset(pCmd, 0, u32CmdLength);
	pCmd[_YNA_Sync] = 0xAA;
	pCmd[_YNA_Mix] = 0x04;
	pCmd[_YNA_Cmd] = 0x00;
	
	/* total length */
	u32Tmp = u32CmdLength - PROTOCOL_YNA_DECODE_LENGTH;
	LittleAndBigEndianTransfer((char *)pCmd + _YNA_Data2, (const char *)(&u32Tmp), 2);
	
	YNAGetCheckSum(pCmd);
	
	pVarialbleCmd = pCmd + PROTOCOL_YNA_DECODE_LENGTH;
	
	/* command serial */
	LittleAndBigEndianTransfer((char *)pVarialbleCmd, (const char *)(&u16Cmd), 2);

	/* command count */
	LittleAndBigEndianTransfer((char *)pVarialbleCmd + 2, (const char *)(&u32Count), 2);

	/* Varialble data length */
	LittleAndBigEndianTransfer((char *)pVarialbleCmd + 4, (const char *)(&u32Length), 2);
	
	/* copy the data */
	memcpy(pVarialbleCmd + 6, pData, u32DataLength);

	/* get the CRC16 of the variable command */
	u32Tmp = CRC16(pVarialbleCmd, 6 + u32DataLength);
	
	LittleAndBigEndianTransfer((char *)pVarialbleCmd + 6 + u32DataLength, 
		(const char *)(&u32Tmp), 2);

	if (pCmdLength != NULL)
	{
		*pCmdLength = u32CmdLength;
	}
	
	return pCmd;
}

void *YNAMakeASimpleVarialbleCmd(uint16_t u16Cmd, void *pData, 
	uint32_t u32DataLength, uint32_t *pCmdLength)
{
	return YNAMakeAnArrayVarialbleCmd(u16Cmd, pData, 1, u32DataLength, pCmdLength);
}

void CopyToUart1Message(void *pData, u32 u32Length)
{
	if ((pData != NULL) && (u32Length != 0))
	{
		void *pBuf = malloc(u32Length);
		if (pBuf != NULL)
		{
			memcpy(pBuf, pData, u32Length);
			if (MessageUartWrite(pBuf, true, _IO_Reserved, u32Length) != 0)
			{
				free (pBuf);
			}	
		}
	}

}
void CopyToUart2Message(void *pData, u32 u32Length)
{
	if ((pData != NULL) && (u32Length != 0))
	{
		void *pBuf = malloc(u32Length);
		if (pBuf != NULL)
		{
			memcpy(pBuf, pData, u32Length);
			if (MessageUart2Write(pBuf, true, _IO_Reserved, u32Length) != 0)
			{
				free (pBuf);
			}	
		}
	}
}

void CopyToUart3Message(void *pData, u32 u32Length)
{
	if ((pData != NULL) && (u32Length != 0))
	{
		void *pBuf = malloc(u32Length);
		if (pBuf != NULL)
		{
			memcpy(pBuf, pData, u32Length);
			if (MessageUart3Write(pBuf, true, _IO_Reserved, u32Length) != 0)
			{
				free (pBuf);
			}	
		}
	}

}

void CopyToUartMessage(void *pData, u32 u32Length)
{
	CopyToUart1Message(pData, u32Length);
	CopyToUart2Message(pData, u32Length);
}

void FlushHIDMsgForBJZS(void *pData, u32 u32Length)
{
	if (IsUSBDeviceConnect())
	{
		u8 u8Buf[12] = {REPORT_ID, 0};
		memcpy(u8Buf + 1, pData, u32Length);
		CopyToUSBMessage(u8Buf, REPORT_IN_SIZE_WITH_ID, _IO_USB_ENDP2);
	}
}



u8 u8YNABuf[PROTOCOL_YNA_ENCODE_LENGTH];

int32_t BaseCmdProcess(StIOFIFO *pFIFO, const StIOTCB *pIOTCB)
{
	uint8_t *pMsg;
	bool boGetVaildBaseCmd = true;
	if (pFIFO == NULL)
	{
		return -1;
	}
	pMsg = (uint8_t *)pFIFO->pData;
	
	if (pMsg[_YNA_Sync] != 0xAA)
	{
		return -1;
	}
	
	if (pMsg[_YNA_Mix] == 0x0C)
	{
		if (pMsg[_YNA_Cmd] == 0x80)
		{
			uint8_t u8EchoBase[PROTOCOL_YNA_DECODE_LENGTH] = {0};
			uint8_t *pEcho = NULL;
			uint32_t u32EchoLength = 0;
			bool boHasEcho = true;
			bool boNeedCopy = true;
			bool boNeedReset = false;
			u8EchoBase[_YNA_Sync] = 0xAA;
			u8EchoBase[_YNA_Mix] = 0x0C;
			u8EchoBase[_YNA_Cmd] = 0x80;
			u8EchoBase[_YNA_Data1] = 0x01;
			switch(pMsg[_YNA_Data3])
			{
				case 0x01:	/* just echo the same command */
				{
					//SetOptionByte(OPTION_UPGRADE_DATA);
					//boNeedReset = true;
				}
				case 0x02:	/* just echo the same command */
				{
					u8EchoBase[_YNA_Data3] = pMsg[_YNA_Data3];
					pEcho = (uint8_t *)malloc(PROTOCOL_YNA_DECODE_LENGTH);
					if (pEcho == NULL)
					{
						boHasEcho = false;
						break;
					}
					u32EchoLength = PROTOCOL_YNA_DECODE_LENGTH;						
					break;
				}
				case 0x03:	/* return the UUID */
				{
					StUID stUID;
					GetUID(&stUID);
					boNeedCopy = false;
					pEcho = YNAMakeASimpleVarialbleCmd(0x8003, 
							&stUID, sizeof(StUID), &u32EchoLength);
					break;
				}
				case 0x05:	/* return the BufLength */
				{
					uint16_t u16BufLength = 0;
					if (pIOTCB != NULL
						 && pIOTCB->pFunGetMsgBufLength != 0)
					{
						u16BufLength = pIOTCB->pFunGetMsgBufLength();
					}
					boNeedCopy = false;
					pEcho = YNAMakeASimpleVarialbleCmd(0x8005, 
							&u16BufLength, sizeof(uint16_t), &u32EchoLength);
					break;
				}
				case 0x09:	/* RESET the MCU */
				{
					NVIC_SystemReset();
					boHasEcho = false;
					break;
				}
				case 0x0B:	/* Get the application's CRC32 */
				{
					uint32_t u32CRC32 = ~0;
					u32CRC32 = AppCRC32(~0);
					boNeedCopy = false;
					pEcho = YNAMakeASimpleVarialbleCmd(0x800B, 
							&u32CRC32, sizeof(uint32_t), &u32EchoLength);
					break;
				}
				case 0x0C:	/* get the version */
				{
					const char *pVersion = APP_VERSOIN;
					boNeedCopy = false;
					pEcho = YNAMakeASimpleVarialbleCmd(0x800C, 
							(void *)pVersion, strlen(pVersion) + 1, &u32EchoLength);
					break;
				}
				
				default:
					boHasEcho = false;
					boGetVaildBaseCmd = false;
					break;
			}
			if (boHasEcho && pEcho != NULL)
			{
				if (boNeedCopy)
				{
					YNAGetCheckSum(u8EchoBase);
					memcpy(pEcho, u8EchoBase, PROTOCOL_YNA_DECODE_LENGTH);
				}
				if (pIOTCB == NULL)
				{
					free(pEcho);
				}
				else if (pIOTCB->pFunMsgWrite == NULL)
				{
					free(pEcho);			
				}
				else if(pIOTCB->pFunMsgWrite(pEcho, true, _IO_Reserved, u32EchoLength) != 0)
				{
					free(pEcho);
				}
			}
			
			/* send all the command in the buffer */
			if (boNeedReset)
			{
				//MessageUartFlush(true); 
				NVIC_SystemReset();
			}
		}
	}
	else if (pMsg[_YNA_Mix] == 0x04)	/* variable command */
	{
		uint32_t u32TotalLength = 0;
		uint32_t u32ReadLength = 0;
		uint8_t *pVariableCmd;
		
		boGetVaildBaseCmd = false;
		
		/* get the total command length */
		LittleAndBigEndianTransfer((char *)(&u32TotalLength), (const char *)pMsg + _YNA_Data2, 2);
		pVariableCmd = pMsg + PROTOCOL_YNA_DECODE_LENGTH;
		u32TotalLength -= 2; /* CRC16 */
		while (u32ReadLength < u32TotalLength)
		{
			uint8_t *pEcho = NULL;
			uint32_t u32EchoLength = 0;
			bool boHasEcho = true;
			uint16_t u16Command = 0, u16Count = 0, u16Length = 0;
			LittleAndBigEndianTransfer((char *)(&u16Command),
				(char *)pVariableCmd, 2);
			LittleAndBigEndianTransfer((char *)(&u16Count),
				(char *)pVariableCmd + 2, 2);
			LittleAndBigEndianTransfer((char *)(&u16Length),
				(char *)pVariableCmd + 4, 2);

			switch (u16Command)
			{
				case 0x800A:
				{
					/* check the crc32 and UUID, and BTEA and check the number */
					int32_t s32Err;
					char *pData = (char *)pVariableCmd + 6;
					StBteaKey stLic;
					StUID stUID;
					uint32_t u32CRC32;
					
					GetUID(&stUID);
					u32CRC32 = AppCRC32(~0);
					GetLic(&stLic, &stUID, u32CRC32, true);
					
					if (memcmp(&stLic, pData, sizeof(StBteaKey)) == 0)
					{
						s32Err = 0;
						
						WriteLic(&stLic, true, 0);
					}
					else
					{
						s32Err = -1;
					}
					pEcho = YNAMakeASimpleVarialbleCmd(0x800A, &s32Err, 4, &u32EchoLength);
					boGetVaildBaseCmd = true;
					break;
				}
				default:
					break;
			}
			
			if (boHasEcho && pEcho != NULL)
			{
				if (pIOTCB == NULL)
				{
					free(pEcho);
				}
				else if (pIOTCB->pFunMsgWrite == NULL)
				{
					free(pEcho);
				}
				else if(pIOTCB->pFunMsgWrite(pEcho, true, _IO_Reserved, u32EchoLength) != 0)
				{
					free(pEcho);
				}
			}
			
			
			u32ReadLength += (6 + (uint32_t)u16Count * u16Length);
			pVariableCmd = pMsg + PROTOCOL_YNA_DECODE_LENGTH + u32ReadLength;
		}
	}
	else
	{
		boGetVaildBaseCmd = false;
	}

	return boGetVaildBaseCmd ? 0: -1;
}


void GlobalStateInit(void)
{
	g_u8CamAddr = 0;
	
	g_boUsingUART2 = true;
	g_boIsPushRodNeedReset = false;
	g_boIsBroadcastPlay = false;
	g_boIsDDRPlay = false;
	g_boIsRockEnable = true;	
}


void ChangeEncodeState(void)
{

}

void YNADecode(u8 *pBuf)
{
	if (g_u32BoolIsEncode)
	{
			
	}
	else
	{
		
	}

}
void YNAEncodeAndGetCheckSum(u8 *pBuf)
{
	if (g_u32BoolIsEncode)
	{
			
	}
	else
	{
		
	}
}


void YNAGetCheckSum(u8 *pBuf)
{
	s32 i, s32End;
	u8 u8Sum = pBuf[0];

	if (g_u32BoolIsEncode)
	{
		s32End = PROTOCOL_YNA_ENCODE_LENGTH - 1;	
	}
	else
	{
		s32End = PROTOCOL_YNA_DECODE_LENGTH - 1;
	}
	for (i = 1; i < s32End; i++)
	{
		u8Sum ^= pBuf[i];
	}
	pBuf[i] = u8Sum;
}

void PelcoDGetCheckSum(u8 *pBuf)
{
	s32 i;
	u8 u8Sum = 0;
	for (i = 1; i < 6; i++)
	{
		u8Sum += pBuf[i];
	}
	pBuf[i] = u8Sum;
}

void SBGetCheckSum(u8 *pBuf)
{
	s32 i;
	u8 u8Sum = 0;
	for (i = 1; i < PROTOCOL_SB_LENGTH - 1; i++)
	{
		u8Sum += pBuf[i];
	}
	pBuf[i] = u8Sum;
}

#define PS_MAX_SET	16
enum
{
	_PS_Normal = 0,
	_PS_MIDI_Channle,
	
	_PS_Reserved
};

enum
{
	_PS_Begin = 0x10,
	_PS_Normal_Begin = 0x10,
	_PS_Normal_Protocol_YNA = _PS_Normal_Begin,
	_PS_Normal_Protocol_BJZS,
	
	_PS_Normal_Reserved,
	_PS_Normal_Max = _PS_Normal_Begin + PS_MAX_SET - 1,
	
	_PS_MIDI_Begin = _PS_Normal_Max + 1,
	_PS_MIDI_Channle_0 = _PS_MIDI_Begin,
	_PS_MIDI_Channle_1,
	_PS_MIDI_Channle_2,
	_PS_MIDI_Channle_3,
	_PS_MIDI_Channle_4,
	_PS_MIDI_Channle_5,
	_PS_MIDI_Channle_6,
	_PS_MIDI_Channle_7,
	_PS_MIDI_Channle_8,
	_PS_MIDI_Channle_9,
	_PS_MIDI_Channle_10,
	_PS_MIDI_Channle_11,
	_PS_MIDI_Channle_12,
	_PS_MIDI_Channle_14,
	_PS_MIDI_Channle_15,
	_PS_MIDI_Reserved,
	_PS_MIDI_Max = _PS_MIDI_Begin + PS_MAX_SET - 1,

};

#if _PS_Normal_Reserved > _PS_Normal_Max
 #error "_PS_Normal_Reserved should not equal or big than _PS_Normal_Max"
#endif

#if _PS_MIDI_Reserved > _PS_MIDI_Max
 #error "_PS_MIDI_Reserved should not equal or big than _PS_MIDI_Max"
#endif

bool ProtocolSelect(StIOFIFO *pFIFO)
{
	const u8 u8KeyMapTrigger[_PS_Reserved] = 
	{
		_Key_PVW_1, _Key_PGM_1,
	};
	const u16 u16LedMapTrigger[_PS_Reserved] = 
	{
		_Led_PVW_1, _Led_PGM_1,
	};

	const u8 u8KeyMap[_PS_Reserved][PS_MAX_SET] = 
	{
		{
			_Key_PGM_1, _Key_PGM_2,
		},
		{
			_Key_PVW_1, _Key_PVW_2, _Key_PVW_3, _Key_PVW_4, 
			_Key_PVW_5, _Key_PVW_6, _Key_PVW_7, _Key_PVW_8, 
			_Key_PVW_9, _Key_PVW_10, _Key_PVW_11, _Key_PVW_12, 
			_Key_Overlay_1, _Key_Overlay_2, _Key_Overlay_3, _Key_Overlay_4,
		
		},
	};
	const u16 u16LedMap[_PS_Reserved][PS_MAX_SET] = 
	{
		{ 
			_Led_PGM_1, _Led_PGM_2,	
		},
		
		{
			_Led_PVW_1, _Led_PVW_2, _Led_PVW_3, _Led_PVW_4, 
			_Led_PVW_5, _Led_PVW_6, _Led_PVW_7, _Led_PVW_8, 
			_Led_PVW_9, _Led_PVW_10, _Led_PVW_11, _Led_PVW_12, 
			_Led_Overlay_1, _Led_Overlay_2, _Led_Overlay_3, _Led_Overlay_4,
		},
	};
	
	u32 u32MsgSentTime;
	u32 u32State = 0;
	u32 i, u32Mode;
	StKeyMixIn *pKeyIn;
	StKeyState *pKey;
	
	if (pFIFO == NULL)
	{
		return false;
	}
		
	pKeyIn = pFIFO->pData;
	if (pKeyIn == NULL)
	{
		return false;
	}

	if (pKeyIn->emKeyType != _Key_Board)
	{
		return false;
	}
	
	pKey = &(pKeyIn->unKeyMixIn.stKeyState[0]);
	
	u32Mode = ~0;
	for (i = 0; i < _PS_Reserved; i++)
	{
		if (pKey->u8KeyValue == u8KeyMapTrigger[i])
		{
			u32Mode = i;
			break;			
		}		
	}
	
	if (u32Mode == ~0)
	{
		return false;
	}


	ChangeAllLedState(false);
	
	u32MsgSentTime = g_u32SysTickCnt;
	while(1)
	{
		if (pKeyIn != NULL)
		{
			pKey = &(pKeyIn->unKeyMixIn.stKeyState[0]);
			if (u32State == 0)
			{
				if (pKey->u8KeyValue == u8KeyMapTrigger[u32Mode])
				{
					u32MsgSentTime = g_u32SysTickCnt;
					if (pKey->u8KeyState == KEY_DOWN)
					{
						ChangeAllLedState(false);
						ChangeLedState(GET_XY(u16LedMapTrigger[u32Mode]), true);
					}
					else if (pKey->u8KeyState == KEY_UP)
					{
						//ChangeLedState(LED(_Led_Switch_Video_Bak5), false);
						u32State = 1;
					}
				}			
			}
			else if (u32State == 1) /* get protocol or bandrate */
			{
				u32 u32Index = ~0;
				for (i = 0; i < PS_MAX_SET; i++)
				{
					if (pKey->u8KeyValue == u8KeyMap[u32Mode][i])
					{
						u32Index = i;
						break;
					}						
				}
				if (u32Index != ~0)
				{
					u32MsgSentTime = g_u32SysTickCnt;
					if (pKey->u8KeyState == KEY_DOWN)
					{
						ChangeLedState(GET_XY(u16LedMap[u32Mode][u32Index]), true);
					}
					else if (pKey->u8KeyState == KEY_UP)
					{
						ChangeLedState(GET_XY(u16LedMap[u32Mode][u32Index]), false);
						u32State = (u32Mode + 1) * PS_MAX_SET + u32Index;
						break;
					}
				}
			}
		}
		
		KeyBufGetEnd(pFIFO);
		
		pFIFO = NULL;
		
		if (SysTimeDiff(u32MsgSentTime, g_u32SysTickCnt) > 5000) /* 10S */
		{
			ChangeAllLedState(false);
			return true;
		}

		
		pKeyIn = NULL;
		pFIFO = KeyBufGetBuf();
		if (pFIFO == NULL)
		{
			continue;
		}
		
		pKeyIn = pFIFO->pData;
		if (pKeyIn == NULL)
		{
			KeyBufGetEnd(pFIFO);

			pFIFO = NULL;

			continue;
		}

		if (pKeyIn->emKeyType != _Key_Board)
		{
			KeyBufGetEnd(pFIFO);
			
			pKeyIn = NULL;
			pFIFO = NULL;
			continue;
		}	
	}
	
	
	KeyBufGetEnd(pFIFO);
	
	
	if (u32State >= 2)
	{
		switch (u32State)
		{
			case _PS_Normal_Protocol_YNA:
			{
				g_emProtocol = _Protocol_YNA;
				break;
			}
			case _PS_Normal_Protocol_BJZS:
			{
				g_emProtocol = _Protocol_BJZS;
				break;
			}
			case _PS_MIDI_Channle_0:
			case _PS_MIDI_Channle_1:
			case _PS_MIDI_Channle_2:
			case _PS_MIDI_Channle_3:
			case _PS_MIDI_Channle_4:
			case _PS_MIDI_Channle_5:
			case _PS_MIDI_Channle_6:
			case _PS_MIDI_Channle_7:
			case _PS_MIDI_Channle_8:
			case _PS_MIDI_Channle_9:
			case _PS_MIDI_Channle_10:
			case _PS_MIDI_Channle_11:
			case _PS_MIDI_Channle_12:
			case _PS_MIDI_Channle_14:
			case _PS_MIDI_Channle_15:
			{
				g_u8MIDIChannel = u32State - _PS_MIDI_Channle_0;
				break;
			}
			
			default:
				break;
		}
		
		
		if (WriteSaveData())
		{
			u32MsgSentTime = g_u32SysTickCnt;
			ChangeAllLedState(true);
			while(SysTimeDiff(u32MsgSentTime, g_u32SysTickCnt) < 1500);/* 延时1s */
			ChangeAllLedState(false);
			return true;
		}

		{
			bool boBlink = true;
			u32 u32BlinkCnt = 0;
			while (u32BlinkCnt < 10)
			{
				boBlink = !boBlink;
				ChangeLedState(GET_XY(u16LedMapTrigger[u32Mode]), boBlink);
				u32MsgSentTime = g_u32SysTickCnt;
				while(SysTimeDiff(u32MsgSentTime, g_u32SysTickCnt) < 100);/* 延时1s */
				u32BlinkCnt++;
			}
		}	
	}
	
	ChangeAllLedState(false);
	return false;
}

static void ChangeLedArrayState(const u16 *pLed, u16 u16Cnt, bool boIsLight)
{
	u16 i;
	for (i = 0; i < u16Cnt; i++)
	{
		ChangeLedState(GET_XY(pLed[i]), boIsLight);
	}
}

void TallyUartSend(u8 Tally1, u8 Tally2)
{
	u32 i, j;
	u8 u8Buf[4] = {0xFC, 0x00, 0x00, 0x00};

	for (j = 0; j < 2; j++)
	{	
		for(i = 0; i < 4; i++)
		{
			if (((Tally1 >> (4 * j + i)) & 0x01) != 0x00)
			{
				u8Buf[1 + j] |= (1 << (i * 2));
			}
			
			if (((Tally2 >> (4 * j + i)) & 0x01) != 0x00)
			{
				u8Buf[1 + j] |= (1 << (i * 2 + 1));			
			}			
		}	
	}
	
	u8Buf[3] = u8Buf[0] ^ u8Buf[1] ^ u8Buf[2];
	
	CopyToUart3Message(u8Buf, 4);
}
u8 g_u8YNATally[2] = {0, 0};

void SetTallyPGM(u8 u8Index, bool boIsLight, bool boIsClear, bool boIsSend)
{
	if (u8Index > 7)
	{
		return;
	}
	if (boIsClear)
	{
		g_u8YNATally[0] = 0;
	}
	if (boIsLight)
	{
		g_u8YNATally[0] |= (1 << u8Index);
	}
	else 
	{
		g_u8YNATally[0] &= ~(1 << u8Index);					
	}
	if (boIsSend)
	{
		TallyUartSend(g_u8YNATally[0], g_u8YNATally[1]);					
	}
	
}

void SetTallyPVW(u8 u8Index, bool boIsLight, bool boIsClear, bool boIsSend)
{
	if (u8Index > 7)
	{
		return;
	}
	if (boIsClear)
	{
		g_u8YNATally[1] = 0;
	}
	if (boIsLight)
	{
		g_u8YNATally[1] |= (1 << u8Index);
	}
	else 
	{
		g_u8YNATally[1] &= ~(1 << u8Index);					
	}
	if (boIsSend)
	{
		TallyUartSend(g_u8YNATally[0], g_u8YNATally[1]);					
	}
	
}


static bool KeyBoardProcess(StKeyMixIn *pKeyIn)
{
	u32 i;
	for (i = 0; i < pKeyIn->u32Cnt; i++)
	{
		u8 *pBuf;
		StKeyState *pKeyState = pKeyIn->unKeyMixIn.stKeyState + i;
		u8 u8KeyValue;
		if (pKeyState->u8KeyState == KEY_KEEP)
		{
			continue;
		}

		u8KeyValue = pKeyState->u8KeyValue;

		pBuf = u8YNABuf;

		memset(pBuf, 0, PROTOCOL_YNA_ENCODE_LENGTH);

		pBuf[_YNA_Sync] = 0xAA;
		pBuf[_YNA_Addr] = g_u8CamAddr;
		pBuf[_YNA_Mix] = 0x07;
		if (pKeyState->u8KeyState == KEY_UP)
		{
			pBuf[_YNA_Data1] = 0x01;
		}

		/* 处理按键 */
		switch (u8KeyValue)
		{			
			case _Key_Make_Record:
			{
				pBuf[_YNA_Cmd] = 0x47;
				pBuf[_YNA_Data2] = 0x02;			
				break;
			}
			case _Key_Make_Live:
			{
				pBuf[_YNA_Cmd] = 0x47;
				pBuf[_YNA_Data2] = 0x00;			
				break;
			}
			case _Key_Make_FullScreen:
			case _Key_Make_MutliRecord:
			{
				pBuf[_YNA_Cmd] = 0x47;
				pBuf[_YNA_Data2] = u8KeyValue - _Key_Make_FullScreen + 0x40;			
				break;

			}
			
			
			case _Key_Ctrl_Machine_Switch1:
			case _Key_Ctrl_Machine_Switch2:
			case _Key_Ctrl_Machine_Switch3:
			case _Key_Ctrl_Machine_Switch4:
			{
				pBuf[_YNA_Cmd] = 0x40;
				pBuf[_YNA_Data1] |= 0x10;
				pBuf[_YNA_Data2] = u8KeyValue - _Key_Ctrl_Machine_Switch1;			
				break;
			}
			case _Key_Ctrl_Machine_Up:
			case _Key_Ctrl_Machine_Down:
			{
				pBuf[_YNA_Cmd] = 0x4C;
				pBuf[_YNA_Data2] = u8KeyValue - _Key_Ctrl_Machine_Up + 0x60;			
				break;
			}

			case _Key_SlowMove_Machine_Choose1:
			case _Key_SlowMove_Machine_Choose2:
			case _Key_SlowMove_Machine_Choose3:
			case _Key_SlowMove_Machine_Choose4:
			{
				pBuf[_YNA_Cmd] = 0x40;
				pBuf[_YNA_Data1] |= 0x20;
				pBuf[_YNA_Data2] = u8KeyValue - _Key_SlowMove_Machine_Choose1;			
				break;
			}
			case _Key_SlowMove_Record_Start:
			case _Key_SlowMove_Record_Stop:
			case _Key_SlowMove_Make_Start:
			case _Key_SlowMove_Make_Stop:
			case _Key_SlowMove_Replay_Start:
			case _Key_SlowMove_Replay_Stop:
			{
				u8 u8Tmp = u8KeyValue - _Key_SlowMove_Record_Start;
				pBuf[_YNA_Cmd] = 0x47;
				pBuf[_YNA_Data1] |= ((u8Tmp >> 1) << 4);
				pBuf[_YNA_Data2] = (u8Tmp & 0x01) + 0x05;			
				break;
				
			}

			case _Key_AI_Present1:
			case _Key_AI_Present2:
			case _Key_AI_Present3:
			case _Key_AI_Present4:
			{
				pBuf[_YNA_Cmd] = 0x40;
				pBuf[_YNA_Data1] |= 0x30;
				pBuf[_YNA_Data2] = u8KeyValue - _Key_AI_Present1;			
				break;
			}

			case _Key_AI_Ctrl_BuildPic:
			case _Key_AI_Walk_Stop:
			case _Key_AI_TP_Charge:
			case _Key_AI_Fun1:
			case _Key_AI_Fun2:
			{
				pBuf[_YNA_Cmd] = 0x4C;
				pBuf[_YNA_Data2] = u8KeyValue - _Key_AI_Ctrl_BuildPic + 0x62;			
				break;
			}
		
			
			case _Key_PGM_1:
			case _Key_PGM_2:
			case _Key_PGM_3:
			case _Key_PGM_4:
			case _Key_PGM_5:
			case _Key_PGM_6:
			{
				pBuf[_YNA_Cmd] = 0x48;
				pBuf[_YNA_Data2] = u8KeyValue - _Key_PGM_1 + 0x02;			
				break;
			}
			case _Key_PGM_7:
			case _Key_PGM_8:
			case _Key_PGM_9:
			case _Key_PGM_10:
			case _Key_PGM_11:
			case _Key_PGM_12:
			{
				pBuf[_YNA_Cmd] = 0x48;
				pBuf[_YNA_Data2] = u8KeyValue - _Key_PGM_7 + 0x80;			
				break;
			}


			case _Key_PVW_1:
			case _Key_PVW_2:
			case _Key_PVW_3:
			case _Key_PVW_4:
			case _Key_PVW_5:
			case _Key_PVW_6:
			{
				pBuf[_YNA_Cmd] = 0x48;
				pBuf[_YNA_Data2] = u8KeyValue - _Key_PVW_1 + 0x08;			
				break;
			}
			case _Key_PVW_7:
			case _Key_PVW_8:
			case _Key_PVW_9:
			case _Key_PVW_10:
			case _Key_PVW_11:
			case _Key_PVW_12:	
			{
				pBuf[_YNA_Cmd] = 0x48;
				pBuf[_YNA_Data2] = u8KeyValue - _Key_PVW_7 + 0x90;			
				break;
			}

			
			case _Key_Cam_Ctrl_Tele:
			{

				if (pKeyState->u8KeyState == KEY_DOWN)
				{
					u8 u8Cmd;
					ChangeLedState(GET_XY(_Led_Cam_Ctrl_Tele), true);
					u8Cmd = u8KeyValue - _Key_Cam_Ctrl_Tele + 1;
					pBuf[_YNA_Cmd] = u8Cmd << 4;
					pBuf[_YNA_Data3] = 0x0F;
				}
				else
				{
					pBuf[_YNA_Data1] = 0x00;

					ChangeLedState(GET_XY(_Led_Cam_Ctrl_Tele), false);

				}
				
				
				break;
			}
			case _Key_Cam_Ctrl_Wide:
			{
				if (pKeyState->u8KeyState == KEY_DOWN)
				{
					u8 u8Cmd;
					ChangeLedState(GET_XY(_Led_Cam_Ctrl_Wide), true);
					u8Cmd = u8KeyValue - _Key_Cam_Ctrl_Tele + 1;
					pBuf[_YNA_Cmd] = u8Cmd << 4;
					pBuf[_YNA_Data3] = 0x0F;					
				}
				else
				{
					pBuf[_YNA_Data1] = 0x00;
					ChangeLedState(GET_XY(_Led_Cam_Ctrl_Wide), false);
				}
				break;
			}
			case _Key_Cam_Ctrl_Present1:
			case _Key_Cam_Ctrl_Present2:
			case _Key_Cam_Ctrl_Present3:
			case _Key_Cam_Ctrl_Present4:
			case _Key_Cam_Ctrl_Present5:
			case _Key_Cam_Ctrl_Present6:
			case _Key_Cam_Ctrl_Present7:
			case _Key_Cam_Ctrl_Present8:
			{
				pBuf[_YNA_Cmd] = 0x40;
				pBuf[_YNA_Data2] = u8KeyValue - _Key_Cam_Ctrl_Present1;			
				break;
			}

			case _Key_Effect_1:
			case _Key_Effect_2:
			case _Key_Effect_3:
			case _Key_Effect_4:
			case _Key_Effect_5:
			case _Key_Effect_6:
			case _Key_Effect_7:
			case _Key_Effect_8:
			{
				pBuf[_YNA_Cmd] = 0x48;
				pBuf[_YNA_Data2] = u8KeyValue - _Key_Effect_1 + 0x0E;			
				break;
			}
					
			case _Key_Overlay_1:
			case _Key_Overlay_2:
			case _Key_Overlay_3:
			case _Key_Overlay_4:
			{
				pBuf[_YNA_Cmd] = 0x4B;
				pBuf[_YNA_Data2] = u8KeyValue - _Key_Overlay_1 + 0x01;			
				break;
			}

			
			default:
				continue;
		}

		
		YNAGetCheckSum(pBuf);
		CopyToUartMessage(pBuf, PROTOCOL_YNA_DECODE_LENGTH);	
	}
	return true;
}

static bool RockProcess(StKeyMixIn *pKeyIn)
{
	u8 *pBuf;
#if SCENCE_MUTUAL
	TurnOffZoomScence();
	PresetNumInit();
#endif
	if (!g_boIsRockEnable)
	{
		return false;
	}

	pBuf = u8YNABuf;

	memset(pBuf, 0, PROTOCOL_YNA_ENCODE_LENGTH);

	pBuf[_YNA_Sync] = 0xAA;
	pBuf[_YNA_Addr] = g_u8CamAddr;
	pBuf[_YNA_Mix] = 0x07;

	pBuf[_YNA_Cmd] = pKeyIn->unKeyMixIn.stRockState.u8RockDir;
	pBuf[_YNA_Data1] = pKeyIn->unKeyMixIn.stRockState.u16RockXValue >> 1;
	pBuf[_YNA_Data2] = pKeyIn->unKeyMixIn.stRockState.u16RockYValue >> 1;

	pBuf[_YNA_Data3] = pKeyIn->unKeyMixIn.stRockState.u16RockZValue >> 3;
	if (g_boIsRockAus)
	{
		pBuf[_YNA_Data1] |= (0x01 << 6);
	}
	YNAGetCheckSum(pBuf);
	CopyToUartMessage(pBuf, PROTOCOL_YNA_DECODE_LENGTH);
	
	if (!g_boUsingUART2)
	{
		return true;
	}
	if (g_emProtocol == _Protocol_PecloD)
	{
		u8 u8Buf[7];
		u8Buf[_PELCOD_Sync] = 0xFF;
		u8Buf[_PELCOD_Addr] = g_u8CamAddr + 1;
		u8Buf[_PELCOD_Cmd1] = 0;
		u8Buf[_PELCOD_Cmd2] = pKeyIn->unKeyMixIn.stRockState.u8RockDir << 1;
		u8Buf[_PELCOD_Data1] = pKeyIn->unKeyMixIn.stRockState.u16RockXValue >> 1;
		u8Buf[_PELCOD_Data2] = pKeyIn->unKeyMixIn.stRockState.u16RockYValue >> 1;
		PelcoDGetCheckSum(u8Buf);
		CopyToUart2Message(u8Buf, 7);
	}
	else
	{
		u8 u8Buf[16];
		u8 u8Cmd = pKeyIn->unKeyMixIn.stRockState.u8RockDir << 1;
		static bool boViscaNeedSendZoomStopCmd = false;
		static bool boViscaNeedSendDirStopCmd = false;
		static u8 u8Priority = 0;
		
		u8Cmd &= (PELCOD_DOWN | PELCOD_UP | 
					PELCOD_LEFT | PELCOD_RIGHT |
					PELCOD_ZOOM_TELE | PELCOD_ZOOM_WIDE);
		
		u8Buf[0] = 0x80 + g_u8CamAddr + 1;
		if (u8Priority == 0)
		{
			if ((u8Cmd & (PELCOD_DOWN | PELCOD_UP | PELCOD_LEFT | PELCOD_RIGHT)) != 0)
			{
				u8Priority = 1;
			}
			else if ((u8Cmd & (PELCOD_ZOOM_TELE | PELCOD_ZOOM_WIDE)) != 0)
			{
				u8Priority = 2;
			}
		}
		
		if (u8Priority == 1)
		{
			if (boViscaNeedSendDirStopCmd && 
				((u8Cmd & (PELCOD_DOWN | PELCOD_UP | PELCOD_LEFT | PELCOD_RIGHT)) == 0))
			{
				/* 81 01 06 01 18 18 03 03 FF */
				u8Buf[1] = 0x01;
				u8Buf[2] = 0x06;
				u8Buf[3] = 0x01;
				u8Buf[4] = 0x00;
				u8Buf[5] = 0x00;
				u8Buf[6] = 0x03;
				u8Buf[7] = 0x03;
				u8Buf[8] = 0xFF;
				CopyToUart2Message(u8Buf, 9);
				boViscaNeedSendDirStopCmd = false;
				if ((u8Cmd & (PELCOD_ZOOM_WIDE | PELCOD_ZOOM_WIDE)) != 0)
				{
					u8Priority = 2;
				}
				else
				{
					u8Priority = 0;					
				}
			}
			else
			{
				u8Buf[1] = 0x01;
				u8Buf[2] = 0x06;
				u8Buf[3] = 0x01;
				if ((u8Cmd & (PELCOD_LEFT | PELCOD_RIGHT)) != 0)
				{
					u32 u32Tmp = 0x17 * (pKeyIn->unKeyMixIn.stRockState.u16RockXValue >> 1);
					u32Tmp /= 0x3F;
					u32Tmp %= 0x18;
					u32Tmp += 1;

					u8Buf[4] = u32Tmp;
					if ((u8Cmd & PELCOD_LEFT) != 0)
					{
						u8Buf[6] = 0x01;
					}
					else
					{
						u8Buf[6] = 0x02;
					}

				}
				else
				{
					u8Buf[4] = 0;
					u8Buf[6] = 0x03;
				}
				
				if ((u8Cmd & (PELCOD_UP | PELCOD_DOWN)) != 0)
				{
					u32 u32Tmp = 0x13 * (pKeyIn->unKeyMixIn.stRockState.u16RockYValue >> 1);
					u32Tmp /= 0x3F;
					u32Tmp %= 0x14;
					u32Tmp += 1;

					u8Buf[5] = u32Tmp;
					if ((u8Cmd & PELCOD_UP) != 0)
					{
						u8Buf[7] = 0x01;
					}
					else
					{
						u8Buf[7] = 0x02;
					}

				}
				else
				{
					u8Buf[5] = 0;
					u8Buf[7] = 0x03;
				}
				u8Buf[8] = 0xFF;
				CopyToUart2Message(u8Buf, 9);	
				boViscaNeedSendDirStopCmd = true;			
			}	
		}
		
		if (u8Priority == 2)
		{
			if (boViscaNeedSendZoomStopCmd && 
					((u8Cmd & (PELCOD_ZOOM_WIDE | PELCOD_ZOOM_TELE)) == 0))
			{
				u8Buf[1] = 0x01;
				u8Buf[2] = 0x04;
				u8Buf[3] = 0x07;
				u8Buf[4] = 0x00;
				u8Buf[5] = 0xFF;
				CopyToUart2Message(u8Buf, 6);
				boViscaNeedSendZoomStopCmd = false;
				u8Priority = 0;
			}
			else if ((u8Cmd & PELCOD_ZOOM_WIDE) == PELCOD_ZOOM_WIDE)
			{
				u32 u32Tmp = 0x05 * (pKeyIn->unKeyMixIn.stRockState.u16RockZValue >> 3);
				u32Tmp /= 0x0F;
				u32Tmp %= 6;
				u32Tmp += 2;
				u8Buf[1] = 0x01;
				u8Buf[2] = 0x04;
				u8Buf[3] = 0x07;
				u8Buf[4] = u32Tmp + 0x30;
				u8Buf[5] = 0xFF;
				CopyToUart2Message(u8Buf, 6);
				boViscaNeedSendZoomStopCmd = true;

			}
			else
			{
				u32 u32Tmp = 0x05 * (pKeyIn->unKeyMixIn.stRockState.u16RockZValue >> 3);
				u32Tmp /= 0x0F;
				u32Tmp %= 6;
				u32Tmp += 2;
				u8Buf[1] = 0x01;
				u8Buf[2] = 0x04;
				u8Buf[3] = 0x07;
				u8Buf[4] = 0x20 + u32Tmp;
				u8Buf[5] = 0xFF;
				CopyToUart2Message(u8Buf, 6);			
				boViscaNeedSendZoomStopCmd = true;
			}
			
		}
		
		if (u8Cmd == 0)
		{
			u8Priority = 0;
		}
	}
	return true;
}

static bool MiniRockProcess(StKeyMixIn *pKeyIn)
{
	u8 *pBuf;
#if SCENCE_MUTUAL
	TurnOffZoomScence();
	PresetNumInit();
#endif
	if (!g_boIsRockEnable)
	{
		return false;
	}

	pBuf = u8YNABuf;

	memset(pBuf, 0, PROTOCOL_YNA_ENCODE_LENGTH);

	pBuf[_YNA_Sync] = 0xAA;
	pBuf[_YNA_Addr] = g_u8CamAddr;
	pBuf[_YNA_Mix] = 0x07;

	pBuf[_YNA_Cmd] = pKeyIn->unKeyMixIn.stRockState.u8RockDir;
	pBuf[_YNA_Data1] = pKeyIn->unKeyMixIn.stRockState.u16RockXValue >> 1;
	pBuf[_YNA_Data2] = pKeyIn->unKeyMixIn.stRockState.u16RockYValue >> 1;

	pBuf[_YNA_Data3] = pKeyIn->unKeyMixIn.stRockState.u16RockZValue >> 3;
	
	pBuf[_YNA_Data1] |= (0x01 << 6);

	YNAGetCheckSum(pBuf);
	CopyToUartMessage(pBuf, PROTOCOL_YNA_DECODE_LENGTH);
	return true;
}
static bool PushPodProcess(StKeyMixIn *pKeyIn)
{
	u8 *pBuf;
	pBuf = u8YNABuf;
	if (pBuf == NULL)
	{
		return false;
	}

	memset(pBuf, 0, PROTOCOL_YNA_ENCODE_LENGTH);

	pBuf[_YNA_Sync] = 0xAA;
	pBuf[_YNA_Addr] = g_u8CamAddr;
	pBuf[_YNA_Mix] = 0x07;

	pBuf[_YNA_Cmd] = 0x80;
	
	pBuf[_YNA_Data1] |= 0x80;

	pBuf[_YNA_Data2] = pKeyIn->unKeyMixIn.u32PushRodValue;
	YNAGetCheckSum(pBuf);
	CopyToUartMessage(pBuf, PROTOCOL_YNA_DECODE_LENGTH);
	return true;
}



static bool CodeSwitchProcess(StKeyMixIn *pKeyIn)
{
	u8 *pBuf;
	u16 u16Index;

	pBuf = u8YNABuf;
	if (pBuf == NULL)
	{
		return false;
	}

	memset(pBuf, 0, PROTOCOL_YNA_ENCODE_LENGTH);

	pBuf[_YNA_Sync] = 0xAA;
	pBuf[_YNA_Addr] = g_u8CamAddr;
	pBuf[_YNA_Mix] = 0x07;

	u16Index = pKeyIn->unKeyMixIn.stCodeSwitchState.u16Index;
	switch (u16Index)
	{
		case 0x00:
		{
			pBuf[_YNA_Cmd] = 0x49;
			pBuf[_YNA_Data1] |= 0x00;
			break;
		}
		case 0x01:
		{
			pBuf[_YNA_Cmd] = 0x49;
			pBuf[_YNA_Data1] |= 0x10;
			break;
		}
		default:
			return false;

	}
	pBuf[_YNA_Data2] = pKeyIn->unKeyMixIn.stCodeSwitchState.u16Cnt;
	YNAGetCheckSum(pBuf);
	CopyToUartMessage(pBuf, PROTOCOL_YNA_DECODE_LENGTH);
	return true;
	
}

static bool VolumeProcess(StKeyMixIn *pKeyIn)
{
	u8 *pBuf;
	pBuf = u8YNABuf;
	if (pBuf == NULL)
	{
		return false;
	}

	memset(pBuf, 0, PROTOCOL_YNA_ENCODE_LENGTH);

	pBuf[_YNA_Sync] = 0xAA;
	pBuf[_YNA_Addr] = g_u8CamAddr;
	pBuf[_YNA_Mix] = 0x06;

	pBuf[_YNA_Cmd] = 0x80;
	pBuf[_YNA_Data3] = pBuf[_YNA_Data2] = pKeyIn->unKeyMixIn.u32VolumeValue;
	YNAGetCheckSum(pBuf);
	CopyToUartMessage(pBuf, PROTOCOL_YNA_DECODE_LENGTH);
	return true;
}

static PFun_KeyProcess s_KeyProcessArr[_Key_Reserved] = 
{
	PushPodProcess, KeyBoardProcess, RockProcess, MiniRockProcess,
	VolumeProcess, CodeSwitchProcess, 
};


const u16 c_u16LedPresent[] = 
{
	_Led_Cam_Ctrl_Present1,
	_Led_Cam_Ctrl_Present2,
	_Led_Cam_Ctrl_Present3,
	_Led_Cam_Ctrl_Present4,
	_Led_Cam_Ctrl_Present5,
	_Led_Cam_Ctrl_Present6,
	_Led_Cam_Ctrl_Present7,
	_Led_Cam_Ctrl_Present8,
};

#define TURN_OFF_PRESENT()			ChangeLedArrayState(c_u16LedPresent, sizeof(c_u16LedPresent) / sizeof(u16), false);


static bool KeyBoardProcessForBJZS(StKeyMixIn *pKeyIn)
{
	u32 i;
	for (i = 0; i < pKeyIn->u32Cnt; i++)
	{
		u8 *pBuf;
		u8 u8BJZSBuf[4];
		StKeyState *pKeyState = pKeyIn->unKeyMixIn.stKeyState + i;
		u8 u8KeyValue;
		bool boSendVisca = false;
		
		u8KeyValue = pKeyState->u8KeyValue;
		if (pKeyState->u8KeyState == KEY_KEEP)
		{
			if ((u8KeyValue >= _Key_Cam_Ctrl_Present1) && 
				(u8KeyValue <= _Key_Cam_Ctrl_Present8))
			{
				goto next;
			}
			if ((u8KeyValue >= _Key_AI_Present1) && 
				(u8KeyValue <= _Key_AI_Present4))
			{
				goto next;
			}
			continue;
		}

		next:		

		pBuf = u8BJZSBuf;

		memset(pBuf, 0, 4);

		/* 处理按键 */
		switch (u8KeyValue)
		{			
			case _Key_Make_Record:
			case _Key_Make_Live:
			case _Key_Make_FullScreen:
			case _Key_Make_MutliRecord:
			{
				const u16 u16Led[4] = 
				{
					_Led_Make_Record,
					_Led_Make_Live,
					_Led_Make_FullScreen,
					_Led_Make_MutliRecord,
				};
				static bool boIsLight[4] = {0};
				u8 u8Key = u8KeyValue - _Key_Make_Record;
				if (pKeyState->u8KeyState == KEY_UP)
				{
					continue;
				}
				boIsLight[u8Key] = !boIsLight[u8Key];
				
				pBuf[_BJZS_Special] = BJZS_CTRL;
				pBuf[_BJZS_Key] = u8Key + 0x21;
				pBuf[_BJZS_Extern] = boIsLight[u8Key];
				
				ChangeLedState(GET_XY(u16Led[u8Key]), boIsLight[u8Key]);			
				break;
			}
			
			case _Key_Ctrl_Machine_Switch1:
			case _Key_Ctrl_Machine_Switch2:
			case _Key_Ctrl_Machine_Switch3:
			case _Key_Ctrl_Machine_Switch4:
			{
				const u16 u16Led[4] = 
				{
					_Led_Ctrl_Machine_Switch1,
					_Led_Ctrl_Machine_Switch2,
					_Led_Ctrl_Machine_Switch3,
					_Led_Ctrl_Machine_Switch4,
				};
				const u8 u8BJZSKey[4] = 
				{
					0x23, 0x24, 0x21, 0x22,
				};

				
				u8 u8Key = u8KeyValue - _Key_Ctrl_Machine_Switch1;
				if (pKeyState->u8KeyState == KEY_UP)
				{
					continue;
				}
				
				pBuf[_BJZS_Special] = 0;
				pBuf[_BJZS_Key] = u8BJZSKey[u8Key];
				pBuf[_BJZS_Extern] = 1;
				
				ChangeLedArrayState(u16Led, sizeof(u16Led) / sizeof(u16), false);
				ChangeLedState(GET_XY(u16Led[u8Key]), true);

				/* camera address */
				g_u8CamAddr = u8Key;
				TURN_OFF_PRESENT();
				
				break;
			}

			case _Key_Ctrl_Machine_Up:
			case _Key_Ctrl_Machine_Down:
			{
				const u16 u16Led[2] = 
				{
					_Led_Ctrl_Machine_Up,
					_Led_Ctrl_Machine_Down,
				};
				const u8 u8BJZSKey[2] = 
				{
					0x26, 0x28,
				};
			
				u8 u8Key = u8KeyValue - _Key_Ctrl_Machine_Up;
				pBuf[_BJZS_Special] = 0;
				pBuf[_BJZS_Key] = u8BJZSKey[u8Key];
				pBuf[_BJZS_Extern] = (pKeyState->u8KeyState == KEY_DOWN) ? 1 : 0;
				
				ChangeLedState(GET_XY(u16Led[u8Key]), pBuf[_BJZS_Extern]);	
				break;
			}

			case _Key_SlowMove_Machine_Choose1:
			case _Key_SlowMove_Machine_Choose2:
			case _Key_SlowMove_Machine_Choose3:
			case _Key_SlowMove_Machine_Choose4:
			{
				const u16 u16Led[] = 
				{
					_Led_SlowMove_Machine_Choose1,
					_Led_SlowMove_Machine_Choose2,
					_Led_SlowMove_Machine_Choose3,
					_Led_SlowMove_Machine_Choose4,
				};
				
				u8 u8Key = u8KeyValue - _Key_SlowMove_Machine_Choose1;
				if (pKeyState->u8KeyState == KEY_UP)
				{
					continue;
				}
				
				pBuf[_BJZS_Special] = BJZS_ALT;
				pBuf[_BJZS_Key] = u8Key + 0x61;
				pBuf[_BJZS_Extern] = 0;
				
				ChangeLedArrayState(u16Led, sizeof(u16Led) / sizeof(u16), false);
				ChangeLedState(GET_XY(u16Led[u8Key]), true);	
				break;
			}
			case _Key_SlowMove_Record_Start:
			case _Key_SlowMove_Record_Stop:
			case _Key_SlowMove_Make_Start:
			case _Key_SlowMove_Make_Stop:
			case _Key_SlowMove_Replay_Start:
			case _Key_SlowMove_Replay_Stop:
			{
				const u16 u16Led[][2] = 
				{
					{_Led_SlowMove_Record_Start, _Led_SlowMove_Record_Stop},
					{_Led_SlowMove_Make_Start, _Led_SlowMove_Make_Stop},
					{_Led_SlowMove_Replay_Start, _Led_SlowMove_Replay_Stop},						
				};
				const u8 u8BJZSSpecial[] = 
				{
					0, BJZS_ALT, BJZS_CTRL,
				};
				
				const u8 u8BJZSKey[] = 
				{
					0x20, 0x1B
				};
				

				
				u8 u8Key = u8KeyValue - _Key_SlowMove_Record_Start;
				u8 u8Array = u8Key >> 1;
				u8Key &= 0x01;
				if (pKeyState->u8KeyState == KEY_UP)
				{
					continue;
				}
				
				pBuf[_BJZS_Special] = u8BJZSSpecial[u8Array];
				pBuf[_BJZS_Key] = u8BJZSKey[u8Key];
				pBuf[_BJZS_Extern] = 0;

				ChangeLedArrayState(u16Led[u8Array], 2, false);
				ChangeLedState(GET_XY(u16Led[u8Array][u8Key]), true);
				break;

			}
			
			case _Key_AI_Present1:
			case _Key_AI_Present2:
			case _Key_AI_Present3:
			case _Key_AI_Present4:
			{
				static u32 u32KeyDownTime[4];
				static bool boIsLongPress[4];
				const u16 u16Led[4] = 
				{
					_Led_AI_Present1,
					_Led_AI_Present2,
					_Led_AI_Present3,
					_Led_AI_Present4,
				};
				u8 u8Key = u8KeyValue - _Key_AI_Present1;
				if (pKeyState->u8KeyState == KEY_DOWN)
				{
					u32KeyDownTime[u8Key] = g_u32SysTickCnt;
					boIsLongPress[u8Key] = false;
					continue;
				}
				else if (pKeyState->u8KeyState == KEY_KEEP)
				{
					if (!boIsLongPress[u8Key])
					{
						if (SysTimeDiff(u32KeyDownTime[u8Key], g_u32SysTickCnt) > 2000)
						{
							boIsLongPress[u8Key] = true;
							ChangeLedBlinkState(GET_XY(u16Led[u8Key]), true);
							StartBuzzer(_Buz_Msg_Set_Preset);
						}
					}
					continue;
				}
				else
				{
					pBuf[_BJZS_Special] = BJZS_CTRL;
					pBuf[_BJZS_Key] = 0x70 + u8Key;
					pBuf[_BJZS_Extern] = boIsLongPress[u8Key];
					
					ChangeLedBlinkState(GET_XY(u16Led[u8Key]), false);
					ChangeLedArrayState(u16Led, sizeof(u16Led) / sizeof(u16), false);
					ChangeLedState(GET_XY(u16Led[u8Key]), true);	
					
				}
			
				break;
			}
			
			case _Key_AI_Ctrl_BuildPic:
			case _Key_AI_Walk_Stop:
			case _Key_AI_TP_Charge:
			{
				const u16 u16Led[] = 
				{
					_Led_AI_Ctrl_BuildPic,
					_Led_AI_Walk_Stop,
					_Led_AI_TP_Charge,
				};
				static bool boIsLight[] = {0};
				u8 u8Key = u8KeyValue - _Key_AI_Ctrl_BuildPic;
				if (pKeyState->u8KeyState == KEY_UP)
				{
					continue;
				}
				boIsLight[u8Key] = !boIsLight[u8Key];
				
				pBuf[_BJZS_Special] = BJZS_CTRL;
				pBuf[_BJZS_Key] = u8Key + 0x75;
				pBuf[_BJZS_Extern] = boIsLight[u8Key];
				
				ChangeLedState(GET_XY(u16Led[u8Key]), boIsLight[u8Key]);			
				break;
			}			
			case _Key_AI_Fun1:
			case _Key_AI_Fun2:
			{
				const u16 u16Led[] = 
				{
					_Led_AI_Fun1,
					_Led_AI_Fun2,
				};
			
				u8 u8Key = u8KeyValue - _Key_AI_Fun1;
				pBuf[_BJZS_Special] = 0;
				pBuf[_BJZS_Key] = u8Key + 0x78;
				pBuf[_BJZS_Extern] = (pKeyState->u8KeyState == KEY_DOWN) ? 1 : 0;
				
				ChangeLedState(GET_XY(u16Led[u8Key]), pBuf[_BJZS_Extern]);	
				break;
			}
			
			case _Key_PGM_1:				
			case _Key_PGM_2:
			case _Key_PGM_3:
			case _Key_PGM_4:
			case _Key_PGM_5:
			case _Key_PGM_6:
			case _Key_PGM_7:
			case _Key_PGM_8:
			case _Key_PGM_9:
			case _Key_PGM_10:
			case _Key_PGM_11:
			case _Key_PGM_12:		/* 24 */
			{
				const u16 u16Led[] = 
				{
					_Led_PGM_1,
					_Led_PGM_2,
					_Led_PGM_3,
					_Led_PGM_4,
					_Led_PGM_5,
					_Led_PGM_6,
					_Led_PGM_7,
					_Led_PGM_8,
					_Led_PGM_9,
					_Led_PGM_10,
					_Led_PGM_11,
					_Led_PGM_12,		/* 24 */
				};
				const u8 u8BJZSKey[] = 
				{
					0x31, 0x32, 0x33, 0x34,
					0x35, 0x36, 0x37, 0x38,
					0x39, 0x30, 0x60, 0x65,
				};

				
				u8 u8Key = u8KeyValue - _Key_PGM_1;
				if (pKeyState->u8KeyState == KEY_UP)
				{
					continue;
				}
				
				pBuf[_BJZS_Special] = 0;
				pBuf[_BJZS_Key] = u8BJZSKey[u8Key];
				pBuf[_BJZS_Extern] = 0;
				
				ChangeLedArrayState(u16Led, sizeof(u16Led) / sizeof(u16), false);
				ChangeLedState(GET_XY(u16Led[u8Key]), true);	
				break;
			}

			case _Key_PVW_1:
			case _Key_PVW_2:
			case _Key_PVW_3:
			case _Key_PVW_4:
			case _Key_PVW_5:
			case _Key_PVW_6:
			case _Key_PVW_7:
			case _Key_PVW_8:
			case _Key_PVW_9:
			case _Key_PVW_10:
			case _Key_PVW_11:
			case _Key_PVW_12:		/* 36 */	
			{
				const u16 u16Led[] = 
				{
					_Led_PVW_1,
					_Led_PVW_2,
					_Led_PVW_3,
					_Led_PVW_4,
					_Led_PVW_5,
					_Led_PVW_6,
					_Led_PVW_7,
					_Led_PVW_8,
					_Led_PVW_9,
					_Led_PVW_10,
					_Led_PVW_11,
					_Led_PVW_12,		/* 24 */
				};
				const u8 u8BJZSKey[] = 
				{
					0x31, 0x32, 0x33, 0x34,
					0x35, 0x36, 0x37, 0x38,
					0x39, 0x30, 0x60, 0x65,
				};

				
				u8 u8Key = u8KeyValue - _Key_PVW_1;
				if (pKeyState->u8KeyState == KEY_UP)
				{
					continue;
				}
				
				pBuf[_BJZS_Special] = BJZS_CTRL;
				pBuf[_BJZS_Key] = u8BJZSKey[u8Key];
				pBuf[_BJZS_Extern] = 0;
				
				ChangeLedArrayState(u16Led, sizeof(u16Led) / sizeof(u16), false);
				ChangeLedState(GET_XY(u16Led[u8Key]), true);	
				break;
			}
			case _Key_Cam_Ctrl_Tele:
			case _Key_Cam_Ctrl_Wide:	
			{
				const u16 u16Led[2] = 
				{
					_Led_Cam_Ctrl_Tele,
					_Led_Cam_Ctrl_Wide,
				};
				const u8 u8BJZSKey[2] = 
				{
					0x25, 0x27,
				};
			
				u8 u8Key = u8KeyValue - _Key_Cam_Ctrl_Tele;
				pBuf[_BJZS_Special] = 0;
				pBuf[_BJZS_Key] = u8BJZSKey[u8Key];
				pBuf[_BJZS_Extern] = (pKeyState->u8KeyState == KEY_DOWN) ? 1 : 0;
				
				ChangeLedState(GET_XY(u16Led[u8Key]), pBuf[_BJZS_Extern]);	
				
				/* visca */
				boSendVisca = true;
				TURN_OFF_PRESENT();
				if (u8KeyValue == _Key_Cam_Ctrl_Tele)
				{
					if (pKeyState->u8KeyState == KEY_DOWN)
					{				
						u8 u8Buf[6];
						
						u8Buf[0] = 0x80 + g_u8CamAddr + 1;
						u8Buf[1] = 0x01;
						u8Buf[2] = 0x04;
						u8Buf[3] = 0x07;
						u8Buf[4] = 0x02;
						u8Buf[5] = 0xFF;
						CopyToUartMessage(u8Buf, 6);
					}
					else
					{
						u8 u8Buf[6];
						
						u8Buf[0] = 0x80 + g_u8CamAddr + 1;
						u8Buf[1] = 0x01;
						u8Buf[2] = 0x04;
						u8Buf[3] = 0x07;
						u8Buf[4] = 0x00;
						u8Buf[5] = 0xFF;
						CopyToUartMessage(u8Buf, 6);				
					}
				}
				else
				{
					if (pKeyState->u8KeyState == KEY_DOWN)
					{				
						u8 u8Buf[6];
						
						u8Buf[0] = 0x80 + g_u8CamAddr + 1;
						u8Buf[1] = 0x01;
						u8Buf[2] = 0x04;
						u8Buf[3] = 0x07;
						u8Buf[4] = 0x03;
						u8Buf[5] = 0xFF;
						CopyToUartMessage(u8Buf, 6);
					}
					else
					{
						u8 u8Buf[6];
						
						u8Buf[0] = 0x80 + g_u8CamAddr + 1;
						u8Buf[1] = 0x01;
						u8Buf[2] = 0x04;
						u8Buf[3] = 0x07;
						u8Buf[4] = 0x00;
						u8Buf[5] = 0xFF;
						CopyToUartMessage(u8Buf, 6);
					}
					
				}
				break;
			}

			case _Key_Cam_Ctrl_Present1:
			case _Key_Cam_Ctrl_Present2:
			case _Key_Cam_Ctrl_Present3:
			case _Key_Cam_Ctrl_Present4:
			case _Key_Cam_Ctrl_Present5:
			case _Key_Cam_Ctrl_Present6:
			case _Key_Cam_Ctrl_Present7:
			case _Key_Cam_Ctrl_Present8:
			{
				const u8 u8BJZSKey[8] = 
				{
					0x61, 0x62, 0x63, 0x64,
					0x23, 0x24, 0x21, 0x22
				};

				static u32 u32KeyDownTime[8];
				static bool boIsLongPress[8];
				u8 u8Key = u8KeyValue - _Key_Cam_Ctrl_Present1;
				if (pKeyState->u8KeyState == KEY_DOWN)
				{
					u32KeyDownTime[u8Key] = g_u32SysTickCnt;
					boIsLongPress[u8Key] = false;
					continue;
				}
				else if (pKeyState->u8KeyState == KEY_KEEP)
				{
					if (!boIsLongPress[u8Key])
					{
						if (SysTimeDiff(u32KeyDownTime[u8Key], g_u32SysTickCnt) > 2000)
						{
							boIsLongPress[u8Key] = true;
							ChangeLedBlinkState(GET_XY(c_u16LedPresent[u8Key]), true);
							StartBuzzer(_Buz_Msg_Set_Preset);
						}
					}
					continue;
				}
				else
				{
					pBuf[_BJZS_Special] = 0;
					pBuf[_BJZS_Key] = u8BJZSKey[u8Key]; //0x61 + u8Key;
					pBuf[_BJZS_Extern] = boIsLongPress[u8Key];
					
					ChangeLedBlinkState(GET_XY(c_u16LedPresent[u8Key]), false);
					ChangeLedArrayState(c_u16LedPresent, sizeof(c_u16LedPresent) / sizeof(u16), false);
					ChangeLedState(GET_XY(c_u16LedPresent[u8Key]), true);

					/* visca */
					boSendVisca = true;
					if (boIsLongPress[u8Key])	/* set */
					{
						u8 u8Buf[7];
						u8Buf[0] = 0x80 + g_u8CamAddr + 1;
						u8Buf[1] = 0x01;
						u8Buf[2] = 0x04;
						u8Buf[3] = 0x3F;
						u8Buf[4] = 0x01;
						u8Buf[5] = u8Key;
						u8Buf[6] = 0xFF;
						CopyToUartMessage(u8Buf, 7);
			
					}
					else	/* call */
					{
						u8 u8Buf[7];
						u8Buf[0] = 0x80 + g_u8CamAddr + 1;
						u8Buf[1] = 0x01;
						u8Buf[2] = 0x04;
						u8Buf[3] = 0x3F;
						u8Buf[4] = 0x02;
						u8Buf[5] = u8Key;
						u8Buf[6] = 0xFF;
						CopyToUartMessage(u8Buf, 7);
					}					
				}
			
				break;
			}


			case _Key_Effect_1:
			case _Key_Effect_2:
			case _Key_Effect_3:
			case _Key_Effect_4:
			case _Key_Effect_5:
			case _Key_Effect_6:		
			case _Key_Effect_7:		
			case _Key_Effect_8:		
			{
				const u16 u16Led[] = 
				{
					_Led_Effect_1,
					_Led_Effect_2,
					_Led_Effect_3,
					_Led_Effect_4,
					_Led_Effect_5,
					_Led_Effect_6,		
					_Led_Effect_7,		
					_Led_Effect_8,		
				};

				u8 u8Key = u8KeyValue - _Key_Effect_1;
				pBuf[_BJZS_Special] = 0;
				pBuf[_BJZS_Key] = u8Key + 0x70;
				pBuf[_BJZS_Extern] = (pKeyState->u8KeyState == KEY_DOWN) ? 1 : 0;
				
				ChangeLedState(GET_XY(u16Led[u8Key]), pBuf[_BJZS_Extern]);	
				break;
			}

			
			case _Key_Overlay_1:
			case _Key_Overlay_2:
			case _Key_Overlay_3:
			case _Key_Overlay_4:
			{
				const u16 u16Led[] = 
				{
					_Led_Overlay_1,
					_Led_Overlay_2,
					_Led_Overlay_3,
					_Led_Overlay_4,
				};
				static bool boIsLight[] = {0};
				u8 u8Key = u8KeyValue - _Key_Overlay_1;
				if (pKeyState->u8KeyState == KEY_UP)
				{
					continue;
				}
				boIsLight[u8Key] = !boIsLight[u8Key];
				
				pBuf[_BJZS_Special] = BJZS_CTRL;
				pBuf[_BJZS_Key] = u8Key + 0x61;
				pBuf[_BJZS_Extern] = boIsLight[u8Key];
				
				ChangeLedState(GET_XY(u16Led[u8Key]), boIsLight[u8Key]);			
				break;
			}			
			
			case _Key_Switch_Speed:
			{
				pBuf[_BJZS_Special] = BJZS_SHIFT;
				pBuf[_BJZS_Key] = 0x6B;
				pBuf[_BJZS_Extern] = (pKeyState->u8KeyState == KEY_DOWN) ? 1 : 0;
				
				ChangeLedState(GET_XY(_Led_Switch_Speed), pBuf[_BJZS_Extern]);

				break;
				
			}
			case _Key_Rock_Btn:
			case _Key_MiniRock_Btn:
			{
				continue;
			}
				
			default:
				break;
		}
		if (!boSendVisca)
		{
			CopyToUartMessage(pBuf, PROTOCOL_BJZS_LENGTH);	
		}
		FlushHIDMsgForBJZS(pBuf, PROTOCOL_BJZS_LENGTH);
	}
	
	return true;
}

static bool RockProcessForBJZS(StKeyMixIn *pKeyIn)
{
	u8 u8BJZSBuf[4];
	u8 *pBuf = u8BJZSBuf;
	u8 u8Dir = 0, x, y, z;
	static u8 u8DirPTState = 0;
	static u8 u8DirZState = 0;

	memset(pBuf, 0, PROTOCOL_BJZS_LENGTH);

	
	u8Dir = pKeyIn->unKeyMixIn.stRockState.u8RockDir;
	x = pKeyIn->unKeyMixIn.stRockState.u16RockXValue >> 1;
	y = pKeyIn->unKeyMixIn.stRockState.u16RockYValue >> 1;
	z = pKeyIn->unKeyMixIn.stRockState.u16RockZValue >> 3;
	
	x >>= 2;
	y >>= 2;
	pBuf[_BJZS_Special] = BJZS_CTRL;
	if ((u8Dir & (_YNA_CAM_LEFT | _YNA_CAM_UP | _YNA_CAM_RIGHT | _YNA_CAM_DOWN)) == 0)
	{
		if (u8DirPTState == 2)
		{
			pBuf[_BJZS_Key] = 0x00;
			pBuf[_BJZS_Extern] = 0x00;
			u8DirPTState = 1;			
		}
	}
	else if ((u8Dir & (_YNA_CAM_LEFT | _YNA_CAM_UP)) == (_YNA_CAM_LEFT | _YNA_CAM_UP))
	{
		pBuf[_BJZS_Key] = 0x25;
		pBuf[_BJZS_Extern] = (y << 4) | x;
		u8DirPTState = 2;
	}
	else if ((u8Dir & (_YNA_CAM_RIGHT | _YNA_CAM_UP)) == (_YNA_CAM_RIGHT | _YNA_CAM_UP))
	{
		pBuf[_BJZS_Key] = 0x26;
		pBuf[_BJZS_Extern] = (y << 4) | x;
		u8DirPTState = 2;
	}
	else if ((u8Dir & (_YNA_CAM_LEFT | _YNA_CAM_DOWN)) == (_YNA_CAM_LEFT | _YNA_CAM_DOWN))
	{
		pBuf[_BJZS_Key] = 0x27;
		pBuf[_BJZS_Extern] = (y << 4) | x;
		u8DirPTState = 2;
	}
	else if ((u8Dir & (_YNA_CAM_RIGHT | _YNA_CAM_DOWN)) == (_YNA_CAM_RIGHT | _YNA_CAM_DOWN))
	{
		pBuf[_BJZS_Key] = 0x28;
		pBuf[_BJZS_Extern] = (y << 4) | x;
		u8DirPTState = 2;
	}
	else
	{
		if (u8Dir & _YNA_CAM_LEFT)
		{
			pBuf[_BJZS_Key] = 0x25;
			pBuf[_BJZS_Extern] = x;			
			u8DirPTState = 2;
		}
		else if(u8Dir & _YNA_CAM_RIGHT) 
		{
			pBuf[_BJZS_Key] = 0x27;
			pBuf[_BJZS_Extern] = x;						
			u8DirPTState = 2;
		}
		
		if (u8Dir & _YNA_CAM_UP)
		{
			pBuf[_BJZS_Key] = 0x26;
			pBuf[_BJZS_Extern] = y;			
			u8DirPTState = 2;
		}
		else if (u8Dir & _YNA_CAM_DOWN)
		{
			pBuf[_BJZS_Key] = 0x28;
			pBuf[_BJZS_Extern] = y;						
			u8DirPTState = 2;
		}
	}
	
	
	if (u8DirPTState != 0)
	{
		//CopyToUartMessage(pBuf, PROTOCOL_BJZS_LENGTH);
		FlushHIDMsgForBJZS(pBuf, PROTOCOL_BJZS_LENGTH);
		if (u8DirPTState == 1)
		{
			u8DirPTState = 0;
		}
	}
	

	pBuf[_BJZS_Special] = 0;	
	if ((u8Dir & (_YNA_CAM_TELE | _YNA_CAM_WIDE)) == 0)
	{
		if (u8DirZState == 2)
		{
			pBuf[_BJZS_Key] = 0x2F;
			pBuf[_BJZS_Extern] = 0x00;
			u8DirZState = 1;			
		}
	}
	else if (u8Dir & _YNA_CAM_TELE)
	{
		pBuf[_BJZS_Key] = 0x2E;
		pBuf[_BJZS_Extern] = z;
		u8DirZState = 2;					
	}
	else if (u8Dir & _YNA_CAM_WIDE)
	{
		pBuf[_BJZS_Key] = 0x2D;
		pBuf[_BJZS_Extern] = z;
		u8DirZState = 2;					
	}
	
	if (u8DirZState != 0)
	{
		//CopyToUartMessage(pBuf, PROTOCOL_BJZS_LENGTH);
		FlushHIDMsgForBJZS(pBuf, PROTOCOL_BJZS_LENGTH);

		if (u8DirZState == 1)
		{
			u8DirZState = 0;
		}
	}
	
	/* visca */
	{
		u8 u8Buf[16];
		u8 u8Cmd = pKeyIn->unKeyMixIn.stRockState.u8RockDir << 1;
		static bool boViscaNeedSendZoomStopCmd = false;
		static bool boViscaNeedSendDirStopCmd = false;
		static u8 u8Priority = 0;
		
		TURN_OFF_PRESENT();
		
		u8Cmd &= (PELCOD_DOWN | PELCOD_UP | 
					PELCOD_LEFT | PELCOD_RIGHT |
					PELCOD_ZOOM_TELE | PELCOD_ZOOM_WIDE);
		
		u8Buf[0] = 0x80 + g_u8CamAddr + 1;
		if (u8Priority == 0)
		{
			if ((u8Cmd & (PELCOD_DOWN | PELCOD_UP | PELCOD_LEFT | PELCOD_RIGHT)) != 0)
			{
				u8Priority = 1;
			}
			else if ((u8Cmd & (PELCOD_ZOOM_TELE | PELCOD_ZOOM_WIDE)) != 0)
			{
				u8Priority = 2;
			}
		}
		
		if (u8Priority == 1)
		{
			if (boViscaNeedSendDirStopCmd && 
				((u8Cmd & (PELCOD_DOWN | PELCOD_UP | PELCOD_LEFT | PELCOD_RIGHT)) == 0))
			{
				/* 81 01 06 01 18 18 03 03 FF */
				u8Buf[1] = 0x01;
				u8Buf[2] = 0x06;
				u8Buf[3] = 0x01;
				u8Buf[4] = 0x00;
				u8Buf[5] = 0x00;
				u8Buf[6] = 0x03;
				u8Buf[7] = 0x03;
				u8Buf[8] = 0xFF;
				CopyToUartMessage(u8Buf, 9);
				boViscaNeedSendDirStopCmd = false;
				if ((u8Cmd & (PELCOD_ZOOM_WIDE | PELCOD_ZOOM_WIDE)) != 0)
				{
					u8Priority = 2;
				}
				else
				{
					u8Priority = 0;					
				}
			}
			else
			{
				u8Buf[1] = 0x01;
				u8Buf[2] = 0x06;
				u8Buf[3] = 0x01;
				if ((u8Cmd & (PELCOD_LEFT | PELCOD_RIGHT)) != 0)
				{
					u32 u32Tmp = 0x17 * (pKeyIn->unKeyMixIn.stRockState.u16RockXValue >> 1);
					u32Tmp /= 0x3F;
					u32Tmp %= 0x18;
					u32Tmp += 1;

					u8Buf[4] = u32Tmp;
					if ((u8Cmd & PELCOD_LEFT) != 0)
					{
						u8Buf[6] = 0x01;
					}
					else
					{
						u8Buf[6] = 0x02;
					}

				}
				else
				{
					u8Buf[4] = 0;
					u8Buf[6] = 0x03;
				}
				
				if ((u8Cmd & (PELCOD_UP | PELCOD_DOWN)) != 0)
				{
					u32 u32Tmp = 0x13 * (pKeyIn->unKeyMixIn.stRockState.u16RockYValue >> 1);
					u32Tmp /= 0x3F;
					u32Tmp %= 0x14;
					u32Tmp += 1;

					u8Buf[5] = u32Tmp;
					if ((u8Cmd & PELCOD_UP) != 0)
					{
						u8Buf[7] = 0x01;
					}
					else
					{
						u8Buf[7] = 0x02;
					}

				}
				else
				{
					u8Buf[5] = 0;
					u8Buf[7] = 0x03;
				}
				u8Buf[8] = 0xFF;
				CopyToUartMessage(u8Buf, 9);	
				boViscaNeedSendDirStopCmd = true;			
			}	
		}
		
		if (u8Priority == 2)
		{
			if (boViscaNeedSendZoomStopCmd && 
					((u8Cmd & (PELCOD_ZOOM_WIDE | PELCOD_ZOOM_TELE)) == 0))
			{
				u8Buf[1] = 0x01;
				u8Buf[2] = 0x04;
				u8Buf[3] = 0x07;
				u8Buf[4] = 0x00;
				u8Buf[5] = 0xFF;
				CopyToUartMessage(u8Buf, 6);
				boViscaNeedSendZoomStopCmd = false;
				u8Priority = 0;
			}
			else if ((u8Cmd & PELCOD_ZOOM_WIDE) == PELCOD_ZOOM_WIDE)
			{
				u32 u32Tmp = 0x05 * (pKeyIn->unKeyMixIn.stRockState.u16RockZValue >> 3);
				u32Tmp /= 0x0F;
				u32Tmp %= 6;
				u32Tmp += 2;
				u8Buf[1] = 0x01;
				u8Buf[2] = 0x04;
				u8Buf[3] = 0x07;
				u8Buf[4] = u32Tmp + 0x30;
				u8Buf[5] = 0xFF;
				CopyToUartMessage(u8Buf, 6);
				boViscaNeedSendZoomStopCmd = true;

			}
			else
			{
				u32 u32Tmp = 0x05 * (pKeyIn->unKeyMixIn.stRockState.u16RockZValue >> 3);
				u32Tmp /= 0x0F;
				u32Tmp %= 6;
				u32Tmp += 2;
				u8Buf[1] = 0x01;
				u8Buf[2] = 0x04;
				u8Buf[3] = 0x07;
				u8Buf[4] = 0x20 + u32Tmp;
				u8Buf[5] = 0xFF;
				CopyToUartMessage(u8Buf, 6);			
				boViscaNeedSendZoomStopCmd = true;
			}
			
		}
		
		if (u8Cmd == 0)
		{
			u8Priority = 0;
		}
	}
	
	return true;
}

static bool MiniRockProcessForBJZS(StKeyMixIn *pKeyIn)
{
	u8 u8BJZSBuf[4];
	u8 *pBuf = u8BJZSBuf;
	u8 u8Dir = 0, x, y;
	static u8 u8DirPTState = 0;

	memset(pBuf, 0, PROTOCOL_BJZS_LENGTH);

	
	u8Dir = pKeyIn->unKeyMixIn.stRockState.u8RockDir;
	x = pKeyIn->unKeyMixIn.stRockState.u16RockXValue >> 1;
	y = pKeyIn->unKeyMixIn.stRockState.u16RockYValue >> 1;
	
	x >>= 2;
	y >>= 2;
	pBuf[_BJZS_Special] = BJZS_ALT;
	if ((u8Dir & (_YNA_CAM_LEFT | _YNA_CAM_UP | _YNA_CAM_RIGHT | _YNA_CAM_DOWN)) == 0)
	{
		if (u8DirPTState == 2)
		{
			pBuf[_BJZS_Key] = 0x00;
			pBuf[_BJZS_Extern] = 0x00;
			u8DirPTState = 1;			
		}
	}
	else if ((u8Dir & (_YNA_CAM_LEFT | _YNA_CAM_UP)) == (_YNA_CAM_LEFT | _YNA_CAM_UP))
	{
		pBuf[_BJZS_Key] = 0x25;
		pBuf[_BJZS_Extern] = (y << 4) | x;
		u8DirPTState = 2;
	}
	else if ((u8Dir & (_YNA_CAM_RIGHT | _YNA_CAM_UP)) == (_YNA_CAM_RIGHT | _YNA_CAM_UP))
	{
		pBuf[_BJZS_Key] = 0x26;
		pBuf[_BJZS_Extern] = (y << 4) | x;
		u8DirPTState = 2;
	}
	else if ((u8Dir & (_YNA_CAM_LEFT | _YNA_CAM_DOWN)) == (_YNA_CAM_LEFT | _YNA_CAM_DOWN))
	{
		pBuf[_BJZS_Key] = 0x27;
		pBuf[_BJZS_Extern] = (y << 4) | x;
		u8DirPTState = 2;
	}
	else if ((u8Dir & (_YNA_CAM_RIGHT | _YNA_CAM_DOWN)) == (_YNA_CAM_RIGHT | _YNA_CAM_DOWN))
	{
		pBuf[_BJZS_Key] = 0x28;
		pBuf[_BJZS_Extern] = (y << 4) | x;
		u8DirPTState = 2;
	}
	else
	{
		if (u8Dir & _YNA_CAM_LEFT)
		{
			pBuf[_BJZS_Key] = 0x25;
			pBuf[_BJZS_Extern] = x;			
			u8DirPTState = 2;
		}
		else if(u8Dir & _YNA_CAM_RIGHT) 
		{
			pBuf[_BJZS_Key] = 0x27;
			pBuf[_BJZS_Extern] = x;						
			u8DirPTState = 2;
		}
		
		if (u8Dir & _YNA_CAM_UP)
		{
			pBuf[_BJZS_Key] = 0x26;
			pBuf[_BJZS_Extern] = y;			
			u8DirPTState = 2;
		}
		else if (u8Dir & _YNA_CAM_DOWN)
		{
			pBuf[_BJZS_Key] = 0x28;
			pBuf[_BJZS_Extern] = y;						
			u8DirPTState = 2;
		}
	}
	
	
	if (u8DirPTState != 0)
	{
		CopyToUartMessage(pBuf, PROTOCOL_BJZS_LENGTH);
		FlushHIDMsgForBJZS(pBuf, PROTOCOL_BJZS_LENGTH);

		if (u8DirPTState == 1)
		{
			u8DirPTState = 0;
		}
	}
		
	return true;
}


static bool PushPodProcessForBJZS(StKeyMixIn *pKeyIn)
{
	u8 u8BJZSBuf[4];
	u8 *pBuf = u8BJZSBuf;
	u16 u16Value = pKeyIn->unKeyMixIn.u32PushRodValue;
	memset(pBuf, 0, PROTOCOL_BJZS_LENGTH);

	pBuf[_BJZS_Special] = BJZS_ALT;
	pBuf[_BJZS_Key] = 0x30;
	
	u16Value = u16Value * 0xFF / PUSH_ROD_MAX_VALUE;
	
	pBuf[_BJZS_Extern] = u16Value;

	CopyToUartMessage(pBuf, PROTOCOL_BJZS_LENGTH);
	FlushHIDMsgForBJZS(pBuf, PROTOCOL_BJZS_LENGTH);
	return true;
}

static bool CodeSwitchProcessForBJZS(StKeyMixIn *pKeyIn)
{
	u8 u8BJZSBuf[4];
	u8 *pBuf;
	u16 u16Index;

	pBuf = u8BJZSBuf;


	memset(pBuf, 0, PROTOCOL_BJZS_LENGTH);

	u16Index = pKeyIn->unKeyMixIn.stCodeSwitchState.u16Index;
	switch (u16Index)
	{
		case 0x00:
		{
			pBuf[_BJZS_Key] = 0x6F;
			break;
		}
		case 0x01:
		{
			pBuf[_BJZS_Key] = 0x6A;
			break;
		}
		default:
			return false;

	}
	
	pBuf[_BJZS_Special] = BJZS_SHIFT;
	pBuf[_BJZS_Extern] = pKeyIn->unKeyMixIn.stCodeSwitchState.u16Cnt;

	CopyToUartMessage(pBuf, PROTOCOL_BJZS_LENGTH);
	FlushHIDMsgForBJZS(pBuf, PROTOCOL_BJZS_LENGTH);

	return true;
}


static PFun_KeyProcess s_KeyProcessForBJZSArr[_Key_Reserved] = 
{
	PushPodProcessForBJZS, KeyBoardProcessForBJZS, RockProcessForBJZS, MiniRockProcessForBJZS,
	NULL, CodeSwitchProcessForBJZS, 
};




bool RockProcessForMIDIWithID(StKeyMixIn *pKeyIn, u32 u32JoyStickID)
{
	u8 u8JoyStickBuf[4] = {0};
	u8 u8JoyStickBufWithID[4] = {0};
	s8 *pBuf = (s8 *)(u8JoyStickBuf);
	s16 s16Tmp = 0;
	u8 u8Dir = 0;

	
	u8Dir = pKeyIn->unKeyMixIn.stRockState.u8RockDir;
	
	if ((u8Dir & _YNA_CAM_LEFT) != 0)
	{
		s16Tmp = (s16)(pKeyIn->unKeyMixIn.stRockState.u16RockXValue);
		s16Tmp = 0 - (s16)(pKeyIn->unKeyMixIn.stRockState.u16RockXValue);

		pBuf[0] = (s8)s16Tmp;
	}
	else if ((u8Dir & _YNA_CAM_RIGHT) != 0)
	{
		s16Tmp = (s16)(pKeyIn->unKeyMixIn.stRockState.u16RockXValue);

		pBuf[0] = (s8)s16Tmp;	
	}

	if ((u8Dir & _YNA_CAM_DOWN) != 0)
	{
		s16Tmp = (s16)(pKeyIn->unKeyMixIn.stRockState.u16RockYValue);

		pBuf[1] = (s8)s16Tmp;
	}
	else if ((u8Dir & _YNA_CAM_UP) != 0)
	{
		s16Tmp = (s16)(pKeyIn->unKeyMixIn.stRockState.u16RockYValue);
		s16Tmp = 0 - (s16)(pKeyIn->unKeyMixIn.stRockState.u16RockYValue);

		pBuf[1] = (s8)s16Tmp;	
	}

	if ((u8Dir & _YNA_CAM_WIDE) != 0)
	{
		s16Tmp = (s16)(pKeyIn->unKeyMixIn.stRockState.u16RockZValue);
		s16Tmp = 0 - (s16)(pKeyIn->unKeyMixIn.stRockState.u16RockZValue);

		pBuf[2] = (s8)s16Tmp;
	}
	else if ((u8Dir & _YNA_CAM_TELE) != 0)
	{
		s16Tmp = (s16)(pKeyIn->unKeyMixIn.stRockState.u16RockZValue);

		pBuf[2] = (s8)s16Tmp;	
	}
			
	if (IsUSBDeviceConnect())
	{
		u8JoyStickBufWithID[0] = u32JoyStickID;
		u8JoyStickBufWithID[1] = u8JoyStickBuf[0];
		u8JoyStickBufWithID[2] = u8JoyStickBuf[1];
		u8JoyStickBufWithID[3] = u8JoyStickBuf[2];
		
		CopyToUSBMessage(u8JoyStickBufWithID, 4, _IO_USB_ENDP2);
	}				
	
	return true;
}

bool RockProcessForMIDI(StKeyMixIn *pKeyIn)
{
	return RockProcessForMIDIWithID(pKeyIn, JOYSTICK_REPORT_ID);
}

bool MINIRockProcessForMIDI(StKeyMixIn *pKeyIn)
{
	return RockProcessForMIDIWithID(pKeyIn, JOYSTICK_MINI_REPORT_ID);
}


static bool PushPodProcessForMIDI(StKeyMixIn *pKeyIn)
{
	u8 u8Midi[4] = {0x0B, 0xB0, 0x0E};
	u16 u16Value = pKeyIn->unKeyMixIn.u32PushRodValue;
	
	u8Midi[1] |= (g_u8MIDIChannel & 0x0F);

	
	u16Value = u16Value * 0x7F / PUSH_ROD_MAX_VALUE;
	
	u8Midi[3] = u16Value;

	if (IsUSBDeviceConnect())
	{
		CopyToUSBMessage(u8Midi, 4, _IO_USB_ENDP1);	
	}				

	return true;
}


static bool CodeSwitchProcessForMIDI(StKeyMixIn *pKeyIn)
{
	
	
	u8 u8Midi[4] = {0x0B, 0xB0, 0x07};
	
	u8Midi[1] |= (g_u8MIDIChannel & 0x0F);

	
	u8Midi[2] = pKeyIn->unKeyMixIn.stCodeSwitchState.u16Index + 0x07;
	if (pKeyIn->unKeyMixIn.stCodeSwitchState.u16Dir)
	{
		u8Midi[3] = pKeyIn->unKeyMixIn.stCodeSwitchState.u16CWCnt;	
		u8Midi[3] += 0x40;
	}
	else
	{
		u8Midi[3] = pKeyIn->unKeyMixIn.stCodeSwitchState.u16CCWCnt;	
	}

	if (IsUSBDeviceConnect())
	{
		CopyToUSBMessage(u8Midi, 4, _IO_USB_ENDP1);	
	}				

	return true;	
}


#define MIDI_KEY_BEGIN		0x20


static bool KeyBoardProcessForMIDI(StKeyMixIn *pKeyIn)
{
	u32 i;

	for (i = 0; i < pKeyIn->u32Cnt; i++)
	{
		StKeyState *pKeyState = pKeyIn->unKeyMixIn.stKeyState + i;
		
		if ((pKeyState->u8KeyValue >= _Key_Ctrl_Reserved_Inner1) ||
			(pKeyState->u8KeyValue < _Key_Ctrl_Begin))
		{
			continue;			
		}
		
		if (pKeyState->u8KeyState == KEY_KEEP)
		{
			continue;
		}
		else
		{
			u8 u8Midi[4] = {0x09, 0x90, 0x00, 0x7F};
			
			
			if (pKeyState->u8KeyState == KEY_UP)
			{
				u8Midi[0] = 0x08; 
				u8Midi[1] = 0x80; 
				u8Midi[3] = 0;			
			}

			u8Midi[1] |= (g_u8MIDIChannel & 0x0F);
			u8Midi[2] = pKeyState->u8KeyValue - _Key_Ctrl_Begin + MIDI_KEY_BEGIN;
			
			if (IsUSBDeviceConnect())
			{
				CopyToUSBMessage(u8Midi, 4, _IO_USB_ENDP1);	
			}				

		}

	}
	return true;
}
static PFun_KeyProcess s_KeyProcessForMIDIArr[_Key_Reserved] = 
{
	PushPodProcessForMIDI, KeyBoardProcessForMIDI, RockProcessForMIDI, MINIRockProcessForMIDI,
	NULL, CodeSwitchProcessForMIDI,
};

bool KeyProcess(StIOFIFO *pFIFO)
{
	StKeyMixIn *pKeyIn = pFIFO->pData;
	
	if (pKeyIn->emKeyType >= _Key_Reserved)
	{
		return false;
	}
	
	if (s_KeyProcessForMIDIArr[pKeyIn->emKeyType] != NULL)
	{
		s_KeyProcessForMIDIArr[pKeyIn->emKeyType](pKeyIn);		
	}

	if (g_emProtocol == _Protocol_YNA)
	{
		if (s_KeyProcessArr[pKeyIn->emKeyType] != NULL)
		{
			return s_KeyProcessArr[pKeyIn->emKeyType](pKeyIn);	
		}
	}
	else
	{
		if (s_KeyProcessForBJZSArr[pKeyIn->emKeyType] != NULL)
		{
			return s_KeyProcessForBJZSArr[pKeyIn->emKeyType](pKeyIn);
		}			
	}
	

	return false;
}



bool PCEchoProcessYNA(StIOFIFO *pFIFO)
{
	u8 *pMsg;
	u8 u8Cmd;
	u8 u8Array;
	bool boIsLight;

	if (pFIFO == NULL)
	{
		return -1;
	}
	pMsg = (u8 *)pFIFO->pData;	
	u8Cmd = pMsg[_YNA_Cmd];
	
	u8Array = pMsg[_YNA_Data1] >> 4;
	
	pMsg[_YNA_Data1] &= 0x0F;
	
	boIsLight = !pMsg[_YNA_Data3];

	if (pMsg[_YNA_Mix] == 0x06)
	{
		return true;
	}

	switch (u8Cmd)
	{
		case 0x40:
		{
			if ((u8Array == 0) && (pMsg[_YNA_Data2] < 8))
			{
				const u16 u16Led[][8] = 
				{
					{
						_Led_Cam_Ctrl_Present1, _Led_Cam_Ctrl_Present2, 
						_Led_Cam_Ctrl_Present3, _Led_Cam_Ctrl_Present4,
						_Led_Cam_Ctrl_Present5, _Led_Cam_Ctrl_Present6, 
						_Led_Cam_Ctrl_Present7, _Led_Cam_Ctrl_Present8,
					},
				};
				ChangeLedArrayState(u16Led[u8Array], 8, false);
				ChangeLedState(GET_XY(u16Led[u8Array][pMsg[_YNA_Data2]]), boIsLight);
				
			}
			else if ((u8Array < 4) && (pMsg[_YNA_Data2] < 4))
			{
				const u16 u16Led[][4] = 
				{
					{
						_Led_Ctrl_Machine_Switch1, _Led_Ctrl_Machine_Switch2, 
						_Led_Ctrl_Machine_Switch3, _Led_Ctrl_Machine_Switch4
					},
					{
						_Led_SlowMove_Machine_Choose1, _Led_SlowMove_Machine_Choose2, 
						_Led_SlowMove_Machine_Choose3, _Led_SlowMove_Machine_Choose4
					},
					{
						_Led_AI_Present1, _Led_AI_Present2, 
						_Led_AI_Present3, _Led_AI_Present4
					},
				
				};
				u8Array = u8Array - 1;
				ChangeLedArrayState(u16Led[u8Array], 4, false);
				ChangeLedState(GET_XY(u16Led[u8Array][pMsg[_YNA_Data2]]), boIsLight);

			}
			break;
		}

		case 0x45:
		{
			break;
		}
		case 0x46:
		{
			break;
		}
		case 0x47:
		{
			u8 u8Led = pMsg[_YNA_Data2];
			switch (u8Led)
			{
				case 0x00:
					ChangeLedState(GET_XY(_Led_Make_Live), boIsLight);
					break;
				case 0x02:
					ChangeLedState(GET_XY(_Led_Make_Record), boIsLight);
					break;
				case 0x05: case 0x06:
				{
					const u16 u16Led[][2] = 
					{
						{_Led_SlowMove_Record_Start, _Led_SlowMove_Record_Stop},
						{_Led_SlowMove_Make_Start, _Led_SlowMove_Make_Stop},
						{_Led_SlowMove_Replay_Start, _Led_SlowMove_Replay_Stop},						
					};
					if (u8Array < 3)
					{
						ChangeLedArrayState(u16Led[u8Array], 2, false);
						ChangeLedState(GET_XY(u16Led[u8Array][pMsg[_YNA_Data2] - 0x05]), boIsLight);

					}
					break;
				}
				case 0x40:
					ChangeLedState(GET_XY(_Led_Make_FullScreen), boIsLight);
					break;
				case 0x41:
					ChangeLedState(GET_XY(_Led_Make_MutliRecord), boIsLight);
					break;
				default:
					break;
			}

			break;
		}
		case 0x48:
		{
			const u16 u16LedPGM[] = 
			{
				_Led_PGM_1, _Led_PGM_2, _Led_PGM_3, _Led_PGM_4,
				_Led_PGM_5, _Led_PGM_6, _Led_PGM_7, _Led_PGM_8,
				_Led_PGM_9, _Led_PGM_10, _Led_PGM_11, _Led_PGM_12,
			};
			const u16 u16LedPVW[] = 
			{
				_Led_PVW_1, _Led_PVW_2, _Led_PVW_3, _Led_PVW_4,
				_Led_PVW_5, _Led_PVW_6, _Led_PVW_7, _Led_PVW_8,
				_Led_PVW_9, _Led_PVW_10, _Led_PVW_11, _Led_PVW_12,
			};

			u8 u8Led = pMsg[_YNA_Data2];
			switch (u8Led)
			{
				case 0x00:
				{
					break;
				}
				case 0x02: case 0x03: case 0x04: case 0x05:
				case 0x06: case 0x07:
				{
					ChangeLedArrayState(u16LedPGM, sizeof(u16LedPGM) / sizeof(u16), false);
					ChangeLedState(GET_XY(u16LedPGM[pMsg[_YNA_Data2] - 0x02]), boIsLight);
					if (pMsg[_YNA_Data1] == 1)
					{
						SetTallyPGM(pMsg[_YNA_Data2] - 0x02, boIsLight, true, true);
					}
					break;
				}
				case 0x08: case 0x09: case 0x0A: case 0x0B:
				case 0x0C: case 0x0D: 
				{
					ChangeLedArrayState(u16LedPVW, sizeof(u16LedPVW) / sizeof(u16), false);
					ChangeLedState(GET_XY(u16LedPVW[pMsg[_YNA_Data2] - 0x08]), boIsLight);
					
					if (pMsg[_YNA_Data1] == 1)
						SetTallyPVW(pMsg[_YNA_Data2] - 0x08, boIsLight, true, true);
					break;
				}
				case 0x0E: case 0x0F: case 0x10: case 0x11:
				case 0x12: case 0x13: case 0x14: case 0x15:
				{
					const u16 u16Led[] = 
					{
						_Led_Effect_1,
						_Led_Effect_2,
						_Led_Effect_3,
						_Led_Effect_4,
						_Led_Effect_5,
						_Led_Effect_6,
						_Led_Effect_7,
						_Led_Effect_8,
					};
					ChangeLedArrayState(u16Led, sizeof(u16Led) / sizeof(u16), false);
					ChangeLedState(GET_XY(u16Led[pMsg[_YNA_Data2] - 0x0E]), boIsLight);
					break;
				}
				case 0x80: case 0x81: case 0x82: case 0x83:
				case 0x84: case 0x85:
				{
					ChangeLedArrayState(u16LedPGM, sizeof(u16LedPGM) / sizeof(u16), false);
					ChangeLedState(GET_XY(u16LedPGM[pMsg[_YNA_Data2] - 0x80 + 6]), boIsLight);
					if (pMsg[_YNA_Data1] == 1)
						SetTallyPGM(pMsg[_YNA_Data2] - 0x80 + 6, boIsLight, true, true);

					break;
				}
				case 0x90: case 0x91: case 0x92: case 0x93:
				case 0x94: case 0x95:
				{
					ChangeLedArrayState(u16LedPVW, sizeof(u16LedPVW) / sizeof(u16), false);
					ChangeLedState(GET_XY(u16LedPVW[pMsg[_YNA_Data2] - 0x90 + 6]), boIsLight);
					if (pMsg[_YNA_Data1] == 1)
						SetTallyPVW(pMsg[_YNA_Data2] - 0x90 + 6, boIsLight, true, true);
					break;
				}
				default:
					break;
			}
			break;
		}
		case 0x49:
		{
			break;
		}
		case 0x4A:
		{
			break;
		}
		case 0x4B:
		{
			u8 u8Led = pMsg[_YNA_Data2];
			if ((u8Led < 5) && (u8Led != 0))
			{
				const u16 u16Led[] = 
				{
					_Led_Overlay_1,
					_Led_Overlay_2,
					_Led_Overlay_3,
					_Led_Overlay_4,
				};
				ChangeLedArrayState(u16Led, sizeof(u16Led) / sizeof(u16), false);
				ChangeLedState(GET_XY(u16Led[u8Led - 1]), boIsLight);

			}

			break;
		}
		case 0x4C:
		{
			u8 u8Led = pMsg[_YNA_Data2];
			switch (u8Led)
			{
				case 0x60: case 0x61:
				case 0x62: case 0x63:
				case 0x64: case 0x65:
				case 0x66:
				{
					const u16 u16Led[] = 
					{
						_Led_Ctrl_Machine_Up,
						_Led_Ctrl_Machine_Down,
						_Led_AI_Ctrl_BuildPic,
						_Led_AI_Walk_Stop,
						_Led_AI_TP_Charge,
						_Led_AI_Fun1,
						_Led_AI_Fun2,
						
					};
					
					ChangeLedState(GET_XY(u16Led[pMsg[_YNA_Data2] - 0x60]), boIsLight);

					break;
				}
				default:
					break;
			}
			break;
		}
		case 0x80:
		{
			break;
		}
		case 0xC0:
		{
			if (pMsg[_YNA_Data2] == 0x01)
			{
				switch (pMsg[_YNA_Data3])
				{
					case 0x00:
					{
						u8 *pBuf = u8YNABuf;
						if (pBuf == NULL)
						{
							return false;
						}

						memset(pBuf, 0, PROTOCOL_YNA_ENCODE_LENGTH);

						pBuf[_YNA_Sync] = 0xAA;
						pBuf[_YNA_Addr] = g_u8CamAddr;
						pBuf[_YNA_Mix] = 0x07;
						pBuf[_YNA_Cmd] = 0xC0;
						pBuf[_YNA_Data2] = 0x01;
						YNAGetCheckSum(pBuf);
						CopyToUartMessage(pBuf, PROTOCOL_YNA_DECODE_LENGTH);
						break;
					}
					case 0x02:
					{
						ChangeAllLedState(false);
						/* maybe we need turn on some light */
						GlobalStateInit();
						break;
					}
					case 0x03:
					{
						u8 *pBuf = u8YNABuf;
						if (pBuf == NULL)
						{
							return false;
						}

						memset(pBuf, 0, PROTOCOL_YNA_ENCODE_LENGTH);

						pBuf[_YNA_Sync] = 0xAA;
						pBuf[_YNA_Addr] = g_u8CamAddr;
						pBuf[_YNA_Mix] = 0x07;
						pBuf[_YNA_Cmd] = 0x80;
						pBuf[_YNA_Data2] = PushRodGetCurValue();
						YNAGetCheckSum(pBuf);
						CopyToUartMessage(pBuf, PROTOCOL_YNA_DECODE_LENGTH);
						break;
					}
				
					default:
						return false;
				}
			}				
			break;
		}
		default :
			return false;
		
	}

	return true;
}

bool PCEchoProcessForBJZS(StIOFIFO *pFIFO)
{
	return true;
}

bool PCEchoProcessForHIDBJZS(StIOFIFO *pFIFO)
{
	return true;	
}

const u16 c_u16LedArrForMIDI[] =
{
	_Led_Make_Record,
	_Led_Make_Live,
	_Led_Make_FullScreen,
	_Led_Make_MutliRecord,
	
	_Led_Ctrl_Machine_Switch1,
	_Led_Ctrl_Machine_Switch2,
	_Led_Ctrl_Machine_Switch3,
	_Led_Ctrl_Machine_Switch4,

	_Led_Ctrl_Machine_Up,
	_Led_Ctrl_Machine_Down,

	_Led_SlowMove_Machine_Choose1,
	_Led_SlowMove_Machine_Choose2,
	_Led_SlowMove_Machine_Choose3,
	_Led_SlowMove_Machine_Choose4,

	_Led_SlowMove_Record_Start,
	_Led_SlowMove_Record_Stop,
	_Led_SlowMove_Make_Start,
	_Led_SlowMove_Make_Stop,
	_Led_SlowMove_Replay_Start,
	_Led_SlowMove_Replay_Stop,
	
	
	_Led_AI_Present1,
	_Led_AI_Present2,
	_Led_AI_Present3,
	_Led_AI_Present4,
	
	_Led_AI_Ctrl_BuildPic,
	_Led_AI_Walk_Stop,
	_Led_AI_TP_Charge,
	_Led_AI_Fun1,
	_Led_AI_Fun2,
	
	_Led_PGM_1,				
	_Led_PGM_2,
	_Led_PGM_3,
	_Led_PGM_4,
	_Led_PGM_5,
	_Led_PGM_6,
	_Led_PGM_7,
	_Led_PGM_8,
	_Led_PGM_9,
	_Led_PGM_10,
	_Led_PGM_11,
	_Led_PGM_12,		/* 24 */


	_Led_PVW_1,
	_Led_PVW_2,
	_Led_PVW_3,
	_Led_PVW_4,
	_Led_PVW_5,
	_Led_PVW_6,
	_Led_PVW_7,
	_Led_PVW_8,
	_Led_PVW_9,
	_Led_PVW_10,
	_Led_PVW_11,
	_Led_PVW_12,		/* 36 */	

	_Led_Cam_Ctrl_Tele,
	_Led_Cam_Ctrl_Wide,	

	_Led_Cam_Ctrl_Present1,
	_Led_Cam_Ctrl_Present2,
	_Led_Cam_Ctrl_Present3,
	_Led_Cam_Ctrl_Present4,
	_Led_Cam_Ctrl_Present5,
	_Led_Cam_Ctrl_Present6,
	_Led_Cam_Ctrl_Present7,
	_Led_Cam_Ctrl_Present8,


	_Led_Effect_1,
	_Led_Effect_2,
	_Led_Effect_3,
	_Led_Effect_4,
	_Led_Effect_5,
	_Led_Effect_6,		
	_Led_Effect_7,		
	_Led_Effect_8,		
	
	_Led_Overlay_1,
	_Led_Overlay_2,
	_Led_Overlay_3,
	_Led_Overlay_4,
	
	0xFFFF,
	0xFFFF,
	0xFFFF,
};
bool PCEchoProcessForMIDI(StIOFIFO *pFIFO)
{
	u8 *pMsg;
	u8 u8Cmd;
	u8 u8Key;
	bool boIsLightON = true;

	if (pFIFO == NULL)
	{
		return false;
	}

	pMsg = (u8 *)pFIFO->pData;
	u8Cmd = (pMsg[1] & 0xF0);

	if ((pMsg[1] & 0x0F) != g_u8MIDIChannel)
	{
		return false;
	}

	if ( u8Cmd == 0x80)
	{
		boIsLightON = false;
	}
	else if ((u8Cmd == 0x90) || (u8Cmd == 0xB0))
	{
		if ((pMsg[3] & 0x7F) == 0)
		{
			boIsLightON = false;
		}
	}
	else
	{
		return false;
	}
		
	
	u8Key = pMsg[2];
	
	if ((u8Key < MIDI_KEY_BEGIN) || 
		(u8Key > (MIDI_KEY_BEGIN + (_Key_Ctrl_Reserved_Inner1 - _Key_Ctrl_Begin))))
	{
		return false;
	}

	u8Key = u8Key - MIDI_KEY_BEGIN + _Key_Ctrl_Begin;
	
	if ((u8Key >= _Key_Ctrl_Begin) && 
		(u8Key <_Key_Ctrl_Reserved_Inner1))
	{
		u16 u16Led = c_u16LedArrForMIDI[u8Key - _Key_Ctrl_Begin];
		if (u16Led != 0xFFFF)
		{
			ChangeLedState(GET_XY(u16Led), boIsLightON);
		}


#if 0		
		if (u8Key >= _Key_PGM_1 && u8Key <= _Key_PGM_4)
		{
			u8Key -= _Key_PGM_1;
			ExternIOCtrl(u8Key, boIsLightON ? Bit_SET : Bit_RESET);
		}
		else if(u8Key >= _Key_PVW_1 && u8Key <= _Key_PVW_4)
		{
			u8Key -= _Key_PVW_1;
			ExternIOCtrl(u8Key + 8, boIsLightON ? Bit_SET : Bit_RESET);
		}
#endif
		
	}
	
	return true;
}
bool PCEchoProcess(StIOFIFO *pFIFO)
{
	if (pFIFO->u8ProtocolType == _Protocol_YNA)
	{
		return PCEchoProcessYNA(pFIFO);
	}
	else if (pFIFO->u8ProtocolType == _Protocol_BJZS)
	{
		return PCEchoProcessForBJZS(pFIFO);
	}
	else if (pFIFO->u8ProtocolType == _Protocol_BJZS_HID)
	{
		return PCEchoProcessForHIDBJZS(pFIFO);
	}
	else if (pFIFO->u8ProtocolType == _Protocol_MIDI)
	{
		return PCEchoProcessForMIDI(pFIFO);
	}
	
	return false;
}


