/*------------------------------------------------------------------------------------------
 * INTRODUCE
 * Brief: This is source file of task command.
 * Author: Duy Vinh To - Team DinhViSo
 * Detail: process commands.
 -------------------------------------------------------------------------------------------*/

/*##########################################################################################
 * INCLUDE
 *##########################################################################################*/
#include "taskCmd.h"
/* C libraries */
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
/* DVS hardware libraries */
#include "STM32F1_flash.h"
#include "STM32F1_uart.h"
#include "STM32F1_rtc.h"
#include "GSM.h"
/* DVS software libraries */
#include "fileSystem.h"
#include "dbgPrint.h"
#include "cli.h"
#include "memMngr.h"
#include "stringlib.h"
#include "netService.h"
/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
/* Other tasks */
#include "taskMain.h"
#include "taskNetworkComm.h"
#include "taskBkpTrkDat.h"
#include "taskLogging.h"
#include "taskRecTrkDat.h"
#include "taskSens.h"
#include "taskCamera.h"
/* Others */
#include "devcfg.h"
#include "devstt.h"
#include "taskNetworkComm.h"
#include "HWmapping.h"
#include "usedFlashPages.h"
#include "FWversion.h"
#include "deviceConfig.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "pb_common.h"
/*##########################################################################################
 * DEFINES
 *##########################################################################################*/
#define BACKSPACE_KEY	0x8
#define CMD_SIZE		256

/*##########################################################################################
 * TYPEDEFS
 *##########################################################################################*/
typedef struct
{
	EXTSRC_CMD_TYPE_t exSrcType;
	uint8_t *src; // Source. It can be number (if source is SMS) or host name/IP (SSH)...
	uint8_t *content;
} EXTSRC_CMD_t; // command from external source: SMS, SSH connection...

/*##########################################################################################
 * VARIABLES
 *##########################################################################################*/
static xQueueHandle extSrcCmdQ = NULL;

static uint8_t *CMD_buf = NULL;
static uint16_t CMD_len = 0;
static uint8_t CMD_got = 0;
/* Command list */
static const uint8_t CMD_RQI_DEVSTT[] = "RQI DEVSTT";
static const uint8_t CMD_RQI_FW[] = "RQI FW";
static const uint8_t CMD_IT_UPGFW[] = "IT_UPGFW";
static const uint8_t CMD_IT_TRACE[] = "IT_TRACE";
static const uint8_t CMD_IT_ECHO[] = "IT_ECHO";
static const uint8_t CMD_IT_DBGGSM[] = "IT_DBGGSM";
static const uint8_t CMD_IT_DBGGNSS[] = "IT_DBGGNSS";
static const uint8_t CMD_IT_SETRBD[] = "IT_SETRBD";
static const uint8_t CMD_IT_LISTCMD[] = "HELP"; // Easy to remember
static const uint8_t CMD_IT_SENDSMS[] = "IT_SENDSMS";
static const uint8_t CMD_CFG[] = "CFG ";
static const uint8_t CMD_GETCFG[] = "GETCFG ";
static const uint8_t CMD_IT_STRESSTEST[] = "IT_STRESSTEST";
static const uint8_t CMD_IT_SENDFILE[] = "IT_SENDFILE";
static const uint8_t CMD_IT_RESET[] = "IT_RESET";
static const uint8_t CMD_IT_RST_EGADCMIN[] = "IT_RST_EGADCMIN";
static const uint8_t CMD_APN[] = "APN ";
/*##########################################################################################
 * FUNC.PROTOTYPES
 *##########################################################################################*/
static void wrtUpgFWver(uint16_t mainVer, uint16_t subVer);
static void calcCRC_bkpRegVal(uint16_t *val);
static void TSKCMD_main(void *pdata);
static void UART1_readc(uint8_t c);
/* Command handlers */
static void TSKCMD_hdl_RQI_DEVSTT(uint8_t *s, uint16_t oSize, uint8_t *o);
static void TSKCMD_hdl_RQI_FW(uint8_t *s, uint16_t oSize, uint8_t *o);
static void TSKCMD_hdl_IT_UPGFW(uint8_t *s, uint16_t oSize, uint8_t *o);
static void TSKCMD_hdl_IT_TRACE(uint8_t *s, uint16_t oSize, uint8_t *o);
static void TSKCMD_hdl_IT_ECHO(uint8_t *s, uint16_t oSize, uint8_t *o);
static void TSKCMD_hdl_IT_DBGGSM(uint8_t *s, uint16_t oSize, uint8_t *o);
static void TSKCMD_hdl_IT_DBGGNSS(uint8_t *s, uint16_t oSize, uint8_t *o);
static void TSKCMD_hdl_IT_SETRBD(uint8_t *s, uint16_t oSize, uint8_t *o);
static void TSKCMD_hdl_IT_LISTCMD(uint8_t *s, uint16_t oSize, uint8_t *o);
static void TSKCMD_hdl_CFG(uint8_t *s, uint16_t oSize, uint8_t *o);
static void TSKCMD_hdl_GETCFG(uint8_t *s, uint16_t oSize, uint8_t *o);
static void TSKCMD_hdl_CFG_GETCFG(uint8_t isGetCfg, uint8_t *s, uint16_t oSize, uint8_t *o);
static void TSKCMD_hdl_IT_SENDSMS(uint8_t *s, uint16_t oSize, uint8_t *o);
static void TSKCMD_hdl_IT_STRESSTEST(uint8_t *s, uint16_t oSize, uint8_t *o);
static void TSKCMD_hdl_IT_SENDFILE(uint8_t *s, uint16_t oSize, uint8_t *o);
static void TSKCMD_hdl_IT_RESET(uint8_t *s, uint16_t oSize, uint8_t *o);
static void TSKCMD_hdl_IT_RST_EGADCMIN(uint8_t *s, uint16_t oSize, uint8_t *o);
static void TSKCMD_hdl_APN(uint8_t *s, uint16_t oSize, uint8_t *o);
/*##########################################################################################
 * FUNCTIONS
 *##########################################################################################*/
/*------------------------------------------------------------------------------------------
 * Brief: calculate CRC of a backup register value.
 * Param: val	|	IN	|	<>
 * Ret:		0	|	OK
 * 			1	|	FAIL
 -------------------------------------------------------------------------------------------*/
void calcCRC_bkpRegVal(uint16_t *val)
{
	uint8_t crc = ((*val >> 8) & 0xF) ^ ((*val >> 4) & 0xF) ^ (*val & 0xF);
	*val &= 0x0FFF;
	*val |= crc << 12;
}
/*------------------------------------------------------------------------------------------
 * Brief: write planned upgrading version.
 * Param:	mainVer	|	IN	|	<>
 * 			subVer	|	IN	|	<>
 -------------------------------------------------------------------------------------------*/
void wrtUpgFWver(uint16_t mainVer, uint16_t subVer)
{
	IRTC_enableBackupAccess();
	calcCRC_bkpRegVal(&mainVer);
	BKP_WriteBackupRegister(BKPDR_UPGFW_MAINVER, mainVer);
	calcCRC_bkpRegVal(&subVer);
	BKP_WriteBackupRegister(BKPDR_UPGFW_SUBVER, subVer);
}
/*----------------------------------------------------------------------------------------
 * Brief: Add external source command
 * Param:	type	|	IN	|	source type. View EXTSRC_CMD_TYPE_t for more details.
 * 			src		|	IN	|	Source. It can be number (if source is SMS) or host name/IP (SSH)...
 * 			content	|	IN	|	Command content.
 * Ret:		0	|	OK
 * 			>0	|	FAIL
 -----------------------------------------------------------------------------------------*/
uint8_t TSKCMD_addExtSrcCMD(EXTSRC_CMD_TYPE_t type, const uint8_t *src, const uint8_t *content)
{
	if (extSrcCmdQ == NULL)
	{
		return 0xFF;
	}
	if (!strlen(content))
	{
		return 0;
	}
	EXTSRC_CMD_t newCmd = {type, NULL, NULL};
	newCmd.src = MM_get(60, strlen(src) + 1);
	newCmd.content = MM_get(60, strlen(content) + 1);
	if ((newCmd.src == NULL) || (newCmd.content == NULL))
	{
		MM_free(newCmd.src);
		MM_free(newCmd.content);
		return 1;
	}
	strlcpy(newCmd.src, src, strlen(src) + 1);
	strlcpy(newCmd.content, content, strlen(content) + 1);
	if (xQueueSend(extSrcCmdQ, &newCmd, 0) != pdTRUE)
	{
		MM_free(newCmd.src);
		MM_free(newCmd.content);
		return 2;
	}
	return 0;
}
/*----------------------------------------------------------------------------------------
 * Brief: setup task handling commands.
 * Param:	size	|	IN	|	task size.
 * 			prio	|	IN	|	task priority.
 * 			tskHdl	|	IO	|	task handler.
 * Ret:	0	|	OK
 * 		>0	|	FAIL
 -----------------------------------------------------------------------------------------*/
uint8_t TSKCMD_setup(uint16_t size, uint32_t prio, xTaskHandle *tskHdl, uint32_t *wotime)
{
	portBASE_TYPE res = xTaskCreate(TSKCMD_main, "CMD", size, NULL, prio, tskHdl);
	if (res == pdTRUE)
	{
		extSrcCmdQ = xQueueCreate(EXTSRC_CMD_QUEUE_SIZE, sizeof(EXTSRC_CMD_t));
	}
	return (res == pdTRUE) ? 0 : 1;
}
/*----------------------------------------------------------------------------------------
 * Brief: logging task: main function.
 * To-do-list:
 -----------------------------------------------------------------------------------------*/
void TSKCMD_main(void *pdata)
{
	/* Setup CLI */
	CLI_setup();
	/* Register commands */
	CLI_regCmdHdl(CMD_RQI_DEVSTT, TSKCMD_hdl_RQI_DEVSTT);
	CLI_regCmdHdl(CMD_RQI_FW, TSKCMD_hdl_RQI_FW);
	CLI_regCmdHdl(CMD_IT_UPGFW, TSKCMD_hdl_IT_UPGFW);
	CLI_regCmdHdl(CMD_IT_TRACE, TSKCMD_hdl_IT_TRACE);
	CLI_regCmdHdl(CMD_IT_ECHO, TSKCMD_hdl_IT_ECHO);
	CLI_regCmdHdl(CMD_IT_DBGGSM, TSKCMD_hdl_IT_DBGGSM);
	CLI_regCmdHdl(CMD_IT_DBGGNSS, TSKCMD_hdl_IT_DBGGNSS);
	CLI_regCmdHdl(CMD_IT_SETRBD, TSKCMD_hdl_IT_SETRBD);
	CLI_regCmdHdl(CMD_IT_LISTCMD, TSKCMD_hdl_IT_LISTCMD);
	CLI_regCmdHdl(CMD_CFG, TSKCMD_hdl_CFG);
	CLI_regCmdHdl(CMD_GETCFG, TSKCMD_hdl_GETCFG);
	CLI_regCmdHdl(CMD_IT_SENDSMS, TSKCMD_hdl_IT_SENDSMS);
	CLI_regCmdHdl(CMD_IT_STRESSTEST, TSKCMD_hdl_IT_STRESSTEST);
	CLI_regCmdHdl(CMD_IT_SENDFILE, TSKCMD_hdl_IT_SENDFILE);
	CLI_regCmdHdl(CMD_IT_RESET, TSKCMD_hdl_IT_RESET);
	CLI_regCmdHdl(CMD_IT_RST_EGADCMIN, TSKCMD_hdl_IT_RST_EGADCMIN);
	CLI_regCmdHdl(CMD_APN, TSKCMD_hdl_APN);
	CMD_buf = MM_get(0, CMD_SIZE);
	if (CMD_buf != NULL)
	{
		UART_setup(1, 115200, UART1_readc);
	}
	while (1)
	{
		if (DEVSTT_access()->rdy2rst)
		{
			//vTaskDelete(NULL);
		}
		if (CMD_got || uxQueueMessagesWaiting(extSrcCmdQ))
		{
			const uint16_t respSize = 4096;
			EXTSRC_CMD_t newCmd;
			uint8_t *resp = MM_get(10, respSize);

			if (resp != NULL)
			{
				if (xQueueReceive(extSrcCmdQ, &newCmd, 0))
				{
					if (strlen(newCmd.content))
					{
						DBG_print("\r\n TSKCMD:Num=%s Cmd=\"%s\"", newCmd.src, newCmd.content);
						/* Process command */
						CLI_processCmd(newCmd.content, respSize, resp);
						/* Reply to sender */
						switch (newCmd.exSrcType)
						{
							case EXSRCCT_SMS:
								TSKNETCOMM_add_sndSMS(newCmd.src, resp);
								break;
							case EXSRCCT_NATS:
								TSKNETCOMM_add_sndFBViaNATS(resp);
								break;
							default:
								break;
						}
					}
					MM_free(newCmd.src);
					MM_free(newCmd.content);
				}
				else if (CMD_got)
				{
					/* Process command */
					CLI_processCmd(CMD_buf, respSize, resp);
					CMD_got = 0;
					CMD_len = 0;
				}
				DBG_print("\r\n %s", resp);
				MM_free(resp);
			}
		}
		vTaskDelay(50);
	}
}
/*----------------------------------------------------------------------------------------
 * Brief: readc callback function.
 -----------------------------------------------------------------------------------------*/
void UART1_readc(uint8_t c)
{
	if (SESSIONCFG_access()->enECHO)
	{
		if ((c == BACKSPACE_KEY) && (CMD_len))
		{
			UART_sendc(1, BACKSPACE_KEY);
			UART_sendc(1, ' ');
			UART_sendc(1, BACKSPACE_KEY);
		}
		else
		{
			UART_sendc(1, c);
		}
	}
	if (CMD_got)
	{
		return;
	}
	if ((c == '\r') || (c == '\n'))
	{
		if (CMD_len)
		{
			CMD_got = 1;
		}
	}
	else
	{
		if (c == BACKSPACE_KEY)
		{
			if (CMD_len)
			{
				CMD_buf[CMD_len--] = 0;
			}
		}
		else
		{
			if (CMD_len < CMD_SIZE)
			{
				CMD_buf[CMD_len++] = c;
				CMD_buf[CMD_len] = 0;
			}
		}
	}
}
/*----------------------------------------------------------------------------------------
 * Brief: Handle command request device status
 -----------------------------------------------------------------------------------------*/
void TSKCMD_hdl_RQI_DEVSTT(uint8_t *s, uint16_t oSize, uint8_t *o)
{
	DEVSTT_genRpt(oSize, o);
}
/*----------------------------------------------------------------------------------------
 * Brief: Handle command request FW version
 -----------------------------------------------------------------------------------------*/
void TSKCMD_hdl_RQI_FW(uint8_t *s, uint16_t oSize, uint8_t *o)
{
	snprintf(o, oSize, "FW:%s\r\nBD:%s", FW_VERSION, FW_BUILTDATE);
}
/*----------------------------------------------------------------------------------------
 * Brief: upgrade FW
 -----------------------------------------------------------------------------------------*/
void TSKCMD_hdl_IT_UPGFW(uint8_t *s, uint16_t oSize, uint8_t *o)
{
	uint8_t sMainVer[6], sSubVer[6];
	uint16_t mainVer = 0, subVer = 0;

	SL_split(s, '.', sMainVer, 6, sSubVer, 6);
	DBG_print("\r\n UPGFW:Main=\"%s\" Sub=\"%s\"", sMainVer, sSubVer);
	mainVer = (uint16_t)atoi(sMainVer);
	subVer = (uint16_t)atoi(sSubVer);
	wrtUpgFWver(mainVer, subVer);
	DEVSTT_access()->rdy2rst = 1;
	snprintf(o, oSize, "OK");
}
/*----------------------------------------------------------------------------------------
 * Brief: toggle trace mode
 -----------------------------------------------------------------------------------------*/
void TSKCMD_hdl_IT_TRACE(uint8_t *s, uint16_t oSize, uint8_t *o)
{
	if (!strcmp(s, "NO"))
	{
		/* Disable all tracing */
		TSKNETCOMM_setup_dbgPrt(NULL);
		GSM_setup_dbgPrt(NULL);
		NETSRVC_setup_dbgPrt(NULL);
		TSKRECTD_setup_dbgPrt(NULL);
		TSKLOG_setup_dbgPrt(NULL);
		TSKBKPTD_setup_dbgPrt(NULL);
		FS_setup_dbgPrt(NULL);
		TSKSENS_setup_dbgPrt(NULL);
		TSKMAIN_setup_dbgPrt(NULL);
		TSKCAM_setup_dbgPrt(NULL);
		MM_setup_dbgPrt(NULL);
	}
	if (!SL_startwith(s, "GSM"))
	{
		// Tracing GSM relating:
		// + GSM module drive.
		// + Task network communication.
		SL_cut(s, "GSM");
		TSKNETCOMM_setup_dbgPrt(atoi(s) ? DBG_print : NULL);
		GSM_setup_dbgPrt(atoi(s) ? DBG_print : NULL);
		NETSRVC_setup_dbgPrt(atoi(s) ? DBG_print : NULL);
	}
	else if (!SL_startwith(s, "GNSS"))
	{
		// Tracing GNSS relating:
		// + GNSS module drive.
		// + Task record tracking data.
		// ! BUT NOT debugging GPS NMEA message. Use command "IT_DBGGNSS" instead.
		SL_cut(s, "GNSS");
		QL70_setup_dbgPrt(atoi(s) ? DBG_print : NULL);
		TSKRECTD_setup_dbgPrt(atoi(s) ? DBG_print : NULL);
	}
	else if (!SL_startwith(s, "FS"))
	{
		// Tracing file system relating:
		// + FieSystem driver.
		// + Task backup tracking data.
		// + Task backup tracking data to flash.
		// + Task logging.
		SL_cut(s, "FS");
		TSKLOG_setup_dbgPrt(atoi(s) ? DBG_print : NULL);
		TSKBKPTD_setup_dbgPrt(atoi(s) ? DBG_print : NULL);
		FS_setup_dbgPrt(atoi(s) ? DBG_print : NULL);

	}
	else if (!SL_startwith(s, "SENS"))
	{
		// Tracing sensor/signal reading relating:
		// + Task sensing.
		SL_cut(s, "SENS");
		TSKSENS_setup_dbgPrt(atoi(s) ? DBG_print : NULL);
	}
	else if (!SL_startwith(s, "CAM"))
	{
		// Tracing camera related activities.
		SL_cut(s, "CAM");
		TSKCAM_setup_dbgPrt(atoi(s) ? DBG_print : NULL);
	}
	else if (!SL_startwith(s, "MEM"))
	{
		// Tracing camera related activities.
		SL_cut(s, "MEM");
		MM_setup_dbgPrt(atoi(s) ? DBG_print : NULL);
	}
	else
	{
		// Simple tracing.
		TSKMAIN_setup_dbgPrt(atoi(s) ? DBG_print : NULL);
		SESSIONCFG_access()->enTRACE = atoi(s) ? 1 : 0;
	}
	snprintf(o, oSize, "OK");
}
/*----------------------------------------------------------------------------------------
 * Brief: toggle echo mode
 -----------------------------------------------------------------------------------------*/
void TSKCMD_hdl_IT_ECHO(uint8_t *s, uint16_t oSize, uint8_t *o)
{
	SESSIONCFG_access()->enECHO = (atoi(s)) ? 1 : 0;
	DBG_print("\r\n ECHO:%u", SESSIONCFG_access()->enTRACE);
	snprintf(o, oSize, "OK");
}
/*----------------------------------------------------------------------------------------
 * Brief: toggle debug GSM mode
 -----------------------------------------------------------------------------------------*/
void TSKCMD_hdl_IT_DBGGSM(uint8_t *s, uint16_t oSize, uint8_t *o)
{
	SESSIONCFG_access()->dbgGSM = (atoi(s)) ? 1 : 0;
	DBG_print("\r\n DBGGSM:%u", SESSIONCFG_access()->dbgGSM);
	snprintf(o, oSize, "OK");
}
/*----------------------------------------------------------------------------------------
 * Brief: toggle debug GNSS mode.
 -----------------------------------------------------------------------------------------*/
void TSKCMD_hdl_IT_DBGGNSS(uint8_t *s, uint16_t oSize, uint8_t *o)
{
	SESSIONCFG_access()->dbgGNSS = (atoi(s)) ? 1 : 0;
	DBG_print("\r\n DBGGNSS:%u", SESSIONCFG_access()->dbgGNSS);
	snprintf(o, oSize, "OK");
}
/*----------------------------------------------------------------------------------------
 * Brief: set rollback tracking data status
 -----------------------------------------------------------------------------------------*/
void TSKCMD_hdl_IT_SETRBD(uint8_t *s, uint16_t oSize, uint8_t *o)
{
	uint8_t sDate[7], sOfs[11];
	DATETIME_t dt;

	SL_split(s, ',', sDate, 7, sOfs, 11);
	DBG_print("\r\n SETRBD:date=\"%s\" Ofs=%s", sDate, sOfs);
	dt = IRTC_s2dt(1, sDate);
	DEVSTT_access()->RBtrkDat.utc = IRTC_dt2utc(dt);
	DEVSTT_access()->RBtrkDat.ofs = atoi(sOfs);
	DEVSTT_access()->RBtrkDat.nextOfs = 0;
	DEVSTT_access()->RBtrkDat.rdy2send = 0;
	DEVSTT_access()->RBtrkDat.size = 0;
	snprintf(o, oSize, "OK");
}
/*----------------------------------------------------------------------------------------
 * Brief: list all supported commands
 -----------------------------------------------------------------------------------------*/
void TSKCMD_hdl_IT_LISTCMD(uint8_t *s, uint16_t oSize, uint8_t *o)
{
	DBG_print("\r\n LISTCMD:");
	CLI_show(DBG_print, "\r\n +%s");
}
/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
void TSKCMD_hdl_IT_SENDSMS(uint8_t *s, uint16_t oSize, uint8_t *o)
{
	uint8_t num[16], content[32];

	SL_split(s, ',', num, 16, content, 32);
	DBG_print("\r\n SENDSMS:\"%s\",\"%s\"", num, content);
	TSKNETCOMM_add_sndSMS(num, content);
}
/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
void TSKCMD_hdl_IT_STRESSTEST(uint8_t *s, uint16_t oSize, uint8_t *o)
{
	SESSIONCFG_access()->stresstest = (atoi(s)) ? 1 : 0;
	DBG_print("\r\n STRESSTEST:%u", SESSIONCFG_access()->stresstest);
	snprintf(o, oSize, "OK");
}
/*----------------------------------------------------------------------------------------
 * Brief: Send a file to server
 -----------------------------------------------------------------------------------------*/
void TSKCMD_hdl_IT_SENDFILE(uint8_t *s, uint16_t oSize, uint8_t *o)
{
	uint8_t *logFname = NULL;
	uint16_t size = strlen(s) + 1;

	logFname = MM_get(MIN2SEC(5), size);
	if (logFname != NULL)
	{
		strlcpy(logFname, s, size);
		memset(&DEVSTT_access()->sndFileViaNATS, 0, sizeof(SNDFILE_STT_t));
		DEVSTT_access()->sndFileViaNATS.fname = logFname;
		DEVSTT_access()->sndFileViaNATS.utc = IRTC_getUTC();
		snprintf(o, oSize, "OK");
	}
	else
	{
		snprintf(o, oSize, "ERROR");
	}
}
/*----------------------------------------------------------------------------------------
 * Brief: configure device/get configuration.
 -----------------------------------------------------------------------------------------*/
void TSKCMD_hdl_CFG(uint8_t *s, uint16_t oSize, uint8_t *o)
{
	TSKCMD_hdl_CFG_GETCFG(0, s, oSize, o);
}
/*----------------------------------------------------------------------------------------
 * Brief: configure device ID
 -----------------------------------------------------------------------------------------*/
void TSKCMD_hdl_GETCFG(uint8_t *s, uint16_t oSize, uint8_t *o)
{
	strcat(s, " "); // To be compatible with TSKCMD_hdl_CFG_GETCFG function.
	TSKCMD_hdl_CFG_GETCFG(1, s, oSize, o);
}
/*----------------------------------------------------------------------------------------
 * Brief: configure device/get configuration.
 -----------------------------------------------------------------------------------------*/
void TSKCMD_hdl_CFG_GETCFG(uint8_t isGetCfg, uint8_t *s, uint16_t oSize, uint8_t *o)
{
	if (!SL_startwith(s, "ID "))
	{
		if (!isGetCfg)
		{
			SL_cut(s, "ID ");
			strlcpy(DEVID_access(), s, DEVID_SIZE);
		}
		snprintf(o, oSize, "ID=\"%s\"", DEVID_access());
	}
	else
	{
		static const uint8_t *DevCfgParam[] = DEVCFG_PARAM;
		uint8_t getAll = 0;

		if (isGetCfg && !strcmp(s, "ALL "))
		{
			getAll = 1;
		}
		if (!getAll)
		{
			snprintf(o, oSize, "ERROR");
		}
		else
		{
			snprintf(o, oSize, "All config:");
		}
		for (uint8_t i = 1; i < 0xFF; i++)
		{
			DBG_print("\r\n %s:Compare:\"%s\"/\"%s\"", __func__, s, DevCfgParam[i]);
			if (!strcmp(DevCfgParam[i], "<END>"))
			{
				break;
			}
			if (!SL_startwith(s, DevCfgParam[i]) || getAll)
			{
				pb_field_iter_t iter;
				//DBG_print("\r\n %s:Find with tag=%u", __func__, i);
				if (pb_field_iter_begin(&iter, DeviceConfigV2_fields, DEVCFG_access()))
				{
					if (pb_field_iter_find(&iter, i))
					{
						if (!isGetCfg)
						{
							SL_cut(s, DevCfgParam[i]);
							SL_cut(s, " ");
							uint32_t tmp = atoi(s);
							memcpy(iter.pData, &tmp, iter.pos->data_size);
							snprintf(o, oSize, "OK");
						}
						else
						{
							uint32_t tmp = *(uint32_t*)iter.pData;
							if (!getAll)
							{
								snprintf(o, oSize, "GETCFG:%s%lu", DevCfgParam[i], tmp);
							}
							else
							{
								snprintf(o, oSize, "%s\r\n%s%lu", o, DevCfgParam[i], tmp);
							}
						}
					}
				}
				if (!getAll)
				{
					break;
				}
			}
		}
	}
#if 0
	else if (!SL_startwith(s, "RDD "))
	{
		if (!isGetCfg)
		{
			SL_cut(s, "RDD ");
			DEVCFG_access()->recDatDist = (uint8_t)atoi(s);
		}
		snprintf(o, oSize, "CFG:RDD=%lu", DEVCFG_access()->recDatDist);
	}
	else if (!SL_startwith(s, "SBRDDT "))
	{
		if (!isGetCfg)
		{
			SL_cut(s, "SBRDDT ");
			DEVCFG_access()->stdbyRecDatDelayTime = (uint8_t)atoi(s);
		}
		snprintf(o, oSize, "CFG:SBRDDT=%lu", DEVCFG_access()->stdbyRecDatDelayTime);
	}
	else if (!SL_startwith(s, "RDDT "))
	{
		if (!isGetCfg)
		{
			SL_cut(s, "RDDT ");
			DEVCFG_access()->recDatDelayTime = (uint8_t)atoi(s);
		}
		snprintf(o, oSize, "CFG:RDDT=%lu", DEVCFG_access()->recDatDelayTime);
	}
	else if (!SL_startwith(s, "SBSDDT "))
	{
		if (!isGetCfg)
		{
			SL_cut(s, "SBSDDT ");
			DEVCFG_access()->stdbySndDatDelayTime = (uint8_t)atoi(s);
		}
		snprintf(o, oSize, "CFG:SDDT=%lu", DEVCFG_access()->stdbySndDatDelayTime);
	}
	else if (!SL_startwith(s, "SDDT "))
	{
		if (!isGetCfg)
		{
			SL_cut(s, "SDDT ");
			DEVCFG_access()->sndDatDelayTime = (uint8_t)atoi(s);
		}
		snprintf(o, oSize, "CFG:SDDT=%lu", DEVCFG_access()->sndDatDelayTime);
	}
	else if (!SL_startwith(s, "APN "))
	{
		if (!isGetCfg)
		{
			uint8_t tmps[APNPROFILE_PARAM_SIZE];
			uint16_t slen = strlen(s);
			SL_cut(s, "APN ");
			for (uint8_t i = 0; i < 3; i++)
			{
				SL_split(s, ',', tmps, APNPROFILE_PARAM_SIZE, s, slen);
				switch (i)
				{
					case 0:
						strlcpy(DEVCFG_access()->APN.APname, tmps, APNNAME_SIZE);
						break;
					case 1:
						strlcpy(DEVCFG_access()->APN.usr, tmps, APNUSR_SIZE);
						break;
					case 2:
						strlcpy(DEVCFG_access()->APN.pwd, tmps, APNPWD_SIZE);
						break;
				}
			}
		}
		snprintf(o, oSize, "CFG:APN:Name=\"%s\" Usr=\"%s\" Pwd=\"%s\""//
				, DEVCFG_access()->APN.APname//
				, DEVCFG_access()->APN.usr//
				, DEVCFG_access()->APN.pwd);
	}
	else if (!SL_startwith(s, "FIXON "))
	{
		if (!isGetCfg)
		{
			SL_cut(s, "FIXON ");
			DEVCFG_access()->fixOn = (uint8_t)atoi(s);
		}
		snprintf(o, oSize, "CFG:FIXON=%u", DEVCFG_access()->fixOn);
	}
	else if (!SL_startwith(s, "REVON "))
	{
		if (!isGetCfg)
		{
			SL_cut(s, "REVON ");
			DEVCFG_access()->revOn = (uint8_t)atoi(s);
		}
		snprintf(o, oSize, "CFG:REVON=%u", DEVCFG_access()->revOn);
	}
	else if (!SL_startwith(s, "FIXDR "))
	{
		if (!isGetCfg)
		{
			SL_cut(s, "FIXDR ");
			DEVCFG_access()->fixDoor = (uint8_t)atoi(s);
		}
		snprintf(o, oSize, "CFG:FIXDR=%u", DEVCFG_access()->fixDoor);
	}
	else if (!SL_startwith(s, "REVDR "))
	{
		if (!isGetCfg)
		{
			SL_cut(s, "REVDR ");
			DEVCFG_access()->revDoor = (uint8_t)atoi(s);
		}
		snprintf(o, oSize, "CFG:REVDR=%u", DEVCFG_access()->revDoor);
	}
	else if (!SL_startwith(s, "OBEO "))
	{
		if (!isGetCfg)
		{
			SL_cut(s, "OBEO ");
			DEVCFG_access()->onByEngineOn = (uint8_t)atoi(s);
		}
		snprintf(o, oSize, "CFG:OBEO=%u", DEVCFG_access()->onByEngineOn);
	}
#endif
}
/*----------------------------------------------------------------------------------------
 * Brief: Reset device.
 -----------------------------------------------------------------------------------------*/
void TSKCMD_hdl_IT_RESET(uint8_t *s, uint16_t oSize, uint8_t *o)
{
	DEVSTT_access()->rdy2rst = 1;
	snprintf(o, oSize, "OK");
}
/*----------------------------------------------------------------------------------------
 * Brief: Reset Engine ADC minimum value.
 -----------------------------------------------------------------------------------------*/
void TSKCMD_hdl_IT_RST_EGADCMIN(uint8_t *s, uint16_t oSize, uint8_t *o)
{
	SESSIONCFG_access()->EG_ADCmin = 0xFFFF;
	snprintf(o, oSize, "OK");
}
/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
void TSKCMD_hdl_APN(uint8_t *s, uint16_t oSize, uint8_t *o)
{
	if (!strncmp(s, "ADD ", 4))
	{
		SL_cut(s, "ADD ");
		APN_PROFILE_t *APN = MM_get(10, sizeof(APN_PROFILE_t));
		if (APN != NULL)
		{
			uint8_t sSize = strlen(s) + 1;
			SL_split(s, ',', APN->OP, APNPROFILE_PARAM_SIZE, s, sSize);
			SL_split(s, ',', APN->APNn, APNPROFILE_PARAM_SIZE, s, sSize);
			SL_split(s, ',', APN->APNu, APNPROFILE_PARAM_SIZE, s, sSize);
			SL_split(s, ',', APN->APNp, APNPROFILE_PARAM_SIZE, s, sSize);
			if (!APNPROFILE_add(*APN))
			{
				strlcpy(o, "OK", oSize);
			}
			else
			{
				strlcpy(o, "ERROR", oSize);
			}
			MM_free(APN);
		}
	}
	else if (!strncmp(s, "DEL ", 4))
	{
		SL_cut(s, "DEL ");
		APNPROFILE_del(s);
		strlcpy(o, "OK", oSize);
	}
	else if (!strncmp(s, "DELALL", 6))
	{
		APNPROFILE_delAll();
		strlcpy(o, "OK", oSize);
	}
	strlcpy(o, "APN:", oSize);
	for (uint8_t i = 0; i < MAX_APNPROFILE; i++)
	{
		if ((APNPROFILE_get(i)->OP[0] == 0xFF) || !strlen(APNPROFILE_get(i)->OP))
		{
			continue;
		}
		snprintf(o, oSize, "%s\r\n[%u]\"%s\",\"%s\",\"%s\",\"%s\"", o//
				, i//
				, APNPROFILE_get(i)->OP//
				, APNPROFILE_get(i)->APNn//
				, APNPROFILE_get(i)->APNu//
				, APNPROFILE_get(i)->APNp//
				);
	}
}

