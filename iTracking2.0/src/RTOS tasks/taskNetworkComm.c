/*------------------------------------------------------------------------------------------
 * INTRODUCE
 * Brief: This is source file of task network communication.
 * Author: Duy Vinh To - Team DinhViSo
 * Detail: upload/download data to/from server.
 -------------------------------------------------------------------------------------------*/

/*##########################################################################################
 * INCLUDE
 *##########################################################################################*/
#include "taskNetworkComm.h"
/* C libraries */
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
/* HW libraries */
#include "GSM.h"
#include "STM32F1_rtc.h"
#include "STM32F1_io.h"
#include "STM32F1_uart.h"
/* SW libraries */
#include "memMngr.h"
#include "netService.h"
#include "dbgPrint.h"
#include "stringlib.h"
#include "fileSystem.h"
/* FreeRTOS libraries */
#include "queue.h"
/* Others */
#include "logResetEvt.h"
#include "trackingData.h"
#include "devcfg.h"
#include "devstt.h"
#include "taskLogging.h"
#include <pb_encode.h>
#include <pb_decode.h>
#include "location.pb.h"
#include "nats-streaming.pb.h"
#include "file.pb.h"
#include "HWmapping.h"
#include "GSM.h"
#include "FWversion.h"
#include "taskCmd.h"

/*##########################################################################################
 * DEFINES
 *##########################################################################################*/
/* Toggle feature */
#define PUBLOC_WITHOUT_PING		1
#define ENABLE_SUBLOC			0
/*----------------*/
#define GSMBUF_SIZE		4096

#define FB_VIA_NATS_CONTENT_SIZE	256

#define NTP_HOST				"vn.pool.ntp.org"
#define NATS_AUTH_KEY			"2I8080bT86aQD45vQkZk1UvVGvrU3qEE"
#define NATS_HOST				"itracking.vn"
#define NATS_PORT				4222
#define SOCKINDEX_NATS_PUB_LOC	0
#define SOCKINDEX_NATS_SUB_LOC	1
#define SOCKINDEX_NATS_PUB_FB	2
#define SOCKINDEX_NATS_SUB_CMD	3
#define SOCKINDEX_NATS_SUB_CFG	4
#define SOCKINDEX_NATS_PUB_FILE	5
#define SOCKINDEX_NTP			5

#define DELAYTIME_CHK_SUB		10//Unit: minute
#define TIMEOUT_2_CHKSOCK		MIN2SEC(1)
/*##########################################################################################
 * TYPEDEFS
 *##########################################################################################*/
typedef struct
{
	U8 *content;
}FB_VIA_NATS_t;

typedef struct
{
	SMS_t *sms;
}SENDSMS_E_t;

typedef enum
{
	NST_CMD = 0,
	NST_CFG,
	NST_LOC,
}NATS_SUB_TYPE_t;

typedef enum
{
	SNDFILEVIANATS_QFILE = 0,
	SNDFILEVIANATS_QPHOTO,
}SNDFILE_VIA_NATS_QUEUE_SEL_t;

/*##########################################################################################
 * FUNC.PROTOTYPES
 *##########################################################################################*/
static void TSKNETCOMM_dbgPrt_pseudo(const uint8_t *s, ...);
static void GSM_readc_callback(uint8_t c);
static void GSM_resetModule(void);
static void GSM_PWRDWNPinSet(uint8_t set);
static void GSM_printcCB(uint8_t c);
static void TSKNETCOMM_main(void *pdata);
static void TSKNETCOMMIR_main(void *pdata);
static uint8_t TSKNETCOMM_NATS_sndFB(void);
static uint8_t TSKNETCOMM_NATS_sub_(NATS_SUB_TYPE_t type);
static void TSKNETCOMM_NATS_sndTrkDat(uint8_t sendRB);
static void TSKNETCOMM_NATS_sndFile(SNDFILE_VIA_NATS_QUEUE_SEL_t queueSel);
static bool Locations_encode_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg);
static bool File_encode_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg);

/*##########################################################################################
 * VARIABLES
 *##########################################################################################*/
static TSKNETCOMM_dbgPrt_CB TSKNETCOMM_dbgPrt = TSKNETCOMM_dbgPrt_pseudo;
static xQueueHandle q_sndSMS = NULL//
					, q_NATS_sndFB = NULL//
					, q_NATS_sndFile = NULL//
					, q_NATS_sndPhoto = NULL//
					;
static uint8_t *GSMbuf = NULL;
static xTaskHandle *TSKNETCOMMIR_hdl;
static xTaskHandle *TSKNETCOMM_hdl;
static uint8_t instantSndTrkDat = 0;
#if ENABLE_SUBLOC
static uint8_t gotfb_on_sub_LOC = 0; // Got feedback on 'LOC' subscriber
#endif // #if ENABLE_SUBLOC
static bool lastSndDat_on = false;
/*##########################################################################################
 * FUNCTIONS
 *##########################################################################################*/
/*-----------------------------------------------------------------------------
 * Brief:
 ------------------------------------------------------------------------------*/
void TSKNETCOMM_dbgPrt_pseudo(const uint8_t *s, ...)
{
	return;
}
/*-----------------------------------------------------------------------------
 * Brief: UART2 RXNE callback
 ------------------------------------------------------------------------------*/
void GSM_readc_callback(uint8_t c)
{
	if (SESSIONCFG_access()->dbgGSM)
	{
		UART_sendc(TERM_UARTPORT, c); // Debug GSM is always throughput to UART1
	}
	GSM_readc(c);
}
/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
void GSM_printcCB(uint8_t c)
{
	if (SESSIONCFG_access()->dbgGSM)
	{
		UART_sendc(TERM_UARTPORT, c); // Debug GSM is always throughput to UART1
	}
#ifdef GSM_DTR_PORT
	IO_wrt(GSM_DTR_PORT, GSM_DTR_PIN, 1);
#endif // #ifdef GSM_DTR_PORT
	UART_sendc(GSM_UARTPORT, c);
#ifdef GSM_DTR_PORT
	IO_wrt(GSM_DTR_PORT, GSM_DTR_PIN, 0);
#endif // #ifdef GSM_DTR_PORT
}
/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
void GSM_resetModule(void)
{
	IO_wrt(GSM_PWRDWN_PORT, GSM_PWRDWN_PIN, 1);
	vTaskDelay(500);
	IO_wrt(GSM_PWRDWN_PORT, GSM_PWRDWN_PIN, 0);
}
/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
void TSKNETCOMM_setup_dbgPrt(TSKNETCOMM_dbgPrt_CB cb)
{
	if (cb == NULL)
	{
		TSKNETCOMM_dbgPrt = TSKNETCOMM_dbgPrt_pseudo;
		return;
	}
	TSKNETCOMM_dbgPrt = cb;
}
/*----------------------------------------------------------------------------------------
 * Brief: setup task network communication.
 * Param:	size	|	IN	|	task size.
 * 			prio	|	IN	|	task priority.
 * 			tskHdl	|	I/O	|	task handler.
 * Ret:	0	|	OK
 * 		>0	|	FAIL
 -----------------------------------------------------------------------------------------*/
uint8_t TSKNETCOMM_setup(uint16_t size, uint32_t prio, xTaskHandle *tskHdl, uint32_t *wotime)
{
	if (GSMbuf == NULL)
	{
		GSMbuf = MM_get(0, GSMBUF_SIZE);
	}
	if (q_sndSMS == NULL)
	{
		q_sndSMS = xQueueCreate(SNDSMS_QUEUE_SIZE, sizeof(SENDSMS_E_t));
	}
	if (q_NATS_sndFB == NULL)
	{
		q_NATS_sndFB = xQueueCreate(SNDFB_VIA_NATS_QUEUE_SIZE, sizeof(FB_VIA_NATS_t));
	}
	if (q_NATS_sndFile == NULL)
	{
		q_NATS_sndFile = xQueueCreate(SNDFILE_VIA_NATS_QUEUE_SIZE, sizeof(File_data1KB*));
	}
	if (q_NATS_sndPhoto == NULL)
	{
		q_NATS_sndPhoto = xQueueCreate(SNDPHOTO_VIA_NATS_QUEUE_SIZE, sizeof(File_data1KB*));
	}
	/* Create task */
	portBASE_TYPE res = xTaskCreate(TSKNETCOMM_main, "NETCOMM", size, NULL, prio, tskHdl);
	if (res != pdTRUE)
	{
		MM_free(GSMbuf);
		vQueueDelete(q_sndSMS);
		vQueueDelete(q_NATS_sndFB);
		vQueueDelete(q_NATS_sndFile);
		vQueueDelete(q_NATS_sndPhoto);
		return 2;
	}
	/* Setup UART2 to communicate with GSM module */
	UART_setup(GSM_UARTPORT, 115200, GSM_readc_callback);
	/* Setup IO controlling power down pin */
#ifdef GSM_DTR_PORT
	IO_setup(GSM_PWRDWN_PORT, GSM_PWRDWN_PIN, IODIR_OPP);
#endif // #ifdef GSM_DTR_PORT
#ifdef GSM_PWRDWN_PORT
	IO_setup(GSM_PWRDWN_PORT, GSM_PWRDWN_PIN, IODIR_OPP);
#endif // #ifdef GSM_PWRDWN_PORT
	/* Setup IO controlling power down pin */
#ifdef GSM_DTR_PORT
	IO_setup(GSM_DTR_PORT, GSM_DTR_PIN, IODIR_OPP);
#endif // #ifdef GSM_DTR_PORT
	/* Setup GSM module */
	GSM_setup(GSMbuf, GSMBUF_SIZE, GSM_printcCB, GSM_resetModule, vTaskDelay);
	TSKNETCOMM_hdl = tskHdl;
	return 0;
}
/*----------------------------------------------------------------------------------------
 * Brief: setup task network communication Instant reacting.
 * 			This task will stop task network communication to process unsolicited event:
 * 			+ New received SMS.
 * 			+ Incoming call.
 * 			+ Socket received data.
 * Param:	size	|	IN	|	task size.
 * 			prio	|	IN	|	task priority.
 * 			tskHdl	|	I/O	|	task handler.
 * Ret:	0	|	OK
 * 		>0	|	FAIL
 * Note: Must setup this task after setup task network communication.
 -----------------------------------------------------------------------------------------*/
uint8_t TSKNETCOMMIR_setup(uint16_t size, uint32_t prio, xTaskHandle *tskHdl, uint32_t *wotime)
{
	portBASE_TYPE res = xTaskCreate(TSKNETCOMMIR_main, "NETCOMMIR", size, NULL, prio, tskHdl);
	if (res != pdTRUE)
	{
		return 1;
	}
	TSKNETCOMMIR_hdl = tskHdl;
	return 0;
}
/*----------------------------------------------------------------------------------------
 * Brief: main function.
 * To-do-list:
 *
 -----------------------------------------------------------------------------------------*/
void TSKNETCOMMIR_main(void *pdata)
{
	uint8_t SMSindex = 0, sockRxDindex = 0xFF;
	uint32_t t_last_scanSMSinbox = 0;

	while (1)
	{
		SMSindex = GSM_get_RxSMSindex();
		sockRxDindex = GSM_get_sockRxDindex();
		if (sockRxDindex != 0xFF)
		{
			const uint16_t MSG_SUBJ_SIZE = 64;
			uint8_t MSG_subject[MSG_SUBJ_SIZE], *rdat = NULL;
			uint8_t rdatLen = 0;

			GSM_rst_sockRxDindex();
			TSKNETCOMM_dbgPrt("\r\n <!>%s:SockRxDat at %u", __func__, sockRxDindex);
			memset(MSG_subject , 0, MSG_SUBJ_SIZE);
			rdat = MM_get(30, TCP_MAXSENDLENGTH);
			if (rdat != NULL)
			{
				uint8_t *MSG_payload = MM_get(30, TCP_MAXSENDLENGTH + 1);
				uint16_t MSG_bytes = (MSG_payload == NULL) ? 0 : TCP_MAXSENDLENGTH;
				uint32_t starttime = IRTC_getSRT();

				TSKNETCOMM_dbgPrt("\r\n %s:svMsgLen=%u", __func__, MSG_bytes);
				/* Stop task network communication */
				vTaskSuspend(*TSKNETCOMM_hdl);
				/* Wait until task network communication is suspended */
				while (xTaskIsTaskSuspended(*TSKNETCOMM_hdl) != pdTRUE)
				{
					vTaskDelay(10);
				}
				TSKNETCOMM_dbgPrt("\r\n %s:wait4suspend:T=%u", __func__, IRTC_getSRT() - starttime);
				rdatLen = GSM_rdSock(sockRxDindex, 500, TCP_MAXSENDLENGTH, rdat);
				TSKNETCOMM_dbgPrt("\r\n %s:Len=%u", __func__, rdatLen);
				TSKNETCOMM_dbgPrt("\r\n %s:wait4rd:T=%u", __func__, IRTC_getSRT() - starttime);
				//if (rdatLen)
				{
					uint8_t msgTyp = 0;
					TSKNETCOMM_dbgPrt("\r\n <SOCKRXDAT>%s</SOCKRXDAT>\r\n", rdat);
					msgTyp = NETSRVC_NATS_hdlUnsolMsg(rdatLen, rdat, MSG_SUBJ_SIZE, MSG_subject, &MSG_bytes, MSG_payload);
					switch (msgTyp)
					{
						case 1: /* PING */
						{
							uint8_t res = 0;
							switch (sockRxDindex)
							{
								case SOCKINDEX_NATS_PUB_LOC:
#if ENABLE_SUBLOC
								case SOCKINDEX_NATS_SUB_LOC:
#endif // #if ENABLE_SUBLOC
								case SOCKINDEX_NATS_SUB_CMD:
								case SOCKINDEX_NATS_SUB_CFG:
									res = NETSRVC_NATS_pong(1, sockRxDindex, NATS_HOST, NATS_PORT);
									if (res)
									{
										/* PONG action is failed */
										switch (sockRxDindex)
										{
											case SOCKINDEX_NATS_PUB_LOC:
												CLRBIT(DEVSTT_access()->net.connect2servBits, C2SVBIT_NATS_PUBLOC);
												break;
											case SOCKINDEX_NATS_SUB_LOC:
												CLRBIT(DEVSTT_access()->net.connect2servBits, C2SVBIT_NATS_SUBLOC);
												break;
											case SOCKINDEX_NATS_SUB_CMD:
												CLRBIT(DEVSTT_access()->net.connect2servBits, C2SVBIT_NATS_SUBCMD);
												break;
											case SOCKINDEX_NATS_SUB_CFG:
												CLRBIT(DEVSTT_access()->net.connect2servBits, C2SVBIT_NATS_SUBCFG);
												break;
											default:
												break;
										}
									}
									break;
								default:
									break;
							}
						}
							break;
						case 2: /* MSG */
							if (strlen(MSG_subject))
							{
								TSKNETCOMM_dbgPrt("\r\n %s:Subject=\"%s\"", __func__, MSG_subject);
								if (SL_search(MSG_subject, ".CFG") != -1)
								{
									pb_istream_t stream;
									DeviceConfigV2 newCfg;

									stream = pb_istream_from_buffer(MSG_payload, MSG_bytes);
									if (!pb_decode(&stream, DeviceConfigV2_fields, &newCfg))
									{
										DBG_print("\r\n %s:Decode failed: %s", __func__, PB_GET_ERROR(&stream));
									}
									else
									{
										memcpy(DEVCFG_access(), &newCfg, sizeof(DeviceConfigV2));
									}
								}
								else if (SL_search(MSG_subject, ".CMD") != -1)
								{
//									uint8_t subject[32], res = 0;
//
//									snprintf(subject, 32, "R.%s.FB", DEVID_access());
//									/* Firstly, echo command to feedback */
//									NETSRVC_NATS_pub(SOCKINDEX_NATS_PUB_FB, NATS_HOST, NATS_PORT, subject, "", strlen(MSG_payload), MSG_payload);
									if (MSG_bytes)
									{
										TSKCMD_addExtSrcCMD(EXSRCCT_NATS, "", MSG_payload);

									}
								}
#if ENABLE_SUBLOC
								else if (SL_search(MSG_subject, ".LOC") != -1)
								{
									if (MSG_bytes)
									{
										TSKLOG_addLogMsg(IRTC_getUTC(), LOGTYPE_LOCFB, MSG_payload);
									}
									gotfb_on_sub_LOC = 1;
								}
#endif // #if ENABLE_SUBLOC
							}
							break;
						case 3: /* ERR */
							GSM_closeSock(sockRxDindex);
							switch (sockRxDindex)
							{
								case SOCKINDEX_NATS_PUB_LOC:
									CLRBIT(DEVSTT_access()->net.connect2servBits, C2SVBIT_NATS_PUBLOC);
									break;
#if ENABLE_SUBLOC
								case SOCKINDEX_NATS_SUB_LOC:
									CLRBIT(DEVSTT_access()->net.connect2servBits, C2SVBIT_NATS_SUBLOC);
									break;
#endif // #if ENABLE_SUBLOC
								case SOCKINDEX_NATS_SUB_CMD:
									CLRBIT(DEVSTT_access()->net.connect2servBits, C2SVBIT_NATS_SUBCMD);
									break;
								case SOCKINDEX_NATS_SUB_CFG:
									CLRBIT(DEVSTT_access()->net.connect2servBits, C2SVBIT_NATS_SUBCFG);
									break;
								default:
									break;
							}
							break;
						default:
							break;
					}
				}
				MM_free(rdat);
				MM_free(MSG_payload);
				/* Resume task network communication */
				vTaskResume(*TSKNETCOMM_hdl);
			}
			else
			{
				TSKNETCOMM_dbgPrt("\r\n <!>%s:Low memory", __func__);
			}
		}
		else if ((IRTC_getSRT() >= (t_last_scanSMSinbox + MIN2SEC(10))) || (SMSindex))
		{
			uint8_t SMSindexList[32], SMSnum = 0;

			/* Stop task network communication */
			vTaskSuspend(*TSKNETCOMM_hdl);
			/* Wait until task network communication is suspended */
			while (xTaskIsTaskSuspended(*TSKNETCOMM_hdl) != pdTRUE)
			{
				vTaskDelay(10);
			}
			if (SMSindex)
			{
				GSM_rst_RxSMSindex();
				/* Only read newest SMS */
				SMSindexList[0] = SMSindex;
				SMSnum = 1;
			}
			else
			{
				/* Get list of inbox SMS */
				SMSnum = GSM_getSMSindexList(32, SMSindexList);
			}
			if (SMSnum > 0)
			{
				for (uint8_t i = 0; i < SMSnum; i++)
				{
					SMS_t *sms = MM_get(30, sizeof(SMS_t));
					if (sms != NULL)
					{
						uint8_t res = GSM_readSMS(SMSindexList[i], sms);
						if (res)
						{
							TSKNETCOMM_dbgPrt("\r\n %s:ReadSMS:res=%u", __func__, res);
						}
						else
						{
							TSKNETCOMM_dbgPrt("\r\n %s:New SMS:\"%s\"", __func__, sms->num);
							TSKNETCOMM_dbgPrt("\r\n<SMS>%s</SMS>", sms->content);
							if (!strlen(sms->num))
							{
								i = SMSnum;
							}
							else
							{
								if (!TSKCMD_addExtSrcCMD(EXSRCCT_SMS, sms->num, sms->content))
								{
									/* Delete this SMS from inbox */
									GSM_delSMS(SMSindexList[i]);
								}
							}
						}
					}
					MM_free(sms);
				}
			}
			else
			{
				/* Empty SMS inbox to ensure */
				GSM_delSMS(0);
			}
			/* Resume task network communication */
			vTaskResume(*TSKNETCOMM_hdl);
			t_last_scanSMSinbox = IRTC_getSRT();
		}
		else
		{
			vTaskDelay(50);
		}
		//vTaskDelay(50);
	}
}
/*----------------------------------------------------------------------------------------
 * Brief: main function.
 * To-do-list:
 *
 -----------------------------------------------------------------------------------------*/
void TSKNETCOMM_main(void *pdata)
{
	//const uint16_t BULKSMSFILE_DAT_SIZE = 4096;
	uint32_t t_last_rqstNTP  = 0//
			, t_last_checkNetStt = 0//
			, t_last_GPRSconnOk = 0//
			//, t_last_rpwGSM = 0//
			//, t_last_sendTrkDat = 0//
			, t_last_chk_NATS_sub_CMD = 0//
			, t_last_chk_NATS_sub_CFG = 0//
			//, t_last_chk_NATS_sub_LOC = 0//
			;
	uint8_t rstGSMct = 0//
			, setup_basic = 1//
			;

	while (1)
	{
		if (DEVSTT_access()->rdy2rst)
		{
			//vTaskDelete(NULL);
		}
		TSKNETCOMM_dbgPrt("\r\n %s:Start cycle at %u", __func__, IRTC_getSRT());
#if 1
		if (IRTC_getSRT() >= (t_last_checkNetStt + 10))
		{
			GSM_getNetwork(NETWORKNAME_SIZE, DEVSTT_access()->net.network);
			GSM_getSQ(&DEVSTT_access()->net.signalQuality);
			GSM_getNetRegStt(&DEVSTT_access()->net.netRegStt);
			t_last_checkNetStt = IRTC_getSRT();
		}
#endif
		TSKNETCOMM_dbgPrt("\r\n %s:TimeNoGPRS=%u sockFC=%u", __func__//
				, IRTC_getSRT() - t_last_GPRSconnOk, DEVSTT_access()->GPRSdbg.sockFC);
		if (//
				(IRTC_getSRT() >= (t_last_GPRSconnOk + MIN2SEC(3)))//
				|| (!t_last_GPRSconnOk)//
				|| (DEVSTT_access()->GPRSdbg.sockFC >= 10)//
			)
		{
			//if ((IRTC_getSRT() >= (t_last_rpwGSM + MIN2SEC(3))) || (!t_last_rpwGSM))
			{
				{
					uint8_t sLog[64];
					snprintf(sLog, 64, "TimeNoGPRS=%lu sockFC=%lu->RST", IRTC_getSRT() - t_last_GPRSconnOk, DEVSTT_access()->GPRSdbg.sockFC);
					TSKLOG_addLogMsg(IRTC_getUTC(), LOGTYPE_GSM, sLog);
				}
				if (rstGSMct >= 30)
				{
					DBG_print("\r\n <!>%s:RST SYS\r\n", __func__);
					LOGRSTEVT_wrt(LOGRSTEVT_BIT_NOGPRSLONGTIME);
					DEVSTT_access()->rdy2rst = 1;
				}
				TSKNETCOMM_dbgPrt("\r\n <!>%s:Reset", __func__);
				GSM_rst();
				memset(&DEVSTT_access()->net, 0, sizeof(NET_STT_t));
				memset(&DEVSTT_access()->GPRSdbg, 0, sizeof(GPRS_DBG_t));
				rstGSMct++;
				setup_basic = 1;
				t_last_GPRSconnOk = IRTC_getSRT();
				//t_last_rpwGSM = IRTC_getSRT();
			}
		}
		if (strlen(DEVSTT_access()->net.network))
		{
			/* GSM is available */
			SENDSMS_E_t send;
			if (xQueueReceive(q_sndSMS, &send, 0) == pdTRUE)
			{
				TSKNETCOMM_dbgPrt("\r\n %s:SendSMS:\"%s\"", __func__, send.sms->num);
				TSKNETCOMM_dbgPrt("\r\n<SMS>%s</SMS>", send.sms->content);
				GSM_sendSMS(*(SMS_t*)send.sms);
				MM_free(send.sms);
			}
			if (setup_basic)
			{
#if (HW_MAIN_VER == 2)
				if (!GSM_setCSCLK(1))
				{
					setup_basic = 0;
				}
#endif
			}
		}
		if (//
				(DEVSTT_access()->net.signalQuality > 0)//
				&& ((DEVSTT_access()->net.netRegStt == NRS_REG) || (DEVSTT_access()->net.netRegStt == NRS_ROAMING))//
				)
		{
			/* GPRS is available */
			t_last_GPRSconnOk = IRTC_getSRT();
			TSKNETCOMM_dbgPrt("\r\n %s:rqstNTP:tLast=%u RTCsync=%u", __func__, t_last_rqstNTP, DEVSTT_access()->RTCsync);
			if ((IRTC_getSRT() >= (t_last_rqstNTP + (DEVSTT_access()->RTCsync ? HOUR2MIN(MIN2SEC(1)) : MIN2SEC(1))))//
					|| (!t_last_rqstNTP))
			{
				uint32_t utc;
				uint8_t res;
				for (uint8_t i = 0; i < 1; i++)
				{
					res = NETSRVC_rqstNTP(SOCKINDEX_NTP, NTP_HOST, &utc);
					GSM_closeSock(SOCKINDEX_NTP);
					TSKNETCOMM_dbgPrt("\r\n %s:rqstNTP:res=%u utc=%u", __func__, res, utc);
					if (!res)
					{
						DEVSTT_access()->GPRSdbg.sockFC -= (DEVSTT_access()->GPRSdbg.sockFC) ? 1 : 0;
						if (abs(IRTC_getUTC() - utc) >= 5)
						{
							IRTC_setByUTC(utc);
						}
						DEVSTT_access()->RTCsync = 1;
						break;
					}
					else
					{
						{
							uint8_t sLog[32];
							snprintf(sLog, 32, "rqstNTP:Res=%u", res);
							TSKLOG_addLogMsg(IRTC_getUTC(), LOGTYPE_GSM, sLog);
						}
						/* Sending UDP be failed doesn't mean sending socket be failed */
						//DEVSTT_access()->GPRSdbg.sockFC++;
					}
				}
				t_last_rqstNTP = IRTC_getSRT();
			}
			/* Publish feedbacks to commands that received via NATS (if needed) */
			TSKNETCOMM_NATS_sndFB();
			/* Send realtime data (if needed) */
#if 0
			if (//
					((IRTC_getSRT() >= (t_last_sendTrkDat + DEVCFG_access()->sndDatDelayTime)) && !DEVSTT_access()->VHstdby)//
					|| ((IRTC_getSRT() >= (t_last_sendTrkDat + MIN2SEC(DEVCFG_access()->stdbySndDatDelayTime))) && DEVSTT_access()->VHstdby)//
					|| instantSndTrkDat//
				)
#endif
			{
				instantSndTrkDat = 0;
				TSKNETCOMM_NATS_sndTrkDat(0);
				//t_last_sendTrkDat = IRTC_getSRT();
			}
			/* Send rollback data (if needed) */
			TSKNETCOMM_NATS_sndTrkDat(1);
			/* send log (if needed) */
			TSKNETCOMM_NATS_sndFile(SNDFILEVIANATS_QFILE);
			/* send photo (if needed) */
			TSKNETCOMM_NATS_sndFile(SNDFILEVIANATS_QPHOTO);
			/* Timeout -> verify subscriber 'CMD' */
			if ((IRTC_getSRT() >= (t_last_chk_NATS_sub_CMD + MIN2SEC(DELAYTIME_CHK_SUB))) //
					|| (!t_last_chk_NATS_sub_CMD)//
					|| !GETBIT(DEVSTT_access()->net.connect2servBits, C2SVBIT_NATS_SUBCMD)//
					)
			{
				if (!TSKNETCOMM_NATS_sub_(NST_CMD))
				{
					SETBIT(DEVSTT_access()->net.connect2servBits, C2SVBIT_NATS_SUBCMD);
					t_last_chk_NATS_sub_CMD = IRTC_getSRT();
				}
				else
				{
					CLRBIT(DEVSTT_access()->net.connect2servBits, C2SVBIT_NATS_SUBCMD);
				}
			}
			/* Timeout -> verify subscriber 'CFG' */
			if ((IRTC_getSRT() >= (t_last_chk_NATS_sub_CFG + MIN2SEC(DELAYTIME_CHK_SUB))) //
					|| (!t_last_chk_NATS_sub_CFG)//
					|| !GETBIT(DEVSTT_access()->net.connect2servBits, C2SVBIT_NATS_SUBCFG)//
					)
			{
				if (!TSKNETCOMM_NATS_sub_(NST_CFG))
				{
					SETBIT(DEVSTT_access()->net.connect2servBits, C2SVBIT_NATS_SUBCFG);
					t_last_chk_NATS_sub_CFG = IRTC_getSRT();
				}
				else
				{
					CLRBIT(DEVSTT_access()->net.connect2servBits, C2SVBIT_NATS_SUBCFG);
				}
			}
		}
		vTaskDelay(30);
	}
}
/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
uint8_t TSKNETCOMM_NATS_sndFB(void)
{
	FB_VIA_NATS_t fb;
	uint8_t ret = 0xFF;

	if (xQueueReceive(q_NATS_sndFB, &fb, 0) == pdTRUE)
	{
		uint8_t subject[32], res = 0;

		TSKNETCOMM_dbgPrt("\r\n %s:SendFBviaNATS:\"%s\"", __func__, fb.content);
		/* Subscribe subject 'Command' to receive command from server */
		snprintf(subject, 32, "R.%s.FB", DEVID_access());
		for (uint8_t rtry = 0; rtry < 2; rtry++)
		{
			GSM_closeSock(SOCKINDEX_NATS_PUB_FB);
			res = NETSRVC_NATS_connect(SOCKINDEX_NATS_PUB_FB, NATS_HOST, NATS_PORT, 1, 0, 0, NATS_AUTH_KEY, "", "", DEVID_access(), FW_VERSION);
			if (res)
			{
				/* Log this fail result */
				uint8_t sLog[32];
				snprintf(sLog, 32, "Connect:Res=%u", res);
				TSKLOG_addLogMsg(IRTC_getUTC(), LOGTYPE_NATS, sLog);
				DEVSTT_access()->GPRSdbg.sockFC++;
				ret = 1;
			}
			else
			{
				res = NETSRVC_NATS_pub(SOCKINDEX_NATS_PUB_FB, NATS_HOST, NATS_PORT, subject, "", strlen(fb.content), fb.content);
				GSM_closeSock(SOCKINDEX_NATS_PUB_FB);
				if (!res)
				{
					ret = 0;
					break;
				}
				else
				{
					uint8_t sLog[32];
					snprintf(sLog, 32, "Sub:Res=%u", res);
					TSKLOG_addLogMsg(IRTC_getUTC(), LOGTYPE_NATS, sLog);
					DEVSTT_access()->GPRSdbg.sockFC++;
					ret = 1;
				}
			}
		}
		MM_free(fb.content);
	}
	return ret;
}
/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
uint8_t TSKNETCOMM_NATS_sub_(NATS_SUB_TYPE_t type)
{
	uint8_t subject[32], res = 0, conn2sub = 0, sockindex = 0xFF;

	switch (type)
	{
		case NST_CMD:
			sockindex = SOCKINDEX_NATS_SUB_CMD;
			snprintf(subject, 32, "R.%s.CMD", DEVID_access());
			break;
		case NST_CFG:
			sockindex = SOCKINDEX_NATS_SUB_CFG;
			snprintf(subject, 32, "R.%s.CFG", DEVID_access());
			break;
#if ENABLE_SUBLOC
		case NST_LOC:
			sockindex = SOCKINDEX_NATS_SUB_LOC;
			snprintf(subject, 32, "R.%s.LOC", DEVID_access());
			break;
#endif // #if ENABLE_SUBLOC
		default:
			return 0xFF;
	}
	for (uint8_t rtry = 0; rtry < 2; rtry++)
	{
		/* Connect */
		GSM_closeSock(sockindex);
		res = NETSRVC_NATS_connect(sockindex, NATS_HOST, NATS_PORT, 1, 0, 0, NATS_AUTH_KEY, "", "", DEVID_access(), FW_VERSION);
		if (res)
		{
			/* Log this fail result */
			uint8_t sLog[32];
			snprintf(sLog, 32, "Connect:Res=%u", res);
			TSKLOG_addLogMsg(IRTC_getUTC(), LOGTYPE_NATS, sLog);
			DEVSTT_access()->GPRSdbg.sockFC++;
			GSM_closeSock(sockindex);
			conn2sub = 0;
		}
		else
		{
			/* Subscribe subject 'CMD' to receive command from server */
			res = NETSRVC_NATS_sub(sockindex, NATS_HOST, NATS_PORT, subject, "", 1);
			if (res)
			{
				uint8_t sLog[64];
				snprintf(sLog, 64, "Sub=\"%s\":Res=%u", subject, res);
				TSKLOG_addLogMsg(IRTC_getUTC(), LOGTYPE_NATS, sLog);
				DEVSTT_access()->GPRSdbg.sockFC++;
				GSM_closeSock(sockindex);
			}
			else
			{
				DEVSTT_access()->GPRSdbg.sockFC -= DEVSTT_access()->GPRSdbg.sockFC ? 1 : 0;
				conn2sub = 1;
				break;
			}
		}
	}
	TSKNETCOMM_dbgPrt("\r\n %s:conn2sub\"%s\"=%u", __func__, subject, conn2sub);
	return !conn2sub;
}

/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
void TSKNETCOMM_NATS_sndTrkDat(uint8_t sendRB)
{
	uint8_t res//
			, keepRBqueueBusy = 0//
			;
	uint16_t sendDatCount = TRKDAT_vault_count(sendRB ? VAULT_ROLLBACK : VAULT_SEND)//
			//, buffersize = 0//
			//, msegLen//
			;
	uint32_t startTime = IRTC_getSRT();

	Locations locs = Locations_init_default;
	pb_ostream_t stream;

	if (!sendDatCount)
	{
		/* There is no data to send */
		return;
	}
#if ENABLE_SUBLOC
	/* Subscribe to 'LOC' to listen feedback of insert data result */
	if (!GETBIT(DEVSTT_access()->net.connect2servBits, C2SVBIT_NATS_SUBLOC))
	{
		if (TSKNETCOMM_NATS_sub_(NST_LOC))
		{
			TSKNETCOMM_dbgPrt("\r\n %s:Cannot Sub 'LOC'", __func__);
			CLRBIT(DEVSTT_access()->net.connect2servBits, C2SVBIT_NATS_SUBLOC);
			return;
		}
		SETBIT(DEVSTT_access()->net.connect2servBits, C2SVBIT_NATS_SUBLOC);
		TSKNETCOMM_dbgPrt("\r\n %s:SUBLOC:spentTime=%u", __func__, IRTC_getSRT() - startTime);
	}
#endif // #if ENABLE_SUBLOC
	//TSKNETCOMM_SNDTRKDAT_SKIPCHK://
	//if (GETBIT(DEVSTT_access()->net.connect2servBits, C2SVBIT_NATS_PUBLOC))
	{
		uint32_t sDatSize = 0;
		uint16_t tmpQlen = 0//
				;
		uint8_t subject[32], replyTo[32], *sDat = NULL;
		Location loc, *tmpQ = NULL;

		snprintf(replyTo, 32, "R.%s.LOC", DEVID_access());
		snprintf(subject, 32, "S.%s.LOC", DEVID_access());
		/* Generating data to send */
		TSKNETCOMM_dbgPrt("\r\n %s:vaultRBsel=%u sndCt=%u", __func__, sendRB, sendDatCount);
		while (1)
		{
			if (!sendDatCount)
			{
				TSKNETCOMM_dbgPrt("\r\n %s:NotEnoughMem", __func__);
				return;
			}
			sDatSize = sendDatCount * Location_size;
			sDat = MM_get(30, sDatSize);
			tmpQ = MM_get(30, sendDatCount * sizeof(Location));
			if ((sDat != NULL) && (tmpQ != NULL))
			{
				break;
			}
			MM_free(sDat);
			MM_free(tmpQ);
			sendDatCount--;
		}
		TSKNETCOMM_dbgPrt("\r\n %s:sndCt=%u", __func__, sendDatCount);
		stream = pb_ostream_from_buffer(sDat, sDatSize);
		while (1)
		{
			if (sendRB)
			{
				if (TRKDAT_vault_count(VAULT_ROLLBACK) == 1)
				{
					if (!keepRBqueueBusy)
					{
						Location pseudoLoc = Location_init_default;
						TRKDAT_vault_add(VAULT_ROLLBACK, pseudoLoc);
						keepRBqueueBusy = 1;
					}
					else
					{
						break;
					}
				}
			}
			res = TRKDAT_vault_get((sendRB ? VAULT_ROLLBACK : VAULT_SEND), &loc);
			if (res)
			{
				break;
			}
//			/* This log is only for debug error  */
//			if (loc.onoff != lastSndDat_on)
//			{
//				lastSndDat_on = loc.onoff;
//			}
			if (sendRB)
			{
				TSKNETCOMM_dbgPrt("\r\n %s:Loc:\"%s\",%u,%f,%f", __func__, loc.deviceID, loc.unixtime,loc.latitude, loc.longitude);
			}
			tmpQ[tmpQlen++] = loc;
			locs.data.funcs.encode = &Locations_encode_callback;
			locs.data.arg = &loc;
			if (!pb_encode(&stream, Locations_fields, &locs))
			{
				TSKNETCOMM_dbgPrt("\r\n %s:Encoding failed: %s", __func__, PB_GET_ERROR(&stream));
				break;
			}
			if (tmpQlen >= sendDatCount)
			{
				break;
			}
			//TSKNETCOMM_dbgPrt("\r\n stream.bytes_written=%u", stream.bytes_written);
		}
		TSKNETCOMM_dbgPrt("\r\n %s:GENDAT:spentTime=%u", __func__, IRTC_getSRT() - startTime);
		/* Send (publish) data to server */
		res = 0;
#if ENABLE_SUBLOC
		gotfb_on_sub_LOC = 0;
#endif // #if ENABLE_SUBLOC
		for (uint8_t rtry = 0; rtry < 2; rtry++)
		{
			if (!GETBIT(DEVSTT_access()->net.connect2servBits, C2SVBIT_NATS_PUBLOC))
			{
				/* No connection -> connect now */
				GSM_closeSock(SOCKINDEX_NATS_PUB_LOC);
				res = NETSRVC_NATS_connect(SOCKINDEX_NATS_PUB_LOC, NATS_HOST, NATS_PORT, 1, 0, 0, NATS_AUTH_KEY, "", "", DEVID_access(), FW_VERSION);
				if (!res)
				{
					/* Successfully connected to server */
					SETBIT(DEVSTT_access()->net.connect2servBits, C2SVBIT_NATS_PUBLOC);
					TSKNETCOMM_dbgPrt("\r\n %s:CONNECT:spentTime=%u", __func__, IRTC_getSRT() - startTime);
				}
				else
				{
					/* Log this fail result */
					uint8_t sLog[32];
					snprintf(sLog, 32, "Connect:Res=%u", res);
					TSKLOG_addLogMsg(IRTC_getUTC(), LOGTYPE_NATS, sLog);
					GSM_closeSock(SOCKINDEX_NATS_PUB_LOC);
					CLRBIT(DEVSTT_access()->net.connect2servBits, C2SVBIT_NATS_PUBLOC);
					DEVSTT_access()->GPRSdbg.sockFC++;
					continue;
				}
			}
			/* Now, publish data */
			res = NETSRVC_NATS_pub(SOCKINDEX_NATS_PUB_LOC, NATS_HOST, NATS_PORT, subject, replyTo, stream.bytes_written, sDat);
			if (!res)
			{
				break;
			}
			/* Send failed -> connection is not good */
			GSM_closeSock(SOCKINDEX_NATS_PUB_LOC);
			CLRBIT(DEVSTT_access()->net.connect2servBits, C2SVBIT_NATS_PUBLOC);
			TSKNETCOMM_dbgPrt("\r\n %s:Pub:Retry", __func__);
		}
		MM_free(sDat);
		TSKNETCOMM_dbgPrt("\r\n %s:PUB:spentTime=%u", __func__, IRTC_getSRT() - startTime);
#if ENABLE_SUBLOC
		for (uint8_t i = 0; i < 100; i++)
		{
			if (gotfb_on_sub_LOC)
			{
				/* Got feedback */
				break;
			}
			vTaskDelay(10);
		}
		if (!gotfb_on_sub_LOC)
		{
			CLRBIT(DEVSTT_access()->net.connect2servBits, C2SVBIT_NATS_SUBLOC);
		}
		TSKNETCOMM_dbgPrt("\r\n %s:GETFB:spentTime=%u", __func__, IRTC_getSRT() - startTime);
		TSKNETCOMM_dbgPrt("\r\n %s:Pub:res=%u fbOnSubLOC=%u", __func__, res, gotfb_on_sub_LOC);
		if (gotfb_on_sub_LOC)
#else
		if (!res)
#endif // #if ENABLE_SUBLOC
		{
			/* Successfully sending data */
			DEVSTT_access()->GPRSdbg.sockFC -= (DEVSTT_access()->GPRSdbg.sockFC) ? 1 : 0;
			if (sendRB)
			{
				/* Updating rollback data offset */
				TSKNETCOMM_dbgPrt("\r\n %s:UpdOfs:%u->%u", __func__, DEVSTT_access()->RBtrkDat.ofs, DEVSTT_access()->RBtrkDat.nextOfs);
				{
					uint8_t sLog[64];
					DATETIME_t dt;

					dt = IRTC_utc2dt(DEVSTT_access()->RBtrkDat.utc, DEFAULT_SYSTEM_GMT);
					snprintf(sLog, 64, "DT=%u-%u-%u,Ofs:%lu->%lu", dt.year, dt.month, dt.day, DEVSTT_access()->RBtrkDat.ofs, DEVSTT_access()->RBtrkDat.nextOfs);
					TSKLOG_addLogMsg(IRTC_getUTC(), LOGTYPE_ROLLBACK, sLog);
				}
				DEVSTT_access()->RBtrkDat.ofs = DEVSTT_access()->RBtrkDat.nextOfs;
			}
			//GSM_closeSock(SOCKINDEX_NATS_PUB_LOC);
		}
		else
		{
			if (res)
			{
				/* Fail by hardware -> log it */
				uint8_t sLog[32];
				snprintf(sLog, 32, "Pub:Res=%u", res);
				TSKLOG_addLogMsg(IRTC_getUTC(), LOGTYPE_NATS, sLog);
				DEVSTT_access()->GPRSdbg.sockFC++;
				GSM_closeSock(SOCKINDEX_NATS_PUB_LOC);
			}
			CLRBIT(DEVSTT_access()->net.connect2servBits, C2SVBIT_NATS_PUBLOC);
			if (!sendRB)
			{
				for (uint16_t i = 0; i < tmpQlen; i++)
				{
					TRKDAT_vault_add(VAULT_MISS, tmpQ[i]);
				}
			}
			else
			{
				/* Add data back into rollback queue */
				for (uint16_t i = 0; i < tmpQlen; i++)
				{
					if (i == 0)
					{
						if (keepRBqueueBusy)
						{
							TRKDAT_vault_get(VAULT_ROLLBACK, &loc);
							keepRBqueueBusy = 0;
						}
					}
					TRKDAT_vault_add(VAULT_ROLLBACK, tmpQ[i]);
				}
			}
		}
		if (keepRBqueueBusy)
		{
			TRKDAT_vault_get(VAULT_ROLLBACK, &loc);
		}
		MM_free(tmpQ);
	}
	TSKNETCOMM_dbgPrt("\r\n %s:spentTime=%u", __func__, IRTC_getSRT() - startTime);
}

/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
uint8_t TSKNETCOMM_add_sndSMS(const uint8_t *num, const uint8_t *content)
{
	if (q_sndSMS == NULL)
	{
		return 0xFF;
	}
	SENDSMS_E_t send;

	if (!strlen(num) || !strlen(content))
	{
		return 0;
	}
	send.sms = MM_get(60, sizeof(SMS_t));
	if (send.sms == NULL)
	{
		return 1;
	}
	strlcpy(send.sms->num, num, PHONENUM_MAXSIZE);
	strlcpy(send.sms->content, content, SMS_CONTENT_MAXSIZE);
	if (xQueueSend(q_sndSMS, &send, 0) != pdTRUE)
	{
		MM_free(send.sms);
		return 2;
	}
	return 0;
}
/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
uint8_t TSKNETCOMM_add_sndFBViaNATS(const uint8_t *content)
{
	uint16_t size = strlen(content) + 1;
	if (q_NATS_sndFB == NULL)
	{
		return 0xFF;
	}
	FB_VIA_NATS_t newFB;
	newFB.content = MM_get(60, size);
	if (newFB.content == NULL)
	{
		return 1;
	}
	strlcpy(newFB.content, content, size);
	if (xQueueSend(q_NATS_sndFB, &newFB, 0) != pdTRUE)
	{
		MM_free(newFB.content);
		return 2;
	}
	return 0;
}

/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
uint8_t TSKNETCOMM_add_sndFileViaNATS(const File_data1KB *fileBlk1KB)
{
	if (q_NATS_sndFile == NULL)
	{
		return 0xFF;
	}
	if ((fileBlk1KB == NULL) || (!fileBlk1KB->data.size))
	{
		return 1;
	}
	if (xQueueSend(q_NATS_sndFile, &fileBlk1KB, 0) != pdTRUE)
	{
		return 2;
	}
	return 0;
}
/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
uint8_t TSKNETCOMM_add_sndPhotoViaNATS(const File_data1KB *fileBlk1KB)
{
	if (q_NATS_sndPhoto == NULL)
	{
		return 0xFF;
	}
	if ((fileBlk1KB == NULL) || (!fileBlk1KB->data.size))
	{
		return 1;
	}
	if (xQueueSend(q_NATS_sndPhoto, &fileBlk1KB, 0) != pdTRUE)
	{
		return 2;
	}
	return 0;
}
/*----------------------------------------------------------------------------------------
 * Brief: check if sending file via NATS queue is free
 * Ret:	1	|	Yes, it's full.
 * 		0	|	No, it's not.
 -----------------------------------------------------------------------------------------*/
uint8_t TSKNETCOMM_chkFull_sndFileViaNATS(void)
{
	if (uxQueueMessagesWaiting(q_NATS_sndFile) >= SNDFILE_VIA_NATS_QUEUE_SIZE)
	{
		return 1;
	}
	return 0;
}
/*----------------------------------------------------------------------------------------
 * Brief: check if sending photo via NATS queue is free
 * Ret:	1	|	Yes, it's full.
 * 		0	|	No, it's not.
 -----------------------------------------------------------------------------------------*/
uint8_t TSKNETCOMM_chkFull_sndPhotoViaNATS(void)
{
	if (uxQueueMessagesWaiting(q_NATS_sndPhoto) >= SNDPHOTO_VIA_NATS_QUEUE_SIZE)
	{
		return 1;
	}
	return 0;
}
/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
void TSKNETCOMM_set_instantSndTrkDat(void)
{
	instantSndTrkDat = 1;
}

/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
void TSKNETCOMM_NATS_sndFile(SNDFILE_VIA_NATS_QUEUE_SEL_t queueSel)
{
	File_data1KB *fileBlk1KB = NULL;
	File_data4KB *fileBlk4KB = NULL;
	uint8_t *buf = NULL//
			, res = 0//
			, subject[32]//
			//, breakSending = 0//
			, finish = 0//
			, oneTimesDatCt = 4// hardware limitation: socket cannot continuously send more than 4KB
			;
	uint32_t startTime = IRTC_getSRT()//
			, tmp_snt_ofs = 0//
			;
	pb_ostream_t stream;
	bool encodeRes = false;
	xQueueHandle queue = (queueSel == SNDFILEVIANATS_QFILE) ? q_NATS_sndFile : q_NATS_sndPhoto;
	SNDFILE_VIA_NATS_t *sttSndFileViaNATS = (queueSel == SNDFILEVIANATS_QFILE) ? &DEVSTT_access()->sndFileViaNATS : &DEVSTT_access()->sndPhotoViaNATS;

	if (!uxQueueMessagesWaiting(queue))
	{
		return;
	}
	snprintf(subject, 32, "S.%s.FLE", DEVID_access());
	fileBlk4KB = MM_get(60, sizeof(File_data4KB));
	if (fileBlk4KB == NULL)
	{
		MM_free(fileBlk4KB);
		return;
	}
	fileBlk4KB->offset = 0xFFFFFFFF;
	while (1)
	{
		if (xQueueReceive(queue, &fileBlk1KB, 100) != pdTRUE)
		{
			TSKNETCOMM_dbgPrt("\r\n %s:Empty queue", __func__);
			break;
		}
		if ((fileBlk4KB->data.size + fileBlk1KB->data.size) > 4096)
		{
			TSKNETCOMM_dbgPrt("\r\n %s:overflow data", __func__);
			break;
		}
		if (fileBlk4KB->offset == 0xFFFFFFFF)
		{
			fileBlk4KB->offset = fileBlk1KB->offset;
		}
		memcpy(&fileBlk4KB->data.bytes[fileBlk4KB->data.size], fileBlk1KB->data.bytes, fileBlk1KB->data.size);
		fileBlk4KB->data.size += fileBlk1KB->data.size;
		tmp_snt_ofs = fileBlk1KB->offset + fileBlk1KB->data.size;
		strlcpy(fileBlk4KB->deviceID, fileBlk1KB->deviceID, 32);
		strlcpy(fileBlk4KB->contentType, fileBlk1KB->contentType, 32);
		fileBlk4KB->unixtime = fileBlk1KB->unixtime;
		fileBlk4KB->has_latitude = fileBlk1KB->has_latitude;
		fileBlk4KB->latitude = fileBlk1KB->latitude;
		fileBlk4KB->has_longitude = fileBlk1KB->has_longitude;
		fileBlk4KB->longitude = fileBlk1KB->longitude;
		fileBlk4KB->finished = fileBlk1KB->finished;
		MM_free(fileBlk1KB);
		oneTimesDatCt--;
		if (!oneTimesDatCt)
		{
			TSKNETCOMM_dbgPrt("\r\n %s:enough data", __func__);
			break;
		}
	}
	TSKNETCOMM_dbgPrt("\r\n %s:ofs=%u size=%u Fin=%u", __func__, fileBlk4KB->offset, fileBlk4KB->data.size, fileBlk4KB->finished);
	buf = MM_get(60, File_data4KB_size);
	if (buf == NULL)
	{
		MM_free(fileBlk4KB);
		return;
	}
	stream = pb_ostream_from_buffer(buf, File_data4KB_size);
	encodeRes = pb_encode(&stream, File_data4KB_fields, fileBlk4KB);
	finish = fileBlk4KB->finished;
	MM_free(fileBlk4KB);
	if (encodeRes != true)
	{
		TSKNETCOMM_dbgPrt("\r\n %s:Encoding failed: %s", __func__, PB_GET_ERROR(&stream));
		goto SNDFILE_VIA_NATS_FAIL;
	}
	TSKNETCOMM_dbgPrt("\r\n %s:streamSize=%u", __func__, stream.bytes_written);
	for (uint8_t rtry = 0; rtry < 2; rtry++)
	{
		if (!GETBIT(DEVSTT_access()->net.connect2servBits, C2SVBIT_NATS_PUBFILE))
		{
			/* No connection -> connect now */
			GSM_closeSock(SOCKINDEX_NATS_PUB_FILE);
			res = NETSRVC_NATS_connect(SOCKINDEX_NATS_PUB_FILE, NATS_HOST, NATS_PORT, 1, 0, 0, NATS_AUTH_KEY, "", "", DEVID_access(), FW_VERSION);
			if (!res)
			{
				/* Successfully connected to server */
				SETBIT(DEVSTT_access()->net.connect2servBits, C2SVBIT_NATS_PUBFILE);
				TSKNETCOMM_dbgPrt("\r\n %s:CONNECT:spentTime=%u", __func__, IRTC_getSRT() - startTime);
			}
			else
			{
				/* Log this fail result */
				uint8_t sLog[32];
				snprintf(sLog, 32, "Connect:Res=%u", res);
				TSKLOG_addLogMsg(IRTC_getUTC(), LOGTYPE_NATS, sLog);
				GSM_closeSock(SOCKINDEX_NATS_PUB_FILE);
				CLRBIT(DEVSTT_access()->net.connect2servBits, C2SVBIT_NATS_PUBFILE);
				DEVSTT_access()->GPRSdbg.sockFC++;
				continue;
			}
		}
		/* Now, publish log */
		res = NETSRVC_NATS_pub(SOCKINDEX_NATS_PUB_FILE, NATS_HOST, NATS_PORT, subject, "", stream.bytes_written, buf);
		if (!res)
		{
			TSKNETCOMM_dbgPrt("\r\n %s:PUB:spentTime=%u", __func__, IRTC_getSRT() - startTime);
			MM_free(buf);
			goto SNDFILE_VIA_NATS_OK;
		}
		/* Send failed -> connection is not good */
		GSM_closeSock(SOCKINDEX_NATS_PUB_FILE);
		CLRBIT(DEVSTT_access()->net.connect2servBits, C2SVBIT_NATS_PUBFILE);
		TSKNETCOMM_dbgPrt("\r\n %s:Pub:Retry", __func__);
	}
	MM_free(buf);
	// Reach here -> sending progress is broken.
	// Free all queuing data.
	TSKNETCOMM_dbgPrt("\r\n %s:Free queue", __func__);
	while (xQueueReceive(queue, &fileBlk1KB, 0) == pdTRUE)
	{
		MM_free(fileBlk1KB);
	}
	SNDFILE_VIA_NATS_FAIL://
	// Reverse read offset.
	TSKNETCOMM_dbgPrt("\r\n %s:rdOfs:%u->%u", __func__, sttSndFileViaNATS->rd_ofs, sttSndFileViaNATS->snt_ofs);
	sttSndFileViaNATS->rd_ofs = sttSndFileViaNATS->snt_ofs;
//	TSKLOG_addLogMsg(IRTC_getUTC(), LOGTYPE_SNDPHOTO, "FAIL");
	return;
	SNDFILE_VIA_NATS_OK://
	// Update sent offset.
	sttSndFileViaNATS->snt_ofs = tmp_snt_ofs;
	TSKNETCOMM_dbgPrt("\r\n %s:sntOfs=%u/%u", __func__, sttSndFileViaNATS->snt_ofs, sttSndFileViaNATS->fsize);
//	TSKLOG_addLogMsg(IRTC_getUTC(), LOGTYPE_SNDPHOTO, "OK");
	if (finish)
	{
		if (sttSndFileViaNATS->delFileAfterDone)
		{
			if (SL_search(sttSndFileViaNATS->fname, ".jpg") != -1)
			{
				uint8_t *newName = MM_get(5, 256);

				if (newName != NULL)
				{
					strlcpy(newName, sttSndFileViaNATS->fname, 256);
					SL_cut(newName, ".jpg");
					FS_renameFile(sttSndFileViaNATS->fname, newName);
					MM_free(newName);
				}
				else
				{
					FS_delFile(sttSndFileViaNATS->fname);
				}
			}
			else
			{
				FS_delFile(sttSndFileViaNATS->fname);
			}

		}
		MM_free(sttSndFileViaNATS->fname);
	}
	return;
}
/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
bool Locations_encode_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg)
{
	//Location *loc = (Location*)*arg;

	//TSKNETCOMM_dbgPrt("\r\n Encode:%s,%f,%f", loc->deviceID, loc->latitude, loc->longitude);
	if (!pb_encode_tag_for_field(stream, field))
	{
		return false;
	}
	return pb_encode_submessage(stream, Location_fields, *arg);
}
/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
bool File_encode_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg)
{
	//Location *loc = (Location*)*arg;

	//TSKNETCOMM_dbgPrt("\r\n Encode:%s,%f,%f", loc->deviceID, loc->latitude, loc->longitude);
	if (!pb_encode_tag_for_field(stream, field))
	{
		return false;
	}
	return pb_encode_submessage(stream, Location_fields, *arg);
}
