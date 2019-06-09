/*------------------------------------------------------------------------------------------
 * INTRODUCE
 * Brief: This is source file of task logging.
 * Author: Duy Vinh To - Team DinhViSo
 * Detail: periodically logging device state into file system.
 -------------------------------------------------------------------------------------------*/

/*##########################################################################################
 * INCLUDE
 *##########################################################################################*/
#include "taskLogging.h"
/* C libraries */
#include "stdio.h"
#include "string.h"
/* HW libraries */
#include "STM32F1_rtc.h"
#include "GSM.h"
/* SW libraries */
#include "fileSystem.h"
#include "dbgPrint.h"
#include "stringlib.h"
#include "memMngr.h"
/* Others */
#include "devstt.h"
#include "devcfg.h"
#include "predefFname.h"
#include "file.pb.h"
#include "taskNetworkComm.h"
#include "pb_encode.h"
/* FreeRTOS */
#include "queue.h"

/*##########################################################################################
 * TYPEDEFS
 *##########################################################################################*/
typedef struct
{
	uint32_t utc;
	LOGTYPE_t type;
	uint8_t *content;
}LOGMSG_t;

typedef struct
{
	uint32_t size;
	void *ptr;
}FILE_DATA_t;
/*##########################################################################################
 * FUNC.PROTOTYPES
 *##########################################################################################*/
static void TSKLOG_dbgPrt_pseudo(const uint8_t *s,...);
static bool File_encode_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg);
static void TSKLOG_main(void *pdata);

/*##########################################################################################
 * VARIABLES
 *##########################################################################################*/
static const uint8_t *logTypeSuffix[] = LOGFILE_SUFFIX;
static TSKLOG_dbgPrt_CB TSKLOG_dbgPrt = TSKLOG_dbgPrt_pseudo;
static xQueueHandle logQ = NULL;

/*##########################################################################################
 * FUNCTIONS
 *##########################################################################################*/
/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
void TSKLOG_dbgPrt_pseudo(const uint8_t *s,...)
{
	return;
}
/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
void TSKLOG_setup_dbgPrt(TSKLOG_dbgPrt_CB cb)
{
	if (cb == NULL)
	{
		TSKLOG_dbgPrt = TSKLOG_dbgPrt_pseudo;
		return;
	}
	TSKLOG_dbgPrt = cb;
}
/*----------------------------------------------------------------------------------------
 * Brief: setup logging task.
 -----------------------------------------------------------------------------------------*/
uint8_t TSKLOG_setup(uint16_t size, uint32_t prio, xTaskHandle *tskHdl, uint32_t *wotime)
{
	portBASE_TYPE res = xTaskCreate(TSKLOG_main, "LOGGING", size, NULL, prio, tskHdl);
	if (res == pdTRUE)
	{
		logQ = xQueueCreate(LOGQUEUE_SIZE, sizeof(LOGMSG_t));
	}
	return (res == pdTRUE) ? 0 : 1;
}
/*----------------------------------------------------------------------------------------
 * Brief:
 *
 -----------------------------------------------------------------------------------------*/
uint8_t TSKLOG_addLogMsg(uint32_t utc, LOGTYPE_t type, const uint8_t *content)
{
	if (logQ == NULL)
	{
		return 0xFF;
	}
	LOGMSG_t newLog = {utc, type, NULL};

	newLog.content = MM_get(60, strlen(content) + 1);
	if (newLog.content == NULL)
	{
		return 1;
	}
	strlcpy(newLog.content, content, strlen(content) + 1);
	if (xQueueSend(logQ, &newLog, 0) != pdTRUE)
	{
		MM_free(newLog.content);
		return 2;
	}
	return 0;
}
/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
bool File_encode_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg)
{
	//Location *loc = (Location*)*arg;
	FILE_DATA_t *file2sndDat = (FILE_DATA_t*)*arg;

	//TSKNETCOMM_dbgPrt("\r\n Encode:%s,%f,%f", loc->deviceID, loc->latitude, loc->longitude);
	if (!pb_encode_tag_for_field(stream, field))
	{
		return false;
	}
	TSKLOG_dbgPrt("\r\n %s:Size=%u", __func__, file2sndDat->size);
	return pb_encode_string(stream, file2sndDat->ptr, file2sndDat->size);
}
/*----------------------------------------------------------------------------------------
 * Brief: load log file to send
 -----------------------------------------------------------------------------------------*/
void TSKLOG_loadlog2snd(void)
{
	if ((DEVSTT_access()->sndFileViaNATS.fname != NULL) && strlen(DEVSTT_access()->sndFileViaNATS.fname))
	{
		if (!DEVSTT_access()->sndFileViaNATS.fsize)
		{
			FS_fsize(DEVSTT_access()->sndFileViaNATS.fname, &DEVSTT_access()->sndFileViaNATS.fsize);
		}
		while (1)
		{
			if (TSKNETCOMM_chkFull_sndFileViaNATS())
			{
				TSKLOG_dbgPrt("\r\n %s:Queue is full", __func__);
				break;
			}
			TSKLOG_dbgPrt("\r\n %s:F=\"%s\" Ofs=%u/%u", __func__, DEVSTT_access()->sndFileViaNATS.fname, DEVSTT_access()->sndFileViaNATS.rd_ofs, DEVSTT_access()->sndFileViaNATS.fsize);
			if (DEVSTT_access()->sndFileViaNATS.rd_ofs >= DEVSTT_access()->sndFileViaNATS.fsize)
			{
				break;
			}
			{
				uint8_t res = 0;
				uint32_t size = 0;
				File_data1KB *fileBlk1KB = MM_get(MIN2SEC(5), sizeof(File_data1KB));
				if (fileBlk1KB == NULL)
				{
					TSKLOG_dbgPrt("\r\n %s:Low memory", __func__);
					break;
				}
				fileBlk1KB->unixtime = DEVSTT_access()->sndFileViaNATS.utc;
				fileBlk1KB->has_latitude = false;
				fileBlk1KB->latitude = 0;
				fileBlk1KB->has_longitude = false;
				fileBlk1KB->longitude = 0;
				strlcpy(fileBlk1KB->contentType, "text/x-log", 32);
				strlcpy(fileBlk1KB->deviceID, DEVID_access(), 16);
				fileBlk1KB->offset = DEVSTT_access()->sndFileViaNATS.rd_ofs;
				size = DEVSTT_access()->sndFileViaNATS.fsize - DEVSTT_access()->sndFileViaNATS.rd_ofs;
				if (size > 1024)
				{
					size = 1024;
				}
				else
				{
					fileBlk1KB->finished = true;
				}
				res = FS_rd(DEVSTT_access()->sndFileViaNATS.fname, fileBlk1KB->offset, size, fileBlk1KB->data.bytes, &size);
				if (res)
				{
					MM_free(fileBlk1KB);
					TSKLOG_dbgPrt("\r\n %s:ReadFile:Res=%u->ERR", __func__, res);
					break;
				}
				fileBlk1KB->data.size = size;
				if (TSKNETCOMM_add_sndFileViaNATS(fileBlk1KB))
				{
					MM_free(fileBlk1KB);
					TSKLOG_dbgPrt("\r\n %s:ReadFile:Cannot add to queue", __func__);
					break;
				}
				DEVSTT_access()->sndFileViaNATS.rd_ofs += size;
			}
		}
	}
}
/*----------------------------------------------------------------------------------------
 * Brief: logging task: main function.
 * To-do-list:
 * + Logging system uptime.
 -----------------------------------------------------------------------------------------*/
void TSKLOG_main(void *pdata)
{
	uint8_t fname[32];
	DATETIME_t dt;

	while (1)
	{
		if (DEVSTT_access()->rdy2rst)
		{
			//vTaskDelete(NULL);
		}
		if (uxQueueMessagesWaiting(logQ))
		{
			LOGMSG_t unsavedlog;

			if (xQueueReceive(logQ, &unsavedlog, 0) == pdTRUE)
			{
				uint8_t *sLog = NULL, res = 0;

				TSKLOG_dbgPrt("\r\n LogType=%u -> \"%s\"", unsavedlog.type, logTypeSuffix[unsavedlog.type]);
				TSKLOG_dbgPrt("\r\n<LOG>%s\r\n</LOG>", unsavedlog.content);
				dt = IRTC_utc2dt(unsavedlog.utc, DEFAULT_SYSTEM_GMT);
				snprintf(fname, 32, "%02u%02u%02u", dt.day, dt.month, dt.year % 100);
				fname[31] = 0;
				res = FS_createDir(fname);
				strlcat(fname, "/", 32);
				strlcat(fname, logTypeSuffix[unsavedlog.type], 32);
				strlcat(fname, ".log", 32);
				res = FS_createFile(fname, 0);
				sLog = MM_get(5, strlen(unsavedlog.content) + 64);
				if (sLog == NULL)
				{
					sLog = unsavedlog.content;
				}
				else
				{
					snprintf(sLog, strlen(unsavedlog.content) + 64, "\r\n[%u-%u-%u %u:%u:%u]:%s"//
							, dt.year % 100, dt.month, dt.day, dt.hour, dt.minute, dt.second//
							, unsavedlog.content);
				}
				res = FS_append(fname, strlen(sLog), sLog);
				MM_free(unsavedlog.content);
				MM_free(sLog);
			}
		}
		TSKLOG_loadlog2snd();
		vTaskDelay(30);
	}
}
