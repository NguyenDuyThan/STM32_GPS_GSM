/*------------------------------------------------------------------------------------------
 * INTRODUCE
 * Brief: This is source file of task backup tracking data into file system.
 * Author: Duy Vinh To - Team DinhViSo
 * Detail:
 -------------------------------------------------------------------------------------------*/

/*##########################################################################################
 * INCLUDE
 *##########################################################################################*/
#include "taskBkpTrkDat.h"
/* C libraries */
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
/* HW libraries */
#include "STM32F1_rtc.h"
/* SW libraries */
#include "trackingData.h"
#include "fileSystem.h"
#include "dbgPrint.h"
#include "memMngr.h"
#include "stringlib.h"
#include "location.pb.h"
#include "devstt.h"
/* Pre-definitions */
#include "predefFname.h"

/*##########################################################################################
 * DEFINE
 *##########################################################################################*/
#define BKPTRKDAT_HEADER	"TD:"
#define BKPTRKDAT_FOOTER	"\r\n"

#define TIMEOUT_SAVEDAT2FS	MIN2SEC(1) // Unit: second
/*##########################################################################################
 * FUNC.PROTOTYPES
 *##########################################################################################*/
static void TSKBKPTD_dbgPrt_pseudo(const uint8_t *s,...);
static void TSKBKPTD_main(void *pdata);
static void TSKBKPTD_saveTrkDat(uint8_t missQueueSel);
static void TSKBKPTD_load2RB(void);

/*##########################################################################################
 * VARIABLES
 *##########################################################################################*/
static TSKBKPTD_dbgPrt_CB TSKBKPTD_dbgPrt = TSKBKPTD_dbgPrt_pseudo;
static uint32_t cycleStartTime = 0;

/*##########################################################################################
 * FUNCTIONS
 *##########################################################################################*/
/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
void TSKBKPTD_dbgPrt_pseudo(const uint8_t *s,...)
{
	return;
}
/*----------------------------------------------------------------------------------------
 * Brief: setup debug print.
 -----------------------------------------------------------------------------------------*/
void TSKBKPTD_setup_dbgPrt(TSKBKPTD_dbgPrt_CB cb)
{
	if (cb == NULL)
	{
		TSKBKPTD_dbgPrt = TSKBKPTD_dbgPrt_pseudo;
		return;
	}
	TSKBKPTD_dbgPrt = cb;
}
/*----------------------------------------------------------------------------------------
 * Brief: setup task.
 -----------------------------------------------------------------------------------------*/
uint8_t TSKBKPTD_setup(uint16_t size, uint32_t prio, xTaskHandle *tskHdl, uint32_t *wotime)
{
	portBASE_TYPE res = xTaskCreate(TSKBKPTD_main, "BKPTRKDAT", size, NULL, prio, tskHdl);
	return (res == pdTRUE) ? 0 : 1;
}

/*----------------------------------------------------------------------------------------
 * Brief: logging task: main function.
 * To-do-list:
 * + Logging system uptime.
 -----------------------------------------------------------------------------------------*/
void TSKBKPTD_main(void *pdata)
{
	uint32_t t_last_bkpMain = 0//
			, t_last_bkpMiss = 0//
			, t_last_bkpDevstt = 0//
			, t_last_loadRB = 0//
			, t_wait_loadRB = 0//
			;

	while (1)
	{
		if (DEVSTT_access()->rdy2rst)
		{
			//vTaskDelete(NULL);
		}
		TSKBKPTD_dbgPrt("\r\n %s:Start cycle at %u", __func__, IRTC_getSRT());
		cycleStartTime = IRTC_getSRT();
		/* Backup main data to file system */
		if ((IRTC_getSRT() >= (t_last_bkpMain + TIMEOUT_SAVEDAT2FS)) || (TRKDAT_vault_count(VAULT_MAIN) >= (MAINTRKDAT_VAULT_SIZE / 2)))
		{
			TSKBKPTD_saveTrkDat(0);
			t_last_bkpMain = IRTC_getSRT();
		}
		/* Backup missed data to file system */
		if ((IRTC_getSRT() >= (t_last_bkpMiss + TIMEOUT_SAVEDAT2FS)) || (TRKDAT_vault_count(VAULT_MISS) >= (MISSTRKDAT_VAULT_SIZE / 2)))
		{
			TSKBKPTD_saveTrkDat(1);
			t_last_bkpMiss = IRTC_getSRT();
		}
		/* Load rollback data to RAM */
		if (DEVSTT_access()->RTCsync)
		{
			TSKBKPTD_dbgPrt("\r\n +TSKBKPTD:LoadRB:tLast=%u tWait=%u", t_last_loadRB, t_wait_loadRB);
			if (IRTC_getSRT() >= (t_last_loadRB + t_wait_loadRB))
			{
				//uint32_t preNextOfs = DEVSTT_access()->RBtrkDat.nextOfs;
				TSKBKPTD_load2RB();
				t_last_loadRB = IRTC_getSRT();
				if ((DEVSTT_access()->RBtrkDat.size == DEVSTT_access()->RBtrkDat.ofs) && (DEVSTT_access()->RBtrkDat.size != 0))
				{
					t_wait_loadRB = MIN2SEC(5);
				}
				else
				{
					t_wait_loadRB = 0;
				}
			}
		}
		/* Backup device status to file system */
		if (IRTC_getSRT() >= (t_last_bkpDevstt + 60))
		{
			FS_createFile(DEVSTTBKP_FNAME, 1);
			FS_wrt(DEVSTTBKP_FNAME, 0, sizeof(DEVSTT_t), DEVSTT_access());
			t_last_bkpDevstt = IRTC_getSRT();
		}
		cycleStartTime = 0;
		vTaskDelay(30);
	}
}
/*----------------------------------------------------------------------------------------
 * Brief: Save backup of main/missed tracking data
 * Param:	missQueueSel	|	I	|	queue to sava is 'Miss' one or not
 -----------------------------------------------------------------------------------------*/
void TSKBKPTD_saveTrkDat(uint8_t missQueueSel)
{
	const uint16_t WRT_SIZE = 1024, LINE_SIZE = 128;
	Location loc;
	DATETIME_t dt;
	VAULTSEL_t vaultSel = (missQueueSel) ? VAULT_MISS : VAULT_MAIN;
	uint8_t sDate[7], fname[32]//
			, *sWrt, sLine[LINE_SIZE]//
			, *predefFname = (missQueueSel) ? MISSTRKDAT_FNAME : MAINTRKDAT_FNAME//
			;
	//uint16_t lineLength = 0;
	//uint32_t preWrtUtc = 0;

	if (!TRKDAT_vault_count(vaultSel))
	{
		return;
	}
	TSKBKPTD_dbgPrt("\r\n %s:vaultSel=%u", __func__, vaultSel);
	sWrt = MM_get(10, WRT_SIZE);
	if (sWrt == NULL)
	{
		return;
	}
	memset(sLine, 0, LINE_SIZE);
	while (1)
	{
		if (TRKDAT_vault_get(vaultSel, &loc))
		{
			break;
		}
		TSKBKPTD_dbgPrt("\r\n %s:UTC=%u", __func__, loc.unixtime);
		dt = IRTC_utc2dt(loc.unixtime, DEFAULT_SYSTEM_GMT);
		TRKDAT_conv2s(loc, BKPTRKDAT_HEADER, BKPTRKDAT_FOOTER, LINE_SIZE, sLine);
		if((strlen(sWrt) + strlen(sLine)) >= WRT_SIZE)
		{
			TSKBKPTD_dbgPrt("\r\n %s:Buffer full", __func__);
			break;
		}
		strlcat(sWrt, sLine, WRT_SIZE);
		memset(sLine, 0, LINE_SIZE);
	}
	snprintf(sDate, 7, "%02u%02u%02u", dt.day, dt.month, dt.year % 100);
	sDate[6] = 0;
	snprintf(fname, 32, "%s/%s", sDate, predefFname);
	fname[31] = 0;
	if (!FS_createDir(sDate))
	{
		if (!FS_createFile(fname, 0))
		{
			if (!FS_append(fname, strlen(sWrt), sWrt))
			{
				if (strlen(sLine))
				{
					TSKBKPTD_dbgPrt("\r\n %s:sLine=\"%s\"", __func__, sLine);
					if (!FS_append(fname, strlen(sLine), sLine))
					{
						/* Append OK */
					}
				}
			}
		}
	}
	MM_free(sWrt);
}

/*----------------------------------------------------------------------------------------
 * Brief: loading rollback data from missed data file.
 -----------------------------------------------------------------------------------------*/
void TSKBKPTD_load2RB(void)
{
	DATETIME_t dt;
	uint8_t sDate[7], fname[32]//
			, *rDat = NULL//
			, *sline = NULL//
			;
	const uint16_t RDAT_SIZE = 4096, SLINE_SIZE = 1024;
	uint32_t rSize = 0;
	uint16_t sline_len = 0;
	int startpos = 0, stoppos = 0;

	if (TRKDAT_vault_count(VAULT_ROLLBACK))
	{
		return;
	}
	dt = IRTC_utc2dt(DEVSTT_access()->RBtrkDat.utc, DEFAULT_SYSTEM_GMT);
	snprintf(sDate, 7, "%02u%02u%02u", dt.day, dt.month, dt.year % 100);
	sDate[6] = 0;
	snprintf(fname, 32, "%s/%s", sDate, MISSTRKDAT_FNAME);
	fname[31] = 0;
	FS_fsize(fname, &DEVSTT_access()->RBtrkDat.size);
	TSKBKPTD_dbgPrt("\r\n %s:d=%u-%u-%u s=%u o=%u"//
			, __func__//
			, dt.day, dt.month, dt.year//
			, DEVSTT_access()->RBtrkDat.size//
			, DEVSTT_access()->RBtrkDat.ofs//
			);
	TSKBKPTD_LOADRB_CHECKCONDT://
	if ((DEVSTT_access()->RBtrkDat.size) && (DEVSTT_access()->RBtrkDat.size > DEVSTT_access()->RBtrkDat.ofs))
	{
		/* There are available data to rollback */
		rDat = MM_get(10, RDAT_SIZE);
		sline = MM_get(10, SLINE_SIZE);
		if ((rDat != NULL) && (sline != NULL))
		{
			//DEVSTT_access()->RBtrkDat.ofs = DEVSTT_access()->RBtrkDat.nextOfs;
			//TSKBKPTD_dbgPrt("\r\n +++TSKBKPTD_loadRB:loadData");
			DEVSTT_access()->RBtrkDat.nextOfs = DEVSTT_access()->RBtrkDat.ofs;
			if (!FS_rd(fname, DEVSTT_access()->RBtrkDat.ofs, RDAT_SIZE, rDat, &rSize))
			{
				//TSKBKPTD_dbgPrt("\r\n +++TSKBKPTD:RB:dat:\r\n\"%s\"", rDat);
				while (1)
				{
					startpos = 0;
					stoppos = 0;
					startpos = SL_search(rDat, BKPTRKDAT_HEADER);
					stoppos = SL_search(rDat, BKPTRKDAT_FOOTER);
					//TSKBKPTD_dbgPrt("\r\n +++TSKBKPTD_loadRB:startpos=%d stoppos=%d", startpos, stoppos);
					if ((startpos >= stoppos) || (startpos == -1) || (stoppos == -1))
					{
						// No completed data line.
						// Try to detect a line instead, and skip this "invalid" line.
						startpos = 0;
						stoppos = SL_search(rDat, "\r\n");
						if (stoppos != -1)
						{
							//Extract this "invalid" line.
							SL_sub(startpos, stoppos + strlen("\r\n") - startpos, rDat, sline);
							// Remove extracted data line from read buffer.
							SL_cut(rDat, sline);
							TSKBKPTD_dbgPrt("\r\n %s:remove invalid line=\"%s\"", __func__, sline);
							DEVSTT_access()->RBtrkDat.nextOfs += strlen(sline);
							continue;
						}
						else
						{
							// There is no "invalid" also.
							break;
						}
					}
					// Extract data line.
					SL_sub(startpos, stoppos + strlen(BKPTRKDAT_FOOTER) - startpos, rDat, sline);
					// Remove extracted data line from read buffer.
					SL_cut(rDat, sline);
					sline_len = strlen(sline);
					// Remove header and footer from data line before converting.
					SL_cut(sline, BKPTRKDAT_HEADER);
					SL_cut(sline, BKPTRKDAT_FOOTER);
					//TSKBKPTD_dbgPrt("\r\n +++TSKBKPTD_loadRB:sline=\"%s\"", sline);
					/* Converting data line to tracking data structure. */
					{
						Location loc = Location_init_default;
						TRKDAT_conv(strlen(sline), sline, &loc);
#if FILTER_INVALID_DATA_ON_LOADRB
						if ((loc.unixtime <= IRTC_getUTC()) && (loc.has_latitude) && (loc.has_longitude))
#endif // #if FILTER_INVALID_DATA_ON_LOADRB
						{
							/* Unix time is reasonable, coordinates is available */
							if (TRKDAT_vault_add(VAULT_ROLLBACK, loc))
							{
								break;
							}
						}
					}
					DEVSTT_access()->RBtrkDat.nextOfs += sline_len;
				}
			}
		}
		MM_free(rDat);
		MM_free(sline);
	}
	else
	{
		/* All rollback data in day has gone */
		TSKBKPTD_dbgPrt("\r\n %s:Find new data on the next day from %u Now=%u", __func__, DEVSTT_access()->RBtrkDat.utc, IRTC_getUTC());
		if (IRTC_getUTC() >= ((HOUR2SEC(24) * 90) + DEVSTT_access()->RBtrkDat.utc))
		{
			DEVSTT_access()->RBtrkDat.utc = IRTC_getUTC() - (HOUR2SEC(24) * 30);
			TSKBKPTD_dbgPrt("\r\n %s:To far in the past:%u", __func__, DEVSTT_access()->RBtrkDat.utc);
		}
		else
		{
			if ((DEVSTT_access()->RBtrkDat.utc / HOUR2SEC(24)) < (IRTC_getUTC() / HOUR2SEC(24)))
			{
				DEVSTT_access()->RBtrkDat.utc += HOUR2SEC(24);
				DEVSTT_access()->RBtrkDat.size = 0;
				DEVSTT_access()->RBtrkDat.ofs = 0;
				DEVSTT_access()->RBtrkDat.nextOfs = 0;
				TSKBKPTD_dbgPrt("\r\n %s:It's ok", __func__);
			}
			else if ((DEVSTT_access()->RBtrkDat.utc / HOUR2SEC(24)) > (IRTC_getUTC() / HOUR2SEC(24)))
			{
				DEVSTT_access()->RBtrkDat.utc = IRTC_getUTC();
				DEVSTT_access()->RBtrkDat.size = 0;
				DEVSTT_access()->RBtrkDat.ofs = 0;
				DEVSTT_access()->RBtrkDat.nextOfs = 0;
				TSKBKPTD_dbgPrt("\r\n +TSKBKPTD_loadRB:pass over");
			}
			TSKBKPTD_dbgPrt("\r\n %s:Nothing???", __func__);
		}
	}
	//TSKBKPTD_dbgPrt("\r\n +++TSKBKPTD_loadRB:END");
}
/*----------------------------------------------------------------------------------------
 * Brief: get cycle runtime
 -----------------------------------------------------------------------------------------*/
uint32_t TSKBKPTD_getCycleRT(void)
{
	if (!cycleStartTime)
	{
		return 0;
	}
	return (RTC_getSRT() - cycleStartTime);
}
