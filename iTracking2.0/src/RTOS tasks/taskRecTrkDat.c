/*------------------------------------------------------------------------------------------
 * INTRODUCE
 * Brief: This is source file of task recording tracking data.
 * Author: Duy Vinh To - Team DinhViSo
 * Detail: periodically recording tracking data.
 -------------------------------------------------------------------------------------------*/

/*##########################################################################################
 * INCLUDE
 *##########################################################################################*/
#include "taskRecTrkDat.h"
/* C libraries */
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
/* DVS hardware libraries */
#include "STM32F1_io.h"
#include "STM32F1_rtc.h"
#include "STM32F1_uart.h"
#include "QuectelL70.h"
/* DVS software libraries */
#include "dbgPrint.h"
#include "trackingData.h"
#include "fileSystem.h"
#include "predefFname.h"
#include "devcfg.h"
#include "devstt.h"
/* FreeRTOS libraries */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "taskLogging.h"
#include "taskNetworkComm.h"

#include "HWmapping.h"
/*##########################################################################################
 * DEFINE
 *##########################################################################################*/
#define SPDSAMPLE_MAX	5
/*##########################################################################################
 * FUNC.PROTOTYPES
 *##########################################################################################*/
static void GNSS_readc(uint8_t c);
static void GNSS_sendc(uint8_t c);
static void TSKRECTD_dbgPrt_pseudo(const uint8_t *s,...);
static void TSKRECTD_main(void *pdata);
/*##########################################################################################
 * VARIABLES
 *##########################################################################################*/
static TSKRECTD_dbgPrt_CB TSKRECTD_dbgPrt = TSKRECTD_dbgPrt_pseudo;
static xQueueHandle SPDsamples = NULL;
/*##########################################################################################
 * FUNCTIONS
 *##########################################################################################*/
/*-----------------------------------------------------------------------------
 * Brief: Rx data callback
 ------------------------------------------------------------------------------*/
void GNSS_readc(uint8_t c)
{
	if (SESSIONCFG_access()->dbgGNSS)
	{
		UART_sendc(TERM_UARTPORT, c); // // Debug GNSS is always throughput to UART1
	}
	QL70_readc(c);
}
/*-----------------------------------------------------------------------------
 * Brief: Rx data callback
 ------------------------------------------------------------------------------*/
void GNSS_sendc(uint8_t c)
{
	UART_sendc(GNSS_UARTPORT, c);
}
/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
void TSKRECTD_dbgPrt_pseudo(const uint8_t *s,...)
{
	return;
}
/*----------------------------------------------------------------------------------------
 * Brief: setup debug print.
 -----------------------------------------------------------------------------------------*/
void TSKRECTD_setup_dbgPrt(TSKRECTD_dbgPrt_CB cb)
{
	if (cb == NULL)
	{
		TSKRECTD_dbgPrt = TSKRECTD_dbgPrt_pseudo;
		return;
	}
	TSKRECTD_dbgPrt = cb;
}
/*----------------------------------------------------------------------------------------
 * Brief: setup task.
 -----------------------------------------------------------------------------------------*/
uint8_t TSKRECTD_setup(uint16_t size, uint32_t prio, xTaskHandle *tskHdl, uint32_t *wotime)
{
	portBASE_TYPE res = xTaskCreate(TSKRECTD_main, "RECTRKDAT", size, NULL, prio, tskHdl);
	if (res == pdTRUE)
	{
		SPDsamples = xQueueCreate(SPDSAMPLE_MAX, sizeof(uint16_t));
		/* Setup UART2 to communicate with GNSS module */
		//UART_setup(GNSS_UARTPORT, GNSS_UART_BRATE, GNSS_readc);
		/* Setup GNSS module */
		QL70_setup(GNSS_sendc, vTaskDelay);
	}
	return (res == pdTRUE) ? 0 : 1;
}
/*----------------------------------------------------------------------------------------
 * Brief: record data task.
 * To-do-list:
 * + Communicate with GNSS module.
 * + Record data into queues.
 -----------------------------------------------------------------------------------------*/
void TSKRECTD_main(void *pdata)
{
	uint32_t t_last_recTrkDat = 0, t_last_VHoff_gotoSleep = 0;
	uint8_t recordRequest = 0, pre_OnOff = 0, pre_OpDr = 0, pre_VHstdby = 0, instantSndTrkDat = 0;
	float preRecLat = 0, preRecLng = 0;
	GNSS_DAT_t GNSSdat, lastQC_GNSSdat;

	IO_setup(GNSS_RST_PORT, GNSS_RST_PIN, IODIR_OPP);
	IO_setup(GNSS_PWRCTRL_PORT, GNSS_PWRCTRL_PIN, IODIR_OPP);
	/* Power on QL70 */
	IO_wrt(GNSS_PWRCTRL_PORT, GNSS_PWRCTRL_PIN, 1);
	/* De-active QL70 reset pin */
	IO_wrt(GNSS_RST_PORT, GNSS_RST_PIN, 0);
	memset(&lastQC_GNSSdat, 0 , sizeof(GNSS_DAT_t));
	while (1)
	{
		if (!DEVSTT_access()->OnOff)
		{
			if (DEVSTT_access()->GNSSdbg.stdby)
			{
				TSKRECTD_dbgPrt("\r\n %s:GNSS is sleeping", __func__);
				memset(&DEVSTT_access()->GNSS, 0, sizeof(GNSS_DAT_t));
				vTaskDelay(100);
				goto TSKRECTD_END_A_CYCLE;
			}
			else
			{
				if (!t_last_VHoff_gotoSleep)
				{
					t_last_VHoff_gotoSleep = IRTC_getSRT();
				}
				else if (IRTC_getSRT() >= (t_last_VHoff_gotoSleep + 60))
				{
					TSKRECTD_dbgPrt("\r\n %s:sleep GNSS", __func__);
					/* Enable reading data interrupt from GNSS module */
					UART_setup(GNSS_UARTPORT, GNSS_UART_BRATE, GNSS_readc);
					DEVSTT_access()->GNSSdbg.stdby = QL70_sleep() ? 0 : 1;
					/* Disable reading data interrupt from GNSS module (avoid causing system hang when writing to flash) */
					UART_setup(GNSS_UARTPORT, GNSS_UART_BRATE, NULL);
				}
				TSKRECTD_dbgPrt("\r\n %s:TimeUntilSleep=%u", __func__, (IRTC_getSRT() > t_last_VHoff_gotoSleep) ? (IRTC_getSRT() - t_last_VHoff_gotoSleep) : 0);
			}
		}
		else
		{
			t_last_VHoff_gotoSleep = 0;
			if (DEVSTT_access()->GNSSdbg.stdby)
			{
				TSKRECTD_dbgPrt("\r\n %s:wakeup GNSS", __func__);
				/* Enable reading data interrupt from GNSS module */
				UART_setup(GNSS_UARTPORT, GNSS_UART_BRATE, GNSS_readc);
				DEVSTT_access()->GNSSdbg.stdby = QL70_wkup() ? 1 : 0;
				/* Disable reading data interrupt from GNSS module (avoid causing system hang when writing to flash) */
				UART_setup(GNSS_UARTPORT, GNSS_UART_BRATE, NULL);
			}
		}
		memset(&GNSSdat, 0 , sizeof(GNSS_DAT_t));
		/* Enable reading data interrupt from GNSS module */
		UART_setup(GNSS_UARTPORT, GNSS_UART_BRATE, GNSS_readc);
		/* Retry until got GNSS data */
		for (uint8_t rtry = 0; rtry < 2; rtry++)
		{
			vTaskDelay(100);
			QL70_getDat(&GNSSdat);
			if (GNSSdat.RMCavail && (GNSSdat.GGAavail || GNSSdat.GSAavail))
			{
				/* GNSS data is available */
				break;
			}
			TSKRECTD_dbgPrt("\r\n %s:rtry=%u", __func__, rtry + 1);
		}
		/* Disable reading data interrupt from GNSS module (avoid causing system hang when writing to flash) */
		UART_setup(GNSS_UARTPORT, GNSS_UART_BRATE, NULL);
		DEVSTT_access()->GNSS = GNSSdat;
		if (GNSSdat.RMCavail && (GNSSdat.GGAavail || GNSSdat.GSAavail))
		{
			DEVSTT_access()->GNSSdbg.stdby = 0;
			DEVSTT_access()->GNSSdbg.tLastAvail = IRTC_getSRT();
			if ( (GNSSdat.fixQuality)//
					//&& (GNSSdat.HDOP <= 6)//
					)
			{
				DEVSTT_access()->GNSSdbg.tLastQC = IRTC_getSRT();
				lastQC_GNSSdat = GNSSdat;
				{
					//DBG_print("\r\n Update UTC");
					DATETIME_t dt;
					uint32_t tmpUtc = 0;

					dt.day = GNSSdat.dt.day;
					dt.month = GNSSdat.dt.mon;
					dt.year = GNSSdat.dt.year + 2000;
					dt.hour = GNSSdat.dt.hour;
					dt.minute = GNSSdat.dt.min;
					dt.second = GNSSdat.dt.sec;
					tmpUtc = IRTC_dt2utc(dt);
					if (abs(IRTC_getUTC() - tmpUtc) >= 3)
					{
						IRTC_setByUTC(tmpUtc);
					}
				}
				DEVSTT_access()->RTCsync = 1;
				if (SPDsamples != NULL)
				{
					if (uxQueueMessagesWaiting(SPDsamples) == SPDSAMPLE_MAX)
					{
						/* Enough samples -> calculate average value */
						uint32_t sum = 0;
						uint16_t tmpSpd = 0;
						uint8_t numOfSample = 0;
						for (uint8_t i = 0; i < SPDSAMPLE_MAX; i++)
						{
							if (xQueueReceive(SPDsamples, &tmpSpd, 0) != pdTRUE)
							{
								break;
							}
							sum += tmpSpd;
							numOfSample++;
							xQueueSend(SPDsamples, &tmpSpd, 0);
						}
						tmpSpd = sum / numOfSample;
						TSKRECTD_dbgPrt("\r\n %s:avgSpd=%u", __func__, tmpSpd);
						DEVSTT_access()->VHstdby = (tmpSpd <= STDBY_SPEED_MAX) ? 1 : 0;
						xQueueReceive(SPDsamples, &tmpSpd, 0); // Remove oldest sample
					}
					xQueueSend(SPDsamples, &GNSSdat.spd, 0);
				}
				else
				{
					TSKRECTD_dbgPrt("\r\n %s:sample queue not existed", __func__);
				}
			}
			else
			{
				TSKRECTD_dbgPrt("\r\n %s:FQ=%u HDOP=%f -> UnQC", __func__, GNSSdat.fixQuality, GNSSdat.HDOP);
			}
		}
		else
		{
			TSKRECTD_dbgPrt("\r\n %s:GGA=%u GSA=%u RMC=%u -> NA", __func__, GNSSdat.GGAavail, GNSSdat.GSAavail, GNSSdat.RMCavail);
			if (IRTC_getSRT() >= (DEVSTT_access()->GNSSdbg.tLastAvail + WAIT_GNSS_UNTIL_RST_TIMEOUT))
			{
				TSKRECTD_dbgPrt("\r\n <!>%s:RST", __func__);
				IO_wrt(GNSS_RST_PORT, GNSS_RST_PIN, 1);
				vTaskDelay(100);
				IO_wrt(GNSS_RST_PORT, GNSS_RST_PIN, 0);
				TSKLOG_addLogMsg(IRTC_getUTC(), LOGTYPE_GNSS, "Rst GNSS");
				DEVSTT_access()->GNSSdbg.tLastAvail = IRTC_getSRT();
			}
		}
		if (DEVSTT_access()->OnOff != pre_OnOff)
		{
			/* Instantly record and send event */
			recordRequest = 1;
			instantSndTrkDat = 1;
		}
		pre_OnOff = DEVSTT_access()->OnOff;
		if (DEVSTT_access()->OpDr != pre_OpDr)
		{
			/* Instantly record and send event */
			recordRequest = 1;
			instantSndTrkDat = 1;
		}
		pre_OpDr = DEVSTT_access()->OpDr;
		if (!pre_VHstdby && DEVSTT_access()->VHstdby)
		{
			/* Instantly record and send event */
			recordRequest = 1;
			instantSndTrkDat = 1;
		}
		if (SESSIONCFG_access()->stresstest)
		{
			/* Stress test: continuously record and sending data */
			recordRequest = 1;
			instantSndTrkDat = 1;
		}
		pre_VHstdby = DEVSTT_access()->VHstdby;
		if (DEVCFG_access()->trackDist && (calcDist(preRecLat, preRecLng, GNSSdat.lat, GNSSdat.lng) >= DEVCFG_access()->trackDist))
		{
			/* Satisfy 'record data distance' condition */
			recordRequest = 1;
		}
		if (DEVCFG_access()->trackDelayTime && ((IRTC_getSRT() >= (t_last_recTrkDat + DEVCFG_access()->trackDelayTime)) && !DEVSTT_access()->VHstdby))
		{
			/* Satisfy 'record data delay time' condition */
			recordRequest = 1;
		}
		if (DEVCFG_access()->stdbyTrackDelayTime && ((IRTC_getSRT() >= (t_last_recTrkDat + MIN2SEC(DEVCFG_access()->stdbyTrackDelayTime))) && DEVSTT_access()->VHstdby))
		{
			/* Satisfy 'record data delay time on standing by' condition */
			recordRequest = 1;
		}
		if (recordRequest)
		{
			recordRequest = 0;
			/* Satisfying GNSS quality condition and device configuration condition */
			Location loc = Location_init_default;
			/* In a limited time, non-QC GNSS data will be replaced by last QC GNSS data: coordinates, speed and course only */
			if (!DEVSTT_access()->GNSS.fixQuality)
			{
				if (IRTC_getSRT() <= (DEVSTT_access()->GNSSdbg.tLastQC + ALLOW_USE_LASTQC_GNSSDAT_MAXTIME))
				{
					TSKRECTD_dbgPrt("\r\n %s:use last QC GNSSdat", __func__);
					if (lastQC_GNSSdat.lat)
					{
						loc.has_latitude = true;
						loc.latitude = lastQC_GNSSdat.lat;
					}
					if (lastQC_GNSSdat.lng)
					{
						loc.has_longitude = true;
						loc.longitude = lastQC_GNSSdat.lng;
					}
					if (lastQC_GNSSdat.spd)
					{
						loc.has_velocity = true;
						loc.velocity = lastQC_GNSSdat.spd;
					}
					if (lastQC_GNSSdat.course)
					{
						loc.has_carHeading = true;
						loc.carHeading = lastQC_GNSSdat.course;
					}
				}
			}
			else
			{
				if (GNSSdat.lat)
				{
					loc.has_latitude = true;
					loc.latitude = GNSSdat.lat;
				}
				if (GNSSdat.lng)
				{
					loc.has_longitude = true;
					loc.longitude = GNSSdat.lng;
				}
				if (GNSSdat.spd)
				{
					loc.has_velocity = true;
					loc.velocity = GNSSdat.spd;
				}
				if (GNSSdat.course)
				{
					loc.has_carHeading = true;
					loc.carHeading = GNSSdat.course;
				}
			}
			strlcpy(loc.deviceID, DEVID_access(), TRKDAT_DEVID_SIZE);
			loc.unixtime = IRTC_getUTC();
			loc.has_onoff = DEVSTT_access()->OnOff ? true : false;
			loc.onoff = loc.has_onoff;
			loc.has_opendoor = DEVSTT_access()->OpDr ? true : false;
			loc.opendoor = loc.has_opendoor;
			//DBG_print("\r\n TSKRECTRKDAT:ID=%s", trkDat.devID);
			if (DEVSTT_access()->RTCsync)
			{
				TRKDAT_vault_add(VAULT_MAIN, loc);
				if (TRKDAT_vault_add(VAULT_SEND, loc))
				{
					Location tmp;
					//DBG_print("\r\n TSKRECTRKDAT:Unload old data");
					TRKDAT_vault_get(VAULT_SEND, &tmp);
					//DBG_print("\r\n TSKRECTRKDAT:add new data");
					TRKDAT_vault_add(VAULT_SEND, loc);
					if (TRKDAT_vault_add(VAULT_MISS, loc))
					{
						Location tmp2;
						//DBG_print("\r\n TSKRECTRKDAT:Unload old data");
						TRKDAT_vault_get(VAULT_MISS, &tmp2);
						//DBG_print("\r\n TSKRECTRKDAT:add new data");
						TRKDAT_vault_add(VAULT_MISS, loc);
					}
				}
				preRecLat = loc.latitude;
				preRecLng = loc.longitude;
			}
			if (instantSndTrkDat)
			{
				TSKNETCOMM_set_instantSndTrkDat();
				instantSndTrkDat = 0;
			}
			t_last_recTrkDat = IRTC_getSRT();
		}
		TSKRECTD_END_A_CYCLE://
		continue;
	}
}
