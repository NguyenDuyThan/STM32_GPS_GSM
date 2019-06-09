/*
 * taskTakePhoto.c
 *
 *  Created on: Mar 24, 2017
 *      Author: dv198
 */


/*##########################################################################################
 * INCLUDE
 *##########################################################################################*/
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "taskSens.h"
#include "dbgPrint.h"
#include "devstt.h"
#include "devcfg.h"

#include "fileSystem.h"
#include "memMngr.h"
#include "stringlib.h"

#include "STM32F1_io.h"
#include "STM32F1_uart.h"
#include "STM32F1_rtc.h"
#include "RS232camera.h"

#include "stm32f10x_bkp.h"

#include "taskCamera.h"
#include "taskNetworkComm.h"

#include "queue.h"

#include "HWmapping.h"
/*##########################################################################################
 * DEFINES
 *##########################################################################################*/
#define RS232CAM_BUF_SIZE		(512 * 3)
#define RS232CAM_PACKET_SIZE	(512 * 2)
/*##########################################################################################
 * FUNC.PROTOTYPES
 *##########################################################################################*/
static void TSKCAM_dbgPrt_pseudo(const uint8_t *s,...);
static void TSKCAM_main(void *pdata);
static void RS232CAM_sdByte(uint8_t byte);
static void TSKCAM_takePhoto(void);
static void TSKCAM_load2snd(void);
/*##########################################################################################
 * VARIABLES
 *##########################################################################################*/
static TSKCAM_dbgPrt_CB TSKCAM_dbgPrt = TSKCAM_dbgPrt_pseudo;
static uint32_t t_last_scanPhoto = 0;
/*##########################################################################################
 * FUNCTIONS
 *##########################################################################################*/
/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
void TSKCAM_dbgPrt_pseudo(const uint8_t *s,...)
{
	return;
}
/*----------------------------------------------------------------------------------------
 * Brief: setup debug print.
 -----------------------------------------------------------------------------------------*/
void TSKCAM_setup_dbgPrt(TSKCAM_dbgPrt_CB cb)
{
	if (cb == NULL)
	{
		TSKCAM_dbgPrt = TSKCAM_dbgPrt_pseudo;
		return;
	}
	TSKCAM_dbgPrt = cb;
}
/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
uint8_t TSKCAM_setup(uint16_t size, uint32_t prio, xTaskHandle *tskHdl, uint32_t *wotime)
{
	portBASE_TYPE res = xTaskCreate(TSKCAM_main, "CAM", size, NULL, prio, tskHdl);
	return (res == pdTRUE) ? 0 : 1;
}
/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
void TSKCAM_main(void *pdata)
{
	uint32_t t_last_takePhoto = 0;
	uint8_t takePhotoNow = 0;

	IO_setup(EXTRS232_PWRCTRL_PORT, EXTRS232_PWRCTRL_PIN, IODIR_OPP);
	IO_wrt(EXTRS232_PWRCTRL_PORT, EXTRS232_PWRCTRL_PIN, 0);
	UART_setup(CAM_UARTPORT, 115200, RS232CAM_buf_wrtByte);
	while (1)
	{
		if ((IRTC_getSRT() >= (t_last_takePhoto + MIN2SEC(DEVSTT_access()->VHstdby ? DEVCFG_access()->stdbyCAMdelay : DEVCFG_access()->stdbyCAMdelay))) //
				|| !t_last_takePhoto)
		{
			takePhotoNow = 1;
			t_last_takePhoto = IRTC_getSRT();
		}
		if (takePhotoNow)
		{
			IO_wrt(EXTRS232_PWRCTRL_PORT, EXTRS232_PWRCTRL_PIN, 1); // Power on camera
			vTaskDelay(50);
			TSKCAM_takePhoto();
			IO_wrt(EXTRS232_PWRCTRL_PORT, EXTRS232_PWRCTRL_PIN, 0); // Power off camera
			takePhotoNow = 0;
		}
		TSKCAM_dbgPrt("\r\n %s:timeUntilTakePhoto=%u/%u", __func__//
				, DEVSTT_access()->VHstdby, IRTC_getSRT() - t_last_takePhoto//
				, MIN2SEC(DEVSTT_access()->VHstdby ? DEVCFG_access()->stdbyCAMdelay : DEVCFG_access()->stdbyCAMdelay)//
				);
		TSKCAM_load2snd();
		vTaskDelay(100);
	}
}
/*----------------------------------------------------------------------------------------
 * Brief: send 1 byte to RS232 camera.
 * Param:	byte	|	IN	|	byte to send.
 * Note: this function is used as callback function by RS232 camera library.
 -----------------------------------------------------------------------------------------*/
void RS232CAM_sdByte(uint8_t byte)
{
	UART_sendc(CAM_UARTPORT, byte);
}
/*----------------------------------------------------------------------------------------
 * Brief: Say 'cheese!' and take a photo.
 -----------------------------------------------------------------------------------------*/
void TSKCAM_takePhoto(void)
{
	uint8_t *RS232CAM_buf = MM_get(MIN2SEC(2), RS232CAM_BUF_SIZE);
	if (RS232CAM_buf != NULL)
	{
		uint32_t photoSize = 0;
		uint16_t pNum = 0;
		uint8_t res = 0;

		RS232CAM_setup(RS232CAM_BUF_SIZE, RS232CAM_buf, RS232CAM_sdByte, vTaskDelay);
		res = RS232CAM_takePhoto((IMG_RES_t)DEVCFG_access()->CAMres, RS232CAM_PACKET_SIZE, 2000, &photoSize, &pNum);
		TSKCAM_dbgPrt("\r\n %s:TakePhoto:res=%u size=%u num=%u", __func__, res, photoSize, pNum);
		if (!res)
		{
			const uint8_t wrtFSpktNum = 4;
			uint8_t *pDat = MM_get(MIN2SEC(2), RS232CAM_PACKET_SIZE * wrtFSpktNum);
			if (pDat != NULL)
			{
				uint16_t pSize = 0, pDatLen = 0;
				uint8_t fname[96], sLat[12] = "0.0", sLng[12] = "0.0";
				uint8_t rtry = 0, haveFile = 0;

				SL_double2s(DEVSTT_access()->GNSS.lat, 12, sLat);
				SL_double2s(DEVSTT_access()->GNSS.lng, 12, sLng);
				snprintf(fname, 96, "%02u%02u%02u/%lu_%s_%s.jpg"//
						, IRTC_getDT(DEFAULT_SYSTEM_GMT).day//
						, IRTC_getDT(DEFAULT_SYSTEM_GMT).month//
						, IRTC_getDT(DEFAULT_SYSTEM_GMT).year % 100//
						, IRTC_getUTC()//
						, sLat//
						, sLng//
						);
				TSKCAM_dbgPrt("\r\n %s:fname \"%s\"", __func__, fname);
				for (uint16_t i = 1; i <= pNum; i++)
				{
					TSKCAM_MAIN_READPACKET://
					res = RS232CAM_rdPhotoPkt(2000, i, RS232CAM_PACKET_SIZE, &pDat[pDatLen], &pSize);
					TSKCAM_dbgPrt("\r\n %s:rdPhotoPkt[%u]:res=%u size=%u", __func__, i, res, pSize);
					if (res)
					{
						if (rtry++ > 3)
						{
							break;
						}
						goto TSKCAM_MAIN_READPACKET;
					}
					pDatLen += pSize;
					if ((i == pNum) || !(i % wrtFSpktNum))
					{
						TSKCAM_MAIN_WRITEPACKET://
						if (!haveFile)
						{
							/* 1st packet -> create file */
							res = FS_createFile(fname, 1);
							TSKCAM_dbgPrt("\r\n %s:createFile:res=%u", __func__, res);
							if (res)
							{
								if (rtry++ > 3)
								{
									break;
								}
								goto TSKCAM_MAIN_WRITEPACKET;
							}
							haveFile = 1;
						}
						res = FS_append(fname, (uint32_t)pDatLen, pDat);
						TSKCAM_dbgPrt("\r\n %s:append:res=%u", __func__, res);
						if (res)
						{
							if (rtry++ > 3)
							{
								break;
							}
							goto TSKCAM_MAIN_WRITEPACKET;
						}
						memset(pDat, 0, RS232CAM_PACKET_SIZE * wrtFSpktNum);
						pDatLen = 0;
					}
					rtry = 0;
				}
				MM_free(pDat);
			}
		}
		RS232CAM_setup(0, NULL, RS232CAM_sdByte, vTaskDelay);
		MM_free(RS232CAM_buf);
	}
	else
	{
		TSKCAM_dbgPrt("\r\n %s:No memory", __func__);
	}
}
/*----------------------------------------------------------------------------------------
 * Brief: load photo data to send
 -----------------------------------------------------------------------------------------*/
void TSKCAM_load2snd(void)
{
	if ((DEVSTT_access()->sndPhotoViaNATS.fname != NULL) && strlen(DEVSTT_access()->sndPhotoViaNATS.fname))
	{
		if (!DEVSTT_access()->sndPhotoViaNATS.fsize)
		{
			FS_fsize(DEVSTT_access()->sndPhotoViaNATS.fname, &DEVSTT_access()->sndPhotoViaNATS.fsize);
		}
		while (1)
		{
			if (DEVSTT_access()->sndPhotoViaNATS.rd_ofs >= DEVSTT_access()->sndPhotoViaNATS.fsize)
			{
				break;
			}
			if (TSKNETCOMM_chkFull_sndPhotoViaNATS())
			{
				//TSKCAM_dbgPrt("\r\n %s:Queue is full", __func__);
				break;
			}
			TSKCAM_dbgPrt("\r\n %s:F=\"%s\" Ofs=%u/%u", __func__, DEVSTT_access()->sndPhotoViaNATS.fname, DEVSTT_access()->sndPhotoViaNATS.rd_ofs, DEVSTT_access()->sndPhotoViaNATS.fsize);
			{
				uint8_t res = 0;
				uint32_t size = 0;
				File_data1KB *fileBlk1KB = MM_get(MIN2SEC(5), sizeof(File_data1KB));
				if (fileBlk1KB == NULL)
				{
					TSKCAM_dbgPrt("\r\n %s:Low memory", __func__);
					break;
				}
				{
					uint8_t tmp[32], fname[96];

					strlcpy(fname, DEVSTT_access()->sndPhotoViaNATS.fname, 96);
					SL_split(fname, '/', tmp, 32, fname, 96);
					if (SL_search(fname, "_") == -1)
					{
						SL_split(fname, '.', tmp, 32, fname, 96);
						fileBlk1KB->unixtime = atoi(tmp);
					}
					else
					{
						SL_cut(fname, ".jpg");
						SL_split(fname, '_', tmp, 32, fname, 96);
						fileBlk1KB->unixtime = atoi(tmp);
						//TSKCAM_dbgPrt("\r\n %s:parse:utc:%s->%u", __func__, tmp, fileBlk1KB->unixtime);
						SL_split(fname, '_', tmp, 32, fname, 96);
						fileBlk1KB->latitude = SL_s2double(tmp);
						//TSKCAM_dbgPrt("\r\n %s:parse:Lat:%s->%f", __func__, tmp, fileBlk1KB->latitude);
						fileBlk1KB->longitude = SL_s2double(fname);
						//TSKCAM_dbgPrt("\r\n %s:parse:Lng:%s->%f", __func__, fname, fileBlk1KB->longitude);
					}
				}
				fileBlk1KB->has_latitude = fileBlk1KB->latitude ? true : false;
				fileBlk1KB->has_longitude = fileBlk1KB->longitude ? true : false;
				strlcpy(fileBlk1KB->contentType, "image/jpg", 32);
				strlcpy(fileBlk1KB->deviceID, DEVID_access(), 16);
				fileBlk1KB->offset = DEVSTT_access()->sndPhotoViaNATS.rd_ofs;
				size = DEVSTT_access()->sndPhotoViaNATS.fsize - DEVSTT_access()->sndPhotoViaNATS.rd_ofs;
				if (size > 1024)
				{
					size = 1024;
				}
				else
				{
					fileBlk1KB->finished = true;
				}
				res = FS_rd(DEVSTT_access()->sndPhotoViaNATS.fname, DEVSTT_access()->sndPhotoViaNATS.rd_ofs, size, fileBlk1KB->data.bytes, &size);
				if (res)
				{
					MM_free(fileBlk1KB);
					TSKCAM_dbgPrt("\r\n %s:ReadFile:Res=%u rSize=%u->ERR", __func__, res, fileBlk1KB->data.size);
					break;
				}
				fileBlk1KB->data.size = size;
				if (TSKNETCOMM_add_sndPhotoViaNATS(fileBlk1KB))
				{
					MM_free(fileBlk1KB);
					TSKCAM_dbgPrt("\r\n %s:ReadFile:Cannot add to queue", __func__);
					break;
				}
				DEVSTT_access()->sndPhotoViaNATS.rd_ofs += size;
			}
		}
		TSKCAM_dbgPrt("\r\n %s:rO=%u sO=%u S=%u", __func__, DEVSTT_access()->sndPhotoViaNATS.rd_ofs, DEVSTT_access()->sndPhotoViaNATS.snt_ofs, DEVSTT_access()->sndPhotoViaNATS.fsize);
	}
	else if ((IRTC_getSRT() >= (t_last_scanPhoto  + MIN2SEC(1))) || !t_last_scanPhoto)
	{
		uint8_t dname[7], fname[96];
		uint32_t utc = IRTC_getUTC();
		uint16_t matchedCt = 0;
		DATETIME_t dt;

		memset(&DEVSTT_access()->sndPhotoViaNATS, 0, sizeof(SNDFILE_VIA_NATS_t));
		TSKCAM_dbgPrt("\r\n %s:Start searching", __func__);
		while (1)
		{
			dt = IRTC_utc2dt(utc, DEFAULT_SYSTEM_GMT);
			uint32_t fsize = 0;

			snprintf(dname, 7, "%02u%02u%02u", dt.day, dt.month, dt.year % 100);
			TSKCAM_dbgPrt("\r\n %s:Dir \"%s\"", __func__, dname);
			FS_searchFileInDir(dname, 1, ".jpg", 96, fname, &matchedCt);
			if (strlen(fname))
			{
				FS_fsize(fname, &fsize);
				if (fsize)
				{
					/* Found photo(s) */
					TSKCAM_dbgPrt("\r\n %s:Found %u photo(s) in \"%s\"", __func__, matchedCt, dname);
					TSKCAM_dbgPrt("\r\n %s:1st photo is \"%s\"", __func__, fname);
					DEVSTT_access()->sndPhotoViaNATS.fname = MM_get(MIN2SEC(10), strlen(fname) + 1);
					if (DEVSTT_access()->sndPhotoViaNATS.fname != NULL)
					{
						strlcpy(DEVSTT_access()->sndPhotoViaNATS.fname, fname, strlen(fname) + 1);
						DEVSTT_access()->sndPhotoViaNATS.delFileAfterDone = 1;
					}
					break;
				}
			}
			else
			{
				if ((utc <= HOUR2SEC(24)) || ((utc + HOUR2SEC(24 * 90)) < IRTC_getUTC()))
				{
					/* Out of range */
					TSKCAM_dbgPrt("\r\n %s:End searching", __func__);
					break;
				}
			}
			/* Backward 1 day */
			utc -= HOUR2SEC(24);
		}
		t_last_scanPhoto = IRTC_getSRT();
	}
}
