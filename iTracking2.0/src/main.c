/*------------------------------------------------------------------------------------------
 * INTRODUCE
 * Brief: This is main file of "iTracking2.0" project.
 * Author: Duy Vinh To - Team DinhViSo
 * Detail:
 -------------------------------------------------------------------------------------------*/
/*##########################################################################################
 * INCLUDE
 *##########################################################################################*/
#include "taskMain.h"
/* C libraries */
#include "string.h"
#include "stdio.h"
/* STM32 StdPeriph libraries */
#include "stm32f10x_iwdg.h"
#include "stm32f10x_exti.h"
#include "misc.h"
/* DVS hardware libraries */
#include "NORflash.h"
#include "NORflash_branchInfo.h"
#include "STM32F1_tim.h"
#include "STM32F1_uart.h"
#include "STM32F1_rtc.h"
#include "STM32F1_io.h"
#include "STM32F1_flash.h"
#include "STM32F1_spi.h"
#include "QuectelL70.h"
#if (HW_MAIN_VERSION == 1)
#include "SIM900.h"
#endif // #if (HW_MAIN_VERSION == 2)
#if (HW_MAIN_VERSION == 2)
#include "QuectelM95.h"
#endif // #if (HW_MAIN_VERSION == 2)
/* DVS software libraries */
#include "fileSystem.h"
#include "dbgPrint.h"
#include "stringlib.h"
#include "memMngr.h"
#include "md5.h"
/* 3rd software libraries */
#include "jsmn.h"
#include "pb.h"
#include <pb_encode.h>
#include <pb_decode.h>
/* Project specified */
#include "logResetEvt.h"
#include "trackingData.h"
#include "devcfg.h"
#include "devstt.h"
#include "predefFname.h"
#include "usedFlashPages.h"
#include "taskLogging.h"
#include "taskRecTrkDat.h"
#include "taskNetworkComm.h"
#include "taskCmd.h"
#include "taskBkpTrkDat.h"
#include "taskSens.h"
#include "taskBkpTrkDatToFlash.h"
#include "taskRFIDreader.h"
#include "taskCamera.h"
/* Others */
#include "diskio.h"
#include "HWmapping.h"
#include <RTOS tasks/predef_tasks_size.h>
#include "FWversion.h"
#include "NVIC_priorities.h"
/*##########################################################################################
 * DEFINES
 *##########################################################################################*/
#define MAX_TASKLIST		16
/*##########################################################################################
 * TYPEDEFS
 *##########################################################################################*/
typedef struct
{
		xTaskHandle th;
		uint32_t wotime;
}TASKWATCHOUT_t;

/*##########################################################################################
 * FUNC.PROTOTYPES
 *##########################################################################################*/
static void TSKMAIN_dbgPrt_pseudo(const uint8_t *s, ...);
static void *MM_alloc(uint32_t size);
static void MM_dealloc(void *addr);
static void TIM2_tickCB(void);
static void NORFL_delayms(uint32_t t);
static void NORFL_wrtCSpin(uint8_t val);
static uint8_t NORFL_wrSPI(uint8_t dat);
static void DBG_printc(uint8_t c);
static void RTC_tick(void);
static void stresstestFlash(void);
static void IWDG_setup(void);
/*##########################################################################################
 * VARIABLES
 *##########################################################################################*/
static TSKMAIN_dbgPrt_CB TSKMAIN_dbgPrt = TSKMAIN_dbgPrt_pseudo;
static TASKWATCHOUT_t taskList[MAX_TASKLIST];

static uint32_t t_last_TSKMAIN_startCycle = 0//
		;
static uint32_t _1msTickCt = 0;
static uint8_t wait4FileSystem = 0;
static uint32_t countup = 0;
/*##########################################################################################
 * FUNCTIONS
 *##########################################################################################*/
/*----------------------------------------------------------------------------------------
 * Brief: debug printing pseudo function
 * 			(Just to avoid hanging until mount true print function).
 * Param:	s	|	I	|	string format
 -----------------------------------------------------------------------------------------*/
void TSKMAIN_dbgPrt_pseudo(const uint8_t *s, ...)
{
	return;
}
/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
void *MM_alloc(uint32_t size)
{
	return pvPortMalloc(size);
}
/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
void MM_dealloc(void *addr)
{
	vPortFree(addr);
}
/*----------------------------------------------------------------------------------------
 * Brief: Delay millisecond
 * Param:	t	|	I	|	delay time
 -----------------------------------------------------------------------------------------*/
void TIM2_tickCB(void)
{
	_1msTickCt++;
}
/*----------------------------------------------------------------------------------------
 * Brief: Delay millisecond
 * Param:	t	|	I	|	delay time
 -----------------------------------------------------------------------------------------*/
void NORFL_delayms(uint32_t t)
{
	_1msTickCt = 0;
	while(_1msTickCt <= t);
//	if (t < 10)
//	{
//		t = 10;
//	}
//	vTaskDelay(t / 10);
}
/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
void NORFL_wrtCSpin(uint8_t val)
{
#if (HW_MAIN_VERSION == 2)
	IO_wrt(EXTFLASH_CS_PORT, EXTFLASH_CS_PIN, val);
#endif // #if (HW_MAIN_VERSION == 2)
}
/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
uint8_t NORFL_wrSPI(uint8_t dat)
{
	return SPI_wr1Byte(1, dat);
}
/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
void TSKMAIN_setup_dbgPrt(TSKMAIN_dbgPrt_CB cb)
{
	if (cb == NULL)
	{
		TSKMAIN_dbgPrt = TSKMAIN_dbgPrt_pseudo;
		return;
	}
	TSKMAIN_dbgPrt = cb;
}
/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------
 * Brief: Main task.
 * To-do-list:
 * + Setup file system.
 * + Setup other tasks.
 -----------------------------------------------------------------------------------------*/
void TSKMAIN_main (void *pdata)
{
	uint32_t prio = 1//
			, t_last_memShow = 0//
			, t_last_save2flash = 0//
			//, t_last_4KmemFreeAvail = 0//
			, t_last_log = 0//
			//, timepass = 0;//
			;
	uint8_t taskList_index = 1//
			, LEDtick = 0//
			//, GSMsignalquality = 0//
			//, sNet[32]//
			;

	DBG_print("\r\n Setup IO(s)");
	/* Setup LED indicator pinout */
	IO_setup(LEDPWR_PORT, LEDPWR_PIN, IODIR_OPP);
#if (HW_MAIN_VERSION == 2)
	IO_setup(LEDGSM_PORT, LEDGSM_PIN, IODIR_OPP);
	IO_setup(LEDLINK_PORT, LEDLINK_PIN, IODIR_OPP);
	IO_setup(LEDGNSS_PORT, LEDGNSS_PIN, IODIR_OPP);
	IO_setup(LEDLOGON_PORT, LEDLOGON_PIN, IODIR_OPP);
	IO_setup(LEDSDC_PORT, LEDSDC_PIN, IODIR_OPP);
	IO_setup(EXTFLASH_CS_PORT, EXTFLASH_CS_PIN, IODIR_OPP);
#endif // #if (HW_MAIN_VERSION == 2)
	IO_wrt(LEDPWR_PORT, LEDPWR_PIN, 0);
#if (HW_MAIN_VERSION == 2)
	IO_wrt(LEDGSM_PORT, LEDGSM_PIN, 0);
	IO_wrt(LEDLINK_PORT, LEDLINK_PIN, 0);
	IO_wrt(LEDGNSS_PORT, LEDGNSS_PIN, 0);
	IO_wrt(LEDLOGON_PORT, LEDLOGON_PIN, 0);
	IO_wrt(LEDSDC_PORT, LEDSDC_PIN, 0);
	IO_wrt(EXTFLASH_CS_PORT, EXTFLASH_CS_PIN, 1);
#endif // #if (HW_MAIN_VERSION == 2)
	DBG_print("\r\n Setup flash access");
	IFLASH_setup(vTaskDelay);
	DBG_print("\r\n Setup RTC");
	{
		IRTC_setup(RTC_tick);
	}
	/* Setup memory manager */
	MM_setup(MM_alloc, MM_dealloc, vTaskDelay);
	/* Load local configuration */
	DBG_print("\r\n Load device ID");
	DEVID_load();
	DBG_print("\r\n ID=%s", DEVID_access());
	DBG_print("\r\n Load local configuration");
	LOCALCFG_load();
	/* Load device configuration */
	DBG_print("\r\n Load device configuration");
	DEVCFG_load();
	/**/
	DBG_print("\r\n Load APN profiles");
	APNPROFILE_load();
#if 1
	/* Setup timer used on fileSystem libraries */
	TIM_setup(2, TIMFREQ_100Hz, disk_timerproc);
	/* Setup file system. */
	FS_setup(vTaskDelay);
	FS_setup_dbgPrt(DBG_print);
	wait4FileSystem = 1;
	{
		/* Mount file system. */
		uint8_t res = FS_mount();
		if (res)
		{
			DBG_print("\r\n Format fileSystem");
			FS_format();
			IWDG_ReloadCounter();
			res = FS_mount();
		}
		DEVSTT_access()->fs.rdy = (!res) ? 1 : 0;
		DBG_print("\r\n FileSystem is %sready", (DEVSTT_access()->fs.rdy) ? "" : "not ");
	}
	if (!DEVSTT_access()->fs.rdy)
	{
		/* Turn off SDcard power if there was no SDcard, or SDcard is broken */
		//IO_wrt(SDC_PWR_PORT, SDC_PWR_PIN, 0);
	}
	FS_setup_dbgPrt(NULL);
	wait4FileSystem = 0;
#endif
#if 0
	{
		U8 manId, memType, memCap;

		//TSKMAIN_dbgPrt("\r\n Setup SPI");
		SPI_setup(1, 1, SPI_SPEED_HIGH);
		/* Setup timer 2: use for delay 1ms */
		//TSKMAIN_dbgPrt("\r\n Setup TIM2");
		TIM_setup(2, TIMFREQ_1KHz, TIM2_tickCB);
		//TSKMAIN_dbgPrt("\r\n Setup NORFL DBG");
		NORFL_setup_dbgPrt(DBG_print);
		//TSKMAIN_dbgPrt("\r\n Setup NORFL");
		NORFL_setup(NORFL_delayms, NORFL_wrSPI, NORFL_wrtCSpin, CYPRESS_S25FL256S_256Mb_PAGE_SIZE, CYPRESS_S25FL256S_256Mb_SECTOR_SIZE);
		//TSKMAIN_dbgPrt("\r\n Read NORFL ID");
		NORFL_readId(&manId, &memType, &memCap, 0, NULL);
		//TSKMAIN_dbgPrt("\r\n disk_initialize:%X,%X,%X", manId, memType, memCap);
		if ((manId != CYPRESS_MANID) //s
				|| (memType != CYPRESS_S25FL256S_256Mb_MEMTYPE) //
				|| (memCap != CYPRESS_S25FL256S_256Mb_MEMCAP))
		{
			DEVSTT_access()->flashAvail = 0;
		}
		else
		{
			DEVSTT_access()->flashAvail = 1;
		}
	}
	DEVSTT_access()->fs.rdy = 0;
#endif
	/* Setup default device configuration */
	/* Load device status from file system */
	if (DEVSTT_access()->fs.rdy)
	{
		U32 rSize = 0;
		DEVSTT_t tmp;
		memset(DEVSTT_access(), 0, sizeof(DEVSTT_t));
		FS_rd(DEVSTTBKP_FNAME, 0, sizeof(DEVSTT_t), &tmp, &rSize);
		if (tmp.RBtrkDat.utc <= 1000000000)
		{
			tmp.RBtrkDat.size = 0;
			tmp.RBtrkDat.nextOfs = 0;
			tmp.RBtrkDat.ofs = 0;
			tmp.RBtrkDat.utc = IRTC_getUTC();
		}
		DEVSTT_access()->RBtrkDat = tmp.RBtrkDat;
		DEVSTT_access()->fs.rdy = 1;
	}
	/**/
	TRKDAT_vault_setup();
	//SESSIONCFG_access()->dbgGSM = 1;
	/**/
	memset(taskList, 0, sizeof(TASKWATCHOUT_t) * MAX_TASKLIST);
#if 1//SETUP_TASKS
	DBG_print("\r\n Setup tasks");
#if 1
	if (!TSKRECTD_setup(TSKRECTD_SIZE, prio++, &taskList[taskList_index].th, &taskList[taskList_index].wotime))
	{
		taskList_index++;
	}
#endif
	if (DEVSTT_access()->flashAvail)
	{
		if (!TSKBKPTRKDAT2FL_setup(TSKBKPTRKDAT2FL_SIZE, prio++, &taskList[taskList_index].th, &taskList[taskList_index].wotime))
		{
			taskList_index++;
		}
	}
	if (DEVSTT_access()->fs.rdy)
	{
#if 1
		if (!TSKBKPTD_setup(TSKBKPTD_SIZE, prio++, &taskList[taskList_index].th, &taskList[taskList_index].wotime))
		{
			taskList_index++;
		}
#endif
#if 1
		if (!TSKLOG_setup(TSKLOG_SIZE, prio++, &taskList[taskList_index].th, &taskList[taskList_index].wotime))
		{
			taskList_index++;
			{
				uint16_t rstEvts = LOGRSTEVT_read();
				uint8_t slog[10];
				DBG_print("\r\n Last Reset causes: %X", rstEvts);
				LOGRSTEVT_wrt(0);
				snprintf(slog, 10, "%X", rstEvts);
				TSKLOG_addLogMsg(IRTC_getUTC(), LOGTYPE_RSTCAUSE, slog);
			}
		}
#endif
#if 1
		if (!TSKCAM_setup(TSKCAM_SIZE, prio++, &taskList[taskList_index].th, &taskList[taskList_index].wotime))
		{
			//TSKCAM_setup_dbgPrt(DBG_print);
			taskList_index++;
		}
#endif
	}
	else
	{
		DBG_print("\r\n Skip tasks use file system");
	}
#if 1
	/* Setup task network communication and GSM module */
	if (!TSKNETCOMM_setup(TSKNETCOMM_SIZE, prio++, &taskList[taskList_index].th, &taskList[taskList_index].wotime))
	{
		taskList_index++;
		TSKNETCOMM_add_sndFBViaNATS(FW_VERSION);
	}
#endif
#if 1
	if (!TSKNETCOMMIR_setup(TSKNETCOMMIR_SIZE, prio++, &taskList[taskList_index].th, &taskList[taskList_index].wotime))
	{
		taskList_index++;
	}
#endif
#if 1
	if (!TSKCMD_setup(TSKCMD_SIZE, prio++, &taskList[taskList_index].th, &taskList[taskList_index].wotime))
	{
		taskList_index++;
	}
#endif
#if 1
	if (!TSKSENS_setup(TSKSENS_SIZE, prio++, &taskList[taskList_index].th, &taskList[taskList_index].wotime))
	{
		taskList_index++;
	}
#endif
#if 0
	if (!TSKRFIDREADER_setup(TSKRFIDREADER_SIZE, prio++, &taskList[taskList_index].th, &taskList[taskList_index].wotime))
	{
		taskList_index++;
	}
#endif
#endif // SETUP_TASKS
	DBG_print("\r\n Initialing device: done\r\n");
	IWDG_ReloadCounter();
	DEVSTT_access()->initDone = 1;
	while( 1)
	{
#if HW_MAIN_VERSION == 2
		if (DEVSTT_access()->sens.BATpct && (DEVSTT_access()->sens.BATpct <= 5))
		{
			/* Battery is too low -> Off system */
			IO_wrt(MCUSYSOFF_PORT, MCUSYSOFF_PIN, 0);
		}
#endif // #if HW_MAIN_VERSION == 2
		if ((IRTC_getSRT() >= HOUR2SEC(24)) && (DEVSTT_access()->VHstdby))
		{
			/* Device need to be refreshed after a working day. */
			DEVSTT_access()->rdy2rst = 1;
		}
		if (t_last_TSKMAIN_startCycle != IRTC_getSRT())
		{
			t_last_TSKMAIN_startCycle = IRTC_getSRT();
			/* Reload watchdog to inform MCU that app is working */
			IWDG_ReloadCounter();
			/* Display debug trace */
			if (SESSIONCFG_access()->enTRACE)
			{
				TSKMAIN_dbgPrt("\r\n SRT=%u", IRTC_getSRT());
			}
			/* Check if it's time to reset device */
			if (DEVSTT_access()->rdy2rst)
			{
				xTaskStatusType *taskStatusList = NULL;
				uint8_t taskCt = uxTaskGetNumberOfTasks();
				uint32_t totalRunTime = 0;

				taskStatusList = MM_get(0, sizeof(xTaskStatusType) * taskCt);
				if (taskStatusList != NULL)
				{
					uxTaskGetSystemState(taskStatusList, taskCt, &totalRunTime);
					for (uint8_t i = 0; i < taskCt; i++)
					{
						if (strcmp(taskStatusList[i].pcTaskName, "MAIN") && strcmp(taskStatusList[i].pcTaskName, "IDLE"))
						{
							/* Delete all tasks except for 'main' and 'idle' tasks */
							DBG_print("\r\n <!>DELETE TASK '%s'", taskStatusList[i].pcTaskName);
							vTaskDelete(taskStatusList[i].xHandle);
						}
					}
				}
				if (DEVSTT_access()->fs.rdy)
				{
					/* Unmount file system before reseting */
					DBG_print("\r\n <!>UNMOUNT FILE SYSTEM\r\n");
					FS_unmount();
				}
				DBG_print("\r\n <!>RESET DEVICE\r\n");
				NVIC_SystemReset();
			}
			if ((IRTC_getSRT() >= (t_last_memShow + 2)) //
					&& (SESSIONCFG_access()->enTRACE)//
					)
			{
				uint32_t totalUM = 0; // Total used memory.

				float AFR = 0; // Allocation fail rate
				TSKMAIN_dbgPrt("\r\n <MEMSHOW>");

				TSKMAIN_dbgPrt("\r\n  1.Tasks:");
				{
					xTaskStatusType *taskStatusList = NULL;
					uint8_t taskCt = uxTaskGetNumberOfTasks();
					uint32_t totalRunTime = 0;

					TSKMAIN_dbgPrt("\r\n   NumOfTsks:%u", taskCt);
					taskStatusList = MM_get(10, sizeof(xTaskStatusType) * taskCt);
					if (taskStatusList != NULL)
					{
						uxTaskGetSystemState(taskStatusList, taskCt, &totalRunTime);
						for (uint8_t i = 0; i < taskCt; i++)
						{
							TSKMAIN_dbgPrt("\r\n   +[%u]'%s':UM=%u RT=%u STT=%u" //
									, i//
									, taskStatusList[i].pcTaskName//
									, taskStatusList[i].usStackHighWaterMark//
									, taskStatusList[i].ulRunTimeCounter//
									, taskStatusList[i].eCurrentState//
							);
						}
					}
				}
				TSKMAIN_dbgPrt("\r\n  2.Mem:");
				TSKMAIN_dbgPrt("\r\n   HeapFreeMem=%u", xPortGetFreeHeapSize());
				//totalUM = MM_show(DBG_print, "\r\n   +%u:S=%u LT=%u/%u");
				MM_show(DBG_print, "\r\n   +[%03u]:S=%04u T=%04u/%04u", &totalUM, &AFR);
				TSKMAIN_dbgPrt("\r\n   TotalUM=%u AFR=%f", totalUM, AFR);
				TSKMAIN_dbgPrt("\r\n  3.TrkDatVault:");
				TSKMAIN_dbgPrt("\r\n   +MAIN:%u", TRKDAT_vault_count(VAULT_MAIN));
				TSKMAIN_dbgPrt("\r\n   +SEND:%u", TRKDAT_vault_count(VAULT_SEND));
				TSKMAIN_dbgPrt("\r\n   +MISS:%u", TRKDAT_vault_count(VAULT_MISS));
				TSKMAIN_dbgPrt("\r\n   +ROLLBACK:%u", TRKDAT_vault_count(VAULT_ROLLBACK));
				TSKMAIN_dbgPrt("\r\n </MEMSHOW>");
				t_last_memShow = IRTC_getSRT();
			}
			{
				uint8_t *sLog = NULL;

				sLog = MM_get(10, 4096);
				if (sLog != NULL)
				{
					DEVSTT_genRpt(4096, sLog);
					if (SESSIONCFG_access()->enTRACE)
					{
						DATETIME_t dt = IRTC_getDT(DEFAULT_SYSTEM_GMT);
						TSKMAIN_dbgPrt("\r\n##### START #####");
						TSKMAIN_dbgPrt("\r\nSysDT=%u-%u-%u %u:%u:%u PS=%u (%s)", dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second, IRTC_getPrescaler(), DEVSTT_access()->RTCsync ? "sync" : "unsync");
						TSKMAIN_dbgPrt(sLog);
						TSKMAIN_dbgPrt("%s\r\n##### END #####");
					}
					if ((IRTC_getSRT() >= (t_last_log + 60)) || (t_last_log == 0))
					{
						TSKLOG_addLogMsg(IRTC_getUTC(), LOGTYPE_DEVSTT, sLog);
						t_last_log = IRTC_getSRT();
					}
					MM_free(sLog);
				}
			}
			/* Auto save device configuration */
#if 1
			if (IRTC_getSRT() >= (t_last_save2flash + 10))
			{
				TSKMAIN_dbgPrt("\r\n %s:Save2flash", __func__);
				taskDISABLE_INTERRUPTS();
				LOCALCFG_save();
				DEVCFG_save();
				DEVID_save();
				APNPROFILE_save();
				taskENABLE_INTERRUPTS();
				t_last_save2flash = IRTC_getSRT();
			}
#endif
			/* Handling memory management */
			MM_timertick1sCB();
		}
		/* controlling indicator LEDs */
#if (HW_MAIN_VERSION == 2)
		if (IRTC_getSRT() <= (DEVSTT_access()->GNSSdbg.tLastAvail + 10))
		{
			if (DEVSTT_access()->GNSS.fixQuality)
			{
				IO_wrt(LEDGNSS_PORT, LEDGNSS_PIN, !(LEDtick % 10) ? 1 : 0);
			}
			else
			{
				if (DEVSTT_access()->GNSS.NumOfSat)
				{
					IO_wrt(LEDGNSS_PORT, LEDGNSS_PIN, !(LEDtick % 5) ? 1 : 0);
				}
				else
				{
					IO_wrt(LEDGNSS_PORT, LEDGNSS_PIN, !(LEDtick % 2) ? 1 : 0);
				}
			}
		}
		else
		{
			IO_wrt(LEDGNSS_PORT, LEDGNSS_PIN, 0);
		}
#endif // #if (HW_MAIN_VERSION == 2)
		IO_wrt(LEDPWR_PORT, LEDPWR_PIN, !(LEDtick % (DEVSTT_access()->sens.PwrConn ? 10 : 20)) ? 1 : 0);
#if (HW_MAIN_VERSION == 2)
		IO_wrt(LEDGSM_PORT, LEDGSM_PIN, ((DEVSTT_access()->net.netRegStt == NRS_REG) || (DEVSTT_access()->net.netRegStt == NRS_ROAMING)) ? (!(LEDtick % 10) ? 1 : 0) : 0);
		IO_wrt(LEDLINK_PORT, LEDLINK_PIN, (GETBIT(DEVSTT_access()->net.connect2servBits, C2SVBIT_NATS_PUBLOC)) ? (!(LEDtick % 10) ? 1 : 0) : 0);
		IO_wrt(LEDSDC_PORT, LEDSDC_PIN, (DEVSTT_access()->fs.rdy) ? (!(LEDtick % 10) ? 1 : 0) : 0);
#endif // #if (HW_MAIN_VERSION == 2)
		if (LEDtick++ >= 100)
		{
			LEDtick = 1;
		}
#if 0
		IO_toggle(LEDPWR_PORT, LEDPWR_PIN);
		IO_wrt(LEDGSM_PORT, LEDGSM_PIN, ((DEVSTT_access()->net.netRegStt == NRS_REG) || (DEVSTT_access()->net.netRegStt == NRS_ROAMING)) ? 1 : 0);
		IO_wrt(LEDLINK_PORT, LEDLINK_PIN, GETBIT(DEVSTT_access()->net.connect2servBits, C2SVBIT_NATS_PUBLOC));
#endif
		vTaskDelay(10);
	}
}
/*----------------------------------------------------------------------------------------
 * Brief: Print character callback function for debug feature.
 -----------------------------------------------------------------------------------------*/
void DBG_printc(uint8_t c)
{
	UART_sendc(TERM_UARTPORT, c);
}
#define TEST_QM95	0
/*----------------------------------------------------------------------------------------
 * Brief: Main function.
 -----------------------------------------------------------------------------------------*/
int main (void)
{
	/* Recover from jump */
	//SystemInit();
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x1A000);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	EXTI_DeInit();
#if !TEST_QM95
#if HW_MAIN_VERSION == 2
	IO_setup(MCUSYSOFF_PORT, MCUSYSOFF_PIN, IODIR_OPP);
	IO_wrt(MCUSYSOFF_PORT, MCUSYSOFF_PIN, 1);
#endif // #if HW_MAIN_VERSION == 2
	/* Setup UART1 for debug interface */
	UART_setup(TERM_UARTPORT, 115200, NULL);
	/* Setup debug interface */
	DBG_setup(DBG_printc);
	DBG_print("\r\n+----- iTracking2.0:APP -----+\r\n");
	DBG_print("\r\n HW_MAIN_VERSION:%u", HW_MAIN_VERSION);
	DBG_print("\r\n FW_BUILTDATE:%s", FW_BUILTDATE);
	DBG_print("\r\n FW_VERSION:%s", FW_VERSION);
	DBG_print("\r\n LOCSIZE:%u", sizeof(Location));
	IWDG_setup();
	/* Create main task */
	memset(taskList, 0, sizeof(TASKWATCHOUT_t) * MAX_TASKLIST);
	xTaskCreate(TSKMAIN_main, "MAIN", TSKMAIN_SIZE, NULL, configMAX_PRIORITIES - 1, taskList[0].th);
	/* Start RTOS */
	DBG_print("\r\n START RTOS\r\n");
	vTaskStartScheduler();
	NVIC_SystemReset();
#else
	void UART1_rdc(uint8_t c)
	{
		UART_sendc(2, c);
	}
	void UART2_rdc(uint8_t c)
	{
		UART_sendc(1, c);
	}
	UART_setup(1, 115200, UART1_rdc);
	UART_setup(2, 115200, UART2_rdc);
	while (1);
#endif
}
/*-----------------------------------------------------------------------------
 * Brief: setup watch dog
 ------------------------------------------------------------------------------*/
void IWDG_setup(void)
{
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);

	/* IWDG timeout equal to 350ms (the timeout may varies due to LSI frequency
	 dispersion) -------------------------------------------------------------*/
	/* Enable write access to IWDG_PR and IWDG_RLR registers */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

	/* IWDG counter clock: 32KHz(LSI) / 32 = 1KHz */
	IWDG_SetPrescaler(IWDG_Prescaler_256);

	/* Set counter reload value to 349 */
	IWDG_SetReload(1799);

	/* Reload IWDG counter */
	IWDG_ReloadCounter();

	/* Enable IWDG (the LSI oscillator will be enabled by hardware) */
	IWDG_Enable();
}
/*-----------------------------------------------------------------------------
 * Brief: RTC tick callback
 ------------------------------------------------------------------------------*/
void RTC_tick(void)
{
	if (wait4FileSystem)
	{
		IO_toggle(LEDPWR_PORT, LEDPWR_PIN);
		IWDG_ReloadCounter();
	}
}
/*-----------------------------------------------------------------------------
 * Brief:
 ------------------------------------------------------------------------------*/
void stresstestFlash(void)
{
	U8 redo = 0;
	U8 dat[] = "This is test data";
	while (redo++ < 10)
	{
		DBG_print("\r\n Redo=%u", redo);
		IFLASH_wrt(FLASH_PAGEADDR(TEST_FLASHPAGE), strlen(dat) + 1, dat);
		IFLASH_massErase(TEST_FLASHPAGE, 1);
	}
}
/*-----------------------------------------------------------------------------
 * Brief: default task that called when RTOS is idle.
 * Note: Enter low power mode.
 ------------------------------------------------------------------------------*/
void vApplicationIdleHook(void)
{
	NVIC_SystemLPConfig(NVIC_LP_SLEEPONEXIT, ENABLE);
}
/*-----------------------------------------------------------------------------
 * Brief: default task that called when RTOS is ticking.
 * Note: Exit low power mode.
 ------------------------------------------------------------------------------*/
void vApplicationTickHook(void)
{
	NVIC_SystemLPConfig(NVIC_LP_SLEEPONEXIT, DISABLE);
}
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/*
 * Minimal __assert_func used by the assert() macro
 * */
void __assert_func(const char *file, int line, const char *func, const char *failedexpr)
{
  while(1)
  {}
}

/*
 * Minimal __assert() uses __assert__func()
 * */
void __assert(const char *file, int line, const char *failedexpr)
{
   __assert_func (file, line, NULL, failedexpr);
}
