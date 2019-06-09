/*------------------------------------------------------------------------------------------
 * INTRODUCE
 * Brief: This is source file of task logging.
 * Author: Duy Vinh To - Team DinhViSo
 * Detail: periodically logging device state into file system.
 -------------------------------------------------------------------------------------------*/

/*##########################################################################################
 * INCLUDE
 *##########################################################################################*/
#include "stdio.h"
#include "string.h"
#include "taskSens.h"
#include "dbgPrint.h"
#include "devstt.h"
#include "devcfg.h"

#include "STM32F1_io.h"
#include "STM32F1_adc.h"
#include "STM32F1_rtc.h"

#include "stm32f10x_bkp.h"

#include "taskLogging.h"

#include "queue.h"

#include "HWmapping.h"
/*##########################################################################################
 * DEFINES
 *##########################################################################################*/
#define ADCVAL_AS_ON	7	00
#define EGON_ADCSAMPLE_MAX	50
#define BAT_ADCVAL_FULL			2543 // ~ (4.1V / 2)
#define BAT_ADCVAL_LDOINPUTMIN	2171 // ~ (3.5V / 2)
#define BAT_ADCVAL_GNSSOFF		1737 // ~ (2.8V / 2)
#define BAT_ADCVAL_SDCOFF		1675 // ~ (2.7V / 2)
#define BAT_ADCVAL_MCUSOFF 		1240 // ~ (2V / 2)
#define BAT_ADCVAL_MAX			BAT_ADCVAL_FULL
#define BAT_ADCVAL_MIN			BAT_ADCVAL_LDOINPUTMIN
#define PWR_ADCVAL_MIN			100
/*##########################################################################################
 * FUNC.PROTOTYPES
 *##########################################################################################*/
static void TSKSENS_dbgPrt_pseudo(const uint8_t *s,...);
static void TSKSENS_main(void *pdata);

/*##########################################################################################
 * VARIABLES
 *##########################################################################################*/
static TSKSENS_dbgPrt_CB TSKSENS_dbgPrt = TSKSENS_dbgPrt_pseudo;
static xQueueHandle EGOn_ADCsamples = NULL;
/*##########################################################################################
 * FUNCTIONS
 *##########################################################################################*/
/*----------------------------------------------------------------------------------------
 * Brief: setup logging task.
 -----------------------------------------------------------------------------------------*/
void TSKSENS_dbgPrt_pseudo(const uint8_t *s,...)
{
	return;
}
/*----------------------------------------------------------------------------------------
 * Brief: setup debug print.
 -----------------------------------------------------------------------------------------*/
void TSKSENS_setup_dbgPrt(TSKSENS_dbgPrt_CB cb)
{
	if (cb == NULL)
	{
		TSKSENS_dbgPrt = TSKSENS_dbgPrt_pseudo;
		return;
	}
	TSKSENS_dbgPrt = cb;
}
/*----------------------------------------------------------------------------------------
 * Brief: setup logging task.
 -----------------------------------------------------------------------------------------*/
uint8_t TSKSENS_setup(uint16_t size, uint32_t prio, xTaskHandle *tskHdl, uint32_t *wotime)
{
	portBASE_TYPE res = xTaskCreate(TSKSENS_main, "SENS", size, NULL, prio, tskHdl);
	if (res == pdTRUE)
	{
		EGOn_ADCsamples = xQueueCreate(EGON_ADCSAMPLE_MAX, sizeof(uint16_t));
	}
	return (res == pdTRUE) ? 0 : 1;
}
/*----------------------------------------------------------------------------------------
 * Brief: task sensing: main function.
 * To-do-list:
 * + Reading signal source: key signal, fuel, wheel pulse...
 * + Reading sensor: temperature, fuel...
 -----------------------------------------------------------------------------------------*/
void TSKSENS_main(void *pdata)
{
	uint16_t ADCval = 0;
	uint8_t pre_EGOn = 0;

	/* Read engine ADC values from backup register */
	SESSIONCFG_access()->EG_ADCmin = BKP_ReadBackupRegister(BKPDR_EG_ADC);
	if (!SESSIONCFG_access()->EG_ADCmin)
	{
		SESSIONCFG_access()->EG_ADCmin = 0xFFFF;
	}
	while (1)
	{
		if (DEVSTT_access()->rdy2rst)
		{
			//vTaskDelete(NULL);
		}
#if HW_MAIN_VERSION == 2
		/* ADC1 channel 8: battery */
		ADC1IN8_rd(&ADCval);
		if (ADCval <= BAT_ADCVAL_MIN)
		{
			DEVSTT_access()->sens.BATpct = 0;
		}
		else if (ADCval >= BAT_ADCVAL_MAX)
		{
			DEVSTT_access()->sens.BATpct = 100;
		}
		else
		{
			DEVSTT_access()->sens.BATpct = (float)(ADCval - BAT_ADCVAL_MIN) * 100 / (float)(BAT_ADCVAL_MAX - BAT_ADCVAL_MIN);
		}
#endif // #if HW_MAIN_VERSION == 2
		/* ADC1 Channel 10: read power to detect if engine were on/off */
		ADC1IN10_rd(&ADCval);
		if (ADCval < PWR_ADCVAL_MIN)
		{
			/* Input power voltage is too low -> disconnected */
			DEVSTT_access()->sens.PwrConn = 0;
			DEVSTT_access()->sens.EGOn = 1;/* Fake value to avoid filter on server */
			TSKSENS_dbgPrt("\r\n %s:ADCval=%u -> PwrConn=0", __func__);
		}
		else
		{
			DEVSTT_access()->sens.PwrConn = 1;
			if (EGOn_ADCsamples != NULL)
			{
				if (uxQueueMessagesWaiting(EGOn_ADCsamples) == EGON_ADCSAMPLE_MAX)
				{
					/* Enough samples -> calculate average value */
					uint32_t sum = 0;
					uint16_t val = 0;
					uint8_t numOfSample = 0;
					for (uint8_t i = 0; i < EGON_ADCSAMPLE_MAX; i++)
					{
						if (xQueueReceive(EGOn_ADCsamples, &val, 0) != pdTRUE)
						{
							break;
						}
						sum += val;
						numOfSample++;
						xQueueSend(EGOn_ADCsamples, &val, 0);
					}
					xQueueReceive(EGOn_ADCsamples, &val, 0); // Remove oldest sample
					if (//
							(DEVSTT_access()->GNSS.RMCavail && (DEVSTT_access()->GNSS.GGAavail || DEVSTT_access()->GNSS.GSAavail))//
							&& (DEVSTT_access()->GNSS.fixQuality)//
							&& !DEVSTT_access()->VHstdby//
							)
					{
						SESSIONCFG_access()->EG_ADCmin = sum / numOfSample * 4 / 5;
					}
					else
					{
						uint16_t tmp = sum / numOfSample;
						if (SESSIONCFG_access()->EG_ADCmin >= tmp)
						{
							SESSIONCFG_access()->EG_ADCmin = tmp * 6 / 5;
						}
					}
					/* Save in backup register */
					IRTC_enableBackupAccess();
					BKP_WriteBackupRegister(BKPDR_EG_ADC, SESSIONCFG_access()->EG_ADCmin);
				}
				xQueueSend(EGOn_ADCsamples, &ADCval, 0);
			}
			else
			{
				TSKSENS_dbgPrt("\r\n %s:sample queue not existed", __func__);
			}
			DEVSTT_access()->sens.EGOn = (ADCval > SESSIONCFG_access()->EG_ADCmin) ? 1 : 0;
			if (pre_EGOn != DEVSTT_access()->sens.EGOn)
			{
				uint8_t sLog[32];

				snprintf(sLog, 32, "EG_ADC=%u/%u ->EGOn=%u", ADCval, SESSIONCFG_access()->EG_ADCmin, DEVSTT_access()->sens.EGOn);
				TSKLOG_addLogMsg(IRTC_getUTC(), LOGTYPE_EGON, sLog);
				pre_EGOn = DEVSTT_access()->sens.EGOn;
			}
			TSKSENS_dbgPrt("\r\n %s:ADCv=%u EGAmin=%u->On=%u", __func__, ADCval, SESSIONCFG_access()->EG_ADCmin, DEVSTT_access()->sens.EGOn);
		}
		/* ADC1 channel 9: Fuel analog input */
		ADC1IN9_rd(&ADCval);
		DEVSTT_access()->sens.fuel = 100 - (ADCval * 100 / 4095);
		/* Read key signal */
		DEVSTT_access()->sens.KSOn = !IO_rd(KEYSIGNAL_PORT, KEYSIGNAL_PIN);
		/* Read door open signal */
		DEVSTT_access()->sens.DROp = IO_rd(DOORSIGNAL_PORT, DOORSIGNAL_PIN);
		/* Tuning results basing on configurations */
		if (DEVCFG_access()->fixOn)
		{
			DEVSTT_access()->OnOff = 1;
		}
		else
		{
			if (DEVCFG_access()->onByEngineOn)
			{
				DEVSTT_access()->OnOff = DEVSTT_access()->sens.EGOn ? 1 : 0;
			}
			else
			{
				DEVSTT_access()->OnOff = DEVSTT_access()->sens.KSOn ? 1 : 0;
				if (DEVCFG_access()->revOn)
				{
					DEVSTT_access()->OnOff = !DEVSTT_access()->OnOff;
				}
			}
		}
		if (DEVCFG_access()->fixDoor)
		{
			DEVSTT_access()->OpDr = 1;
		}
		else
		{
			DEVSTT_access()->OpDr = DEVSTT_access()->sens.DROp;
			if (DEVCFG_access()->revDoor)
			{
				DEVSTT_access()->OpDr = !DEVSTT_access()->OpDr;
			}
		}
		vTaskDelay(50);
	}
}
