/*
 * taskRFIDreader.c
 *
 *  Created on: Feb 23, 2017
 *      Author: dv198
 */

/*##########################################################################################
 * INCLUDE
 *##########################################################################################*/
#include "taskRFIDreader.h"

#include "STM32F1_io.h"
#include "STM32F1_uart.h"
#include "DVS_RFIDreader.h"

#include "dbgPrint.h"

#include "devstt.h"

#include "HWmapping.h"
/*##########################################################################################
 * DEFINE
 *##########################################################################################*/

/*##########################################################################################
 * TYPEDEFS
 *##########################################################################################*/

/*##########################################################################################
 * FUNC.PROTOTYPES
 *##########################################################################################*/
static void TSKRFIDREADER_main(void *pdata);
static void DVSRFID_printc(uint8_t c);
/*##########################################################################################
 * VARIABLES
 *##########################################################################################*/

/*##########################################################################################
 * FUNCTIONS
 *##########################################################################################*/
/*----------------------------------------------------------------------------------------
 * Brief: print a character to DVS RFID reader
 * Param:	c	|	I	|	character
 -----------------------------------------------------------------------------------------*/
void DVSRFID_printc(uint8_t c)
{
	UART_sendc(5, c);
}
/*----------------------------------------------------------------------------------------
 * Brief: setup task RFID reader
 * Param:	size	|	IN	|	task size.
 * 			prio	|	IN	|	task priority.
 * 			tskHdl	|	I/O	|	task handler.
 * Ret:	0	|	OK
 * 		>0	|	FAIL
 -----------------------------------------------------------------------------------------*/
uint8_t TSKRFIDREADER_setup(uint16_t size, uint32_t prio, xTaskHandle *tskHdl, uint32_t *wotime)
{
	/* Create task */
	portBASE_TYPE res = xTaskCreate(TSKRFIDREADER_main, "RFIDREADER", size, NULL, prio, tskHdl);
	if (res != pdTRUE)
	{
		return 1;
	}
	/* Setup UART5 to communicate with DVS RFID module */
	UART_setup(5, 115200, DVSRFID_readc);
	/* Setup IO controlling power down pin */
	IO_setup(EXTRS232_PWRCTRL_PORT, EXTRS232_PWRCTRL_PIN, IODIR_OPP);
	IO_wrt(EXTRS232_PWRCTRL_PORT, EXTRS232_PWRCTRL_PIN, 1);
	/* Setup GNSS module */
	DVSRFID_setup(DVSRFID_printc, vTaskDelay);
	return 0;
}
/*----------------------------------------------------------------------------------------
 * Brief: main function.
 * To-do-list:
 *
 -----------------------------------------------------------------------------------------*/
void TSKRFIDREADER_main(void *pdata)
{

	while (1)
	{
		if (DEVSTT_access()->rdy2rst)
		{
			//vTaskDelete(NULL);
		}
		/* Read driver information (if available) */
		{
			uint8_t lic[32]= "", name[64] = "", uid[16] = "";
			if (!DVSRFID_readDrvInfo(32, lic, 64, name, 16, uid))
			{
				DBG_print("\r\n %s:Lic=\"%s\" Name=\"%s\" UID=\"%s\"\r\n", __func__, lic, name, uid);
			}
		}
		if (DVSRFID_setLED(LEDINDEX_GSM, GETBIT(DEVSTT_access()->net.connect2servBits, C2SVBIT_NATS_PUBLOC)))
		{
			//DBG_print("\r\n %s:SetLEDGSM:FAIL", __func__);
		}
		if (DVSRFID_setLED(LEDINDEX_GPS, (DEVSTT_access()->GNSS.RMCavail && (DEVSTT_access()->GNSS.GGAavail || DEVSTT_access()->GNSS.GSAavail)) ? 1 : 0))
		{
			//DBG_print("\r\n %s:SetLEDGPS:FAIL", __func__);
		}
		vTaskDelay(50);
	}
}
