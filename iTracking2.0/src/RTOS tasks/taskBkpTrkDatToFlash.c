/*
 * taskBkpTrkDatToFlash.c
 *
 *  Created on: Jan 20, 2017
 *      Author: dv198
 *
 * Describe flash data structure:
 * 32MB flash = 512 sectors (64KB per sector).
 * Each sector:
 * +-----------------------------------------+
 * | available seat indexer zone: 64 bytes   |
 * | - Each bit stand for the availability   |
 * |   of data in sector.                    |
 * | - 1:free - 0:occupied                   |
 * +-----------------------------------------+
 * | Data[0]: 128 bytes                      |
 * | - Attribute byte:                       |
 * |  + Bit 0: missed flag. 0:Y 1:N          |
 * |  + Bit 1: rollback flag. 0:Y 1:N        |
 * | - Location data: 127 bytes              |
 * +-----------------------------------------+
 * | Data[1]: 128 bytes                      |
 * | - Attribute byte:                       |
 * |  + Bit 0: missed flag. 0:Y 1:N          |
 * |  + Bit 1: rollback flag. 0:Y 1:N        |
 * | - Location data: 127 bytes              |
 * +-----------------------------------------+
 * | Data[2]: 128 bytes                      |
 * | - Attribute byte:                       |
 * |  + Bit 0: missed flag. 0:Y 1:N          |
 * |  + Bit 1: rollback flag. 0:Y 1:N        |
 * | - Location data: 127 bytes              |
 * +-----------------------------------------+
 *  ...
 * +-----------------------------------------+
 * | Data[510]: 128 bytes                    |
 * | - Attribute byte:                       |
 * |  + Bit 0: missed flag. 0:Y 1:N          |
 * |  + Bit 1: rollback flag. 0:Y 1:N        |
 * | - Location data: 127 bytes              |
 * +-----------------------------------------+
 * (Total: 511 data per sector)
 */
/*##################################################################################
 * INCLUDE
 *##################################################################################*/
#include "taskBkpTrkDatToFlash.h"
/* C libraries */
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
/* HW libraries */
#include "STM32F1_rtc.h"
#include "NORflash.h"
#include "NORflash_branchInfo.h"
/* SW libraries */
#include "trackingData.h"
#include "dbgPrint.h"
#include "memMngr.h"
#include "stringlib.h"
#include "location.pb.h"
#include "devstt.h"
/* Pre-definitions */
/*##################################################################################
 * DEFINES
 *##################################################################################*/

/*##################################################################################
 * TYPEDEFS
 *##################################################################################*/

/*##################################################################################
 * FUNC.PROTOTYPES
 *##################################################################################*/
static void TSKBKPTRKDAT2FL_main(void *pdata);
static void TSKBKPTRKDAT2FL_bkpDat(void);
/*##################################################################################
 * VARIABLES
 *##################################################################################*/
static uint32_t cycleStartTime = 0;
/*##################################################################################
 * FUNCTIONS
 *##################################################################################*/
/*----------------------------------------------------------------------------------------
 * Brief: setup task.
 -----------------------------------------------------------------------------------------*/
uint8_t TSKBKPTRKDAT2FL_setup(uint16_t size, uint32_t prio, xTaskHandle *tskHdl, uint32_t *wotime)
{
	portBASE_TYPE res = xTaskCreate(TSKBKPTRKDAT2FL_main, "BKPTRKDAT2FL", size, NULL, prio, tskHdl);
	return (res == pdTRUE) ? 0 : 1;
}

/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
void TSKBKPTRKDAT2FL_main(void *pdata)
{
	DBG_print("\r\n TSKBKPTRKDAT2FL:Start");
	while (1)
	{
		if (DEVSTT_access()->rdy2rst)
		{
			//vTaskDelete(NULL);
		}
		cycleStartTime = IRTC_getSRT();
		TSKBKPTRKDAT2FL_bkpDat();
		vTaskDelay(100);
	}
}
/*----------------------------------------------------------------------------------------
 * Brief:
 -----------------------------------------------------------------------------------------*/
void TSKBKPTRKDAT2FL_bkpDat(void)
{
	Location loc;
	uint32_t sectAddr = 0;
	uint8_t res = 0;

	if (!TRKDAT_vault_get(VAULT_MAIN, &loc))
	{
		uint32_t freeSeat = 0, wAddr = 0;
		uint8_t availseat[64];

		DBG_print("\r\n ---TSKBKPTRKDAT2FL:SectAddr=%X", sectAddr);
		wAddr = sectAddr + 64; /* Shift address out of available seat indexer zone */
		NORFL_rdDat(sectAddr, 64, availseat);
		for (uint8_t i = 0; i < 64; i++)
		{
			for (uint8_t i2 = 0; i2 < 8; i2++)
			{
				if ((availseat[i] >> i2) & 0x1)
				{
					freeSeat++;
				}
				else
				{
					wAddr += 128 * freeSeat;
					goto TSKBKPTRKDAT2FL_COUNTFREESEAT_END;
				}
			}
		}
		TSKBKPTRKDAT2FL_COUNTFREESEAT_END://
		if (freeSeat > 511)
		{
			freeSeat = 511;
		}
		DBG_print("\r\n ---TSKBKPTRKDAT2FL:freeSeat=%u", freeSeat);
		if (freeSeat == 0)
		{
			/* Go to next sector */
			sectAddr += CYPRESS_S25FL256S_256Mb_TOTAL_SIZE;
			goto TSKBKPTRKDAT2FL_ENDBKP;
		}
		else if (freeSeat == 511)
		{
			res = NORFL_eraseSect(sectAddr, 500);
			DBG_print("\r\n ---TSKBKPTRKDAT2FL:Erase:%X:res=%u", sectAddr);
		}
		wAddr++;
		res = NORFL_progPg(wAddr, sizeof(Location), &loc, 500);
		DBG_print("\r\n ---TSKBKPTRKDAT2FL:PP:%X:res=%u", wAddr, res);
		if (NORFL_cmpDat(wAddr, sizeof(Location), &loc))
		{
			DBG_print("\r\n ---TSKBKPTRKDAT2FL:W:E");
			//NORFL_eraseSect(sectAddr, 500);
			sectAddr += CYPRESS_S25FL256S_256Mb_TOTAL_SIZE;
		}
		TSKBKPTRKDAT2FL_ENDBKP://
		vTaskDelay(1);
	}
}
