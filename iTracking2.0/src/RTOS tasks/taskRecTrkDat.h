/*------------------------------------------------------------------------------------------
 * INTRODUCE
 * Brief: This is header file of task recording tracking data.
 * Author: Duy Vinh To - Team DinhViSo
 * Detail: periodically recording tracking data.
 -------------------------------------------------------------------------------------------*/

#ifndef TASKRECTRKDAT_H_
#define TASKRECTRKDAT_H_

/*##########################################################################################
 * INCLUDE
 *##########################################################################################*/
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"

/*##########################################################################################
 * DEFINES
 *##########################################################################################*/
#define ALLOW_USE_LASTQC_GNSSDAT_MAXTIME	30 // Seconds
#define WAIT_GNSS_UNTIL_RST_TIMEOUT			30 // Seconds
/*##########################################################################################
 * TYPEDEFS
 *##########################################################################################*/

typedef void (*TSKRECTD_dbgPrt_CB)(const uint8_t *s, ...); // debug printing.
/*##########################################################################################
 * FUNC.PROTOTYPES
 *##########################################################################################*/
extern void TSKRECTD_setup_dbgPrt(TSKRECTD_dbgPrt_CB cb);
extern uint8_t TSKRECTD_setup(uint16_t size, uint32_t prio, xTaskHandle *tskHdl, uint32_t *wotime);

#endif /* TASKRECTRKDAT_H_ */
