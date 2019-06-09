/*------------------------------------------------------------------------------------------
 * INTRODUCE
 * Brief: This is header file of task backup tracking data into file system.
 * Author: Duy Vinh To - Team DinhViSo
 * Detail:
 -------------------------------------------------------------------------------------------*/

#ifndef TASKBKPTRKDAT_H_
#define TASKBKPTRKDAT_H_

/*##########################################################################################
 * INCLUDE
 *##########################################################################################*/
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"

/*##########################################################################################
 * TYPEDEFS
 *##########################################################################################*/
typedef void (*TSKBKPTD_dbgPrt_CB)(const uint8_t *s, ...); // debug printing.

/*##########################################################################################
 * FUNC.PROTOTYPES
 *##########################################################################################*/
extern void TSKBKPTD_setup_dbgPrt(TSKBKPTD_dbgPrt_CB cb);
extern uint8_t TSKBKPTD_setup(uint16_t size, uint32_t prio, xTaskHandle *tskHdl, uint32_t *wotime);
extern uint32_t TSKBKPTD_getCycleRT(void);

#endif /* TASKBKPTRKDAT_H_ */
