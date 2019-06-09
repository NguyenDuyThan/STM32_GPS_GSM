/*
 * taskBkpTrkDatToFlash.h
 *
 *  Created on: Jan 20, 2017
 *      Author: dv198
 */

#ifndef RTOS_TASKS_TASKBKPTRKDATTOFLASH_H_
#define RTOS_TASKS_TASKBKPTRKDATTOFLASH_H_

/*##################################################################################
 * INCLUDE
 *##################################################################################*/
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
/*##################################################################################
 * DEFINES
 *##################################################################################*/

/*##################################################################################
 * TYPEDEFS
 *##################################################################################*/

/*##################################################################################
 * FUNC.PROTOTYPES
 *##################################################################################*/
extern uint8_t TSKBKPTRKDAT2FL_setup(uint16_t size, uint32_t prio, xTaskHandle *tskHdl, uint32_t *wotime);
extern uint32_t TSKBKPTRKDAT2FL_getCycleRT(void);

#endif /* RTOS_TASKS_TASKBKPTRKDATTOFLASH_H_ */
