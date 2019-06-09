/*
 * taskSens.h
 *
 *  Created on: Dec 30, 2016
 *      Author: dv198
 */

#ifndef RTOS_TASKS_TASKSENS_H_
#define RTOS_TASKS_TASKSENS_H_

/*##########################################################################################
 * INCLUDE
 *##########################################################################################*/
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"

/*##########################################################################################
 * TYPEDEFS
 *##########################################################################################*/
typedef void (*TSKSENS_dbgPrt_CB)(const uint8_t *s, ...); // debug printing.

/*##########################################################################################
 * FUNC.PROTOTYPES
 *##########################################################################################*/
extern void TSKSENS_setup_dbgPrt(TSKSENS_dbgPrt_CB cb);
extern uint8_t TSKSENS_setup(uint16_t size, uint32_t prio, xTaskHandle *tskHdl, uint32_t *wotime);

#endif /* RTOS_TASKS_TASKSENS_H_ */
