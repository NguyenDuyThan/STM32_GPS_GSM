/*
 * taskTakePhoto.h
 *
 *  Created on: Mar 24, 2017
 *      Author: dv198
 */

#ifndef RTOS_TASKS_TASKCAMERA_H_
#define RTOS_TASKS_TASKCAMERA_H_

/*##########################################################################################
 * INCLUDE
 *##########################################################################################*/
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"

/*##########################################################################################
 * TYPEDEFS
 *##########################################################################################*/
typedef void (*TSKCAM_dbgPrt_CB)(const uint8_t *s, ...); // debug printing.

/*##########################################################################################
 * FUNC.PROTOTYPES
 *##########################################################################################*/
extern void TSKCAM_setup_dbgPrt(TSKCAM_dbgPrt_CB cb);
extern uint8_t TSKCAM_setup(uint16_t size, uint32_t prio, xTaskHandle *tskHdl, uint32_t *wotime);

#endif /* RTOS_TASKS_TASKCAMERA_H_ */
