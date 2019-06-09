/*------------------------------------------------------------------------------------------
 * INTRODUCE
 * Brief: This is header file of task command.
 * Author: Duy Vinh To - Team DinhViSo
 * Detail: process commands.
 -------------------------------------------------------------------------------------------*/

#ifndef TASKCMD_H_
#define TASKCMD_H_

/*------------------------------------------------------------------------------------------
 * INCLUDES
 -------------------------------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
/*------------------------------------------------------------------------------------------
 * DEFINES
 -------------------------------------------------------------------------------------------*/
#define EXTSRC_CMD_QUEUE_SIZE	32
/*------------------------------------------------------------------------------------------
 * TYPEDEFS
 -------------------------------------------------------------------------------------------*/
typedef enum
{
	EXSRCCT_SMS = 0,
	EXSRCCT_NATS = 1,
} EXTSRC_CMD_TYPE_t; // command from external source: SMS, SSH connection...
/*------------------------------------------------------------------------------------------
 * FUNC.PROTOTYPES
 -------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------
 * Brief: setup task handling commands.
 * Param:	size	|	IN	|	task size.
 * 			prio	|	IN	|	task priority.
 * 			tskHdl	|	IO	|	task handler.
 * Ret:	0	|	OK
 * 		>0	|	FAIL
 -----------------------------------------------------------------------------------------*/
extern uint8_t TSKCMD_setup(uint16_t size, uint32_t prio, xTaskHandle *tskHdl, uint32_t *wotime);
/*----------------------------------------------------------------------------------------
 * Brief: Add external source command
 * Param:	type	|	IN	|	source type. View EXTSRC_CMD_TYPE_t for more details.
 * 			src		|	IN	|	Source. It can be number (if source is SMS) or hostname/IP (SSH)...
 * 			content	|	IN	|	Command content.
 * Ret:		0	|	OK
 * 			>0	|	FAIL
 -----------------------------------------------------------------------------------------*/
extern uint8_t TSKCMD_addExtSrcCMD(EXTSRC_CMD_TYPE_t type, const uint8_t *src, const uint8_t *content);

#endif /* TASKCMD_H_ */
