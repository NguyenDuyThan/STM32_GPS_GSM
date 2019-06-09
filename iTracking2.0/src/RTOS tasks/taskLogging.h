/*------------------------------------------------------------------------------------------
 * INTRODUCE
 * Brief: This is header file of task logging.
 * Author: Duy Vinh To - Team DinhViSo
 * Detail: periodically logging device state into file system.
 -------------------------------------------------------------------------------------------*/

#ifndef TASKLOGGING_H_
#define TASKLOGGING_H_

/*##########################################################################################
 * INCLUDE
 *##########################################################################################*/
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"

/*##########################################################################################
 * DEFINE
 *##########################################################################################*/
#define LOGQUEUE_SIZE	32
#define LOGFILE_SUFFIX  {"default", "nats", "EGOn", "devstt", "gsm", "rollback", "locfb", "sndPhoto", "rstcause", "gnss"}

#define SNDLOG_BLKSIZE	(1024 * 1)//(1024 * 4)

/*##########################################################################################
 * TYPEDEFS
 *##########################################################################################*/
typedef enum
{
	LOGTYPE_DEFAULT = 0, //
	LOGTYPE_NATS = 1, // NATS protocol
	LOGTYPE_EGON = 2, // Engine on
	LOGTYPE_DEVSTT = 3, // device status
	LOGTYPE_GSM = 4, // GSM
	LOGTYPE_ROLLBACK = 5, // Rollback data information
	LOGTYPE_LOCFB = 6, // sending data feedback
	LOGTYPE_SNDPHOTO = 7, // Send photo
	LOGTYPE_RSTCAUSE = 8, // Reset cause
	LOGTYPE_GNSS = 9, //
} LOGTYPE_t;

typedef void (*TSKLOG_dbgPrt_CB)(const uint8_t *s, ...); // debug printing.

/*##########################################################################################
 * VARIABLES
 *##########################################################################################*/

/*##########################################################################################
 * FUNC.PROTOTYPES
 *##########################################################################################*/
extern void TSKLOG_setup_dbgPrt(TSKLOG_dbgPrt_CB cb);
extern uint8_t TSKLOG_setup(uint16_t size, uint32_t prio, xTaskHandle *tskHdl, uint32_t *wotime);
extern uint8_t TSKLOG_addLogMsg(uint32_t utc, LOGTYPE_t type, const uint8_t *content);

#endif /* TASKLOGGING_H_ */
