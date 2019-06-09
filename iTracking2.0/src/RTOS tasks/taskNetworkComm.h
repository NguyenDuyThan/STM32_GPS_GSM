/*------------------------------------------------------------------------------------------
 * INTRODUCE
 * Brief: This is header file of task network communication.
 * Author: Duy Vinh To - Team DinhViSo
 * Detail: upload/download data to/from server.
 -------------------------------------------------------------------------------------------*/

#ifndef TASKNETWORKCOMM_H_
#define TASKNETWORKCOMM_H_

/*##########################################################################################
 * INCLUDE
 *##########################################################################################*/
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "file.pb.h"

/*##########################################################################################
 * DEFINES
 *##########################################################################################*/
#define SNDSMS_QUEUE_SIZE			32
#define SNDFB_VIA_NATS_QUEUE_SIZE	32
#define SNDFILE_VIA_NATS_QUEUE_SIZE		2
#define SNDPHOTO_VIA_NATS_QUEUE_SIZE	2

/*##########################################################################################
 * TYPEDEFS
 *##########################################################################################*/
typedef void (*TSKNETCOMM_dbgPrt_CB)(const uint8_t *s, ...); // debug printing.

/*##########################################################################################
 * FUNC.PROTOTYPES
 *##########################################################################################*/
extern void TSKNETCOMM_setup_dbgPrt(TSKNETCOMM_dbgPrt_CB cb);
extern uint8_t TSKNETCOMM_setup(uint16_t size, uint32_t prio, xTaskHandle *tskHdl, uint32_t *wotime);
extern uint8_t TSKNETCOMMIR_setup(uint16_t size, uint32_t prio, xTaskHandle *tskHdl, uint32_t *wotime);
extern uint8_t TSKNETCOMM_add_sndSMS(const uint8_t *num, const uint8_t *content);
extern uint8_t TSKNETCOMM_add_sndFBViaNATS(const uint8_t *content);
extern uint8_t TSKNETCOMM_add_sndFileViaNATS(const File_data1KB *fileBlk1KB);
extern uint8_t TSKNETCOMM_add_sndPhotoViaNATS(const File_data1KB *fileBlk1KB);
extern uint8_t TSKNETCOMM_chkFull_sndFileViaNATS(void);
extern uint8_t TSKNETCOMM_chkFull_sndPhotoViaNATS(void);
extern void TSKNETCOMM_set_instantSndTrkDat(void);

#endif /* TASKNETWORKCOMM_H_ */
