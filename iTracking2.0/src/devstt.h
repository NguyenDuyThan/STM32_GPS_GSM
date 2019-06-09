/*------------------------------------------------------------------------------------------
 * INTRODUCE
 * Brief: This is header file containing all function relating to device status.
 * Author: Duy Vinh To - Team DinhViSo
 * Detail:
 -------------------------------------------------------------------------------------------*/
#ifndef DEVSTT_H_
#define DEVSTT_H_

/*------------------------------------------------------------------------------------------
 * INCLUDES
 -------------------------------------------------------------------------------------------*/
#if (HW_MAIN_VERSION == 1)
#include "SIM900.h"
#endif // #if (HW_MAIN_VERSION == 2)
#if (HW_MAIN_VERSION == 2)
#include "QuectelM95.h"
#endif // #if (HW_MAIN_VERSION == 2)
#include "QuectelL70.h"
#include "macro.h"
/*------------------------------------------------------------------------------------------
 * DEFINES
 -------------------------------------------------------------------------------------------*/
#define NETWORKNAME_SIZE	64
#define STDBY_SPEED_MAX		7 //Max speed that vehicle is considered to be standing by
/* Connect to server bits */
#define C2SVBIT_NATS_PUBLOC		0
#define C2SVBIT_NATS_SUBLOC		1
#define C2SVBIT_NATS_SUBCMD		2
#define C2SVBIT_NATS_SUBCFG		3
#define C2SVBIT_NATS_PUBFILE	4

/*------------------------------------------------------------------------------------------
 * TYPEDEFS
 -------------------------------------------------------------------------------------------*/
typedef unsigned char U8;
typedef unsigned short U16;
typedef unsigned long U32;

typedef struct
{
	U8 rdy;
	U32 size;
}FS_STT_t;//File system status
typedef struct
{
	U8 rdy2send;
	U32 ofs; // current rollback file offset.
	U32 nextOfs; // next rollback file offset.
	U32 size; // rollback file size.
	U32 utc; // rollback UTC.
}RBTRKDAT_STT_t; // rollback tracking data status

typedef struct
{
	U32 utc;
	U8 rdy2send;
	U8 fin; // Final/finish
	U32 ofs; // current offset.
	U32 nextOfs; // next offset.
	U32 fsize; // rollback file size.
	U8 *fname; // pointer to filename.
	U32 size; // log content size.
	U8 *content; // log content to send.
}SNDFILE_STT_t;

typedef struct
{
	U32 utc; // Unix time.
	U8 *fname; // pointer to filename.
	U32 rd_ofs; // read offset.
	U32 snt_ofs; // sent offset.
	U32 fsize; // rollback file size.
	U8 delFileAfterDone;
}SNDFILE_VIA_NATS_t;

typedef struct
{
	U32 httpTotalCnt; // HTTP post/get action total counter.
	U32 httpFC; // HTTP post/get action fail counter.
	U32 sockFC; // Socket fail counter (send/receive).
}GPRS_DBG_t;

typedef struct
{
	U8 signalQuality;
	U8 network[NETWORKNAME_SIZE];
	NETREGSTT_t netRegStt;
	U8 connect2servBits; // View 'Connect to server bits' definitions
}NET_STT_t;

typedef struct
{
	U8 KSOn; // Key signal on
	U8 EGOn; // Engine on
	U16 fuel; // Analog signal
	U32 WP; // Wheel pulse
	U8 DROp; // Open door signal
	float BATpct; // Battery percentage
	U8 PwrConn; // Power connection
}SENS_t;

typedef struct
{
	U32 tLastQC; // last time GNSS quality is certified.
	U32 tLastAvail; // last time required NMEA messages are available.
	U8 stdby; // standing by
}GNSS_DBG_t;

typedef struct
{
	RBTRKDAT_STT_t RBtrkDat;
	NET_STT_t net;
	GNSS_DAT_t GNSS;
	FS_STT_t fs;
	GPRS_DBG_t GPRSdbg;
	U8 RTCsync;
	SENS_t sens; // Sensing data
	GNSS_DBG_t GNSSdbg;
	U8 flashAvail; // Flash memory is available
	U8 OnOff; // 'OnOff' data to backup and sending to server
	U8 OpDr; // 'Open door' data to backup and sending to server
	U8 VHstdby; // 'Vehicle is standing by'
	//SNDFILE_STT_t sndFile;
	SNDFILE_VIA_NATS_t sndFileViaNATS; //
	SNDFILE_VIA_NATS_t sndPhotoViaNATS; //
	U8 rdy2rst; // Ready to reset device
	U8 initDone;
}DEVSTT_t;
/*------------------------------------------------------------------------------------------
 * FUNC.PROTOTYPES
 -------------------------------------------------------------------------------------------*/
extern DEVSTT_t *DEVSTT_access(void);
extern void DEVSTT_genRpt(U16 size, U8 *s);

#endif /* DEVSTT_H_ */
