/*
 * logResetEvt.h
 *
 *  Created on: Feb 27, 2017
 *      Author: dv198
 */

#ifndef LOGRESETEVT_H_
#define LOGRESETEVT_H_

/*##########################################################################################
 * DEFINES
 *##########################################################################################*/
#define LOGRSTEVT_BIT_BUSFAULT			0x1
#define LOGRSTEVT_BIT_MEMMANAGE			0x2
#define LOGRSTEVT_BIT_USAGEFAULT		0x4
#define LOGRSTEVT_BIT_HARDFAULT			0x8
#define LOGRSTEVT_BIT_NOGPRSLONGTIME	0x10

/*##########################################################################################
 * TYPEDEFS
 *##########################################################################################*/
typedef unsigned char U8;
typedef unsigned short U16;
typedef unsigned long U32;

/*##########################################################################################
 * FUNC.PROTOTYPES
 *##########################################################################################*/
extern U16 LOGRSTEVT_read(void);
extern void LOGRSTEVT_wrt(U16 rstEvtBits);

#endif /* LOGRESETEVT_H_ */
