/*------------------------------------------------------------------------------------------
 * INTRODUCE
 * Brief: This is header file contains all function prototypes, definitions
 * 			that relating to tracking data.
 * Author: Duy Vinh To - Team DinhViSo
 * Detail:
 -------------------------------------------------------------------------------------------*/

#ifndef DVS_TRACKINGDATA_H_
#define DVS_TRACKINGDATA_H_

/*------------------------------------------------------------------------------------------
 * INCLUDES
 -------------------------------------------------------------------------------------------*/
#include "location.pb.h"
/*------------------------------------------------------------------------------------------
 * DEFINES
 -------------------------------------------------------------------------------------------*/
#define LOCATION_DEVICEID_SIZE	16
#define LOCATION_DRIVERID_SIZE	16


#define TRKDAT_DEVID_SIZE	11
#define TRKDAT_DRVID_SIZE	11

#define VHSTT_KSON_BIT		0x1	// Key signal is on
#define VHSTT_DROP_BIT		0x2 // door is opened
#define VHSTT_ACON_BIT		0x4 // Air conditioner is on.

#define MAINTRKDAT_VAULT_SIZE		32
#define MISSTRKDAT_VAULT_SIZE		32
#define SENDTRKDAT_VAULT_SIZE		32
#define ROLLBACKTRKDAT_VAULT_SIZE	32
/*------------------------------------------------------------------------------------------
 * TYPEDEFS
 -------------------------------------------------------------------------------------------*/
typedef unsigned char U8;
typedef unsigned short U16;
typedef unsigned long U32;

/*
typedef struct
{
	U8 devID[TRKDAT_DEVID_SIZE];
	U8 drvID[TRKDAT_DRVID_SIZE];
	U32 utc;
	double lat;
	double lng;
	U16 spd; // Speed. Unit: Km/h.
	U16 course; // similar to "heading" concept.
	U32 vhStt; // state table of logical vehicle signals: 1 and 0.
	U32 wng; // warning table.
	U32 WPPS; // Wheel pulse per second.
	double fuel; // Unit: %.
	int temp; // Temperature. Unit: celsius.
}TRKDAT_t;
*/

typedef enum
{
	VAULT_MAIN,
	VAULT_MISS,
	VAULT_SEND,
	VAULT_ROLLBACK,
}VAULTSEL_t;
/*------------------------------------------------------------------------------------------
 * FUNC.PROTOTYPES
 -------------------------------------------------------------------------------------------*/
extern void TRKDAT_vault_setup(void);
extern U8 TRKDAT_vault_add(VAULTSEL_t vaultSel, Location dat);
extern U8 TRKDAT_vault_get(VAULTSEL_t vaultSel, Location *dat);
extern U16 TRKDAT_vault_count(VAULTSEL_t vaultSel);
//extern void TRKDAT_conv2s_oldformat(Location dat, U16 size, U8 *s);
extern void TRKDAT_conv(U16 len, U8 *s, Location *loc);
extern U16 TRKDAT_conv2s(Location loc, const U8 *header, const U8 *footer, U16 size, U8 *s);
#endif /* DVS_TRACKINGDATA_H_ */
