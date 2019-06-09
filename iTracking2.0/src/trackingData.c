/*------------------------------------------------------------------------------------------
 * INTRODUCE
 * Brief: This is source file contains all functions and variables that relating to tracking data.
 * Author: Duy Vinh To - Team DinhViSo
 * Detail:
 -------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------
 * INCLUDES
 -------------------------------------------------------------------------------------------*/
#include "trackingData.h"
#include "string.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "dbgPrint.h"
#include "memMngr.h"
#include "stdlib.h"
#include "stringlib.h"
//#include "location.pb.h"
/*------------------------------------------------------------------------------------------
 * DEFINES
 -------------------------------------------------------------------------------------------*/
#define LOC_PARAM_LIST	{"ID:"\
						, "DRVID:"\
						, "LAT:"\
						, "LNG:"\
						, "UTC:"\
						, "SPD:"\
						, "FUEL:"\
						, "TEMP:"\
						, "COURSE:"\
						, "KO:"\
						, "OD:"\
						}
#define LOC_PARAM_MAXINDEX			11
#define LOC_PARAM_INDEX_DEVICEID	0
#define LOC_PARAM_INDEX_DRIVERID	1
#define LOC_PARAM_INDEX_LATITUDE	2
#define LOC_PARAM_INDEX_LONGITUDE	3
#define LOC_PARAM_INDEX_UNIXTIME	4
#define LOC_PARAM_INDEX_SPEED		5
#define LOC_PARAM_INDEX_FUEL		6
#define LOC_PARAM_INDEX_TEMPERATURE	7
#define LOC_PARAM_INDEX_COURSE		8
#define LOC_PARAM_INDEX_KEYON		9
#define LOC_PARAM_INDEX_OPENDOOR	10

/*------------------------------------------------------------------------------------------
 * TYPEDEFS
 -------------------------------------------------------------------------------------------*/
typedef struct
{
	Location *dat;
}TRKDATQ_t;
/*------------------------------------------------------------------------------------------
 * VARIABLES
 -------------------------------------------------------------------------------------------*/
//static Location mainTrkDatVault[MAINTRKDAT_VAULT_SIZE];
//static U16 mainTrkDatVaultIn = 0;
//static U16 mainTrkDatVaultOut = 0;
//static U16 mainTrkDatVaultCount = 0;

//static Location missTrkdatVault[MISSTRKDAT_VAULT_SIZE];
//static U16 missTrkdatVaultIn = 0;
//static U16 missTrkdatVaultOut = 0;
//static U16 missTrkDatVaultCount = 0;

//static Location sendTrkDatVault[SENDTRKDAT_VAULT_SIZE];
//static U16 sendTrkDatVaultIn = 0;
//static U16 sendTrkDatVaultOut = 0;
//static U16 sendTrkDatVaultCount = 0;

static xQueueHandle mainTrkDat_queue = NULL;
static xQueueHandle sendTrkDat_queue = NULL;
static xQueueHandle missTrkDat_queue = NULL;
static xQueueHandle rollbackTrkDat_queue = NULL;

static U8 vaultGuard = 0;
static const U8 *locParamList[] = LOC_PARAM_LIST;
/*------------------------------------------------------------------------------------------
 * FUNC.PROTOTYPES
 -------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------
 * FUNCTIONS
 -------------------------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------------------
 * Brief: setup tracking data vault.
 -----------------------------------------------------------------------------------------*/
void TRKDAT_vault_setup(void)
{
	mainTrkDat_queue = xQueueCreate(MAINTRKDAT_VAULT_SIZE, sizeof(TRKDATQ_t));
	if (mainTrkDat_queue == NULL)
	{
		DBG_print("\r\n TRKDAT_vault_setup:MAIN:FAIL");
	}
	sendTrkDat_queue = xQueueCreate(SENDTRKDAT_VAULT_SIZE, sizeof(TRKDATQ_t));
	if (sendTrkDat_queue == NULL)
	{
		DBG_print("\r\n TRKDAT_vault_setup:SEND:FAIL");
	}
	missTrkDat_queue = xQueueCreate(MISSTRKDAT_VAULT_SIZE, sizeof(TRKDATQ_t));
	if (missTrkDat_queue == NULL)
	{
		DBG_print("\r\n TRKDAT_vault_setup:MISS:FAIL");
	}
	rollbackTrkDat_queue = xQueueCreate(ROLLBACKTRKDAT_VAULT_SIZE, sizeof(TRKDATQ_t));
	if (rollbackTrkDat_queue == NULL)
	{
		DBG_print("\r\n TRKDAT_vault_setup:ROLLBACK:FAIL");
	}
}
/*----------------------------------------------------------------------------------------
 * Brief: add new data into tracking data vault.
 * Param:	vaultSel	|	IN	|	vault selection: 0:main 1:miss
 * 			dat	|			IN	|	adding data.
 * Ret:		0	|	OK
 * 			>0	|	FAIL
 -----------------------------------------------------------------------------------------*/
U8 TRKDAT_vault_add(VAULTSEL_t vaultSel, Location dat)
{
	TRKDATQ_t trkDatQ = {NULL};
	xQueueHandle queue;

//	while (vaultGuard)
//	{
//		vTaskDelay(10);
//	}
	vaultGuard = 1;
	switch (vaultSel)
	{
		case VAULT_MAIN:
			queue = mainTrkDat_queue;
			break;
		case VAULT_MISS:
			queue = missTrkDat_queue;
			break;
		case VAULT_SEND:
			queue = sendTrkDat_queue;
			break;
		case VAULT_ROLLBACK:
			queue = rollbackTrkDat_queue;
			break;
		default:
			vaultGuard = 0;
			return 0xFF;
	}
	trkDatQ.dat = MM_get(3600 * 2, sizeof(Location));
	if (trkDatQ.dat == NULL)
	{
		vaultGuard = 0;
		return 2;
	}
	memcpy(trkDatQ.dat, &dat, sizeof(Location));
	if (xQueueSend(queue, &trkDatQ, 0) == pdTRUE)
	{
		vaultGuard = 0;
		return 0;
	}
	MM_free(trkDatQ.dat);
	vaultGuard = 0;
	return 1;
}
/*----------------------------------------------------------------------------------------
 * Brief: Get data from tracking data vault.
 * Param:	dat	|	OUT	|	get data.
 * Ret:		0	|	OK
 * 			>0	|	FAIL
 -----------------------------------------------------------------------------------------*/
U8 TRKDAT_vault_get(VAULTSEL_t vaultSel, Location *dat)
{
	TRKDATQ_t trkDatQ;
	xQueueHandle queue;

//	while (vaultGuard)
//	{
//		vTaskDelay(10);
//	}
	vaultGuard = 1;
	switch (vaultSel)
	{
		case VAULT_MAIN:
			queue = mainTrkDat_queue;
			break;
		case VAULT_MISS:
			queue = missTrkDat_queue;
			break;
		case VAULT_SEND:
			queue = sendTrkDat_queue;
			break;
		case VAULT_ROLLBACK:
			queue = rollbackTrkDat_queue;
			break;
		default:
			vaultGuard = 0;
			return 0xFF;
	}
	if (xQueueReceive(queue, &trkDatQ, 0) == pdTRUE)
	{
		memcpy(dat, trkDatQ.dat, sizeof(Location));
		MM_free(trkDatQ.dat);
		vaultGuard = 0;
		return 0;
	}
	vaultGuard = 0;
	return 1;
}
/*----------------------------------------------------------------------------------------
 * Brief: Get number of data in tracking data vault.
 * Ret:		data count
 -----------------------------------------------------------------------------------------*/
U16 TRKDAT_vault_count(VAULTSEL_t vaultSel)
{
	switch (vaultSel)
	{
		case VAULT_MAIN:
			return uxQueueMessagesWaiting(mainTrkDat_queue);
		case VAULT_MISS:
			return uxQueueMessagesWaiting(missTrkDat_queue);
		case VAULT_SEND:
			return uxQueueMessagesWaiting(sendTrkDat_queue);
		case VAULT_ROLLBACK:
			return uxQueueMessagesWaiting(rollbackTrkDat_queue);
		default:
			return 0xFFFF;
	}
}
#if 0
/*----------------------------------------------------------------------------------------
 * Brief: convert tracking data into string.
 * Param:	dat		|	IN	|	data to convert.
 * 			size	|	IN	|	max string size.
 * 			s		|	OUT	|	output string.
 -----------------------------------------------------------------------------------------*/
void TRKDAT_conv2s_oldformat(Location dat, U16 size, U8 *s)
{
	U8 sLat[12], sLng[12], sFuel[12];

	TRKDAT_double2s(dat.lat, 0, 12, sLat);
	TRKDAT_double2s(dat.lng, 0, 12, sLng);
	TRKDAT_double2s(dat.fuel, 1, 12, sFuel);
	s[size - 1] = 0;
	snprintf(s, size, "%s,%s,%s,%s,%lu,%u,%u,%u,%s,%u,%u,%u" //
				, dat.devID //
				, dat.drvID //
				, sLat //
				, sLng //
				, dat.utc //
				, dat.spd //
				, (dat.vhStt & VHSTT_ACON_BIT) ? 1 : 0 //
				, (dat.vhStt & VHSTT_DROP_BIT) ? 1 : 0 //
				, sFuel //
				, 0 //
				, (dat.vhStt & VHSTT_KSON_BIT) ? 1 : 0 //
				, 0 //
				);
	snprintf(s, size, "%s,%u,%u", s //
				, 0 //
				, ((dat.temp >= 0) ? dat.temp : (-dat.temp + 200)) //acceptable range 0-255
				);
	snprintf(s, size, "%s,%u", s, 0);
	snprintf(s, size, "%s,0", s);
	snprintf(s, size, "%s,%u", s, 0);
	snprintf(s, size, "%s,0,0", s);
	snprintf(s, size, "%s,0", s);
	snprintf(s, size, "%s,%d", s, dat.course);
	snprintf(s, size, "%s,%u", s, 0);
	snprintf(s, size, "%s,%u", s, 0);
}
#endif
/*----------------------------------------------------------------------------------------
 * Brief: convert Location data to string.
 * Param:	loc		|	IN	|	Location data to convert.
 * 			header	|	IN	|	header string.
 * 			footer	|	IN	|	footer string.
 * 			size	|	IN	|	max string size.
 * 			s		|	OUT	|	output string.
 * Ret:		output string length.
 * Tip:	User can input size = 0 to get output string length without actual conversion.
 -----------------------------------------------------------------------------------------*/
U16 TRKDAT_conv2s(Location loc, const U8 *header, const U8 *footer, U16 size, U8 *s)
{
	U8 sTmp[12];
	U16 actualLen = 0;

	actualLen += strlen(header) + strlen(footer) + 2;
	if (size > 0)
	{
		strlcpy(s, header, size);
		strlcat(s, "{", size);
		snprintf(s, size, "%s%s%s,", s, locParamList[LOC_PARAM_INDEX_DEVICEID], loc.deviceID);
	}
	actualLen += 1 + strlen(locParamList[LOC_PARAM_INDEX_DEVICEID]) + strlen(loc.deviceID); //1 is ',' character.
	if (loc.has_rfID)
	{
		if (size > 0)
		{
			snprintf(s, size, "%s%s%s,", s, locParamList[LOC_PARAM_INDEX_DRIVERID], loc.rfID);
		}
		actualLen += 1 + strlen(locParamList[LOC_PARAM_INDEX_DRIVERID]) + strlen(loc.rfID);
	}
	if (loc.has_latitude)
	{
		SL_double2s(loc.latitude, 12, sTmp);
		if (size > 0)
		{
			snprintf(s, size, "%s%s%s,", s, locParamList[LOC_PARAM_INDEX_LATITUDE], sTmp);
		}
		actualLen += 1 + strlen(locParamList[LOC_PARAM_INDEX_LATITUDE]) + strlen(sTmp);
	}
	if (loc.has_longitude)
	{
		SL_double2s(loc.longitude, 12, sTmp);
		if (size > 0)
		{
			snprintf(s, size, "%s%s%s,", s, locParamList[LOC_PARAM_INDEX_LONGITUDE], sTmp);
		}
		actualLen += 1 + strlen(locParamList[LOC_PARAM_INDEX_LONGITUDE]) + strlen(sTmp);
	}
	snprintf(sTmp, 12, "%u", loc.unixtime);
	if (size > 0)
	{
		snprintf(s, size, "%s%s%s,", s, locParamList[LOC_PARAM_INDEX_UNIXTIME], sTmp);
	}
	actualLen += 1 + strlen(locParamList[LOC_PARAM_INDEX_UNIXTIME]) + strlen(sTmp);
	if (loc.has_velocity)
	{
		snprintf(sTmp, 12, "%u", loc.velocity);
		if (size > 0)
		{
			snprintf(s, size, "%s%s%s,", s, locParamList[LOC_PARAM_INDEX_SPEED], sTmp);
		}
		actualLen += 1 + strlen(locParamList[LOC_PARAM_INDEX_SPEED]) + strlen(sTmp);
	}
	if (loc.has_fuel)
	{
		SL_double2s(loc.fuel, 12, sTmp);
		if (size > 0)
		{
			snprintf(s, size, "%s%s%s,", s, locParamList[LOC_PARAM_INDEX_FUEL], sTmp);
		}
		actualLen += 1 + strlen(locParamList[LOC_PARAM_INDEX_FUEL]) + strlen(sTmp);
	}
	if (loc.has_temp)
	{
		SL_double2s(loc.temp, 12, sTmp);
		if (size > 0)
		{
			snprintf(s, size, "%s%s%s,", s, locParamList[LOC_PARAM_INDEX_TEMPERATURE], sTmp);
		}
		actualLen += 1 + strlen(locParamList[LOC_PARAM_INDEX_TEMPERATURE]) + strlen(sTmp);
	}
	if (loc.has_carHeading)
	{
		snprintf(sTmp, 12, "%u", loc.carHeading);
		if (size > 0)
		{
			snprintf(s, size, "%s%s%s,", s, locParamList[LOC_PARAM_INDEX_COURSE], sTmp);
		}
		actualLen += 1 + strlen(locParamList[LOC_PARAM_INDEX_COURSE]) + strlen(sTmp);
	}
	if (loc.has_onoff)
	{
		snprintf(sTmp, 12, "%u", loc.onoff);
		if (size > 0)
		{
			snprintf(s, size, "%s%s%s,", s, locParamList[LOC_PARAM_INDEX_KEYON], sTmp);
		}
		actualLen += 1 + strlen(locParamList[LOC_PARAM_INDEX_KEYON]) + strlen(sTmp);
	}
	if (loc.has_opendoor)
	{
		snprintf(sTmp, 12, "%u", loc.opendoor);
		if (size > 0)
		{
			snprintf(s, size, "%s%s%s,", s, locParamList[LOC_PARAM_INDEX_OPENDOOR], sTmp);
		}
		actualLen += 1 + strlen(locParamList[LOC_PARAM_INDEX_OPENDOOR]) + strlen(sTmp);
	}
	if (size > 0)
	{
		strlcat(s, "}", size);
		strlcat(s, footer, size);
		s[size - 1] = 0;
	}
	return actualLen;
}
/*----------------------------------------------------------------------------------------
 * Brief: convert string to tracking data.
 * Param:	dat		|	IN	|	data to convert.
 * 			size	|	IN	|	max string size.
 * 			s		|	OUT	|	output string.
 -----------------------------------------------------------------------------------------*/
void TRKDAT_conv(U16 len, U8 *s, Location *loc)
{
	U8 sParam[16], *sTmp;
	U16 startpos = 0;

	for(U16 i = 0; i < len; i++)
	{
		if (s[i] == '{')
		{
			startpos = i + 1;
			break;
		}
	}
	sTmp = &s[startpos];
	//DBG_print("\r\n TRKDAT_conv:sTmp=\"%s\"", sTmp);
	for(U8 i = 0; i < 255; i++)
	{
		SL_split(sTmp, ',', sParam, 16, sTmp, len - startpos);
		if (!strlen(sParam))
		{
			break;
		}
		//DBG_print("\r\n TRKDAT_conv:sParam=\"%s\"", sParam);
		if (!SL_cut(sParam, locParamList[LOC_PARAM_INDEX_DRIVERID]))
		{
			strlcpy(loc->rfID, sParam, LOCATION_DRIVERID_SIZE);
			loc->has_rfID = true;
			//DBG_print("\r\n TRKDAT_conv:drvID=\"%s\"", dat->drvID);
		}
		else if (!SL_cut(sParam, locParamList[LOC_PARAM_INDEX_DEVICEID]))
		{
			strlcpy(loc->deviceID, sParam, LOCATION_DEVICEID_SIZE);
			//DBG_print("\r\n TRKDAT_conv:devID=\"%s\"", dat->devID);
		}
		else if (!SL_cut(sParam, locParamList[LOC_PARAM_INDEX_LATITUDE]))
		{
			loc->latitude = SL_s2double(sParam);
			loc->has_latitude = true;
		}
		else if (!SL_cut(sParam, locParamList[LOC_PARAM_INDEX_LONGITUDE]))
		{
			loc->longitude = SL_s2double(sParam);
			loc->has_longitude = true;
		}
		else if (!SL_cut(sParam, locParamList[LOC_PARAM_INDEX_UNIXTIME]))
		{
			loc->unixtime = atoi(sParam);
		}
		else if (!SL_cut(sParam, locParamList[LOC_PARAM_INDEX_SPEED]))
		{
			loc->velocity = atoi(sParam);
			loc->has_velocity = true;
		}
		else if (!SL_cut(sParam, locParamList[LOC_PARAM_INDEX_FUEL]))
		{
			loc->fuel = SL_s2double(sParam);
			loc->has_fuel = true;
		}
		else if (!SL_cut(sParam, locParamList[LOC_PARAM_INDEX_TEMPERATURE]))
		{
			loc->temp = SL_s2double(sParam);
			loc->has_temp = true;
		}
		else if (!SL_cut(sParam, locParamList[LOC_PARAM_INDEX_COURSE]))
		{
			loc->carHeading = atoi(sParam);
			loc->has_carHeading = true;
		}
		else if (!SL_cut(sParam, locParamList[LOC_PARAM_INDEX_KEYON]))
		{
			loc->onoff = atoi(sParam);
			loc->has_onoff = true;
		}
		else if (!SL_cut(sParam, locParamList[LOC_PARAM_INDEX_OPENDOOR]))
		{
			loc->opendoor = atoi(sParam);
			loc->has_opendoor = true;
		}
	}
}
