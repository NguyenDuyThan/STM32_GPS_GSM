/*------------------------------------------------------------------------------------------
 * INTRODUCE
 * Brief: This is source file containing all function relating to device configuration.
 * Author: Duy Vinh To - Team DinhViSo
 * Detail:
 -------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------
 * INCLUDES
 -------------------------------------------------------------------------------------------*/
#include "devcfg.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "stringlib.h"
#include "jsmn.h"
#include "memMngr.h"
#include "STM32F1_flash.h"
#include "usedFlashPages.h"
#include <pb_encode.h>
#include <pb_decode.h>
/*------------------------------------------------------------------------------------------
 * DEFINES
 -------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------
 * VARIABLES
 -------------------------------------------------------------------------------------------*/
const APN_PROFILE_t df_APNprofile = {"default", DF_DEVCFG_APNNAME, DF_DEVCFG_APNUSR, DF_DEVCFG_APNPWD};
static APN_PROFILE_t APNprofiles[MAX_APNPROFILE];
static DeviceConfigV2 mainDevCfg = DeviceConfigV2_init_default;
static localConfig mainLocalCfg = localConfig_init_default;
static SESSIONCFG_t mainSessionCfg = {0, 1};
static U8 deviceID[DEVID_SIZE] = "";
/*------------------------------------------------------------------------------------------
 * FUNC.PROTOTYPES
 -------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------
 * Brief: access device configuration
 --------------------------------------------------------------------------------------*/
DeviceConfigV2 *DEVCFG_access(void)
{
	return &mainDevCfg;
}
/*-------------------------------------------------------------------------------------
 * Brief: load device configuration from flash to RAM
 --------------------------------------------------------------------------------------*/
void DEVCFG_load(void)
{
	pb_istream_t stream;
	uint16_t streamSize = 0;
	uint8_t useDefaultConfig = 1;

	IFLASH_rd(FLASH_PAGEADDR(DEVCFG_FLASHPAGE), 2, &streamSize);
	//DBG_print("\r\n StreamSize=%u", streamSize);
	if (!streamSize || (streamSize > (FLASH_PAGESIZE - 2)))
	{

	}
	else
	{
		uint8_t *rDat = MM_get(60, streamSize);
		if (rDat != NULL)
		{
			stream = pb_istream_from_buffer(rDat, streamSize);
			IFLASH_rd(FLASH_PAGEADDR(DEVCFG_FLASHPAGE) + 2, streamSize, rDat);
			//DBG_print("\r\n<stream>%s</stream>", rDat);
			if (!pb_decode(&stream, DeviceConfigV2_fields, DEVCFG_access()))
			{
				DBG_print("\r\n %s:Decode failed: %s", __func__, PB_GET_ERROR(&stream));
			}
			else
			{
				useDefaultConfig = 0;

			}
			MM_free(rDat);
		}
	}
	if (useDefaultConfig)
	{
		DeviceConfigV2 tmp = DeviceConfigV2_init_default;
		DBG_print("\r\n Use default");
		memcpy(DEVCFG_access(), &tmp, sizeof(DeviceConfigV2));
		DEVCFG_access()->CAMdelay = 5;
		DEVCFG_access()->CAMres = 3;
		DEVCFG_access()->trackDelayTime = 0;
		DEVCFG_access()->onByEngineOn = 1;
	}
}
/*-------------------------------------------------------------------------------------
 * Brief: Save device configuration from RAM to flash
 --------------------------------------------------------------------------------------*/
void DEVCFG_save(void)
{
	const uint16_t wDatSize = DeviceConfigV2_size;
	uint8_t *wDat = MM_get(60, wDatSize);
	if (wDat != NULL)
	{
		{
			pb_ostream_t stream;

			memset(wDat, 0, wDatSize);
			stream = pb_ostream_from_buffer(&wDat[2], wDatSize - 2);
			if (!pb_encode(&stream, DeviceConfigV2_fields, DEVCFG_access()))
			{
				DBG_print("\r\n %s:Encode failed: %s", __func__, PB_GET_ERROR(&stream));
			}
			else
			{
				uint8_t res = 0;
				//TSKMAIN_dbgPrt("\r\n StreamSize=%u RawSize=%u", stream.bytes_written, sizeof(DeviceConfigV2));
				memcpy(wDat, &stream.bytes_written, 2);
				res = IFLASH_wrt(FLASH_PAGEADDR(DEVCFG_FLASHPAGE), stream.bytes_written + 2, wDat);
				//TSKMAIN_dbgPrt("\r\n WriteConfig:res=%u", res);
			}
		}
		{
			pb_ostream_t stream;

			memset(wDat, 0, wDatSize);
			stream = pb_ostream_from_buffer(&wDat[2], wDatSize - 2);
			if (!pb_encode(&stream, localConfig_fields, LOCALCFG_access()))
			{
				DBG_print("\r\n %s:Encode failed: %s", __func__, PB_GET_ERROR(&stream));
			}
			else
			{
				uint8_t res = 0;
				//TSKMAIN_dbgPrt("\r\n StreamSize=%u RawSize=%u", stream.bytes_written, sizeof(localConfig));
				memcpy(wDat, &stream.bytes_written, 2);
				res = IFLASH_wrt(FLASH_PAGEADDR(LOCALCFG_FLASHPAGE), stream.bytes_written + 2, wDat);
				//TSKMAIN_dbgPrt("\r\n WriteConfig:res=%u", res);
			}
		}
		MM_free(wDat);
	}
}
#if 0
/*-------------------------------------------------------------------------------------
 * Brief: Appy new configuration to main device configuration
 * Param:	newCfg	|	I	|	new configuration
 --------------------------------------------------------------------------------------*/
void DEVCFG_apply(DeviceConfig newCfg)
{
	DEVCFG_access()->revOn = newCfg.revOn;
	DEVCFG_access()->fixOn = newCfg.fixOn;
	DEVCFG_access()->revDoor = newCfg.revDoor;
	DEVCFG_access()->fixDoor = newCfg.fixDoor;
	DEVCFG_access()->onByEngineOn = newCfg.onByEngineOn;
	DEVCFG_access()->recDatDist = newCfg.recDatDist;
	DEVCFG_access()->recDatDelayTime = newCfg.recDatDelayTime;
	if (newCfg.has_stdbyRecDatDelayTime)
	{
		DEVCFG_access()->stdbyRecDatDelayTime = newCfg.stdbyRecDatDelayTime;
	}
	if (newCfg.has_sndDatDelayTime)
	{
		DEVCFG_access()->sndDatDelayTime = newCfg.sndDatDelayTime;
	}
	if (newCfg.has_stdbySndDatDelayTime)
	{
		DEVCFG_access()->stdbySndDatDelayTime = newCfg.stdbySndDatDelayTime;
	}
	if (newCfg.has_APN)
	{
		if (newCfg.APN.has_APname)
		{
			strlcpy(DEVCFG_access()->APN.APname, newCfg.APN.APname, APNNAME_SIZE);
		}
		if (newCfg.APN.has_usr)
		{
			strlcpy(DEVCFG_access()->APN.usr, newCfg.APN.usr, APNUSR_SIZE);
		}
		if (newCfg.APN.has_pwd)
		{
			strlcpy(DEVCFG_access()->APN.pwd, newCfg.APN.pwd, APNPWD_SIZE);
		}
	}
}
#endif
/*-------------------------------------------------------------------------------------
 * Brief: Load device ID from flash to RAM
 --------------------------------------------------------------------------------------*/
void DEVID_load(void)
{
	IFLASH_rd(FLASH_PAGEADDR(DEVID_FLASHPAGE), DEVID_SIZE, deviceID);
	if ((deviceID[0] == 0xFF)//
			|| !strlen(deviceID) //
			|| (strlen(deviceID) >= DEVID_SIZE))
	{
		/* Invalid ID ? -> Use default one */
		strlcpy(deviceID, DF_DEVCFG_DEVID, DEVID_SIZE);
	}
}
/*-------------------------------------------------------------------------------------
 * Brief: Save device ID from RAM to flash
 --------------------------------------------------------------------------------------*/
void DEVID_save(void)
{
	IFLASH_wrt(FLASH_PAGEADDR(DEVID_FLASHPAGE), DEVID_SIZE, deviceID);
}
/*-------------------------------------------------------------------------------------
 * Brief: access device ID
 --------------------------------------------------------------------------------------*/
U8 *DEVID_access(void)
{
	return deviceID;
}
/*-------------------------------------------------------------------------------------
 * Brief: Load local configuration from flash to RAM
 --------------------------------------------------------------------------------------*/
void LOCALCFG_load(void)
{
	pb_istream_t stream;
	uint16_t streamSize = 0;
	uint8_t useDefaultConfig = 1;

	IFLASH_rd(FLASH_PAGEADDR(LOCALCFG_FLASHPAGE), 2, &streamSize);
	//DBG_print("\r\n StreamSize=%u", streamSize);
	if (!streamSize || (streamSize > (FLASH_PAGESIZE - 2)))
	{

	}
	else
	{
		uint8_t *rDat = MM_get(60, streamSize);
		if (rDat != NULL)
		{
			stream = pb_istream_from_buffer(rDat, streamSize);
			IFLASH_rd(FLASH_PAGEADDR(LOCALCFG_FLASHPAGE) + 2, streamSize, rDat);
			//DBG_print("\r\n<stream>%s</stream>", rDat);
			if (!pb_decode(&stream, localConfig_fields, LOCALCFG_access()))
			{
				DBG_print("\r\n %s:Decode failed: %s", __func__, PB_GET_ERROR(&stream));
			}
			else
			{
				useDefaultConfig = 0;
			}
			MM_free(rDat);
		}
	}
	if (useDefaultConfig)
	{
		localConfig tmp = localConfig_init_default;
		DBG_print("\r\n Use default");
		memcpy(LOCALCFG_access(), &tmp, sizeof(localConfig));
	}
}
/*-------------------------------------------------------------------------------------
 * Brief: Save local configuration from RAM to flash
 --------------------------------------------------------------------------------------*/
void LOCALCFG_save(void)
{
	const uint16_t wDatSize = DeviceConfigV2_size;
	uint8_t *wDat = MM_get(60, wDatSize);
	if (wDat != NULL)
	{
		pb_ostream_t stream;

		memset(wDat, 0, wDatSize);
		stream = pb_ostream_from_buffer(&wDat[2], wDatSize - 2);
		if (!pb_encode(&stream, localConfig_fields, LOCALCFG_access()))
		{
			DBG_print("\r\n %s:Encode failed: %s", __func__, PB_GET_ERROR(&stream));
		}
		else
		{
			uint8_t res = 0;
			//TSKMAIN_dbgPrt("\r\n StreamSize=%u RawSize=%u", stream.bytes_written, sizeof(localConfig));
			memcpy(wDat, &stream.bytes_written, 2);
			res = IFLASH_wrt(FLASH_PAGEADDR(LOCALCFG_FLASHPAGE), stream.bytes_written + 2, wDat);
			//TSKMAIN_dbgPrt("\r\n WriteConfig:res=%u", res);
		}
		MM_free(wDat);
	}
}
/*-------------------------------------------------------------------------------------
 * Brief: access local configuration
 --------------------------------------------------------------------------------------*/
localConfig *LOCALCFG_access(void)
{
	return &mainLocalCfg;
}
/*-------------------------------------------------------------------------------------
 * Brief: access session configuration
 --------------------------------------------------------------------------------------*/
SESSIONCFG_t *SESSIONCFG_access(void)
{
	return &mainSessionCfg;
}
/*-------------------------------------------------------------------------------------
 * Brief: Load APN profiles from flash to RAM
 --------------------------------------------------------------------------------------*/
void APNPROFILE_load(void)
{
	U8 countFreeSlot = 0;
	IFLASH_rd(FLASH_PAGEADDR(APN_PROFILES_FLASHPAGE), sizeof(APNprofiles), APNprofiles);
	for (U8 index = 0; index < MAX_APNPROFILE; index++)
	{
		if (APNprofiles[index].OP[0] == 0xFF)
		{
			memset(&APNprofiles[index], 0, sizeof(APN_PROFILE_t));
			countFreeSlot++;
		}
	}
}
/*-------------------------------------------------------------------------------------
 * Brief: Save APN profiles from RAM to flash
 --------------------------------------------------------------------------------------*/
void APNPROFILE_save(void)
{
	IFLASH_wrt(FLASH_PAGEADDR(APN_PROFILES_FLASHPAGE), sizeof(APNprofiles), APNprofiles);
}
/*-------------------------------------------------------------------------------------
 * Brief: Get an APN profile with index
 * Param:	index	|	IN	|	index
 * Return: APN profile.
 --------------------------------------------------------------------------------------*/
const APN_PROFILE_t *APNPROFILE_get(U8 index)
{
	return &APNprofiles[index];
}
/*-------------------------------------------------------------------------------------
 * Brief: Look up an APN profile with input operator (network name).
 * Param:	OP	|	IN	|	Operator
 * Return: APN profile.
 --------------------------------------------------------------------------------------*/
const APN_PROFILE_t *APNPROFILE_lookup(const U8 *OP)
{
	strupr(OP);
	for (U8 index = 0; index < MAX_APNPROFILE; index++)
	{
		//if (strlen(APNprofiles[index].OP))
		{
			if (SL_search(OP, APNprofiles[index].OP) != -1)
			{
				return &APNprofiles[index];
			}
		}
	}
	return &df_APNprofile;
}
/*-------------------------------------------------------------------------------------
 * Brief: Delete an APN profile with input operator (network name).
 * Param:	OP	|	IN	|	Operator
 * Return: APN profile.
 --------------------------------------------------------------------------------------*/
void APNPROFILE_del(const U8 *OP)
{
	strupr(OP);
	for (U8 index = 0; index < MAX_APNPROFILE; index++)
	{
		//if (strlen(APNprofiles[index].OP))
		{
			if (SL_search(OP, APNprofiles[index].OP) != -1)
			{
				memset(&APNprofiles[index], 0, sizeof(APN_PROFILE_t));
				return;
			}
		}
	}
}
/*-------------------------------------------------------------------------------------
 * Brief: delete an APN profile.
 * Param: newProfile	|	IN	|	APN profile to add
 --------------------------------------------------------------------------------------*/
void APNPROFILE_delAll(void)
{
	memset(APNprofiles, 0, sizeof(APN_PROFILE_t) * MAX_APNPROFILE);
}
/*-------------------------------------------------------------------------------------
 * Brief: Add new APN profile.
 * Param: newProfile	|	IN	|	APN profile to add
 * Ret:	0	|	OK
 * 		>0	|	FAIL
 --------------------------------------------------------------------------------------*/
U8 APNPROFILE_add(APN_PROFILE_t newProfile)
{
	/* Check if it is matched with existed profile */
	strupr(newProfile.OP);
	for (U8 index = 0; index < MAX_APNPROFILE; index++)
	{
		//if (strlen(APNprofiles[index].OP))
		{
			if (!strcmp(newProfile.OP, APNprofiles[index].OP))
			{
				/* Matched operator with existed profile -> Update this profile */
				APNprofiles[index] = newProfile;
				return 0;
			}
		}
	}
	/* Insert to 1st found free slot */
	for (U8 index = 0; index < MAX_APNPROFILE; index++)
	{
		if (!strlen(APNprofiles[index].OP))
		{
			APNprofiles[index] = newProfile;
			return 0;
		}
	}
	/* No free slot */
	return 1;
}
