/*------------------------------------------------------------------------------------------
 * INTRODUCE
 * Brief: This is header file containing all function relating to device configuration.
 * Author: Duy Vinh To - Team DinhViSo
 * Detail:
 -------------------------------------------------------------------------------------------*/

#ifndef DEVCFG_H_
#define DEVCFG_H_

/*------------------------------------------------------------------------------------------
 * INCLUDES
 -------------------------------------------------------------------------------------------*/
#include "deviceConfig.pb.h"
/*------------------------------------------------------------------------------------------
 * DEFINES
 -------------------------------------------------------------------------------------------*/
#define DEVID_SIZE		11
#define DEVCFG_HOSTDATA_SIZE	64

/* Default device configuration */
#define DF_DEVCFG_DEVID					"0000000000"
#define DF_DEVCFG_RECDATDIST			30
#define DF_DEVCFG_RECDATDELAYTIME		10
#define DF_DEVCFG_STDBYRECDATDELAYTIME	10
#define DF_DEVCFG_SNDDATDELAYTIME		10
#define DF_DEVCFG_STDBYSNDDATDELAYTIME	10
#define DF_DEVCFG_APNNAME				"m-wap"//"e-connect"
#define DF_DEVCFG_APNUSR				"mms"//""
#define DF_DEVCFG_APNPWD				"mms"//""
#define APNPROFILE_PARAM_SIZE			64
#define DF_DEVCFG_HOSTDATA				"itracking.vn"
#define DF_DEVCFG_HOSTINFCFG			"itracking.vn"
#define DF_DEVCFG_FTP_FW_HOST			"itracking.vn"
#define DF_DEVCFG_FTP_FW_USR			""
#define DF_DEVCFG_FTP_FW_PWD			""
#define DF_DEVCFG_FTP_FW_PATH			"iTracking2.0"
#define DF_DEVCFG_FTP_FW_FNAME_PREFIX	"iTracking2.0V"
#define DF_DEVCFG_FTP_FW_FNAME_SUFFIX	".bin"

/* ATTENTION: below 'DEVCFG_PARAM' must be matched with order of field tag in 'DeviceConfig.pb.h' */
/* Always put '<END>' at the end to break scanning for appropriate config parameter, OR CAUSE CRITICAL ERROR */
#define DEVCFG_PARAM  {"<START>"\
						,"FIXON "\
						,"REVON "\
						,"FIXDR "\
						,"REVDR "\
						,"OFFBUZZ "\
						,"TRKDT "\
						,"TRKDST "\
						,"SBTRKDT "\
						,"OBEO "\
						,"CAMD "\
						,"CAMR "\
						,"SBCAMD " \
						,"<END>"\
						}

/*------------------------------------------------------------------------------------------
 * TYPEDEFS
 -------------------------------------------------------------------------------------------*/
typedef unsigned char U8;
typedef unsigned short U16;
typedef unsigned long U32;

typedef struct
{
	U8 enTRACE;
	U8 enECHO;
	U8 dbgGSM;
	U8 dbgGNSS;
	U8 stresstest;
	U16 EG_ADCmin; // Engine ADC minimum value.
}SESSIONCFG_t; // Session configuration (reset to default on reset).

typedef struct
{
	U8 OP[APNPROFILE_PARAM_SIZE]; // Operator
	U8 APNn[APNPROFILE_PARAM_SIZE];
	U8 APNu[APNPROFILE_PARAM_SIZE];
	U8 APNp[APNPROFILE_PARAM_SIZE];
}APN_PROFILE_t;

#define MAX_APNPROFILE		(FLASH_PAGESIZE / sizeof(APN_PROFILE_t))
/*------------------------------------------------------------------------------------------
 * VARIABLES
 -------------------------------------------------------------------------------------------*/
//U8 DEVCFG_enDBG = 0;

/*------------------------------------------------------------------------------------------
 * FUNC.PROTOTYPES
 -------------------------------------------------------------------------------------------*/
extern void DEVID_load(void);
extern void DEVID_save(void);
extern U8 *DEVID_access(void);
extern DeviceConfigV2 *DEVCFG_access(void);
extern void DEVCFG_save(void);
extern void DEVCFG_load(void);
extern void DEVCFG_apply(DeviceConfigV2 newCfg);
extern localConfig *LOCALCFG_access(void);
extern void LOCALCFG_load(void);
extern void LOCALCFG_save(void);
extern SESSIONCFG_t *SESSIONCFG_access(void);
extern void APNPROFILE_load(void);
/* CAUTION: because this will be saved to MCU flash, avoid call it when interrupt can interfere write process (cause system hang) */
extern void APNPROFILE_save(void);
extern const APN_PROFILE_t *APNPROFILE_get(U8 index);
extern const APN_PROFILE_t *APNPROFILE_lookup(const U8 *OP);
extern void APNPROFILE_del(const U8 *OP);
extern void APNPROFILE_delAll(void);
extern U8 APNPROFILE_add(APN_PROFILE_t newProfile);


#endif /* DEVCFG_H_ */
