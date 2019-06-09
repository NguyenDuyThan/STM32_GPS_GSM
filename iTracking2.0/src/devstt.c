/*------------------------------------------------------------------------------------------
 * INTRODUCE
 * Brief: This is source file containing all function relating to device status.
 * Author: Duy Vinh To - Team DinhViSo
 * Detail:
 -------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------
 * INCLUDES
 -------------------------------------------------------------------------------------------*/
#include "devstt.h"
#include "STM32F1_rtc.h"
#include "stdio.h"
#include "string.h"
#include "stringlib.h"
/*------------------------------------------------------------------------------------------
 * DEFINES
 -------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------
 * VARIBLES
 -------------------------------------------------------------------------------------------*/
DEVSTT_t mainDevStt;
/*------------------------------------------------------------------------------------------
 * FUNCTIONS
 -------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------
 * Brief: access device status
 -----------------------------------------------------------------------------------------*/
DEVSTT_t *DEVSTT_access(void)
{
	return &mainDevStt;
}
/*----------------------------------------------------------------------------------------
 * Brief: Generate device status report
 -----------------------------------------------------------------------------------------*/
void DEVSTT_genRpt(U16 size, U8 *s)
{
	DATETIME_t dt;
	U8 sHDOP[6], sLat[12], sLng[12], sBATpct[6];

	snprintf(s, size, "\r\nSRT=%lu", IRTC_getSRT());
	strlcat(s, "\r\n-GNSS:", size);
	if (DEVSTT_access()->GNSSdbg.stdby)
	{
		snprintf(s, size, "%s stdby", s);
	}
	else
	{
		snprintf(s, size, "%s\r\n+GGA=%u GSA=%u RMC=%u", s//
				, DEVSTT_access()->GNSS.GGAavail//
				, DEVSTT_access()->GNSS.GSAavail//
				, DEVSTT_access()->GNSS.RMCavail);
		snprintf(s, size, "%s\r\n+HDOP=%s SAT=%u F3D=%u FQ=%u", s//
				, SL_double2s(DEVSTT_access()->GNSS.HDOP, 6, sHDOP)//
				, DEVSTT_access()->GNSS.NumOfSat//
				, DEVSTT_access()->GNSS.fix3D//
				, DEVSTT_access()->GNSS.fixQuality);
		snprintf(s, size, "%s\r\n+Coor=%s,%s Spd=%u C=%u", s//
				, SL_double2s(DEVSTT_access()->GNSS.lat, 12, sLat)//
				, SL_double2s(DEVSTT_access()->GNSS.lng, 12, sLng)//
				, DEVSTT_access()->GNSS.spd//
				, DEVSTT_access()->GNSS.course);
		snprintf(s, size, "%s\r\n+DT:%02u%02u%02u-%02u%02u%02u", s//
				, DEVSTT_access()->GNSS.dt.year//
				, DEVSTT_access()->GNSS.dt.mon//
				, DEVSTT_access()->GNSS.dt.day//
				, DEVSTT_access()->GNSS.dt.hour//
				, DEVSTT_access()->GNSS.dt.min//
				, DEVSTT_access()->GNSS.dt.sec);
	}
	snprintf(s, size, "%s\r\n-Net:", s);
	snprintf(s, size, "%s\r\n+SQ=%u OP=\"%s\" NRS=%u", s//
			, DEVSTT_access()->net.signalQuality//
			, DEVSTT_access()->net.network//
			, DEVSTT_access()->net.netRegStt);
	snprintf(s, size, "%s\r\n+SockF=%lu Conn2ServBits=0x%X", s//
			, DEVSTT_access()->GPRSdbg.sockFC//
			, DEVSTT_access()->net.connect2servBits//
			);
	snprintf(s, size, "%s\r\n-RB:", s);
	dt = IRTC_utc2dt(DEVSTT_access()->RBtrkDat.utc, DEFAULT_SYSTEM_GMT);
	snprintf(s, size, "%s\r\n+D=%u-%u-%u Ofs=%lu Sz=%lu", s//
			, dt.year, dt.month, dt.day//
			, DEVSTT_access()->RBtrkDat.ofs//
			, DEVSTT_access()->RBtrkDat.size);
	snprintf(s, size, "%s\r\n+SndPhotoNATS:\"%s\" S=%lu rO=%lu sO=%lu", s//
			, (DEVSTT_access()->sndPhotoViaNATS.fname == NULL) ? "" : DEVSTT_access()->sndPhotoViaNATS.fname//
			, DEVSTT_access()->sndPhotoViaNATS.fsize//
			, DEVSTT_access()->sndPhotoViaNATS.rd_ofs//
			, DEVSTT_access()->sndPhotoViaNATS.snt_ofs//
			, DEVSTT_access()->sndPhotoViaNATS.utc//
			);
	if ((DEVSTT_access()->sndFileViaNATS.fname != NULL) && (strlen(DEVSTT_access()->sndFileViaNATS.fname)))
	{
		snprintf(s, size, "%s\r\n+SndPhotoNATS:\"%s\" S=%lu rO=%lu sO=%lu utc=%lu", s//
				, DEVSTT_access()->sndFileViaNATS.fname//
				, DEVSTT_access()->sndFileViaNATS.fsize//
				, DEVSTT_access()->sndFileViaNATS.rd_ofs//
				, DEVSTT_access()->sndFileViaNATS.snt_ofs//
				, DEVSTT_access()->sndFileViaNATS.utc//
				);
	}
	snprintf(s, size, "%s\r\n-SENS:BAT=%s KSOn=%u EGOn=%u DROp=%u WP=%lu fuel=%u", s//
			, SL_double2s(DEVSTT_access()->sens.BATpct, 6, sBATpct)//
			, DEVSTT_access()->sens.KSOn//
			, DEVSTT_access()->sens.EGOn//
			, DEVSTT_access()->sens.DROp//
			, DEVSTT_access()->sens.WP//
			, DEVSTT_access()->sens.fuel//
			);
}
