/* Martin Thomas 4/2009 */

#if !_FS_NORTC
#include "STM32F1_rtc.h"

unsigned long get_fattime(void)
{
	unsigned long res;
	DATETIME_t dt;

	dt = IRTC_getDT(DEFAULT_SYSTEM_GMT);
	res = ((dt.year % 100) << 25)//
			| (dt.month << 21)//
			| (dt.day << 16)//
			| (dt.hour << 11)//
			| (dt.minute << 5)//
			| (dt.second >> 1);

	return res;
}
#endif // _FS_NORTC
