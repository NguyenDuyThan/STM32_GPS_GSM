/*------------------------------------------------------------------------------------------
 * INTRODUCE
 * Brief: This is main file of "iTracking2.0 bootloader" project.
 * Author: Duy Vinh To - Team DinhViSo
 * Detail:
 -------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------
 * INCLUDES
 -------------------------------------------------------------------------------------------*/
/* C libraries */
#include "stddef.h"
#include "string.h"
#include "stdio.h"
/* HW libraries */
#include "STM32F1_uart.h"
#include "STM32F1_tim.h"
#include "STM32F1_rtc.h"
#include "STM32F1_flash.h"
#include "STM32F1_io.h"
#include "QuectelM95.h"
#include "fileSystem.h"
/* SW libraries */
#include "md5.h"
#include "dbgPrint.h"
/* STM32 CMSIS standard libraries */
#include "stm32f10x_bkp.h"
#include "stm32f10x_iwdg.h"
#include "stm32f10x_rcc.h"
#include "misc.h"
/* Others */
#include "diskio.h"
#include "macro.h"
#include <HWmapping_iTrackingV2.3.h>
/*------------------------------------------------------------------------------------------
 * DEFINES
 -------------------------------------------------------------------------------------------*/
#define SAVE_DOWNLOADFW_2_FS	0
#define DEBUG_QM95				0
#define APP_FLASHADDR   		FLASH_PAGEADDR(APP_FLASHPAGE_ORIGIN)
#define APP_FLASHMAXSIZE		FLASH_PAGESIZE * (APP_FLASHPAGE_NUM)// From page 52 to page 179
#define FIRMWARE_FNAME			"fw.bin"
#define MD5_FNAME				"cs.md5"
#define BKP_FIRMWARE_FNAME		"bkpfw.bin"
#define BKP_MD5_FNAME			"bkpcs.md5"
#define FTP_HOST_DEFAULT		"itracking.vn"
#define FTP_PORT_DEFAULT		21
#define FTP_USER_DEFAULT		"firmWare"
#define FTP_PWD_DEFAULT			"7>.*?;W!e6~[/9:"
#define FTP_PATH_DEFAULT		"/iTracking2.0/"
#define FTP_FW_MD5_FNAME_PREFIX	"iTracking2.0V"
#define FTP_FW_FNAME_SUFFIX		".bin"
#define FTP_MD5_FNAME_SUFFIX	".md5"

#define QM95_BUFSIZE			(1024 * 5)
#define MD5RESULT_SIZE			16
/*------------------------------------------------------------------------------------------
 * TYPEDEFS
 -------------------------------------------------------------------------------------------*/
typedef void (*pFunction)(void);
/*------------------------------------------------------------------------------------------
 * VARIABLES
 -------------------------------------------------------------------------------------------*/
static uint32_t TIM2_tickcounter = 0;
static uint32_t timerTickUntilToggleLEDstt = 0;
static uint8_t QM95_buf[QM95_BUFSIZE];
/*------------------------------------------------------------------------------------------
 * FUNC.PROTOTYPES
 -------------------------------------------------------------------------------------------*/
static uint8_t checkCRC_bkpRegVal(uint16_t val);
static void calcCRC_bkpRegVal(uint16_t *val);
static uint8_t getBootSel(void);
static void wrtBootSel(uint8_t bootsel);
static void getUpgFWver(uint16_t *mainVer, uint16_t *subVer);
static void wrtUpgFWver(uint16_t mainVer, uint16_t subVer);
static void jump2app(void);
static void DBG_sendc(uint8_t c);
static void RTC_tick(void);
static void IWDG_setup(void);
static void TIM2_tick_callback(void);
static void QM95_PWRDWNPinSet_callback(uint8_t set);
static void TIM2_delay10ms(uint32_t t);
static uint8_t backupFWtoFS(void);
static uint8_t upgradeFromFS(uint8_t *isUpgraded);
static void QM95_sendc(uint8_t c);
static void UART2_readc(uint8_t c);
static void NVIC_setup(void);
static U8 upgradeViaFTP(uint16_t mainVer, uint16_t subVer);
static uint8_t genMD5fromFile(const uint8_t *fname, uint8_t MD5out[16]);
static void genMD5fromFlashzone(uint32_t startAddr, uint32_t len, uint8_t MD5out[16]);
static uint8_t writeFile2flash(const uint8_t *fname, uint32_t startAddr);
static uint8_t saveFlash2File(const uint8_t *fname, uint32_t startAddr, uint32_t size);
static uint8_t convTxt2MD5(const uint8_t *text, uint8_t *MD5);
/*------------------------------------------------------------------------------------------
 * FUNCTIONS
 -------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------
 * Brief: initial UART1
 -------------------------------------------------------------------------------------------*/
void TIM2_delay10ms(uint32_t t)
{
	TIM2_tickcounter = 0;
	while (TIM2_tickcounter < t);
}
/*------------------------------------------------------------------------------------------
 * Brief: initial UART1
 -------------------------------------------------------------------------------------------*/
void DBG_sendc(uint8_t c)
{
	UART_sendc(1, c);
}
/*------------------------------------------------------------------------------------------
 * Brief: initial UART1
 -------------------------------------------------------------------------------------------*/
void QM95_sendc(uint8_t c)
{
	UART_sendc(2, c);
#if DEBUG_QM95
	UART_sendc(1, c);
#endif
}
/*------------------------------------------------------------------------------------------
 * Brief: initial UART1
 -------------------------------------------------------------------------------------------*/
void UART2_readc(uint8_t c)
{
	QM95_readc(c);
#if DEBUG_QM95
	UART_sendc(1, c);
#endif
}
/*------------------------------------------------------------------------------------------
 * Brief:
 -------------------------------------------------------------------------------------------*/
void NVIC_setup(void)
{
	NVIC_InitTypeDef NVIC_InitStruct;
	uint8_t prio = 0;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	for (prio = 1; prio < 0xFF; prio++)
	{
		switch (prio)
		{
			case 1:
				NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
				break;
			case 2:
				NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
				break;
			case 3:
				NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
				break;
			case 4:
				NVIC_InitStruct.NVIC_IRQChannel = RTC_IRQn;
				break;
			default:
				return;
		}
		NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = prio;
		NVIC_Init(&NVIC_InitStruct);
	}
}
/*-----------------------------------------------------------------------------
 * Brief: setup watch dog
 ------------------------------------------------------------------------------*/
void IWDG_setup(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);

	/* IWDG timeout equal to 350ms (the timeout may varies due to LSI frequency
	 dispersion) -------------------------------------------------------------*/
	/* Enable write access to IWDG_PR and IWDG_RLR registers */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

	/* IWDG counter clock: 32KHz(LSI) / 32 = 1KHz */
	IWDG_SetPrescaler(IWDG_Prescaler_256);

	/* Set counter reload value to 349 */
	IWDG_SetReload(1799);

	/* Reload IWDG counter */
	IWDG_ReloadCounter();

	/* Enable IWDG (the LSI oscillator will be enabled by hardware) */
	IWDG_Enable();
}
/*------------------------------------------------------------------------------------------
 * Brief: Jump to application flash.
 -------------------------------------------------------------------------------------------*/
void jump2app(void)
{
	pFunction Jump_To_Application;
	uint32_t JumpAddress;

	/* Initialize user application's Stack Pointer */
	__set_MSP(*(__IO uint32_t*) APP_FLASHADDR);
	/* Jump to user application */
	JumpAddress = *(__IO uint32_t*) (APP_FLASHADDR + 4);
	Jump_To_Application = (pFunction) JumpAddress;
	/* */
	__disable_irq();
	/* Make a jump */
	Jump_To_Application();
}
/*------------------------------------------------------------------------------------------
 * Brief: check CRC of a backup register value.
 * Param: val	|	IN	|	<>
 * Ret:		0	|	OK
 * 			1	|	FAIL
 -------------------------------------------------------------------------------------------*/
uint8_t checkCRC_bkpRegVal(uint16_t val)
{
	if (((val >> 12) & 0xF) == (((val >> 8) & 0xF) ^ ((val >> 4) & 0xF) ^ (val & 0xF)))
	{
		return 0;
	}
	return 1;
}
/*------------------------------------------------------------------------------------------
 * Brief: calculate CRC of a backup register value.
 * Param: val	|	IN	|	<>
 * Ret:		0	|	OK
 * 			1	|	FAIL
 -------------------------------------------------------------------------------------------*/
void calcCRC_bkpRegVal(uint16_t *val)
{
	uint8_t crc = ((*val >> 8) & 0xF) ^ ((*val >> 4) & 0xF) ^ (*val & 0xF);
	*val &= 0x0FFF;
	*val |= crc << 12;
}
/*------------------------------------------------------------------------------------------
 * Brief: get boot selection.
 * Ret:	boot selection.
 -------------------------------------------------------------------------------------------*/
uint8_t getBootSel(void)
{
	uint16_t bkpRegVal = BKP_ReadBackupRegister(BKPDR_BOOTSEL);
	return (checkCRC_bkpRegVal(bkpRegVal)) ? 0 : bkpRegVal;
}
/*------------------------------------------------------------------------------------------
 * Brief: write boot selection.
 * Param:	bootsel	|	IN	|	boot selection.
 * Note:
 * + Boot selection = 0: Boot into bootloader.
 * + Boot selection = 1: Boot into application.
 -------------------------------------------------------------------------------------------*/
void wrtBootSel(uint8_t bootsel)
{
	uint16_t bkpRegVal = 0;
//	bkpRegVal = BKP_ReadBackupRegister(BKP_DR1);
//	bkpRegVal = (!GETBIT(bootsel, 0)) ? RSTBIT(bkpRegVal, 0) : SETBIT(bkpRegVal, 0);
//	bkpRegVal = (!GETBIT(bootsel, 1)) ? RSTBIT(bkpRegVal, 1) : SETBIT(bkpRegVal, 1);
	bkpRegVal = bootsel;
	calcCRC_bkpRegVal(&bkpRegVal);
	IRTC_enableBackupAccess();
	BKP_WriteBackupRegister(BKPDR_BOOTSEL, bkpRegVal);
}
/*------------------------------------------------------------------------------------------
 * Brief: get planned upgrading version.
 * Param:	mainVer	|	OUT	|	<>
 * 			subVer	|	OUT	|	<>
 -------------------------------------------------------------------------------------------*/
void getUpgFWver(uint16_t *mainVer, uint16_t *subVer)
{
	uint16_t bkpRegVal = BKP_ReadBackupRegister(BKPDR_UPGFW_MAINVER);
	if (checkCRC_bkpRegVal(bkpRegVal))
	{
		*mainVer = 0;
	}
	else
	{
		*mainVer = bkpRegVal & 0x0FFF;
	}
	 bkpRegVal = BKP_ReadBackupRegister(BKPDR_UPGFW_SUBVER);
	if (checkCRC_bkpRegVal(bkpRegVal))
	{
		*subVer = 0;
	}
	else
	{
		*subVer = bkpRegVal & 0x0FFF;
	}
}
/*------------------------------------------------------------------------------------------
 * Brief: write planned upgrading version.
 * Param:	mainVer	|	IN	|	<>
 * 			subVer	|	IN	|	<>
 -------------------------------------------------------------------------------------------*/
void wrtUpgFWver(uint16_t mainVer, uint16_t subVer)
{
	IRTC_enableBackupAccess();
	calcCRC_bkpRegVal(&mainVer);
	BKP_WriteBackupRegister(BKPDR_UPGFW_MAINVER, mainVer);
	calcCRC_bkpRegVal(&subVer);
	BKP_WriteBackupRegister(BKPDR_UPGFW_SUBVER, subVer);
}
/*------------------------------------------------------------------------------------------
 * Brief: Jump to application flash.
 * Note:
 -------------------------------------------------------------------------------------------*/
int main(void)
{
	uint16_t upgFWmainVer = 0, upgFWsubVer = 0;
	uint8_t bootsel = 0, upgViaFTP_rtry = 0, renBkpFW = 0, res = 0;

	SystemInit();
	IWDG_setup();
	/* Setup debug feature */
	UART_setup(1, 115200, NULL);
	DBG_setup(DBG_sendc);
	DBG_print("\r\n+----- iTracking2.0:BOOTLOADER -----+\r\n");
	bootsel = getBootSel();
	if (bootsel == 1)
	{
		/* Clear boot option */
		wrtBootSel(0);
		/* Jump to main application */
		DBG_print("\r\n Boot to app");
		jump2app();
	}
	getUpgFWver(&upgFWmainVer, &upgFWsubVer);
	/* Clear planned upgrading FW versions */
	//DBG_print("\r\n Clear upgrade FW version");
	wrtUpgFWver(0, 0);
	DBG_print("\r\n Boot selection:%u", bootsel);
	DBG_print("\r\n FW to upgrade:%u.%u", upgFWmainVer, upgFWsubVer);
	if (!upgFWmainVer && !upgFWsubVer)
	{
		/* Clear boot option */
		wrtBootSel(0);
		/* Jump to main application */
		jump2app();
	}
	DBG_print("\r\n Setup system:");
	/* Re-setup NVIC priorities */
	NVIC_setup();
	/* Setup LED indicator pinout */
	IO_setup(LEDPWR_PORT, LEDPWR_PIN, IODIR_OPP);
	IO_setup(LEDGSM_PORT, LEDGSM_PIN, IODIR_OPP);
	IO_setup(LEDLINK_PORT, LEDLINK_PIN, IODIR_OPP);
	IO_setup(LEDGNSS_PORT, LEDGNSS_PIN, IODIR_OPP);
	IO_setup(LEDLOGON_PORT, LEDLOGON_PIN, IODIR_OPP);
	IO_setup(LEDSDC_PORT, LEDSDC_PIN, IODIR_OPP);
	IO_wrt(LEDGSM_PORT, LEDGSM_PIN, 0);
	/* Setup RTC */
	IRTC_setup(IRTCCS_LSE, 100000, RTC_tick);
	/* Setup internal flash */
	IFLASH_setup();
	/* Setup timer 2: main delay function "TIM2_delay10ms" depend on this */
	TIM_setup(2, TIMFREQ_100Hz, TIM2_tick_callback);
	/* Setup file system mid layer */
	FS_setup(TIM2_delay10ms);
	FS_setup_dbgPrt(DBG_print);
	/* Setup Quectel M95 mid layer */
	IO_setup(QM95_PWRDWN_PORT, QM95_PWRDWN_PIN, IODIR_OPP);
	//QM95_setup_dbgPrt(DBG_print);
	QM95_setup(QM95_buf, QM95_BUFSIZE, QM95_sendc, NULL, QM95_PWRDWNPinSet_callback, TIM2_delay10ms);
	QM95_setup_dbgPrt(DBG_print);
	UART_setup(2, 115200, UART2_readc);
#if 1
	DBG_print(" Done");
	DBG_print("\r\n Mount file system...");
	if (!FS_mount())
	{
		DBG_print(" Done");
		IO_wrt(LEDSDC_PORT, LEDSDC_PIN, 1);
		if (upgFWmainVer)
		{
			backupFWtoFS();
			goto MAIN_UPGRADEFW_VIA_GPRS;
		}
		else
		{
			uint8_t isUpgraded = 0;
			/* Upgrade with available firmware in file system */
			if (!upgradeFromFS(&isUpgraded))
			{
				/* Successfully updated firmware from file system */
				if (isUpgraded)
				{
					goto MAIN_READY2BOOT2APP;
				}
			}
		}
	}
	else
	{
		DBG_print(" Error");
	}
#endif
	if (!upgFWmainVer)
	{
		/* There is no upgrading request */
		DBG_print("\r\n No upgrading request");
		goto MAIN_READY2BOOT2APP;
	}
	MAIN_UPGRADEFW_VIA_GPRS://
	DBG_print("\r\n Repowering GSM");
	QM95_rpw(1000);
	{
		NETREGSTT_t netRegstt;
		uint32_t timestart = IRTC_getSRT();
		while (1)
		{
			if (IRTC_getSRT() >= (timestart + 60))
			{
				DBG_print(" <!> NO GSM");
				goto MAIN_READY2BOOT2APP;
			}
			if (!QM95_getNetRegStt(&netRegstt))
			{
				if ((netRegstt == NRS_REG) || (netRegstt == NRS_ROAMING))
				{
					DBG_print(" Done");
					IO_wrt(LEDGSM_PORT, LEDGSM_PIN, 1);
					break;
				}
			}
			TIM2_delay10ms(100);
			DBG_print(".");
		}
	}
//	{
//		NETREGSTT_t nrs;
//		DBG_print("\r\n Check network status");
//		if (QM95_getNetRegStt(&nrs))
//		{
//			goto MAIN_READY2BOOT2APP;
//		}
//		DBG_print("\r\n Net registration status:%s", QM95_explainNetRegStt(nrs));
//	}
	/* Backup firmware to file system */
	res = upgradeViaFTP(upgFWmainVer, upgFWsubVer);
	if (res)
	{
		//goto MAIN_READY2BOOT2APP;
//		wrtBootSel(1);
//		TIM2_delay10ms(50);
//		NVIC_SystemReset();
		if (res > 2)
		{
			renBkpFW = 1;
		}
		if (upgViaFTP_rtry++ < 3)
		{
			goto MAIN_UPGRADEFW_VIA_GPRS;
		}
	}
	else
	{
		renBkpFW = 0;
	}
#if 0
	/* Upgrade with downloaded firmware */
	{
		uint8_t isUpgraded = 0;
		if (!upgradeFromFS(&isUpgraded))
		{
			/* Successfully upgraded firmware from file system */
			if (isUpgraded)
			{
				//
			}
		}
	}
#endif
	MAIN_READY2BOOT2APP://
	/* Next reboot will jump into main application */
	DBG_print("\r\n Change boot selection");
	if (renBkpFW)
	{
		DBG_print("\r\n Ready to load backup firmware");
		FS_renameFile(BKP_FIRMWARE_FNAME, FIRMWARE_FNAME);
		FS_renameFile(BKP_MD5_FNAME, MD5_FNAME);
		/* Clear boot option */
		wrtBootSel(0);
		/* Write fake fw version */
		wrtUpgFWver(0, 1); // Main version is zero -> upgrade FW from file system
	}
	else
	{
		wrtBootSel(1);
	}
	DBG_print("\r\n Reboot \r\n");
	TIM2_delay10ms(50);
	NVIC_SystemReset();
	while(1)
	{
		DBG_print("\r\n ERROR:%u", IRTC_getSRT());
		TIM2_delay10ms(100);
	}
	return 0;
}
/*-----------------------------------------------------------------------------
 * Brief: RTC tick callback
 ------------------------------------------------------------------------------*/
void RTC_tick(void)
{
//	if (IRTC_getSRT() < (t_last_TSKMAIN_startCycle + 30))
//	{
//		//IWDG_ReloadCounter();
//	}
}
/*-----------------------------------------------------------------------------
 * Brief: RTC tick callback
 ------------------------------------------------------------------------------*/
void TIM2_tick_callback(void)
{
	TIM2_tickcounter++;
	disk_timerproc();
	timerTickUntilToggleLEDstt++;
	if (!(timerTickUntilToggleLEDstt % 10))
	{
		IO_toggle(LEDPWR_PORT, LEDPWR_PIN);
	}
	if (timerTickUntilToggleLEDstt >= 100)
	{
		timerTickUntilToggleLEDstt = 1;
		IWDG_ReloadCounter();
	}
}
/*-----------------------------------------------------------------------------
 * Brief: Quectel M95 power down pin set callback
 ------------------------------------------------------------------------------*/
void QM95_PWRDWNPinSet_callback(uint8_t set)
{
	IO_wrt(QM95_PWRDWN_PORT, QM95_PWRDWN_PIN, set);
}
/*-----------------------------------------------------------------------------
 * Brief: Backup firmware to file system
 * Ret:	0	|	OK
 * 		>0	|	FAIL
 ------------------------------------------------------------------------------*/
uint8_t backupFWtoFS(void)
{
	uint8_t MD5fileContent[(MD5RESULT_SIZE * 2) + 1]//
			, MD5flash[MD5RESULT_SIZE]//
			, MD5file[MD5RESULT_SIZE];

	DBG_print("\r\n Start backup firmware to file system");
	DBG_print("\r\n Save firmware to file system...");
	if (saveFlash2File(BKP_FIRMWARE_FNAME, APP_FLASHADDR, APP_FLASHMAXSIZE))
	{
		DBG_print(" <!> Error");
		return 1;
	}
	DBG_print(" Done");
	DBG_print("\r\n Generate MD5 from firmware in file system...");
	if (genMD5fromFile(BKP_FIRMWARE_FNAME, MD5file))
	{
		DBG_print(" <!> Error");
		return 2;
	}
	DBG_print(" Done");
	DBG_print("\r\n Generate MD5 from firmware in flash...");
	genMD5fromFlashzone(APP_FLASHADDR, APP_FLASHMAXSIZE, MD5flash);
	DBG_print(" Done");
	DBG_print("\r\n Compare MD5:");
	if (memcmp(MD5file, MD5flash, MD5RESULT_SIZE))
	{
		DBG_print(" <!> Unmatched");
		return 3;
	}
	DBG_print(" Matched");
	memset(MD5fileContent, 0, (MD5RESULT_SIZE * 2) + 1);
	for (uint8_t i = 0; i < MD5RESULT_SIZE; i++)
	{
		snprintf(MD5fileContent, (MD5RESULT_SIZE * 2) + 1, "%s%02x", MD5fileContent, MD5file[i]);
	}
	DBG_print("\r\n Save MD5 to file system...");
	if (FS_createFile(BKP_MD5_FNAME, 1))
	{
		DBG_print(" <!> Error");
		return 4;
	}
	if (FS_wrt(BKP_MD5_FNAME, 0, strlen(MD5fileContent), MD5fileContent))
	{
		DBG_print(" <!> Error");
		return 5;
	}
	DBG_print(" Done");
	return 0;
}
/*-----------------------------------------------------------------------------
 * Brief: Upgrade firmware from file system
 * Param:	isUpgraded	|	OUT	|	inform whether firmware is upgraded or not
 * 									(running firmware is already upgraded before).
 * Ret:	0	|	OK
 * 		>0	|	FAIL
 ------------------------------------------------------------------------------*/
uint8_t upgradeFromFS(uint8_t *isUpgraded)
{
	uint8_t MD5fileContent[(MD5RESULT_SIZE * 2) + 1]//
			, MD5res1[MD5RESULT_SIZE]//
			, MD5res2[MD5RESULT_SIZE]//
			, MD5res3[MD5RESULT_SIZE]//
			, res;
	uint32_t fsize = 0//
			, rlen = 0;

	DBG_print("\r\n Start upgrade firmware from file system");
	DBG_print("\r\n Find firmware in file system...");
	if (FS_chkFileExist(FIRMWARE_FNAME))
	{
		DBG_print(" <!> Not found");
		return 1;
	}
	DBG_print(" Found");
	DBG_print("\r\n Get firmware file size...");
	fsize = FS_fsize(FIRMWARE_FNAME);
	DBG_print(" %u(Bytes)", fsize);
	DBG_print("\r\n Find MD5 file...");
	if (FS_chkFileExist(MD5_FNAME))
	{
		DBG_print(" <!> Not found");
		return 2;
	}
	DBG_print(" Found");
	DBG_print("\r\n Read MD5 file...");
	rlen = MD5RESULT_SIZE * 2;
	if (FS_rd(MD5_FNAME, 0, &rlen, MD5fileContent))
	{
		DBG_print(" <!> Error");
		return 3;
	}
	MD5fileContent[MD5RESULT_SIZE * 2] = 0;
	DBG_print(" \"%s\"", MD5fileContent);
	if (convTxt2MD5(MD5fileContent, MD5res1))
	{
		DBG_print(" <!> Wrong MD5 format");
		return 31;
	}
	DBG_print("\r\n Generate MD5 from firmware in file system...");
	if (genMD5fromFile(FIRMWARE_FNAME, MD5res2))
	{
		DBG_print("\r\n <!> Error on reading");
		return 4;
	}
	DBG_print(" Done");
	DBG_print("\r\n MD5 result:\"");
	for (uint8_t i = 0; i < MD5RESULT_SIZE; i++)
	{
		DBG_print("%02x", MD5res2[i]);
	}
	DBG_print("\"");
	DBG_print("\r\n Compare MD5 between MD5 file and firmware file:");
	if (memcmp(MD5res1, MD5res2, MD5RESULT_SIZE))
	{
		DBG_print(" <!> Unmatched");
		return 5;
	}
	DBG_print(" Matched");
	DBG_print("\r\n Generate MD5 from firmware in flash...");
	genMD5fromFlashzone(APP_FLASHADDR, fsize, MD5res3);
	DBG_print(" Done");
	DBG_print("\r\n MD5 result:\"");
	for (uint8_t i = 0; i < MD5RESULT_SIZE; i++)
	{
		DBG_print("%02x", MD5res3[i]);
	}
	DBG_print("\"");
	DBG_print("\r\n Compare MD5 between firmware file and firmware in flash:");
	if (memcmp(MD5res2, MD5res3, MD5RESULT_SIZE))
	{
		DBG_print(" <!> Unmatched");
		goto UPGRADEFROMFS_UPGRADE;
	}
	DBG_print(" Matched");
	*isUpgraded = 0;
	goto UPGRADEFROMFS_FINISH_INWIN;
	UPGRADEFROMFS_UPGRADE://
	DBG_print("\r\n Erase flash...");
	if (IFLASH_massErase(APP_FLASHPAGE_ORIGIN, APP_FLASHPAGE_NUM))
	{
		DBG_print(" <!> Error");
		return 6;
	}
	DBG_print(" Done");
	DBG_print("\r\n Upgrade firmware from file system...");
	if (writeFile2flash(FIRMWARE_FNAME, APP_FLASHADDR))
	{
		DBG_print(" <!> Error on reading");
		return 7;
	}
	DBG_print(" Done");
	DBG_print("\r\n Generate MD5 from firmware in flash...");
	genMD5fromFlashzone(APP_FLASHADDR, fsize, MD5res3);
	DBG_print(" Done");
	DBG_print("\r\n MD5 result:\"");
	for (uint8_t i = 0; i < MD5RESULT_SIZE; i++)
	{
		DBG_print("%02x", MD5res3[i]);
	}
	DBG_print("\"");
	DBG_print("\r\n Compare MD5 between firmware file and firmware in flash:");
	if (memcmp(MD5res2, MD5res3, MD5RESULT_SIZE))
	{
		DBG_print(" <!> Unmatched");
		return 9;
	}
	DBG_print(" Matched");
	*isUpgraded = 1;
	UPGRADEFROMFS_FINISH_INWIN://
	{
		uint8_t newFname[64];

		snprintf(newFname, 64, "%lu%s", IRTC_getUTC(), FIRMWARE_FNAME);
		DBG_print("\r\n Rename firmware file...");
		FS_renameFile(FIRMWARE_FNAME, newFname);
		DBG_print(" Done");
		snprintf(newFname, 64, "%lu%s", IRTC_getUTC(), MD5_FNAME);
		DBG_print("\r\n Rename MD5 file...");
		FS_renameFile(MD5_FNAME, newFname);
		DBG_print(" Done");
	}
	return 0;
}
/*-----------------------------------------------------------------------------
 * Brief: Upgrade firmware via FTP protocol
 * Param:	isUpgraded	|	OUT	|	inform whether firmware is upgraded or not (running firmware is already upgraded before).
 * Ret:	0	|	OK
 * 		>0	|	FAIL
 ------------------------------------------------------------------------------*/
U8 upgradeViaFTP(uint16_t mainVer, uint16_t subVer)
{
	const uint16_t RSIZE = FLASH_PAGESIZE;
	DLFTP_HANDLE_RECVDAT_CALLBACK_t hdl;
	uint8_t res, rtry = 0;
	uint32_t remotefsize = 0, localfsize = 0, flashAddr2wrt = APP_FLASHADDR;
	uint8_t rBuf[RSIZE]//
			, FTPfwFname[32]//
			, FTPmd5Fname[32]//
			, MD5res[MD5RESULT_SIZE]//
			, MD5res2[MD5RESULT_SIZE]//
			;
	uint8_t handlerFW(uint32_t size, uint8_t *dat)
	{
		uint8_t res = 0;

		res = IFLASH_wrt(flashAddr2wrt, size, dat);
		localfsize += size;
		//DBG_print("\r\n %s:A=%X Size=%u/%u:Res=%u", __func__, flashAddr2wrt, size, localfsize, res);
		flashAddr2wrt += size;
		DBG_print(".");
#if SAVE_DOWNLOADFW_2_FS
		FS_append("dlfw.bin", size, dat);
#endif
		return res;
	}
	uint8_t handlerMD5(uint32_t size, uint8_t *dat)
	{
		uint8_t res;

#if SAVE_DOWNLOADFW_2_FS
		FS_createFile("dlfw.md5", 1);
		FS_append("dlfw.md5", 0, dat);
#endif
		res = convTxt2MD5(dat, MD5res);
		DBG_print(".");
		return res;
	}

	snprintf(FTPfwFname, 32, "%s%u.%u%s", FTP_FW_MD5_FNAME_PREFIX, mainVer, subVer, FTP_FW_FNAME_SUFFIX);
	snprintf(FTPmd5Fname, 32, "%s%u.%u%s", FTP_FW_MD5_FNAME_PREFIX, mainVer, subVer, FTP_MD5_FNAME_SUFFIX);
	DBG_print("\r\n Request firmware size on FTP");
	for (rtry = 0; rtry < 5; rtry++)
	{
		res = QM95_FTPgetsize(FTP_HOST_DEFAULT//
								, FTP_PORT_DEFAULT//
								, FTP_USER_DEFAULT//
								, FTP_PWD_DEFAULT//
								, FTP_PATH_DEFAULT//
								, FTPfwFname//
								, &remotefsize);
		if (!res)
		{
			DBG_print("\r\n FTP:firmwareSize=%u(Bytes)", remotefsize);
			IO_wrt(LEDLINK_PORT, LEDLINK_PIN, 1);
			goto UPGRADEVIAFTP_DOWNLOADMD5FILE;
		}
		DBG_print("\r\n FTP:res=%u->ERR", res);
	}
	return 1;//
	UPGRADEVIAFTP_DOWNLOADMD5FILE://
	hdl.handler = handlerMD5;
	hdl.rBuf = rBuf;
	hdl.rBufSize = RSIZE;
	DBG_print("\r\n Download MD5 file via FTP");
	for (rtry = 0; rtry < 5; rtry++)
	{
		//FS_createFile(MD5_FNAME, 1);
		DBG_print("\r\n ");
		res = QM95_runFTPdownload(FTP_HOST_DEFAULT//
									, FTP_PORT_DEFAULT//
									, FTP_USER_DEFAULT//
									, FTP_PWD_DEFAULT//
									, FTP_PATH_DEFAULT//
									, FTPmd5Fname//
									, 0, hdl);
		if (!res)
		{
			DBG_print(" Done");
			goto UPGRADEVIAFTP_DOWNLOADFWFILE;//
		}
		DBG_print("\r\n FTP:res=%u->ERR");
	}
	return 2;
	UPGRADEVIAFTP_DOWNLOADFWFILE://
	hdl.handler = handlerFW;
	hdl.rBuf = rBuf;
	hdl.rBufSize = RSIZE;
	DBG_print("\r\n Erase flash...");
	IFLASH_massErase(APP_FLASHPAGE_ORIGIN, APP_FLASHPAGE_NUM);
	DBG_print(" Done");
	DBG_print("\r\n Download firmware via FTP");
#if SAVE_DOWNLOADFW_2_FS
	FS_createFile("dlfw.bin", 1);
#endif
	for (rtry = 0; rtry < 5; rtry++)
	{
		DBG_print("\r\n ");
		res = QM95_runFTPdownload(FTP_HOST_DEFAULT//
									, FTP_PORT_DEFAULT//
									, FTP_USER_DEFAULT//
									, FTP_PWD_DEFAULT//
									, FTP_PATH_DEFAULT//
									, FTPfwFname//
									, localfsize, hdl);
		if (!res)
		{
			DBG_print(" Done");
			DBG_print("\r\n Local:fsize=%u/%u", localfsize, remotefsize);
			if (localfsize >= remotefsize)
			{
				genMD5fromFlashzone(APP_FLASHADDR, localfsize, MD5res2);
				DBG_print("\r\n MD5 comparing...");
				if (!memcmp(MD5res, MD5res2, MD5RESULT_SIZE))
				{
					DBG_print(" Matched");
					goto UPGRADEVIAFTP_ENDINSUCCESS;//
				}
				DBG_print(" Unmatched");
				flashAddr2wrt = APP_FLASHADDR;
				localfsize = 0;
				break;
			}
			rtry = 0;
		}
		DBG_print("\r\n FTP:res=%u->ERR");
	}
	return 3;
	UPGRADEVIAFTP_ENDINSUCCESS://
	return 0;
}
/*-----------------------------------------------------------------------------
 * Brief: Convert text string to MD5.
 * Param:
 * Ret:	0	|	OK
 * 		>0	|	FAIL
 ------------------------------------------------------------------------------*/
uint8_t convTxt2MD5(const uint8_t *text, uint8_t *MD5)
{
	if (strlen(text) < (MD5RESULT_SIZE * 2))
	{
		return 1;
	}
	for (uint8_t i = 0; i < MD5RESULT_SIZE; i++)
	{
		if ((text[i * 2] >= '0') && (text[i * 2] <= '9'))
		{
			MD5[i] = (text[i * 2] - '0') << 4;
		}
		else if ((text[i * 2] >= 'a') && (text[i * 2] <= 'f'))
		{
			MD5[i] = (text[i * 2] - 'a' + 10) << 4;
		}
		else if ((text[i * 2] >= 'A') && (text[i * 2] <= 'F'))
		{
			MD5[i] = (text[i * 2] - 'A' + 10) << 4;
		}
		else
		{
			return 2;
		}
		if ((text[(i * 2) + 1] >= '0') && (text[(i * 2) + 1] <= '9'))
		{
			MD5[i] |= (text[(i * 2) + 1] - '0');
		}
		else if ((text[(i * 2) + 1] >= 'a') && (text[(i * 2) + 1] <= 'f'))
		{
			MD5[i] |= (text[(i * 2) + 1] - 'a' + 10);
		}
		else if ((text[(i * 2) + 1] >= 'A') && (text[(i * 2) + 1] <= 'F'))
		{
			MD5[i] |= (text[(i * 2) + 1] - 'A' + 10);
		}
		else
		{
			return 3;
		}
	}
	return 0;
}
/*-----------------------------------------------------------------------------
 * Brief: Generate MD5 checksum from a file.
 * Param: 	fname	|	IN	|	filename.
 * 			MD5out	|	OUT	|	16 bytes MD5 checksum.
 * Ret:	0	|	OK
 * 		>0	|	FAIL
 ------------------------------------------------------------------------------*/
uint8_t genMD5fromFile(const uint8_t *fname, uint8_t MD5out[MD5RESULT_SIZE])
{
	const uint16_t RBUF_SIZE = 1024 * 8;
	uint8_t rbuf[RBUF_SIZE];
	uint32_t rlen = 0, ofs = 0;
	MD5_CTX ctx;

	MD5_Init(&ctx);
	while (1)
	{
		rlen = RBUF_SIZE;
		if (FS_rd(fname, ofs, &rlen, rbuf))
		{
			return 1;
		}
		//DBG_print("\r\n Offset=%u readLength=%u", ofs, rlen);
		DBG_print(".");
		MD5_Update(&ctx, rbuf, rlen);
		if (rlen < RBUF_SIZE)
		{
			//DBG_print("\r\n Finished");
			break;
		}
		ofs += rlen;
	}
	MD5_Final(MD5out, &ctx);
	return 0;
}
/*-----------------------------------------------------------------------------
 * Brief: Generate MD5 checksum from a flash zone.
 * Param: 	startAddr	|	IN	|	start address.
 * 			len			|	IN	|	length of flash zone.
 * 			MD5out		|	OUT	|	16 bytes MD5 checksum.
 ------------------------------------------------------------------------------*/
void genMD5fromFlashzone(uint32_t startAddr, uint32_t len, uint8_t MD5out[MD5RESULT_SIZE])
{
	const uint16_t RBUF_SIZE = 1024 * 8;
	uint8_t rbuf[RBUF_SIZE];
	uint32_t ofs = 0, rlen;
	MD5_CTX ctx;

	//DBG_print("\r\n Len=%u", len);
	MD5_Init(&ctx);
	while (1)
	{
		rlen = ((len - ofs) > RBUF_SIZE) ? RBUF_SIZE : (len - ofs);
		IFLASH_rd(ofs + startAddr, RBUF_SIZE, rbuf);
		//DBG_print("\r\n Offset=%u ReadLength=%u", ofs, rlen);
		//DBG_print(".");
		MD5_Update(&ctx, rbuf, rlen);
		if (rlen < RBUF_SIZE)
		{
			//DBG_print("\r\n Finished");
			break;
		}
		ofs += rlen;
	}
	MD5_Final(MD5out, &ctx);
}
/*-----------------------------------------------------------------------------
 * Brief: write file content to flash
 * Param: 	fname		|	IN	|	file name.
 * 			startAddr	|	IN	|	start address.
 * Ret:	0	|	OK
 * 		>0	|	FAIL
 ------------------------------------------------------------------------------*/
uint8_t writeFile2flash(const uint8_t *fname, uint32_t startAddr)
{
	const uint16_t RBUF_SIZE = 1024 * 8;
	uint8_t rbuf[RBUF_SIZE];
	uint32_t ofs = 0, rlen = 0;

	while (1)
	{
		rlen = RBUF_SIZE;
		if (FS_rd(fname, ofs, &rlen, rbuf))
		{
			//DBG_print("\r\n <!> Error on reading firmware file!");
			return 1;
		}
		//DBG_print("\r\n Offset=%u readLength=%u", ofs, rlen);
		DBG_print(".");
		if (IFLASH_wrt(ofs + startAddr, rlen, rbuf))
		{
			//DBG_print("\r\n <!> Error on flashing!");
			return 2;
		}
		if (rlen < RBUF_SIZE)
		{
			//DBG_print("\r\n Finished");
			break;
		}
		ofs += rlen;
	}
	return 0;
}
/*-----------------------------------------------------------------------------
 * Brief: Save flash content to a file
 * Param: 	fname		|	IN	|	file name.
 * 			startAddr	|	IN	|	start address.
 * 			size		|	IN	|	flash size to save.
 * Ret:	0	|	OK
 * 		>0	|	FAIL
 ------------------------------------------------------------------------------*/
uint8_t saveFlash2File(const uint8_t *fname, uint32_t startAddr, uint32_t size)
{
	const uint16_t RBUF_SIZE = 1024 * 8;
	uint8_t rbuf[RBUF_SIZE];
	uint32_t ofs = 0, rlen = 0;

	FS_createFile(fname, 1);
	while (1)
	{
		rlen = size - ofs;
		if (!rlen)
		{
			break;
		}
		if (rlen > RBUF_SIZE)
		{
			rlen = RBUF_SIZE;
		}
		DBG_print(".");
		IFLASH_rd(startAddr, rlen, rbuf);
		if (FS_append(fname, rlen, rbuf))
		{
			return 1;
		}
		ofs += rlen;
		startAddr += rlen;
	}
	return 0;
}
