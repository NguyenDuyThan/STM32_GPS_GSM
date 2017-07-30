#include <datastruct.pb.h>
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "time.h"

#include "datastruct.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "pb.h"
#include "pb_common.h"

#define Buzzer		GPIO_Pin_12
#define GSM_LED		GPIO_Pin_5
#define GPS_LED		GPIO_Pin_5
#define PWR_GPS		GPIO_Pin_8
#define PWR_GSM		GPIO_Pin_1
#define GSM_RST_Pin	GPIO_Pin_3
#define GSM_RST_Port	GPIOC

#define bufLogSize	1024
#define bufRxDataUart2Size	512
#define bufRxDataUart1Size	0xFF
#define bufRxDataUart3Size   0xFF

#define chuoiConnect	 "CONNECT {\"verbose\":true,\"pedantic\":false,\"ssl_required\":false,\"auth_token\":\"2I8080bT86aQD45vQkZk1UvVGvrU3qEE\",\"name\":\"1410974732\",\"lang\":\"c\",\"version\":\"2.10_DVS\"}\r\n"
#define debugA	0

#define debugInfo(msg)	debugPrintCB("[InFor] ", __func__, msg, debugWriteLog)
#define debugWarn(msg)	debugPrintCB("[Warn] ", __func__, msg, debugWriteLog)
#define debugError(msg)	debugPrintCB("[ERROR] ", __func__, msg, debugWriteLog)
#define findStrSim(strfind, time)	checkStrCB(bufRxDataUart2, strfind, time, checkStr)
#define cmdCtrl(cmd)	checkCmd(bufRxDataUart1, cmd, 1)
#define debugBufQTM95(buf)	debugPrintCB("\tBuffer Received: ", __func__, buf, debugWriteLog)
#define resetBufQTM95()	resetBuf( bufRxDataUart2, &indexBufRxDataUart2, bufRxDataUart2Size)
/**********************************************************************************/
typedef void (*printcb)(char *, const char *, char *);
typedef int (*checkStrcb)(char *, char *, int);
/***** Debug ****/
int indexBufRxDataUart1 = 0;
char bufRxDataUart1[bufRxDataUart1Size];
char bufLog[bufLogSize];
int indexBufLog = 0;
/***** SIM *****/
int indexBufRxDataUart2 = 0;
char bufRxDataUart2[bufRxDataUart2Size];

char strPub[512];
char strPubSend[512];

struct debugFlag {
	int log;
	int test;
};
struct debugFlag dbQuecTelM95, db;

typedef struct sysFlag_t {
	int runOpen;
	int sendSub;
	int sendPub;
	int dataOther;
	int QIRDI;
	int tcp;
	int nat;
	int ping;
	int newmsg;
} sysFlag;
sysFlag QTM95;

msgDataSend msgDataLocal;
msgDataSend msgDataServer;

enum cmdEnum {
	NotFound = 0, sendPUB, sendSUB, run, resetQTM95, test, QIRDI,
};
enum cmdEnum cmd;
enum statusQTM95Enum {
	NotRespond, Good, Warn,
};
enum statusQTM95Enum statusQTM95;

/***** GPS *****/
int gpggalenght = 0;
int gpgsalenght = 0;
int gprmclenght = 0;

int indexBufRxDataUart3 = 0;
char bufRxDataUart3[bufRxDataUart3Size];
char chuoiphanbiet[5];
char GPGGA[90];
char GPRMC[90];
char GPGSA[90];
char GPVTG[90];
char gpgga[5] = "GPGGA";
char gpgsa[5] = "GPGSA";
char gprmc[5] = "GPRMC";
char gpvtg[5] = "GPVTG";
char chuoitrichra[90];
char *chuoicontro;
/*****************************************************************************/
void GPIO_Configuration(void);
void USARTx_Configuration(USART_TypeDef* USARTx);
void NVIC_Configuration(void);

static void TaskA(void *pvParameters);
static void TaskB(void *pvParameters);
static void TaskC(void *pvParameters);
static void TaskD(void *pvParameters);

extern void splitDataGPS();
extern void getDataUart1();
extern void getDataUart2();

void USARTx_SendString(USART_TypeDef* USARTx, char *Str);
void debugPrintCB(char *label, const char *title, char *Str, printcb endcb);

int checkStrCB(char *source, char *Strs, int time, checkStrcb endcheckStrcb);
void GSM_Cmd(char *AT);

void debugWriteLog(char *label, const char *title, char *Str);
void debugPrint(char *strs);

int splitStr(char *source, char *start, char *stop, char *dis, int stat);
int checkStr(char *source, char *strSign, int timeOut);
int resetBuf(char *strBuf, int *indexBuf, int lenBuf);
int clcBuf(char *buf, char *sign, int *indexBuf);
int checkCmd(char *source, char *strSign, int timeOut);

void specFc();
int checkReqQTM95();
void resetQTM95fc();
int turnOffEchoM95();
int turnOnEchoM95();
int checkSIM();
int checkGSM();
int checkGPRS();

int configFGCNT();
int configQICSGP();
int configQINDI();
int configQIMUX();
int configQIMODE();
int configQIDNSIP();
int configTCP();
int configQIRD(char *sign);
int sendDOS(char *data);
int sendPONG();
int sendPub(char *strP);
int sendSub();
int openTCP();
int connectNat();
int closeTCP();
int checkTCP();

int encodeDat(char *dis);
int decodeDat(char *indat);
int getMSG(char *msg, char *dis);
int getDataServer();

void Delayus(__IO uint32_t nCount);

/************************ Config **************************/

void GPIO_Configuration(void) {
	GPIO_DeInit(GPIOA);
	GPIO_DeInit(GPIOB);
	GPIO_DeInit(GPIOC);

	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	/* Configure output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPS_LED | Buzzer;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = PWR_GSM | GSM_RST_Pin;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = PWR_GPS;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/************************** USART config **************************************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
	/********************************* PORTA *************************************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	/*________________________________ OUTPUT ___________________________________*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;    // TX - USART1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/*________________________________ OUTPUT ___________________________________*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;    // TX - USART2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/*________________________________ OUTPUT ___________________________________*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;    // TX - USART3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*________________________________ INPUT ____________________________________*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;    // RX - USART1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/*________________________________ INPUT ____________________________________*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;    // RX - USART2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/*________________________________ INPUT ____________________________________*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;    // RX - USART3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/*___________________________________________________________________________*/

}

void USARTx_Configuration(USART_TypeDef* USARTx) {
	USART_InitTypeDef USART_InitStructure;

	if (USARTx == USART1)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	else if (USARTx == USART2)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	else if (USARTx == USART3)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	USART_DeInit(USARTx);
	USART_InitStructure.USART_BaudRate = 115200;    // Cau hinh BaudRate
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; // Cau hinh so Bit du lieu trong 1 khung truyen/nhan
																// USART_WordLength_8b
																// USART_WordLength_9b
	USART_InitStructure.USART_StopBits = USART_StopBits_1; // Cau hinh so Bit STOP trong khung truyen
														   // USART_StopBits_1
														   // USART_StopBits_0_5
														   // USART_StopBits_2
														   // USART_StopBits_1_5
	USART_InitStructure.USART_Parity = USART_Parity_No; // Cau hinh su dung che do Parity
														// USART_Parity_No
														// USART_Parity_Even
														// USART_Parity_Odd
	USART_InitStructure.USART_HardwareFlowControl =
	USART_HardwareFlowControl_None; // Cau hinh che do dieu khien theo luong
									// USART_HardwareFlowControl_None
									// USART_HardwareFlowControl_RTS
									// USART_HardwareFlowControl_CTS
									// USART_HardwareFlowControl_RTS_CTS
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // Cau hinh che do truyen nhan
																	// USART_Mode_Rx
																	// USART_Mode_Tx
	USART_Init(USARTx, &USART_InitStructure);    // Cau hinh USART1

	USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE); // Xay ra ngat khi thanh ghi du lieu nhan cua USART1 day
	//USART_ITConfig(USARTx, USART_IT_TXE, ENABLE);    // Xay ra ngat khi thanh ghi du lieu truyen cua USART1 trong

	USART_Cmd(USARTx, ENABLE);                  // Kich hoat USART1
	USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
}

void NVIC_Configuration(void) {
	NVIC_InitTypeDef NVIC_InitStructure;
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
//	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn ;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void USARTx_Sendchar(USART_TypeDef* USARTx, char Data) {
	while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
		;
	USART_SendData(USARTx, Data);
}

char USARTx_Getchar(USART_TypeDef* USARTx) {
	char Data;
	while (USART_GetFlagStatus(USARTx, USART_FLAG_RXNE) == RESET)
		;
	Data = (char) USART_ReceiveData(USARTx);
	return Data;
}

void USARTx_SendString(USART_TypeDef* USARTx, char *Str) {
	while (*Str) {
		USARTx_Sendchar(USARTx, *Str);
		Str++;
	}
}

/**********************************************************/

void splitDataGPS() {
	unsigned char data;
	data = USART_ReceiveData(USART3);
	if (data == '$') {
		indexBufRxDataUart3 = 1;
	} else if (data == '\n') {
		//bufRxDataUart3[indexBufRxDataUart3] = 0;
		bufRxDataUart3[indexBufRxDataUart3] = '\0';
		indexBufRxDataUart3 = 0;
		strncpy(chuoiphanbiet, bufRxDataUart3, 5);
		if (strcmp(chuoiphanbiet, gpgga) == 0) {
			strcpy(GPGGA, bufRxDataUart3);
			gpggalenght = indexBufRxDataUart3;
		} else if (strcmp(chuoiphanbiet, gpgsa) == 0) {
			strcpy(GPGSA, bufRxDataUart3);
			gpgsalenght = indexBufRxDataUart3;
		} else if (strcmp(chuoiphanbiet, gprmc) == 0) {
			strcpy(GPRMC, bufRxDataUart3);
			gprmclenght = indexBufRxDataUart3;
		}
	}
	if (indexBufRxDataUart3) {
		bufRxDataUart3[indexBufRxDataUart3 - 1] = data;
		indexBufRxDataUart3++;
	}
}

void getDataUart1() {
	unsigned char data;
	data = USART_ReceiveData(USART1);
	bufRxDataUart1[indexBufRxDataUart1] = data;
	indexBufRxDataUart1++;
}

void getDataUart2() {
	unsigned char data;
	data = (USART_ReceiveData(USART2) & 0x7F);
	if (indexBufRxDataUart2 == bufRxDataUart2Size)
		indexBufRxDataUart2 = 0;
	bufRxDataUart2[indexBufRxDataUart2] = data;
	indexBufRxDataUart2++;
}

void debugPrintCB(char *label, const char *title, char *Str, printcb endcb) {
	if (db.log)
		endcb(label, title, Str);
}

int checkStrCB(char *source, char *Strs, int time, checkStrcb endcheckStrcb) {
	return endcheckStrcb(source, Strs, time);
}

void debugWriteLog(char *label, const char *title, char *Str) {
	if ((Str != NULL) && (label != NULL) && (title != NULL)) {
		char chuoi[strlen(Str) + 20];
//		char *chuoi = pvPortMalloc(strlen(Str) + 20);
		memset(chuoi, 0, strlen(Str) + 20);
		strcpy(chuoi, label);
		strcat(chuoi, title);
		strcat(chuoi, ": ");
		strcat(chuoi, Str);
		strcat(chuoi, "\n\r");
		USARTx_SendString(USART1, chuoi);
//		vPortFree(chuoi);
//		if (strlen(bufLog) >= (bufLogSize - 200)) {
//			strcpy(&bufLog[0], &bufLog[0] + strlen(chuoi));
//			strcat(bufLog, chuoi);
//		} else {
//			strcat(bufLog, chuoi);
//		}
	}
}

void debugPrint(char *strs) {
	USARTx_SendString(USART1, strs);
	USARTx_SendString(USART1, "\r\n");
}

/*---------------------------- GSM_Cmd -------------------------------------------
 * Brief: send AT command to module SIM over UART2
 * Param:	AT			|	IN	|	input AT command to send (Ex: "AT+CPIN?").
 * Ret:		Not.
 -------------------------------------------------------------------------------*/

void GSM_Cmd(char *AT) {
	char *cmd = pvPortMalloc(strlen(AT) + 5);
	memset(cmd, 0, strlen(AT) + 5);
	strcpy(cmd, AT);
	strcat(cmd, "\r\n");
	USARTx_SendString(USART2, cmd);
	vPortFree(cmd);
}

/*---------------------------- checkStr -------------------------------------------
 * Brief: check a string in other string
 * Param:	source		|	IN	|	input source string.
 * 			strSign		|	IN	|	string will be search in source.
 * 			timeOut		|	IN	|	times (1 is 10 ms) to waiting for data in source to search.
 * Ret:		response if success 1, otherwise 0.
 -------------------------------------------------------------------------------*/

int checkStr(char *source, char *strSign, int timeOut) {
	int timePass = 0;
	if (source != NULL) {
		for (timePass = 0; timePass < timeOut; timePass++) {
			if (strstr(source, strSign) != NULL) {
				return 1;
			} else {
				vTaskDelay(10);
			}
		}
	}
	return 0;
}

/*----------------------------- splitStr ----------------------------------------
 * Brief: split string with string start and string stop
 * Param:	source			|	IN	|	input string will be split.
 * 			start			|	IN	|	word or string will be start with.
 * 			stop			|	IN	|	word or string will be stop with.
 * 			dis				|	OUT	|	store the string between start and stop after split.
 * 			stat			|	IN	|	status 1 if want to keep [start] and [stop], 0 for not.
 * Ret:		response if success 1, otherwise 0.
 * Example: source: "hello world"
 * 			start: "el"
 * 			stop:  "r"
 * 			->>> if stat is 1 dis: "ello wor"
 * 				if stat is 0 dis: "lo wo"
 -------------------------------------------------------------------------------*/

int splitStr(char *source, char *start, char *stop, char *dis, int stat) {
	int ret = 0;
	if ((source != NULL) && (start != NULL) && (stop != NULL)) {
		char temp[250];
		memset(temp, 0, 255);
		strcpy(temp, source);
		if (stat) {
			if (strstr(temp, start) != NULL) {
				strcpy(&temp[0], strstr(temp, start));
				if (strstr(temp, stop) != NULL) {
					char temp1[200];
					strcpy(&temp1[0], strstr(temp, stop) + strlen(stop));
					strncpy(dis, temp, strlen(temp) - strlen(temp1));
					ret = 1;
				} else
					ret = 0;
			} else
				ret = 0;
		} else {
			if (strstr(temp, start) != NULL) {
				strcpy(&temp[0], strstr(temp, start) + strlen(start));
				if (strstr(temp, stop) != NULL) {
					char temp1[200];
					strcpy(&temp1[0], strstr(temp, stop));
					strncpy(dis, temp, strlen(temp) - strlen(temp1));
					ret = 1;
				} else
					ret = 0;
			} else
				ret = 0;
		}
	}
	return ret;
}

/*---------------------------- clcBuf -------------------------------------------
 * Brief: clear buf before sign
 * Param:	buf			|	IN	|	input buffer will be clear.
 * 			sign		|	IN	|	word or string will be clear.
 * 			indexBuf	|	IN	|	only accept response with this header.
 * Ret:		response if success 1, otherwise 0.
 -------------------------------------------------------------------------------*/

int clcBuf(char *buf, char *sign, int *indexBuf) {
	char ret = 0;
	if ((buf != NULL) && (sign != NULL)) {
		if (strstr(buf, sign) != NULL) {
			strcpy(&buf[0], strstr(buf, sign) + strlen(sign));
			*indexBuf = strlen(buf);
			ret = 1;
		} else
			ret = 0;
	}
	return ret;
}

int checkCmd(char *source, char *strSign, int timeOut) {
	if (strstr(source, strSign)) {
		vTaskDelay(10);
		if (strstr(source, "buart2")) {
			debugInfo("buf current: ");
			debugInfo(bufRxDataUart2);
			return 1;
		}
		if (strstr(source, "test"))
			return test;
		if (strstr(source, "resetQTM95"))
			return resetQTM95;
		if (strstr(source, "run"))
			return run;
		if (strstr(source, "sendPUB")) {
			if (strstr(source, "msg") != NULL)
				strcpy(&strPub[0], strstr(source, "msg") + strlen("msg"));
			return sendPUB;
		}
		if (strstr(source, "sendSUB")) {
			return sendSUB;
		} else
			return NotFound;
	} else {
		return 100;
	}
}

int resetBuf(char *strBuf, int *indexBuf, int lenBuf) {
	memset(strBuf, 0, lenBuf);
	*indexBuf = 0;
	if (strlen(strBuf) >= 1)
		return 0;
	else
		return 1;
}

/*********************************************************************************/

int main(void) {
	dbQuecTelM95.log = 0;
	db.log = 1;

	GPIO_Configuration();
	USARTx_Configuration(USART1);
	USARTx_Configuration(USART2);
	USARTx_Configuration(USART3);
	NVIC_Configuration();

	time_t t;
	srand((unsigned) time(&t));

	debugInfo("\n-------- *** Main *** ---------");
	debugInfo("------- Test iTracking --------");
	debugInfo(">>>>>>> Setup HardWare <<<<<<<");
	GPIO_SetBits(GPIOA, PWR_GPS);
	GPIO_SetBits(GPIOC, PWR_GSM);

	debugInfo(">>>>>>> OK.");
	debugInfo("Waiting for data to show...");

	xTaskCreate(TaskB, (signed char* )"TaskB", 512, NULL, 4, NULL);
	xTaskCreate(TaskA, (signed char* )"TaskA", 128, NULL, 3, NULL);
	xTaskCreate(TaskC, (signed char* )"TaskC", 128, NULL, 2, NULL);
	xTaskCreate(TaskD, (signed char* )"TaskD", 128, NULL, 1, NULL);
	vTaskStartScheduler();
}

static void TaskA(void *pvParameters) {

	for (;;) {
#if debugA
		int index = 0;
		//int i;
		//char *a = (char *)malloc(100*sizeof(char));
		//char **b = (char **)malloc(100*sizeof(char));;
		char temp[100];
		char title[6];//1
		char time_UTC[11];//2
		char latitude[10];//3
		char NS[2];//4
		char longtitude[10];//5
		char EW[2];//6
		char qualitiGPS[2];//7
		char number_satellites[5];//8
		char horizontal_dilution[10];//9
		char antenna_sea[4];//10
		char unit_antenna[2];//11
		char Geoidal_separation[4];//12
		char unit_Geoidal[2];//13
		char Differential_GPS[4];//14
		char station_ID[5];//15
		char checksum[3];//16
		strcpy(temp, GPRMC);
		USARTx_SendString(USART1, (char*) GPRMC);
		//char chuoisao[]="*";
		int st = strlen(GPRMC) - 4;
		//int dauphay = ',';
		//strncpy(chuoitrichra, GPGGA, n);
		//chuoitam  = strchr(GPGGA, dauphay);
		//int st = strlen(chuoicontro);
		//int m= (st-2);
		//char *test;
		//test=strstr(temp,chuoisao);
		//USARTx_SendString(USART1,(char*)test);
		//USARTx_SendString(USART1,(char*)"\n");
		//int check=strlen(temp);
		char *p;
//    	if(st<10)
//    	{
//    		USARTx_SendString(USART1,(char*)"TaskA: Waiting for data getting...\n");
//        	USARTx_SendString(USART1,(char*)GPGGA);
//        	USARTx_SendString(USART1,(char*)GPGSA);
//        	USARTx_SendString(USART1,(char*)GPRMC);
//    	}
//    	else if (st>=10)
		{
			USARTx_SendString(USART1, (char*) "^^^OK received data^^^\n");
			p = strtok(temp, ",*"); //cat chuoi bang cac ky tu
			////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
			while (index < 16)//for(index ; index<16 ; index++ )//while(p != NULL)
			{
				switch (index) {
					case 0:
					//USARTx_SendString(USART1,(char*)"1\n");
					strcpy(title, p);
					index++;
					break;//optional
					case 1:
					//USARTx_SendString(USART1,(char*)"2\n");
					strcpy(time_UTC, p);
					index++;
					break;//optional
					case 2:
					//USARTx_SendString(USART1,(char*)"3\n");
					strcpy(latitude, p);
					index++;
					break;//optional
					case 3:
					//USARTx_SendString(USART1,(char*)"4\n");
					strcpy(NS, p);
					index++;
					break;//optional
					case 4:
					//USARTx_SendString(USART1,(char*)"5\n");
					strcpy(longtitude, p);
					index++;
					break;
					case 5:
					//USARTx_SendString(USART1,(char*)"6\n");
					strcpy(EW, p);
					index++;
					break;
					case 6:
					//USARTx_SendString(USART1,(char*)"7\n");
					strcpy(qualitiGPS, p);
					index++;
					break;
					case 7:
					//USARTx_SendString(USART1,(char*)"8\n");
					strcpy(number_satellites, p);
					index++;
					break;
					case 8:
					//USARTx_SendString(USART1,(char*)"9\n");
					strcpy(horizontal_dilution, p);
					index++;
					break;
					case 9:
					//USARTx_SendString(USART1,(char*)"10\n");
					strcpy(antenna_sea, p);
					index++;
					break;
					case 10:
					//USARTx_SendString(USART1,(char*)"11\n");
					strcpy(unit_antenna, p);
					index++;
					break;
					case 11:
					//USARTx_SendString(USART1,(char*)"12\n");
					strcpy(Geoidal_separation, p);
					index++;
					break;
					case 12:
					//USARTx_SendString(USART1,(char*)"13\n");
					strcpy(unit_Geoidal, p);
					index++;
					break;
					case 13:
					//USARTx_SendString(USART1,(char*)"14\n");
					strcpy(Differential_GPS, p);
					index++;
					break;
					case 14:
					//USARTx_SendString(USART1,(char*)"15\n");
					strcpy(station_ID, p);
					index++;
					break;
					//case 15 :
					//USARTx_SendString(USART1,(char*)"16\n");
					//    strcpy(checksum,p);
					//	index++;
					//    break;
					default://Optional
					index++;
					break;
				}
				//strcpy(b[index],p);
				//index++;
				p = strtok(NULL, ",*");//cat chuoi tu vi tri dung lai truoc do
			}

			USARTx_SendString(USART1, (char*) "Data getting from: ");
			USARTx_SendString(USART1, (char*) title);
			USARTx_SendString(USART1, (char*) "\n");
			USARTx_SendString(USART1, (char*) "Thoi gian UTC: ");
			USARTx_SendString(USART1, (char*) time_UTC);
			USARTx_SendString(USART1, (char*) "\n");
			USARTx_SendString(USART1, (char*) "Vi do: ");
			USARTx_SendString(USART1, (char*) latitude);
			USARTx_SendString(USART1, (char*) "\n");
			USARTx_SendString(USART1, (char*) "Huong vi do: ");
			USARTx_SendString(USART1, (char*) NS);
			USARTx_SendString(USART1, (char*) "\n");
			USARTx_SendString(USART1, (char*) "Kinh do: ");
			USARTx_SendString(USART1, (char*) longtitude);
			USARTx_SendString(USART1, (char*) "\n");
			USARTx_SendString(USART1, (char*) "Huong kinh do: ");
			USARTx_SendString(USART1, (char*) EW);
			USARTx_SendString(USART1, (char*) "\n");
			USARTx_SendString(USART1, (char*) "Chat luong GPS: ");
			USARTx_SendString(USART1, (char*) qualitiGPS);
			USARTx_SendString(USART1, (char*) "\n");
			USARTx_SendString(USART1, (char*) "So ve tinh nhin thay: ");
			USARTx_SendString(USART1, (char*) number_satellites);
			USARTx_SendString(USART1, (char*) "\n");
			USARTx_SendString(USART1, (char*) "Duong chan troi: ");
			USARTx_SendString(USART1, (char*) horizontal_dilution);
			USARTx_SendString(USART1, (char*) "\n");
			USARTx_SendString(USART1,
					(char*) "Do cao anten so voi nuoc bien: ");
			USARTx_SendString(USART1, (char*) antenna_sea);
			USARTx_SendString(USART1, (char*) " ");
			USARTx_SendString(USART1, (char*) unit_antenna);
			USARTx_SendString(USART1, (char*) "\n");
			USARTx_SendString(USART1, (char*) "Do tach roi hinh hoc: ");
			USARTx_SendString(USART1, (char*) Geoidal_separation);
			USARTx_SendString(USART1, (char*) " ");
			USARTx_SendString(USART1, (char*) unit_Geoidal);
			USARTx_SendString(USART1, (char*) "\n");
			USARTx_SendString(USART1, (char*) "Sai lech du lieu GPS: ");
			USARTx_SendString(USART1, (char*) Differential_GPS);
			USARTx_SendString(USART1, (char*) "\n");
			USARTx_SendString(USART1, (char*) "ID kenh tham chieu vi sai: ");
			USARTx_SendString(USART1, (char*) station_ID);
			USARTx_SendString(USART1, (char*) "\n");
			USARTx_SendString(USART1, (char*) "Ma kiem tra: ");
			USARTx_SendString(USART1, (char*) checksum);
			USARTx_SendString(USART1, (char*) "\n");

			/*************************** part data temp ok 1 ****************************/
			USARTx_SendString(USART1,
					(char*) "******Luu GPGGA co gia tri la:\n");
			USARTx_SendString(USART1, (char*) GPGGA);
			USARTx_SendString(USART1, (char*) "\n");
			//USARTx_SendString(USART1,(char*)"*****Luu GPGSA co gia tri la:\n");
			//USARTx_SendString(USART1,(char*)GPGSA);
			//USARTx_SendString(USART1,(char*)"\n");
			//USARTx_SendString(USART1,(char*)"*****Luu GPRMC co gia tri la:\n");
			//USARTx_SendString(USART1,(char*)GPRMC);
			//USARTx_SendString(USART1,(char*)"\n");
		}
#endif // debugA
		vTaskDelay(1000);
	}
}

static void TaskB(void *pvParameters) {
	uint8_t timeOpenTCP = 0;
	startSIM: QTM95.runOpen = 20;
	QTM95.tcp = 0;
	QTM95.nat = 0;
	resetBufQTM95();
	if (checkSIM() != Good) {
		debugError("SIM Fail");
	} else if (checkGSM() != Good) {
		debugError("GSM Fail");
	} else
		debugInfo("signal GSM Good");
	resetBufQTM95();
	while (1) {
		specFc();
		if (QTM95.QIRDI) {
			if (!getDataServer()) {
				debugError("Cannot get data");
			}
			QTM95.QIRDI = 0;
		}
		if (QTM95.ping >= 260) {
			QTM95.nat = 0;
			QTM95.ping = 0;
			QTM95.runOpen = 20;
			debugWarn("Waiting for PING overtime, try to connect again...");
		}
		if (!QTM95.nat) {
			QTM95.runOpen++;
		} else {
			QTM95.ping++;
			QTM95.runOpen = 0;
		}
		if (QTM95.runOpen >= 20) {
			QTM95.sendSub = 1;
			timeOpenTCP++;
			if (checkGPRS() != Good) {
				debugError("GPRS not working, try to check again...");
			} else if (!openTCP()) {
				debugError("Open TCP fail");
			} else if (!(connectNat())) {
				debugError(
						"Connect to Nat fail, try to close TCP and connect again...");
				if (closeTCP()) {
					debugWarn("TCP closed");
				} else {
					debugError("TCP cannot close");
				}
			} else {
				debugInfo("TCP ok, Wating for Nat respond...");
				QTM95.runOpen = 0;
			}
			if ((timeOpenTCP >= 3)) {
				debugWarn("cannot connect, try to retset QTM95...");
				resetQTM95fc();
				timeOpenTCP = 0;
				goto startSIM;
			}
		}
		if (QTM95.sendSub) {
			if (QTM95.nat) {
				if (sendSub()) {
					debugInfo("SUB ok...");
					QTM95.sendSub = 0;
				} else {
					debugError("SUB fail, try send again...");
				}
			} else {
				debugWarn(
						"Waiting for connect Nat to server, Please wait Send SUB...");
			}
		}
		if (QTM95.sendPub) {
			if (QTM95.nat) {
				char *buffer = pvPortMalloc(msgDataSend_size);
				memset(buffer, 0, msgDataSend_size);
				memset(&msgDataLocal, 0, sizeof(msgDataSend));
				strncpy(msgDataLocal.text1, strPub, strlen(strPub));
				sprintf(msgDataLocal.text2, "Combo + %s", msgDataLocal.text1);
				msgDataLocal.num1 = rand() % 50;
				msgDataLocal.num2 = rand() % 10;
				if (encodeDat(buffer)) {
					if (sendPub(buffer)) {
						debugInfo("Publish data to server is ok ^.^");
						QTM95.sendPub = 0;
					} else {
						debugError(
								"PUB pb_byte_tfail, try to check connect...");
					}
				} else {
					debugError("Encode Data fail.");
				}
				vPortFree(buffer);
			} else {
				debugWarn(
						"Waiting for connect Nat to server. Please wait Send data to server...");
			}
		}
		if (QTM95.newmsg) {
			debugInfo("New data: ");
			char *bufferrec = pvPortMalloc(msgDataSend_size);
			memset(bufferrec, 0, msgDataSend_size);
			sprintf(bufferrec, "\nText1: %sNum1: %d\r\nText2: %sNum2: %d",
					msgDataServer.text1, msgDataServer.num1,
					msgDataServer.text2, msgDataServer.num2);
			debugPrint(bufferrec);
			vPortFree(bufferrec);
			QTM95.newmsg = 0;
		}
		vTaskDelay(500);
	}
}

static void TaskC(void *pvParameters) {

	for (;;) {
		switch (cmdCtrl("cmd")) {
		case NotFound:
			debugWarn("command not found^^^\n\r Please check again");
			break;
		case run:
			debugInfo("received Command: run");
			QTM95.runOpen = 1;
			break;
		case sendPUB:
			debugInfo("received Command: sendPUB");
			QTM95.sendPub = 1;
			break;
		case sendSUB:
			debugInfo("received Command: sendSUB");
			QTM95.sendSub = 1;
			break;
		case resetQTM95:
			debugInfo("received Command: resetQTM95");
			resetQTM95fc();
			break;
		default:
			break;
		}
		resetBuf(bufRxDataUart1, &indexBufRxDataUart1, bufRxDataUart1Size);
		vTaskDelay(100);
	}
}

static void TaskD(void *pvParameters) {

	for (;;) {
		vTaskDelay(1000);
	}
}

/****************************************************************************/

void specFc() {
	if (checkReqQTM95() == QIRDI) {
		QTM95.QIRDI = 1;
	}
	resetBufQTM95();
}

int checkReqQTM95() {
	if (findStrSim("QIRDI:", 100)) {
		return QIRDI;
	}
	return NotFound;
}

void resetQTM95fc() {
	GPIO_SetBits(GPIOC, GSM_RST_Pin);
	vTaskDelay(200);
	GPIO_ResetBits(GPIOC, GSM_RST_Pin);
	vTaskDelay(2000);
	debugWarn("Reset Module QuecTelM95 OK");
}

int turnOffEchoM95() {
	GSM_Cmd("ATE0");
	vTaskDelay(500);
	if (strstr(bufRxDataUart2, "OK")) {
		return Good;
	} else {
		return NotRespond;
	}
}

int turnOnEchoM95() {
	GSM_Cmd("ATE1");
	vTaskDelay(500);
	if (strstr(bufRxDataUart2, "OK")) {
		return Good;
	} else {
		return NotRespond;
	}
}

int checkSIM() {
	char ret = 0;
	GSM_Cmd("AT+CPIN?");
	if (findStrSim("READY", 500)) {
		ret = Good;
	} else {
		ret = NotRespond;
	}
	return ret;
}

int checkGSM() {
	char ret = 0;
	GSM_Cmd("AT+GCAP");
	if (findStrSim("GSM", 500)) {
		ret = Good;
	} else {
		ret = NotRespond;
	}
	return ret;
}

int checkGPRS() {
	char ret = 0;
	GSM_Cmd("AT+CREG?");
	if (findStrSim("+CREG: 0,1", 1000)) {
		ret = Good;
	} else {
		ret = NotRespond;
	}
	return ret;
}

int configFGCNT() {
	char ret = 0;
	GSM_Cmd("AT+QIFGCNT?");
	if (findStrSim("OK", 1000)) {
		if (findStrSim("+QIFGCNT: 0,", 500)) {
			ret = Good;
		} else {
			GSM_Cmd("AT+QIFGCNT=0");
			ret = Good;
		}
	} else
		ret = NotRespond;
	return ret;
}

int configQICSGP() {
	GSM_Cmd("AT+QICSGP=1,\"m-wap\"");
	vTaskDelay(500);
	if (strstr(bufRxDataUart2, "OK")) {
		return Good;
	} else if (strstr(bufRxDataUart2, "ERROR")) {
		return Warn;
	} else {
		return NotRespond;
	}
}

int configQIMUX() {
	GSM_Cmd("AT+QIMUX?");
	vTaskDelay(500);
	if (strstr(bufRxDataUart2, "OK")) {
		if (strstr(bufRxDataUart2, "+QIMUX: 0")) {
			return Good;
		} else {
			GSM_Cmd("AT+QIMUX=0");
			vTaskDelay(500);
			if (strstr(bufRxDataUart2, "OK")) {
				return Good;
			} else {
				return NotRespond;
			}
		}
	} else {
		return NotRespond;
	}
}

int configQIMODE() {
	GSM_Cmd("AT+QIMODE?");
	vTaskDelay(500);
	if (strstr(bufRxDataUart2, "OK")) {
		if (strstr(bufRxDataUart2, "+QIMOE: 0")) {
			return Good;
		} else {
			GSM_Cmd("AT+QIMODE=0");
			vTaskDelay(500);
			if (strstr(bufRxDataUart2, "OK")) {
				return Good;
			} else {
				return NotRespond;
			}
		}
	} else {
		return NotRespond;
	}
}

int configQINDI() {
	GSM_Cmd("AT+QINDI?");
	vTaskDelay(500);
	if (strstr(bufRxDataUart2, "OK")) {
		if (strstr(bufRxDataUart2, "+QINDI: 1")) {
			return Good;
		} else {
			GSM_Cmd("AT+QINDI=1");
			vTaskDelay(500);
			if (strstr(bufRxDataUart2, "OK")) {
				return Good;
			} else {
				return NotRespond;
			}
		}
	} else {
		return NotRespond;
	}
}

int configQIDNSIP() {
	GSM_Cmd("AT+QIDNSIP?");
	vTaskDelay(500);
	if (strstr(bufRxDataUart2, "OK")) {
		if (strstr(bufRxDataUart2, "+QIDNSIP: 1")) {
			return Good;
		} else {
			GSM_Cmd("AT+QIDNSIP=1");
			vTaskDelay(500);
			if (strstr(bufRxDataUart2, "OK")) {
				return Good;
			} else {
				return NotRespond;
			}
		}
	} else {
		return NotRespond;
	}
}

int configTCP() {
	if (!configFGCNT()) {
		return 0;
	} else if (!configQICSGP()) {
		return 0;
	} else if (!configQINDI()) {
		return 0;
	} else if (!configQIMUX()) {
		return 0;
	} else if (!configQIDNSIP()) {
		return 0;
	} else {
		resetBufQTM95();
		return 1;
	}
}

int closeTCP() {
	char ret = 0;
	GSM_Cmd("AT+QICLOSE");
	if (findStrSim("CLOSE OK", 1000)) {
		ret = 1;
	} else {
		GSM_Cmd("AT+QIDEACT");
		if (findStrSim("DEACT OK", 1000)) {
			ret = 1;
		} else
			ret = 0;
	}
	resetBufQTM95();
	return ret;
}

int sendDOS(char *data) {
	char ret = 0;
	char len = 0;
	len = strlen(data);
	char *cmd = pvPortMalloc(15);
	memset(cmd, 0, 15);
	sprintf(cmd, "AT+QISEND=%d", len);
	GSM_Cmd(cmd);
	if (findStrSim("> ", 1000)) {
		GSM_Cmd(data);
		GSM_Cmd("0x1A");
		if (findStrSim("SEND OK", 1000)) {
			ret = 1;
		} else if (findStrSim("SEND FAIL", 1)) {
			ret = 0;
		}
	} else if (findStrSim("ERROR", 1)) {
		ret = 0;
	}
	vPortFree(cmd);
	return ret;
}

int openTCP() {
	uint8_t ret = 0;
	if (!configTCP()) {
		debugError("configTCP error, try to config again...");
		ret = 0;
	} else {
		GSM_Cmd("AT+QIOPEN= \"TCP\", \"itracking.vn\", 4222");
		if (findStrSim("CONNECT", 1000)) {
			if (findStrSim("FAIL", 1)) {
				ret = 0;
			} else
				ret = 1;
		}
	}
	resetBufQTM95();
	return ret;
}

int connectNat() {
	uint8_t ret = 0;
	if (sendDOS(chuoiConnect)) {
		ret = 1;
	} else
		ret = 0;
	return ret;
}

int configQIRD(char *sign) {
	GSM_Cmd("AT+QIRD=0,1,0,1024");
	if (findStrSim(sign, 500)) {
		return Good;
	} else {
		return NotRespond;
	}
}

int sendPONG() {
	char ret = 0;
	char *cmd = pvPortMalloc(10);
	memset(cmd, 0, 10);
	sprintf(cmd, "PONG\r\n");
	if (sendDOS(cmd)) {
		ret = 1;
	} else
		ret = 0;
	vPortFree(cmd);
	return ret;
}

int sendPub(char *strP) {
	char ret = 0;
	int i = strlen(strP);
	char *strPsend = pvPortMalloc(i + 33);
	memset(strPsend, 0, i + 33);
	sprintf(strPsend, "PUB R.0000000000.CMD %d\r\n%s\r\n", strlen(strP), strP);
	if (sendDOS(strPsend)) {
		ret = 1;
	} else
		ret = 0;
	return ret;
}

int sendSub() {
	char ret = 0;
	if (sendDOS("SUB R.0000000000.CMD 1\r\n")) {
		ret = 1;
	} else
		ret = 0;
	return ret;
}

int encodeDat(char *dis) {
	char ret = 0;
	char buffer[msgDataSend_size];
	memset(buffer, 0, msgDataSend_size);
	pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
	if (pb_encode(&stream, msgDataSend_fields, &msgDataLocal)) {
		strncpy(dis, buffer, strlen(buffer));
		ret = 1;
	} else {
		ret = 0;
	}
	return ret;
}

int decodeDat(char *indat) {
	memset(&msgDataServer, 0, sizeof(msgDataSend));
	char ret = 0;
	char buffer[msgDataSend_size];
	memset(buffer, 0, msgDataSend_size);
	strncpy(buffer, indat, strlen(indat));
	pb_istream_t stream = pb_istream_from_buffer(buffer, sizeof(buffer));
	if (pb_decode(&stream, msgDataSend_fields, &msgDataServer)) {
		ret = 1;
	} else
		ret = 0;
	return ret;
}

int getMSG(char *msg, char *dis) {
	char ret = 0;
	char temp[256];
	memset(temp, 0, 256);
	char num[5];
	memset(num, 0, 5);
	strcpy(temp, msg);
	if (strstr(temp, "MSG") != NULL) {
		strcpy(&temp[0], strstr(temp, "MSG") + 23);
		if (strstr(temp, "\n") != NULL) {
			char temp1[200];
			strcpy(&temp1[0], strstr(temp, "\n") + strlen("\n"));
			strncpy(num, temp, strlen(temp) - strlen(temp1));
			if (num != NULL) {
				int lenmsg = atoi(num);
				strncpy(dis, temp1, lenmsg);
				ret = 1;
			}
		} else
			ret = 0;
	} else
		ret = 0;
	return ret;
}

int getDataServer() {
	uint8_t ret = 0;
	if (configQIRD("\nOK")) {
		QTM95.dataOther = 1;
		if (findStrSim("+OK", 1)) {
			debugInfo("Connected to NAT");
			QTM95.nat = 1;
		}
		if (findStrSim("ERR", 1)) {
			debugWarn("Connect NATs time out");
			debugWarn("Try to connect again");
			debugBufQTM95(bufRxDataUart2);
			QTM95.nat = 0;
		}
		if (strstr(bufRxDataUart2, "PING") != NULL) {
			debugInfo("Detected PING>>>>>");
			if (sendPONG()) {
				debugInfo("Da PONG lai ok...");
				QTM95.ping = 0;
				QTM95.nat = 1;
			} else {
				debugWarn("PONG lai FAIL, try to send PONG again");
				sendPONG();
				debugBufQTM95(bufRxDataUart2);
			}
			QTM95.dataOther = 0;
		}
		if (strstr(bufRxDataUart2, "MSG") != NULL) {
			char *MSG = pvPortMalloc(msgDataSend_size);
			memset(MSG, 0, msgDataSend_size);
			if (splitStr(bufRxDataUart2, "MSG", "\r\nOK", MSG, 1)) {
				char *strMSG = pvPortMalloc(msgDataSend_size);
				memset(strMSG, 0, msgDataSend_size);
				getMSG(MSG, strMSG);
				if (decodeDat(strMSG)) {
					QTM95.newmsg = 1;
				} else {
					debugError("Decode Data fail.");
					debugError(MSG);
					debugInfo("MSG:");
					debugInfo(strMSG);
				}
				vPortFree(strMSG);
			}
			vPortFree(MSG);
			QTM95.dataOther = 0;
		}
		if (QTM95.dataOther == 1) {
			QTM95.dataOther = 0;
		}
		ret = 1;
	} else {
		ret = 0;
	}
	return ret;
}

int checkTCP() {
	uint8_t ret = 0;
	GSM_Cmd("AT+QISTAT");
	if (findStrSim("CONNECT OK", 100)) {
		ret = 1;
	}
	return ret;
}

void Delayus(__IO uint32_t nCount) {
	while (nCount--) {
	}
}

/*
 void vApplicationIdleHook( void )
 {

 }
 */
#ifdef  USE_FULL_ASSERT
void assert_failed(char* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif
