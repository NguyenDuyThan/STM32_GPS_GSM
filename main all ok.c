#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"

#define Buzzer		GPIO_Pin_12
#define GSM_LED		GPIO_Pin_5
#define GPS_LED		GPIO_Pin_5
#define PWR_GPS		GPIO_Pin_8
#define PWR_GSM		GPIO_Pin_1
#define GSM_RST_Pin	GPIO_Pin_3
#define GSM_RST_Port	GPIOC
#define null	NULL
#define bufLogSize	4096
#define bufRxDataUart2Size	4096
#define bufRxDataUart1Size	0xFF
#define bufRxDataUart3Size   0xFF

#define chuoiConnect	 "CONNECT {\"verbose\":true,\"pedantic\":false,\"ssl_required\":false,\"auth_token\":\"2I8080bT86aQD45vQkZk1UvVGvrU3qEE\",\"name\":\"1410974732\",\"lang\":\"c\",\"version\":\"2.10_DVS\"}"
#define simnotres	"Sim Not Respond"
#define debugA	0

#define debugInfo(msg)	debugPrintCB("[InFor] ", __func__, msg, debugWriteLog)
#define debugWarn(msg)	debugPrintCB("[Warn] ", __func__, msg, debugWriteLog)
#define debugError(msg)	debugPrintCB("[ERROR] ", __func__, msg, debugWriteLog)
#define findStrSim(strfind, time)	bufCB(bufRxDataUart2, strfind, time, echo, checkStr)
#define cmdCtrl(cmd)	checkCmd(bufRxDataUart1, cmd, 1)
#define debugBufQTM95(buf)	debugPrintCB("\tBuffer Received: ", __func__, buf, debugWriteLog)
#define resetBufQTM95()	resetBuf( bufRxDataUart2, &indexBufRxDataUart2, bufRxDataUart2Size)
/**********************************************************************************/
typedef void (*printcb)(char *, const char *, char *);
typedef int (*bufcb)(char *, char *, int, char *);
/***** Debug ****/
int indexBufRxDataUart1 = 0;
char bufRxDataUart1[bufRxDataUart1Size];
char bufLog[bufLogSize];
int indexBufLog = 0;
/***** SIM *****/
int indexBufRxDataUart2 = 0;
char bufRxDataUart2[bufRxDataUart2Size];

char controlZ[2] = { 0x1A, '\0' };
char strtemp[1024];
char CtrlZtemp[50];

int j = 0;
int k = 0;
char echo[bufRxDataUart2Size];
char strPub[1024];
char strPubSend[1024];
int lenStrPub = 0;
char atcmd[100];

struct debugFlag {
	int log;
	int datSer;
};
struct debugFlag dbQuecTelM95, db;

struct sysFlag {
	int datRec;
	int runOpen;
};
struct sysFlag QTM95;
enum cmdEnum {
	NotFound = 0,
	sendConnect,
	sendPUB,
	sendSUB,
	run,
	bufU2,
	rstbufU2,
	viewLog,
	resetQTM95,
	logQTM950,
	logQTM951,
	QIRDI,
	datSerQTM950,
	datSerQTM951,
	datRecQTM950,
	datRecQTM951,
};
enum cmdEnum cmd;
enum statusQTM95Enum {
	NotRespond, Good, Warn,
};
enum statusQTM95Enum statusQTM95;

int Revice_Data;

int gpggalenght = 0;
int gpgsalenght = 0;
int gprmclenght = 0;

int debugcheckSIM = 0;
int debugcheckGSM = 0;
int debugcheckGPRS = 0;
int debugconfigFGCNT = 0;
int debugconfigQICSGP = 0;
int debugconfigQINDI = 0;
int debugconfigQIMUX = 0;
int debugconfigQIMODE = 0;
int debugconfigQIDNSIP = 0;
int debugconfigQIREGAPP = 0;
int debugconfigQIACT = 0;
int debugconfigTCP = 0;
int debugopenTCP = 0;
int debugsendCONNECT = 0;
int debugsendStrCONNECT = 0;
int debugopenSocket = 0;
int debugconfigQIRD = 0;
/***** GPS *****/
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
//int bufCB(char *source, char *Strs, int time, bufcb endBufcb);
int bufCB(char *source, char *Strs, int time, char *dis, bufcb endBufcb);
void GSM_Cmd(char *AT);
void debugWriteLog(char *label, const char *title, char *Str);
void debugPrint(char *strs);
//int checkStr(char *source, char *strSign, int timeOut);
int checkStr(char *source, char *strSign, int timeOut, char *dis);
int resetBuf(char *strBuf, int *indexBuf, int lenBuf);
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
int configQIRD();
int sendPONG();
int sendPub();
int sendSub();
int openTCP();
int sendCONNECT();
int openSocket();

void Delayus(__IO uint32_t nCount);

/********************* Config *************************/

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

void USARTx_SendChar(USART_TypeDef* USARTx, uint8_t Data) {
	while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
		;
	USART_SendData(USARTx, Data);
}

uint8_t USARTx_GetChar(USART_TypeDef* USARTx) {
	uint8_t Data;
	while (USART_GetFlagStatus(USARTx, USART_FLAG_RXNE) == RESET)
		;
	Data = (uint8_t) USART_ReceiveData(USARTx);
	return Data;
}

void USARTx_SendString(USART_TypeDef* USARTx, char *Str) {
	while (*Str) {
		USARTx_SendChar(USARTx, *Str);
		Str++;
	}
}

/**********************************************************/

void splitDataGPS() {
	unsigned char data;
	data = USART_ReceiveData(USART3);
	if (data == '$') {
		indexBufRxDataUart3 = 1;
		Revice_Data = 0;
	} else if (data == '\n') {
		//bufRxDataUart3[indexBufRxDataUart3] = 0;
		bufRxDataUart3[indexBufRxDataUart3] = '\0';
		indexBufRxDataUart3 = 0;
		Revice_Data = 1;    	//Da co chuoi du lieu duoc nhan
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
		Revice_Data = 0;
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
//	if (!(checkReqQTM95() == NotFound))
//		specFc();
}

void debugPrintCB(char *label, const char *title, char *Str, printcb endcb) {
	if (db.log)
		endcb(label, title, Str);
}

int bufCB(char *source, char *Strs, int time, char *dis, bufcb endBufcb) {
	return endBufcb(source, Strs, time, dis);
}

void debugWriteLog(char *label, const char *title, char *Str) {
	if ((Str != NULL) && (label != NULL) && (title != NULL)) {
		char chuoi[1024];
		memset(chuoi, 0, 1024);
		strcpy(chuoi, label);
		strcat(chuoi, title);
		strcat(chuoi, ": ");
		strcat(chuoi, Str);
		strcat(chuoi, "\n\r");
		USARTx_SendString(USART1, chuoi);
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
}

void GSM_Cmd(char *AT) {
	char cmd[250];
	strcpy(cmd, AT);
	strcat(cmd, "\r\n");
	USARTx_SendString(USART2, cmd);
}

/*
 int checkStr(char *source, char *strSign, int timeOut) {
 char temp[255];
 memset(temp, 0, 255);

 int timePass = 0;
 for (timePass = 0; timePass < timeOut; timePass++) {
 if (strstr(source, strSign)) {
 return 1;
 } else {
 vTaskDelay(10);
 }
 }
 return 0;
 }
 */

int checkStr(char *source, char *strSign, int timeOut, char *dis) {
	int timePass = 0;
//	char temp[sizeof(source)];
	for (timePass = 0; timePass < timeOut; timePass++) {
		if (strstr(source, strSign) != NULL) {
//			memset(temp, 0, sizeof(temp));
//			strcpy(temp, source);
//			int lt = strlen(temp);
//			strcpy(&source[0], strstr(temp, strSign) + strlen(strSign));
//			if (strstr(source, "OK") != NULL)
//				strcpy(&source[0], strstr(source, "OK") + strlen("OK"));
//			int len = strlen(source);
//			indexBufRxDataUart2 = len;
//			if (dis != NULL) {
//				memset(dis, 0, sizeof(dis));
//				strncpy(dis, temp, lt - len);
//			}
//			debugPrint(dis);
////			if (dis != NULL) {
////				debugPrint("im 3");
////				strcpy(dis, source);
////			}
			return 1;
		} else {
			vTaskDelay(10);
		}
	}
//	debugPrint("im 4");
	if (dis != NULL) {
//		debugPrint("im 5");
		strcpy(dis, source);
	}
	return 0;
}

int checkCmd(char *source, char *strSign, int timeOut) {
	if (strstr(source, strSign)) {
		vTaskDelay(10);
		if (strstr(source, "resetQTM95"))
			return resetQTM95;
		if (strstr(source, "logQTM950"))
			return logQTM950;
		if (strstr(source, "logQTM951"))
			return logQTM951;
		if (strstr(source, "QIRDI"))
			return QIRDI;
		if (strstr(source, "datSerQTM950"))
			return datSerQTM950;
		if (strstr(source, "datSerQTM951"))
			return datSerQTM951;
		if (strstr(source, "datRecQTM950"))
			return datRecQTM950;
		if (strstr(source, "datRecQTM951"))
			return datRecQTM951;
		if (strstr(source, "run"))
			return run;
		if (strstr(source, "sendConnect"))
			return sendConnect;
		if (strstr(source, "sendPUB")) {
			if (strstr(source, "msg") != NULL)
				strcpy(&strPub[0], strstr(source, "msg") + strlen("msg"));
			return sendPUB;
		}
		if (strstr(source, "sendSUB"))
			return sendSUB;
		if (strstr(source, "bufU2"))
			return bufU2;
		if (strstr(source, "rstbufU2"))
			return rstbufU2;
		if (strstr(source, "rstlog")) {
//			resetBuf(bufLog, &indexBufLog, bufLogSize);
			return viewLog;
		}
		if (strstr(source, "viewLog")) {
			return viewLog;
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
	dbQuecTelM95.datSer = 0;
	QTM95.datRec = 0;
	db.log = 0;

	GPIO_Configuration();
	USARTx_Configuration(USART1);
	USARTx_Configuration(USART2);
	USARTx_Configuration(USART3);
	NVIC_Configuration();

	GPIO_SetBits(GPIOA, PWR_GPS);
	GPIO_SetBits(GPIOC, PWR_GSM);
	debugInfo("Waiting for data to show...");

	xTaskCreate(TaskB, (signed char* )"TaskB", 128, NULL, 4, NULL);
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
		uint8_t horizontal_dilution[10];//9
		char antenna_sea[4];//10
		char unit_antenna[2];//11
		char Geoidal_separation[4];//12
		char unit_Geoidal[2];//13
		char Differential_GPS[4];//14
		char station_ID[5];//15
		char checksum[3];//16
		strcpy(temp, GPRMC);
		USARTx_SendString(USART1, (uint8_t*) GPRMC);
		//char chuoisao[]="*";
		int st = strlen(GPRMC) - 4;
		//int dauphay = ',';
		//strncpy(chuoitrichra, GPGGA, n);
		//chuoitam  = strchr(GPGGA, dauphay);
		//int st = strlen(chuoicontro);
		//int m= (st-2);
		//char *test;
		//test=strstr(temp,chuoisao);
		//USARTx_SendString(USART1,(uint8_t*)test);
		//USARTx_SendString(USART1,(uint8_t*)"\n");
		//int check=strlen(temp);
		char *p;
//    	if(st<10)
//    	{
//    		USARTx_SendString(USART1,(uint8_t*)"TaskA: Waiting for data getting...\n");
//        	USARTx_SendString(USART1,(uint8_t*)GPGGA);
//        	USARTx_SendString(USART1,(uint8_t*)GPGSA);
//        	USARTx_SendString(USART1,(uint8_t*)GPRMC);
//    	}
//    	else if (st>=10)
		{
			USARTx_SendString(USART1, (uint8_t*) "^^^OK received data^^^\n");
			p = strtok(temp, ",*"); //cat chuoi bang cac ky tu
			////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
			while (index < 16)//for(index ; index<16 ; index++ )//while(p != NULL)
			{
				switch (index) {
					case 0:
					//USARTx_SendString(USART1,(uint8_t*)"1\n");
					strcpy(title, p);
					index++;
					break;//optional
					case 1:
					//USARTx_SendString(USART1,(uint8_t*)"2\n");
					strcpy(time_UTC, p);
					index++;
					break;//optional
					case 2:
					//USARTx_SendString(USART1,(uint8_t*)"3\n");
					strcpy(latitude, p);
					index++;
					break;//optional
					case 3:
					//USARTx_SendString(USART1,(uint8_t*)"4\n");
					strcpy(NS, p);
					index++;
					break;//optional
					case 4:
					//USARTx_SendString(USART1,(uint8_t*)"5\n");
					strcpy(longtitude, p);
					index++;
					break;
					case 5:
					//USARTx_SendString(USART1,(uint8_t*)"6\n");
					strcpy(EW, p);
					index++;
					break;
					case 6:
					//USARTx_SendString(USART1,(uint8_t*)"7\n");
					strcpy(qualitiGPS, p);
					index++;
					break;
					case 7:
					//USARTx_SendString(USART1,(uint8_t*)"8\n");
					strcpy(number_satellites, p);
					index++;
					break;
					case 8:
					//USARTx_SendString(USART1,(uint8_t*)"9\n");
					strcpy(horizontal_dilution, p);
					index++;
					break;
					case 9:
					//USARTx_SendString(USART1,(uint8_t*)"10\n");
					strcpy(antenna_sea, p);
					index++;
					break;
					case 10:
					//USARTx_SendString(USART1,(uint8_t*)"11\n");
					strcpy(unit_antenna, p);
					index++;
					break;
					case 11:
					//USARTx_SendString(USART1,(uint8_t*)"12\n");
					strcpy(Geoidal_separation, p);
					index++;
					break;
					case 12:
					//USARTx_SendString(USART1,(uint8_t*)"13\n");
					strcpy(unit_Geoidal, p);
					index++;
					break;
					case 13:
					//USARTx_SendString(USART1,(uint8_t*)"14\n");
					strcpy(Differential_GPS, p);
					index++;
					break;
					case 14:
					//USARTx_SendString(USART1,(uint8_t*)"15\n");
					strcpy(station_ID, p);
					index++;
					break;
					//case 15 :
					//USARTx_SendString(USART1,(uint8_t*)"16\n");
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

			USARTx_SendString(USART1, (uint8_t*) "Data getting from: ");
			USARTx_SendString(USART1, (uint8_t*) title);
			USARTx_SendString(USART1, (uint8_t*) "\n");
			USARTx_SendString(USART1, (uint8_t*) "Thoi gian UTC: ");
			USARTx_SendString(USART1, (uint8_t*) time_UTC);
			USARTx_SendString(USART1, (uint8_t*) "\n");
			USARTx_SendString(USART1, (uint8_t*) "Vi do: ");
			USARTx_SendString(USART1, (uint8_t*) latitude);
			USARTx_SendString(USART1, (uint8_t*) "\n");
			USARTx_SendString(USART1, (uint8_t*) "Huong vi do: ");
			USARTx_SendString(USART1, (uint8_t*) NS);
			USARTx_SendString(USART1, (uint8_t*) "\n");
			USARTx_SendString(USART1, (uint8_t*) "Kinh do: ");
			USARTx_SendString(USART1, (uint8_t*) longtitude);
			USARTx_SendString(USART1, (uint8_t*) "\n");
			USARTx_SendString(USART1, (uint8_t*) "Huong kinh do: ");
			USARTx_SendString(USART1, (uint8_t*) EW);
			USARTx_SendString(USART1, (uint8_t*) "\n");
			USARTx_SendString(USART1, (uint8_t*) "Chat luong GPS: ");
			USARTx_SendString(USART1, (uint8_t*) qualitiGPS);
			USARTx_SendString(USART1, (uint8_t*) "\n");
			USARTx_SendString(USART1, (uint8_t*) "So ve tinh nhin thay: ");
			USARTx_SendString(USART1, (uint8_t*) number_satellites);
			USARTx_SendString(USART1, (uint8_t*) "\n");
			USARTx_SendString(USART1, (uint8_t*) "Duong chan troi: ");
			USARTx_SendString(USART1, (uint8_t*) horizontal_dilution);
			USARTx_SendString(USART1, (uint8_t*) "\n");
			USARTx_SendString(USART1,
					(uint8_t*) "Do cao anten so voi nuoc bien: ");
			USARTx_SendString(USART1, (uint8_t*) antenna_sea);
			USARTx_SendString(USART1, (uint8_t*) " ");
			USARTx_SendString(USART1, (uint8_t*) unit_antenna);
			USARTx_SendString(USART1, (uint8_t*) "\n");
			USARTx_SendString(USART1, (uint8_t*) "Do tach roi hinh hoc: ");
			USARTx_SendString(USART1, (uint8_t*) Geoidal_separation);
			USARTx_SendString(USART1, (uint8_t*) " ");
			USARTx_SendString(USART1, (uint8_t*) unit_Geoidal);
			USARTx_SendString(USART1, (uint8_t*) "\n");
			USARTx_SendString(USART1, (uint8_t*) "Sai lech du lieu GPS: ");
			USARTx_SendString(USART1, (uint8_t*) Differential_GPS);
			USARTx_SendString(USART1, (uint8_t*) "\n");
			USARTx_SendString(USART1, (uint8_t*) "ID kenh tham chieu vi sai: ");
			USARTx_SendString(USART1, (uint8_t*) station_ID);
			USARTx_SendString(USART1, (uint8_t*) "\n");
			USARTx_SendString(USART1, (uint8_t*) "Ma kiem tra: ");
			USARTx_SendString(USART1, (uint8_t*) checksum);
			USARTx_SendString(USART1, (uint8_t*) "\n");

			/*************************** part data temp ok 1 ****************************/
			USARTx_SendString(USART1,
					(uint8_t*) "******Luu GPGGA co gia tri la:\n");
			USARTx_SendString(USART1, (uint8_t*) GPGGA);
			USARTx_SendString(USART1, (uint8_t*) "\n");
			//USARTx_SendString(USART1,(uint8_t*)"*****Luu GPGSA co gia tri la:\n");
			//USARTx_SendString(USART1,(uint8_t*)GPGSA);
			//USARTx_SendString(USART1,(uint8_t*)"\n");
			//USARTx_SendString(USART1,(uint8_t*)"*****Luu GPRMC co gia tri la:\n");
			//USARTx_SendString(USART1,(uint8_t*)GPRMC);
			//USARTx_SendString(USART1,(uint8_t*)"\n");
		}
#endif // debugA
		vTaskDelay(1000);
	}
}

static void TaskB(void *pvParameters) {

	for (;;) {
		if (QTM95.runOpen) {
			resetBuf(bufRxDataUart2, &indexBufRxDataUart2, bufRxDataUart2Size);
			if (checkSIM() == Good) {
				if (checkGSM() == Good) {
					if (checkGPRS() == Good) {
						if (configTCP() == Good) {
							if (openTCP() == Good) {
								debugInfo("Success connect...");
							} else
								debugPrint("openSocket fail");
						} else
							debugPrint("configTCP fail");
					} else
						debugPrint("checkGPRS fail");
				} else
					debugPrint("checkGSM fail");
			} else
				debugPrint("checkSIM fail");
			QTM95.runOpen = 0;
		}
		specFc();
		if (strstr(bufRxDataUart2, "PING") != NULL) {
			debugPrint("Detected PING>>>>>");
			resetBuf(bufRxDataUart2, &indexBufRxDataUart2, bufRxDataUart2Size);
			sendPONG();
			resetBuf(bufRxDataUart2, &indexBufRxDataUart2, bufRxDataUart2Size);
			j = 0;
		} else {
			j++;
			if (j == 180) {
				debugPrint(
						"Waiting for PING over 3 minute, auto send CONNECT again!!!!");
				resetBuf(bufRxDataUart2, &indexBufRxDataUart2,
				bufRxDataUart2Size);
				sendCONNECT();
				resetBuf(bufRxDataUart2, &indexBufRxDataUart2,
				bufRxDataUart2Size);
				j = 0;
			}
		}
		vTaskDelay(1000);
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
			sendPub();
			break;
		case sendSUB:
			debugInfo("received Command: sendSUB");
			sendSub();
			break;
		case sendConnect:
			debugInfo("received Command: sendConnect");
			resetBuf(bufRxDataUart2, &indexBufRxDataUart2, bufRxDataUart2Size);
			sendCONNECT();
			break;
		case logQTM951:
			debugInfo("received Command: logQTM951");
			dbQuecTelM95.log = 1;
			break;
		case logQTM950:
			debugInfo("received Command: logQTM950");
			dbQuecTelM95.log = 0;
			break;
		case resetQTM95:
			debugInfo("received Command: resetQTM95");
			resetQTM95fc();
			break;
		case datSerQTM950:
			debugInfo("received Command: datSerQTM950");
			dbQuecTelM95.datSer = 0;
			break;
		case datSerQTM951:
			debugInfo("received Command: datSerQTM951");
			dbQuecTelM95.datSer = 1;
			break;
		case datRecQTM950:
			debugInfo("received Command: datRecQTM950");
			QTM95.datRec = 0;
			break;
		case datRecQTM951:
			debugInfo("received Command: datRecQTM951");
			QTM95.datRec = 1;
			break;
		case bufU2:
			debugInfo("received Command: bufU2");
			USARTx_SendString(USART1, bufRxDataUart2);
			break;
		case rstbufU2:
			debugInfo("received Command: rstbufU2");
			resetBuf(bufRxDataUart2, &indexBufRxDataUart2, bufRxDataUart2Size);
			break;
		case viewLog:
			debugInfo("received Command: viewLog");
			USARTx_SendString(USART1, bufLog);
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
		debugInfo("co data from Server");
		configQIRD();
	}
}

int checkReqQTM95() {
	if (strstr(bufRxDataUart2, "QIRDI:") != null)
		return QIRDI;
	return NotFound;
}

void resetQTM95fc() {
	GPIO_SetBits(GPIOC, GSM_RST_Pin);
	vTaskDelay(100);
	GPIO_ResetBits(GPIOC, GSM_RST_Pin);
	vTaskDelay(1000);
	debugWarn("Reset Module QuecTelM95 OK");
}

int turnOffEchoM95() {
	GSM_Cmd("ATE0");
	vTaskDelay(500);
	if (strstr(bufRxDataUart2, "OK")) {
		return Good;
//	} else if (findStrSim("ERROR", 1)) {
//		debugWarn("fail \n\t More detail:");
//		debugBufQTM95(echo);
//		return Warn;
	} else {
//		debugError(simnotres);
//		debugBufQTM95(echo);
		return NotRespond;
	}
}

int turnOnEchoM95() {
	GSM_Cmd("ATE1");
	vTaskDelay(500);
	if (strstr(bufRxDataUart2, "OK")) {
//		debugInfo("OK");
		return Good;
//	} else if (findStrSim("ERROR", 1)) {
//		debugWarn("fail\n\t More detail:");
//		debugBufQTM95(echo);
//		return Warn;
	} else {
//		debugError(simnotres);
//		debugBufQTM95(echo);
		return NotRespond;
	}
}

int checkSIM() {
	GSM_Cmd("AT+CPIN?");
	vTaskDelay(500);
	if (strstr(bufRxDataUart2, "READY")) {
		return Good;
	} else {
		return Warn;
	}
}

int checkGSM() {
	GSM_Cmd("AT+GCAP");
	vTaskDelay(500);
	if (strstr(bufRxDataUart2, "+CGSM")) {
		return Good;
	} else {
		return Warn;
	}
}

int checkGPRS() {
	GSM_Cmd("AT+CREG?");
	vTaskDelay(500);
	if (strstr(bufRxDataUart2, "+CREG: 0,1")) {
		return Good;
	} else {
		return Warn;
	}
}

int configFGCNT() {
	GSM_Cmd("AT+QIFGCNT?");
	vTaskDelay(500);
	if (strstr(bufRxDataUart2, "OK")) {
		if (strstr(bufRxDataUart2, "+QIFGCNT: 0,")) {
			return Good;
		} else {
			GSM_Cmd("AT+QIFGCNT=0");
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
//	} else if (!configQIMODE()) {
//		return 0;
	} else if (!configQIDNSIP()) {
		return 0;
	} else {
		return 1;
	}
}

int openTCP() {
	GSM_Cmd("AT+QIOPEN= \"TCP\", \"itracking.vn\", 4222");
//	vTaskDelay(800);
//	while (!(strstr(bufRxDataUart2, "CONNECT OK"))
//			|| (strstr(bufRxDataUart2, "ALREADY"))) {
//			;
//	}
	checkStr(bufRxDataUart2, "CONNECT", 1000, null);
	if (strstr(bufRxDataUart2, "FAIL")) {
		GSM_Cmd("AT+QICLOSE");
		QTM95.runOpen = 1;
		k++;
		if (k == 3) {
			resetQTM95fc();
			k = 0;
		}
		return 0;
	}
	GSM_Cmd("AT+QISEND=164");
	vTaskDelay(500);
	if (strstr(bufRxDataUart2, "> ")) {
		GSM_Cmd(chuoiConnect);
		GSM_Cmd(controlZ);
		vTaskDelay(1000);
		if (strstr(bufRxDataUart2, "SEND OK")) {
			vTaskDelay(1000);
			configQIRD();
			if (strstr(bufRxDataUart2, "+QIRDI")) {
				configQIRD();
				vTaskDelay(1000);
				if (strstr(bufRxDataUart2, "+QIRDI")) {
					configQIRD();
					vTaskDelay(1000);
					USARTx_SendString(USART1, bufRxDataUart2);
					return Good;
				} else {
					USARTx_SendString(USART1, bufRxDataUart2);
					return Good;
				}
			}
			USARTx_SendString(USART1, bufRxDataUart2);
			return Good;
		} else {
			QTM95.runOpen = 1;
			USARTx_SendString(USART1, bufRxDataUart2);
			return 0;
		}
	} else {
		QTM95.runOpen = 1;
		USARTx_SendString(USART1, bufRxDataUart2);
		return 0;
	}
//	 else {
//		USARTx_SendString(USART1, bufRxDataUart2);
//		return NotRespond;
//	}
	return 0;
}

int sendCONNECT() {
	specFc();
	resetBuf(bufRxDataUart2, &indexBufRxDataUart2, bufRxDataUart2Size);
	GSM_Cmd("AT+QISEND=164");
	vTaskDelay(500);
	if (strstr(bufRxDataUart2, "ERROR"))
		QTM95.runOpen = 1;
	if (strstr(bufRxDataUart2, "> ")) {
		GSM_Cmd(chuoiConnect);
		GSM_Cmd(controlZ);
		vTaskDelay(1000);
		if (strstr(bufRxDataUart2, "SEND OK")) {
			vTaskDelay(1000);
//			configQIRD();
			specFc();
			USARTx_SendString(USART1, bufRxDataUart2);
			return Good;
		} else if (strstr(bufRxDataUart2, "SEND FAIL")) {
			USARTx_SendString(USART1, bufRxDataUart2);
			GSM_Cmd("AT+QICLOSE");
			vTaskDelay(500);
			QTM95.runOpen = 1;
			return Warn;
		} else {
			QTM95.runOpen = 1;
			USARTx_SendString(USART1, bufRxDataUart2);
			return Warn;
		}
	} else {
		QTM95.runOpen = 1;
		USARTx_SendString(USART1, bufRxDataUart2);
		return NotRespond;
	}
}

int openSocket() {
	if (openTCP() == Good) {
		if (sendCONNECT() == Good) {
			return 1;
		} else {
			debugInfo("sendCONNECT fail");
			return 0;
		}
	} else {
		debugInfo("OpenTCP fail");
		return 0;
	}
}

int configQIRD() {
	GSM_Cmd("AT+QIRD=0,1,0,1024");
	vTaskDelay(1000);
	if (strstr(bufRxDataUart2, "+OK")) {
		return Good;
	} else if (strstr(bufRxDataUart2, "PING")) {
		return Good;
	} else {
		return NotRespond;
	}
}

int sendPONG() {
	GSM_Cmd("AT+QISEND=6");
	vTaskDelay(500);
	if (strstr(bufRxDataUart2, "ERROR"))
		QTM95.runOpen = 1;
	if (strstr(bufRxDataUart2, "> ")) {
		GSM_Cmd("PONG");
		GSM_Cmd(controlZ);
		vTaskDelay(1000);
		if (strstr(bufRxDataUart2, "SEND OK")) {
			vTaskDelay(1000);
//			configQIRD();
			specFc();
			USARTx_SendString(USART1, bufRxDataUart2);
			return Good;

		} else if (strstr(bufRxDataUart2, "SEND FAIL")) {
			USARTx_SendString(USART1, bufRxDataUart2);
			GSM_Cmd("AT+QICLOSE");
			vTaskDelay(500);
			return Warn;
		} else {
			QTM95.runOpen = 1;
			USARTx_SendString(USART1, bufRxDataUart2);
			return Warn;
		}
	} else {
		QTM95.runOpen = 1;
		USARTx_SendString(USART1, bufRxDataUart2);
		return NotRespond;
	}
	return 0;
}

int sendPub() {
	specFc();
	resetBuf(bufRxDataUart2, &indexBufRxDataUart2, bufRxDataUart2Size);
	int i = strlen(strPub);
	if (i >= 10) i=i+27;
	else i= i +26;
	memset(atcmd, 0, sizeof(atcmd));
	sprintf(atcmd, "AT+QISEND=%d", i);
	GSM_Cmd(atcmd);//
//	GSM_Cmd("AT+QISEND=38");
	vTaskDelay(500);
	if (strstr(bufRxDataUart2, "ERROR"))
		QTM95.runOpen = 1;
	if (strstr(bufRxDataUart2, "> ")) {
		sprintf(strPubSend, "PUB R.1111111111.CMD %d\r\n%s\r\n",strlen(strPub), strPub);
//		strcat(strPubSend, strPub);
		GSM_Cmd(strPubSend);
//		GSM_Cmd("PUB R.1111111111.CMD 11\r\n12345678901\r\n");
		GSM_Cmd(controlZ);
		vTaskDelay(1000);
		if (strstr(bufRxDataUart2, "SEND OK")) {
			vTaskDelay(1000);
//			configQIRD();
			specFc();
			USARTx_SendString(USART1, bufRxDataUart2);
			return Good;
		} else if (strstr(bufRxDataUart2, "SEND FAIL")) {
			USARTx_SendString(USART1, bufRxDataUart2);
			GSM_Cmd("AT+QICLOSE");
			vTaskDelay(500);
			QTM95.runOpen = 1;
			return Warn;
		} else {
			QTM95.runOpen = 1;
			USARTx_SendString(USART1, bufRxDataUart2);
			return Warn;
		}
	} else return 0;
}

int sendSub() {
	specFc();
	resetBuf(bufRxDataUart2, &indexBufRxDataUart2, bufRxDataUart2Size);
	GSM_Cmd("AT+QISEND=24");
	vTaskDelay(500);
	if (strstr(bufRxDataUart2, "ERROR"))
		QTM95.runOpen = 1;
	if (strstr(bufRxDataUart2, "> ")) {
		GSM_Cmd("SUB R.1111111111.CMD 1\r\n");
		GSM_Cmd(controlZ);
		vTaskDelay(1000);
		USARTx_SendString(USART1, bufRxDataUart2);
		if (strstr(bufRxDataUart2, "SEND OK")) {
			vTaskDelay(1000);
//			configQIRD();
			specFc();
			USARTx_SendString(USART1, bufRxDataUart2);
			return Good;

		} else if (strstr(bufRxDataUart2, "SEND FAIL")) {
			USARTx_SendString(USART1, bufRxDataUart2);
			GSM_Cmd("AT+QICLOSE");
			vTaskDelay(500);
			QTM95.runOpen = 1;
			return Warn;
		} else {
			USARTx_SendString(USART1, bufRxDataUart2);
			return Warn;
		}
	} else {
		QTM95.runOpen = 1;
		USARTx_SendString(USART1, bufRxDataUart2);
		return NotRespond;
	}
	return 0;
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
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif

