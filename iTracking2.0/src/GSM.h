/*
 * GSM.h
 *
 *  Created on: Apr 15, 2017
 *      Author: dv198
 */

#ifndef GSM_H_
#define GSM_H_

/*---------------------------------------------------------------------------*/
#if (HW_MAIN_VERSION == 1) /*------------------------------------------------*/
#define GSM_setup					SIM900_setup
#define GSM_setup_dbgPrt 			SIM900_setup_dbgPrt
#define GSM_rst						SIM900_rst
#define GSM_readc					SIM900_readc
#define GSM_getSMSindexList 		SIM900_getSMSindexList
#define GSM_get_RxSMSindex			SIM900_get_RxSMSindex
#define GSM_rst_RxSMSindex 			SIM900_rst_RxSMSindex
#define GSM_readSMS					SIM900_readSMS
#define GSM_sendSMS					SIM900_sendSMS
#define GSM_delSMS					SIM900_delSMS
#define GSM_get_sockRxDindex 		SIM900_get_sockRxDindex
#define GSM_rst_sockRxDindex 		SIM900_rst_sockRxDindex
#define GSM_setAvoidSockRxDatEvt	SIM900_setAvoidSockRxDatEvt
#define GSM_iniSock					SIM900_iniSock
#define GSM_closeSock 				SIM900_closeSock
#define GSM_sendSock				SIM900_sendSock
#define GSM_rdSock 					SIM900_rdSock
#define GSM_getNetwork				SIM900_getNetwork
#define GSM_getSQ					SIM900_getSQ
#define GSM_getNetRegStt			SIM900_getNetRegStt
#define GSM_setCSCLK				SIM900_setCSCLK
#endif // #if (HW_MAIN_VERSION == 1)
#if (HW_MAIN_VERSION == 2) /*------------------------------------------------*/
#define GSM_setup					QM95_setup
#define GSM_setup_dbgPrt 			QM95_setup_dbgPrt
#define GSM_rst						QM95_rst
#define GSM_readc					QM95_readc
#define GSM_getSMSindexList 		QM95_getSMSindexList
#define GSM_get_RxSMSindex			QM95_get_RxSMSindex
#define GSM_rst_RxSMSindex 			QM95_rst_RxSMSindex
#define GSM_readSMS					QM95_readSMS
#define GSM_sendSMS					QM95_sendSMS
#define GSM_delSMS					QM95_delSMS
#define GSM_get_sockRxDindex 		QM95_get_sockRxDindex
#define GSM_rst_sockRxDindex 		QM95_rst_sockRxDindex
#define GSM_setAvoidSockRxDatEvt	QM95_setAvoidSockRxDatEvt
#define GSM_iniSock					QM95_iniSock
#define GSM_closeSock 				QM95_closeSock
#define GSM_sendSock				QM95_sendSock
#define GSM_rdSock 					QM95_rdSock
#define GSM_getNetwork				QM95_getNetwork
#define GSM_getSQ					QM95_getSQ
#define GSM_getNetRegStt			QM95_getNetRegStt
#define GSM_setCSCLK				QM95_setCSCLK
#endif // #if (HW_MAIN_VERSION == 2)
/*---------------------------------------------------------------------------*/
#endif /* GSM_H_ */
