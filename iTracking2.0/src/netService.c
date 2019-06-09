/*
 * netService.c
 *
 *  Created on: Oct 29, 2016
 *      Author: dv198
 */
/*##########################################################################################
 * INCLUDE
 *##########################################################################################*/
#include "netService.h"
#include "FreeRTOS.h"
#include "task.h"

#include "devstt.h"
#include "devcfg.h"

#include "stdio.h"
#include "string.h"
#include "stdlib.h"

#include "GSM.h"

#include "stringlib.h"
#include "macro.h"
/*##########################################################################################
 * DEFINES
 *##########################################################################################*/

#define NATS_RESP_TIMEOUT	3// Unit:second

/* Below is the list of NATS protocol (header only) messages from server (Not error message)
 * For more details, visit: http://nats.io/documentation/internals/nats-protocol/
 * */
#define NATS_SVMSG_LIST {"+OK\r\n"\
						, "PING\r\n"\
						, "PONG\r\n"\
						, "MSG "\
						, "INFO "\
						, "-ERR "\
						}
#define NATS_SVMSG_MAXINDEX	6
/* NATS message index, it's also the bit index of 'NETSRVC_NATS_SVmsg_index' variable  */
#define NATS_SVMSG_INDEX_OK			0
#define NATS_SVMSG_INDEX_PING		1
#define NATS_SVMSG_INDEX_PONG		2
#define NATS_SVMSG_INDEX_MSG		3
#define NATS_SVMSG_INDEX_INFO		4
#define NATS_SVMSG_INDEX_ERR		5
/* Below is list of NATS protocol error messages (header is "-ERR ")
 * Attention: All error message will close the connection, except the 1st one:"'Invalid Subject'"
 * For more details, visit: http://nats.io/documentation/internals/nats-protocol/
 * */
#define NATS_EMSG_LIST {"'Invalid Subject'"\
						, "'Unknown Protocol Operation'"\
						, "'Authorization Violation'"\
						, "'Authorization Timeout'"\
						, "'Parser Error'"\
						, "'Stale Connection'"\
						, "'Slow Consumer'"\
						, "'Maximum Payload Exceeded'"\
						}
#define NATS_EMSG_MAXINDEX				8
/* NATS error message index */
#define NATS_EMSG_INDEX_INVALIDSUB		0
#define NATS_EMSG_INDEX_UNKNOWNPROTOOP	1
#define NATS_EMSG_INDEX_AUTHVIOLATION	2
#define NATS_EMSG_INDEX_AUTHTIMEOUT		3
#define NATS_EMSG_INDEX_PARSERERR		4
#define NATS_EMSG_INDEX_STALECONN		5
#define NATS_EMSG_INDEX_SLOWCONSUMER	6
#define NATS_EMSG_INDEX_MAXPAYLOADEXC	7
/* Bit definitions of NATS streaming (NATSS) protocol message from server */
#define NATSS_PROTO_SERVMSG_CONNRESP	0x01
#define NATSS_PROTO_SERVMSG_SUBRESP		0x02
#define NATSS_PROTO_SERVMSG_PUBACK		0x04
#define NATSS_PROTO_SERVMSG_MSGPROTO	0x08
#define NATSS_PROTO_SERVMSG_CLOSERESP	0x10
/*##########################################################################################
 * TYPEDEFS
 *##########################################################################################*/

/*##########################################################################################
 * FUNC.PROTOTYPES
 *##########################################################################################*/
static void NETSRVC_dbgPrt_pseudo(const U8 *s,...);

/*##########################################################################################
 * VARIABLES
 *##########################################################################################*/
static NETSRVC_dbgPrt_CB NETSRVC_dbgPrt = NETSRVC_dbgPrt_pseudo;
/* Below is NATS protocol's variables */
static const U8 *NETSRVC_NATS_Emsg_list[] = NATS_EMSG_LIST;
static const U8 *NETSRVC_NATS_SVmsg_list[] = NATS_SVMSG_LIST;
static U8 NETSRVC_NATS_SVmsg_index = 0; /* View 'Bit definitions of NATS protocol message from server' */
static U8 NETSRVC_NATS_Emsg_index = 0;
/*##########################################################################################
 * FUNCTIONS
 *##########################################################################################*/
/*--------------------------------------------------------------------------------------
 * Brief:
 ---------------------------------------------------------------------------------------*/
void NETSRVC_dbgPrt_pseudo(const U8 *s,...)
{
	return;
}
/*--------------------------------------------------------------------------------------
 * Brief:
 ---------------------------------------------------------------------------------------*/
void NETSRVC_setup_dbgPrt(NETSRVC_dbgPrt_CB cb)
{
	if (cb == NULL)
	{
		NETSRVC_dbgPrt = NETSRVC_dbgPrt_pseudo;
		return;
	}
	NETSRVC_dbgPrt = cb;
}
/*--------------------------------------------------------------------------------------
 * Brief: connect to a NATS server.
 * Param:
 * Ret:		0	|	OK
 * 			>0	|	FAIL
 ---------------------------------------------------------------------------------------*/
U8 NETSRVC_NATS_connect(U8 sockindex, const U8 *host, U16 port//
		, U8 verbose//
		, U8 pedantic//
		, U8 ssl_required//
		, const U8 *auth_token//
		, const U8 *user//
		, const U8 *pass//
		, const U8 *name//
		, const U8 *version)
{
	const U16 SSIZE = 256;
	U8 s[SSIZE]//, rs[SSIZE];
		, res//
		, ec = 0xFF//
		;
	DATABLOCK_t sdat[1];

	snprintf(s, SSIZE, "CONNECT {");
	snprintf(s, SSIZE, "%s\"verbose\":%s", s, (verbose) ? "true" : "false");
	snprintf(s, SSIZE, "%s,\"pedantic\":%s", s, (pedantic) ? "true" : "false");
	snprintf(s, SSIZE, "%s,\"ssl_required\":%s", s, (ssl_required) ? "true" : "false");
	if (strlen(auth_token))
	{
		snprintf(s, SSIZE, "%s,\"auth_token\":\"%s\"", s, auth_token);
	}
	if (strlen(user))
	{
		snprintf(s, SSIZE, "%s,\"user\":\"%s\"", s, user);
		snprintf(s, SSIZE, "%s,\"pass\":\"%s\"", s, pass);
	}
	if (strlen(name))
	{
		snprintf(s, SSIZE, "%s,\"name\":\"%s\"", s, name);
	}
	snprintf(s, SSIZE, "%s,\"lang\":\"%s\"", s, "c");
	snprintf(s, SSIZE, "%s,\"version\":\"%s\"", s, version);
	strlcat(s, "}\r\n", SSIZE);

	NETSRVC_dbgPrt("\r\n %s:SI=%u msg=\"%s\"", __func__, sockindex, s);
	/* Setup APN */
	res = GSM_iniSock(APNPROFILE_lookup(DEVSTT_access()->net.network)->APNn//
			, APNPROFILE_lookup(DEVSTT_access()->net.network)->APNu //
			, APNPROFILE_lookup(DEVSTT_access()->net.network)->APNp);
	if (res)
	{
		ec = 1;
		goto NETSRVC_NATS_CONNECT_RETURN_EC;
	}
	/* Clear message '+OK' bit */
	CLRBIT(NETSRVC_NATS_SVmsg_index, NATS_SVMSG_INDEX_OK);
	CLRBIT(NETSRVC_NATS_SVmsg_index, NATS_SVMSG_INDEX_ERR);
	/* Connect and send message */
	sdat[0].dat = s;
	sdat[0].size = strlen(s);
	res = GSM_sendSock(sockindex, 0, host, port, 1, sdat, NULL);
	if (res)
	{
		ec = 2;
		goto NETSRVC_NATS_CONNECT_RETURN_EC;
	}
	/* Wait for feedback */
	for (U8 t = 0; t < (NATS_RESP_TIMEOUT * 10); t++)
	{
		if (GETBIT(NETSRVC_NATS_SVmsg_index, NATS_SVMSG_INDEX_OK))
		{
			NETSRVC_dbgPrt("\r\n %s:DONE", __func__);
			return 0;
		}
		if (GETBIT(NETSRVC_NATS_SVmsg_index, NATS_SVMSG_INDEX_ERR))
		{
			ec = 3;
			goto NETSRVC_NATS_CONNECT_RETURN_EC;
		}
		vTaskDelay(10);
	}
	ec = 4;
	NETSRVC_NATS_CONNECT_RETURN_EC:// Return error code
	return ec;
}
/*--------------------------------------------------------------------------------------
 * Brief: Subscribe to a subject that get message from a NATS server.
 * Param:
 * Ret:		0	|	OK
 * 			>0	|	FAIL
 ---------------------------------------------------------------------------------------*/
U8 NETSRVC_NATS_sub(U8 sockindex, const U8 *host, U16 port//
		, const U8 *subject//
		, const U8 *queueGroup//
		, U32 sid//
		)
{
	const U16 HEADERSIZE = 64;
	U8 msg[HEADERSIZE]//
		, res//
		, ec = 0xFF//
		;
	DATABLOCK_t sdat[1];

	snprintf(msg, HEADERSIZE, "SUB %s %s%s%lu\r\n", subject, queueGroup, (strlen(queueGroup)) ? " " : "", sid);
	NETSRVC_dbgPrt("\r\n %s:SI=%u msg=\"%s\"", __func__, sockindex, msg);
	/* Setup APN */
	res = GSM_iniSock(APNPROFILE_lookup(DEVSTT_access()->net.network)->APNn//
			, APNPROFILE_lookup(DEVSTT_access()->net.network)->APNu //
			, APNPROFILE_lookup(DEVSTT_access()->net.network)->APNp);
	if (res)
	{
		ec = 1;
		goto NETSRVC_NATS_SUB_RETURN_EC;
	}
	/* Clear message '+OK' bit */
	CLRBIT(NETSRVC_NATS_SVmsg_index, NATS_SVMSG_INDEX_OK);
	CLRBIT(NETSRVC_NATS_SVmsg_index, NATS_SVMSG_INDEX_ERR);
	/* Connect and send message */
	sdat[0].dat = msg;
	sdat[0].size = strlen(msg);
	res = GSM_sendSock(sockindex, 0, host, port, 1, sdat, NULL);
	if (res)
	{
		ec = 2;
		goto NETSRVC_NATS_SUB_RETURN_EC;
	}
	/* Wait for feedback */
	for (U8 t = 0; t < (NATS_RESP_TIMEOUT * 10); t++)
	{
		if (GETBIT(NETSRVC_NATS_SVmsg_index, NATS_SVMSG_INDEX_OK))
		{
			NETSRVC_dbgPrt("\r\n %s:DONE", __func__);
			return 0;
		}
		if (GETBIT(NETSRVC_NATS_SVmsg_index, NATS_SVMSG_INDEX_ERR))
		{
			ec = 3;
			goto NETSRVC_NATS_SUB_RETURN_EC;
		}
		vTaskDelay(10);
	}
	ec = 4;
	NETSRVC_NATS_SUB_RETURN_EC:// return error code
	NETSRVC_dbgPrt("\r\n %s:EC=%u", __func__, ec);
	return ec;
}
/*--------------------------------------------------------------------------------------
 * Brief: Publish a subject to a NATS server.
 * Param:
 * Ret:		0	|	OK
 * 			>0	|	FAIL
 ---------------------------------------------------------------------------------------*/
U8 NETSRVC_NATS_pub(U8 sockindex, const U8 *host, U16 port//
		, const U8 *subject//
		, const U8 *reply_to//
		, U32 bytes//
		, const U8 *payload//
		)
{
	const U16 HEADERSIZE = 128;
	U16 timeout = 0;
	U8 sHeader[HEADERSIZE]//
	   , sFooter[2] = {'\r', '\n'}//
	   , res//
	   , ec = 0xFF//
	   ;
	DATABLOCK_t sdat[3];

	snprintf(sHeader, HEADERSIZE, "PUB %s %s%s%lu\r\n", subject, reply_to, (strlen(reply_to)) ? " " : "", bytes);
	NETSRVC_dbgPrt("\r\n %s:SI=%u header=\"%s\"", __func__, sockindex, sHeader);
	/* Setup APN */
	res = GSM_iniSock(APNPROFILE_lookup(DEVSTT_access()->net.network)->APNn//
				, APNPROFILE_lookup(DEVSTT_access()->net.network)->APNu //
				, APNPROFILE_lookup(DEVSTT_access()->net.network)->APNp);
	if (res)
	{
		ec = 1;
		goto NETSRVC_NATS_PUB_RETURN_EC;
	}
	/* Clear message '+OK' bit */
	CLRBIT(NETSRVC_NATS_SVmsg_index, NATS_SVMSG_INDEX_OK);
	CLRBIT(NETSRVC_NATS_SVmsg_index, NATS_SVMSG_INDEX_ERR);
	/* Connect and send message */
	sdat[0].dat = sHeader;
	sdat[0].size = strlen(sHeader);
	sdat[1].dat = payload;
	sdat[1].size = bytes;
	sdat[2].dat = sFooter;
	sdat[2].size = 2;
	res = GSM_sendSock(sockindex, 0, host, port, 3, sdat, NULL);
	if (res)
	{
		ec = 2;
		goto NETSRVC_NATS_PUB_RETURN_EC;
	}
//	res = GSM_sendSock(sockindex, 0, host, port, strlen(sHeader), sHeader, NULL);
//	if (res)
//	{
//		ec = 2;
//		goto NETSRVC_NATS_PUB_RETURN_EC;
//	}
//	res = GSM_sendSock(sockindex, 0, "", port, bytes, payload, &sntBytes);
//	if (res)
//	{
//		ec = 3;
//		goto NETSRVC_NATS_PUB_RETURN_EC;
//	}
//	res = GSM_sendSock(sockindex, 0, "", port, 2, "\r\n", NULL);
//	if (res)
//	{
//		ec = 4;
//		goto NETSRVC_NATS_PUB_RETURN_EC;
//	}
	/* Wait for feedback */
	timeout = (bytes / 500) + 2;
	NETSRVC_dbgPrt("\r\n %s:WaitFB:Timeout=%u", __func__, timeout);
	for (U8 t = 0; t < (timeout * 10); t++)
	{
		if (GETBIT(NETSRVC_NATS_SVmsg_index, NATS_SVMSG_INDEX_OK))
		{
			NETSRVC_dbgPrt("\r\n %s:DONE", __func__);
			return 0;
		}
		if (GETBIT(NETSRVC_NATS_SVmsg_index, NATS_SVMSG_INDEX_ERR))
		{
			ec = 5;
			goto NETSRVC_NATS_PUB_RETURN_EC;
		}
		vTaskDelay(10);
	}
	ec = 6;
	NETSRVC_NATS_PUB_RETURN_EC://Return error code
	NETSRVC_dbgPrt("\r\n %s:EC=%u", __func__, ec);
	return ec;
}


#if 0
/*--------------------------------------------------------------------------------------
 * Brief: connect request to a NATS streaming server.
 * Param:
 * 			clientID		|	I	|
 * 			heartbeatInbox	|	I	|
 * Ret:		0	|	OK
 * 			>0	|	FAIL
 ---------------------------------------------------------------------------------------*/
U8 NETSRVC_NATSS_connRqst(const U8 *host, U16 port//
		, const U8 *clientID//
		, const U8 *heartbeatInbox)
{
	const U16 SSIZE = 256;
	U8 s[SSIZE], rs[SSIZE];
	U8 res;

	snprintf(s, SSIZE, "ConnectRequest {");
	snprintf(s, SSIZE, "%s\"clientID\":\"%s\"", s, clientID);
	if (strlen(heartbeatInbox))
	{
		snprintf(s, SSIZE, "%s,\"heartbeatInbox\":\"%s\"", s, heartbeatInbox);
	}
	strlcat(s, "}\r\n", SSIZE);
	sLen = strlen(s);
	res = GSM_iniSock(APNPROFILE_lookup(DEVSTT_access()->net.network)->APNn//
			, APNPROFILE_lookup(DEVSTT_access()->net.network)->APNu //
			, APNPROFILE_lookup(DEVSTT_access()->net.network)->APNp);
	if (res)
	{
		NETSRVC_dbgPrt("\r\n %s:setupAPN:res=%u->ERR", __func__, res);
		return 1;
	}
	GSM_setAvoidSockRxDatEvt(1);
	res = GSM_sendSock(0, host, port, strlen(s), s, NULL);
	if (res)
	{
		NETSRVC_dbgPrt("\r\n %s:sendSock:res=%u->ERR", __func__, res);
		GSM_closeSock();
		GSM_setAvoidSockRxDatEvt(0);
		return 2;
	}
	GSM_rdSock(10000, SSIZE, rs);
	NETSRVC_dbgPrt("\r\n %s:resp=\"%s\"", __func__, rs);
	GSM_setAvoidSockRxDatEvt(0);
	return 0;
}
#endif
/*--------------------------------------------------------------------------------------
 * Brief: NATS: ping to server
 ---------------------------------------------------------------------------------------*/
U8 NETSRVC_NATS_ping(U8 sockindex, const U8 *host, U16 port)
{
	const U16 MSGSIZE = 7;
	U8 msg[MSGSIZE]//
		, res//
		, ec = 0xFF//
		;
	DATABLOCK_t sdat[1];

	snprintf(msg, MSGSIZE, "PING\r\n");
	/* Setup APN */
	res = GSM_iniSock(APNPROFILE_lookup(DEVSTT_access()->net.network)->APNn//
			, APNPROFILE_lookup(DEVSTT_access()->net.network)->APNu //
			, APNPROFILE_lookup(DEVSTT_access()->net.network)->APNp);
	if (res)
	{
		ec = 1;
		goto NETSRVC_NATS_PING_RETURN_EC;
	}
	/* Clear message '+OK' bit */
	//NETSRVC_dbgPrt("\r\n %s:SVmsg_index(before)=%X", __func__, NETSRVC_NATS_SVmsg_index);
	CLRBIT(NETSRVC_NATS_SVmsg_index, NATS_SVMSG_INDEX_OK);
	CLRBIT(NETSRVC_NATS_SVmsg_index, NATS_SVMSG_INDEX_PONG);
	CLRBIT(NETSRVC_NATS_SVmsg_index, NATS_SVMSG_INDEX_ERR);
	//NETSRVC_dbgPrt("\r\n %s:SVmsg_index(after)=%X", __func__, NETSRVC_NATS_SVmsg_index);
	/* Connect and send message */
	sdat[0].dat = msg;
	sdat[0].size = strlen(msg);
	res = GSM_sendSock(sockindex, 0, host, port, 1, sdat, NULL);
	if (res)
	{
		ec = 2;
		goto NETSRVC_NATS_PING_RETURN_EC;
	}
	/* Wait for feedback */
	for (U8 t = 0; t < (NATS_RESP_TIMEOUT * 10); t++)
	{
		if (GETBIT(NETSRVC_NATS_SVmsg_index, NATS_SVMSG_INDEX_OK) || GETBIT(NETSRVC_NATS_SVmsg_index, NATS_SVMSG_INDEX_PONG))
		{
			NETSRVC_dbgPrt("\r\n %s:%u:DONE", __func__, sockindex);
			return 0;
		}
		if (GETBIT(NETSRVC_NATS_SVmsg_index, NATS_SVMSG_INDEX_ERR))
		{
			ec = 3;
			goto NETSRVC_NATS_PING_RETURN_EC;
		}
		vTaskDelay(10);
	}
	ec = 4;
	NETSRVC_NATS_PING_RETURN_EC:// return error code
	NETSRVC_dbgPrt("\r\n %s:EC=%u", __func__, ec);
	return ec;
}
/*--------------------------------------------------------------------------------------
 * Brief: NATS: send 'PONG' message
 * Param:	byPing		|	IN	|	check if got 'PING' or not
 * 			sockindex	|	IN	|	socket index
 * 			host		|	IN	|	host name / IP
 * 			port		|	IN	|	TCP port
 * Ret:	0		|	OK
 * 		>0		|	FAIL
 * 		0xFF	|	send PONG by PING but there was no PING
 ---------------------------------------------------------------------------------------*/
U8 NETSRVC_NATS_pong(U8 byPing, U8 sockindex, const U8 *host, U16 port)
{
	const U16 MSGSIZE = 7;
	U8 msg[MSGSIZE];
	U8 res = 0, ec = 0xFF;
	DATABLOCK_t sdat[1];

	if (byPing)
	{
		if (GETBIT(NETSRVC_NATS_SVmsg_index, NATS_SVMSG_INDEX_PING))
		{
			//NETSRVC_dbgPrt("\r\n %s:Ping detected", __func__);
			CLRBIT(NETSRVC_NATS_SVmsg_index, NATS_SVMSG_INDEX_PING);
		}
		else
		{
			return 0xFF;
		}
	}
	snprintf(msg, MSGSIZE, "PONG\r\n");
	CLRBIT(NETSRVC_NATS_SVmsg_index, NATS_SVMSG_INDEX_OK);
	CLRBIT(NETSRVC_NATS_SVmsg_index, NATS_SVMSG_INDEX_ERR);
	sdat[0].dat = msg;
	sdat[0].size = strlen(msg);
	res = GSM_sendSock(sockindex, 0, host, port, 1, sdat, NULL);
	if (res)
	{
		ec = 1;
		goto NETSRVC_NATS_PONG_RETURN_EC;
	}
	/* Wait for feedback */
	for (U8 t = 0; t < (NATS_RESP_TIMEOUT * 10); t++)
	{
		if (byPing || GETBIT(NETSRVC_NATS_SVmsg_index, NATS_SVMSG_INDEX_OK))
		{
			/* Received '+OK', or sending 'PONG' by 'PING' doesn't require feedback */
			NETSRVC_dbgPrt("\r\n %s:DONE", __func__);
			return 0;
		}
		if (GETBIT(NETSRVC_NATS_SVmsg_index, NATS_SVMSG_INDEX_ERR))
		{
			ec = 2;
			goto NETSRVC_NATS_PONG_RETURN_EC;
		}
		vTaskDelay(10);
	}
	ec = 3;
	NETSRVC_NATS_PONG_RETURN_EC:// Return error code
	NETSRVC_dbgPrt("\r\n %s:EC=%u", __func__, ec);
	return ec;
}
/*--------------------------------------------------------------------------------------
 * Brief: Handling NATS unsol message.
 * Param:	len				|	I	| 	whole message length
 * 			msg				|	I	|	message.
 * 			MSG_subjectSize	|	I	|	server message (with header is 'MSG ') subject's max size.
 * 			MSG_subject		|	O	|	server message (with header is 'MSG ') subject
 * 			MSG_bytes		|	I/O	|	server message (with header is 'MSG ') max length/ returned server message length.
 * 			MSG_payload		|	O	|	server message (with header is 'MSG ').
 * Ret: 0	| 	nothing.
 * 		1	|	it's NATS PING.
 * 		2	|	it's NATS MSG.
 * 		3	|	it's NATS ERR.
 ---------------------------------------------------------------------------------------*/
U8 NETSRVC_NATS_hdlUnsolMsg(U16 len, U8 *msg, U16 MSG_subjectSize, U8 *MSG_subject, U16 *MSG_bytes, U8 *MSG_payload)
{
	U8 ret = 0;
	NETSRVC_NATS_HDLUNSOLMSG_BEGIN://
	/* Classify message type */
	for (U8 i = 0; i < NATS_SVMSG_MAXINDEX; i++)
	{
		//NETSRVC_dbgPrt("\r\n %s:cmpMsg \"%s\"/\"%s\"", __func__, msg, NETSRVC_NATS_SVmsg_list[i]);
		if (!SL_startwith(msg, NETSRVC_NATS_SVmsg_list[i]))
		{
			/* Set appropriate bit with message type */
			//NETSRVC_dbgPrt("\r\n %s:MatchedSVmsgIndex=%u", __func__, i);
			SETBIT(NETSRVC_NATS_SVmsg_index, i);
			switch (i)
			{
				case NATS_SVMSG_INDEX_PING:
					ret = 1;
				case NATS_SVMSG_INDEX_OK:
				case NATS_SVMSG_INDEX_PONG:
					SL_cut(msg, NETSRVC_NATS_SVmsg_list[i]);
					if (strlen(msg))
					{
						goto NETSRVC_NATS_HDLUNSOLMSG_BEGIN;
					}
					break;
				case NATS_SVMSG_INDEX_INFO:
				{
					int end = SL_search(msg, "\r\n");
					SL_sub(end + 2, len - (end + 2), msg, msg);
					if (strlen(msg))
					{
						goto NETSRVC_NATS_HDLUNSOLMSG_BEGIN;
					}
				}
					break;
				case NATS_SVMSG_INDEX_ERR:
					/* This is error message */
					ret = 3;
					/* Classify error message type */
					for (U8 i2 = 0; i2 < NATS_EMSG_MAXINDEX; i2++)
					{
						//NETSRVC_dbgPrt("\r\n %s:cmpEMsg \"%s\"", __func__, NETSRVC_NATS_SVmsg_list[i2]);
						if (!SL_startwith(&msg[strlen(NETSRVC_NATS_SVmsg_list[NATS_SVMSG_INDEX_ERR])], NETSRVC_NATS_Emsg_list[i2]))
						{
							/* Set appropriate bit with error message type */
							SETBIT(NETSRVC_NATS_Emsg_index, i2);
							if (i2 > NATS_EMSG_INDEX_INVALIDSUB)
							{
								/* All errors except for 'Invalid Subject' will close connection */
							}
							break;
						}
					}
					break;
				case NATS_SVMSG_INDEX_MSG:
					/* This is server message (with header is 'MSG '). */
					ret = 2;
				{
					int tmp = 0;
					U16 tmpStart = 0, tmpLen = 0;
					U8 tmpS[11];

					/* Parse subject */
					if (MSG_subjectSize)
					{
						tmp = SL_search(msg, " ");
						if (tmp != -1)
						{
							tmpStart = tmp + 1;
							tmp = SL_search(&msg[tmpStart], " ");
							if (tmp != -1)
							{
								tmpLen = tmp;
								SL_sub(tmpStart, tmpLen, msg, MSG_subject);
							}
						}
					}
					tmpStart = 0;
					tmpLen = 0;
					tmp = SL_search(msg, "\r\n");
					if (tmp != -1)
					{
						tmpStart = tmp;
						for (U16 i = tmpStart; i > 0; i--)
						{
							if (msg[i] == ' ')
							{
								/* Parse 'Payload length' parameter from message */
								strlcpy(tmpS, &msg[i + 1], tmpLen);
								tmpLen = atoi(tmpS); // Put 'Payload length' value to 'tmpLen' now.
								break;
							}
							if (tmpLen < 11)
							{
								tmpLen++; // Avoid overflow
							}
						}
					}
					if (*MSG_bytes != 0)
					{
						/* Allow returning 'Payload' */
						tmpStart += 2;//Move to start index of 'payload' parameter.
						/* Copy 'payload' to 'MSG_payload' */
						memcpy(MSG_payload, &msg[tmpStart], (*MSG_bytes > tmpLen) ? tmpLen : *MSG_bytes);
					}
					/* Return 'Payload length' */
					*MSG_bytes = tmpLen;
				}
				default:
					break;
			}
			break;
		}
	}
	NETSRVC_dbgPrt("\r\n %s:Msg=%X EMsg=%X", __func__, NETSRVC_NATS_SVmsg_index, NETSRVC_NATS_Emsg_index);
	return ret;
}
/*--------------------------------------------------------------------------------------
 * Brief: Request NTP server for UTC.
 * Param:
 * Ret:		0	|	OK
 * 			>0	|	FAIL
 ---------------------------------------------------------------------------------------*/
U8 NETSRVC_rqstNTP(U8 sockindex, const U8 *host, U32 *utc)
{
	U8 NTPpacket[48], rxNTPpacket[48], ec = 0xFF;
	DATABLOCK_t sdat[1];

	memset(NTPpacket, 0, 48);
	memset(rxNTPpacket, 0, 48);
	NTPpacket[0] = 0xE3; // LI, Version, Mode
	NTPpacket[1] = 0; // Stratum, or type of clock
	NTPpacket[2] = 6; // Polling Interval
	NTPpacket[3] = 0xEC; // Peer Clock Precision
	// 8 bytes of zero for Root Delay & Root Dispersion
	NTPpacket[12] = 49;
	NTPpacket[13] = 0x4E;
	NTPpacket[14] = 49;
	NTPpacket[15] = 52;
	if (GSM_iniSock(APNPROFILE_lookup(DEVSTT_access()->net.network)->APNn//
			, APNPROFILE_lookup(DEVSTT_access()->net.network)->APNu //
			, APNPROFILE_lookup(DEVSTT_access()->net.network)->APNp))
	{
		ec = 1;
		goto NETSRVC_RQSTNTP_RETURN_EC;
	}
	GSM_setAvoidSockRxDatEvt(1);
	sdat[0].dat = NTPpacket;
	sdat[0].size = 48;
	if (GSM_sendSock(sockindex, 1, host, 123, 1, sdat, NULL))
	{
		GSM_setAvoidSockRxDatEvt(0);
		ec = 2;
		goto NETSRVC_RQSTNTP_RETURN_EC;
	}
	if (!GSM_rdSock(sockindex, 3000, 48, rxNTPpacket))
	{
		GSM_setAvoidSockRxDatEvt(0);
		ec = 3;
		goto NETSRVC_RQSTNTP_RETURN_EC;
	}
	GSM_setAvoidSockRxDatEvt(0);
	*utc = (rxNTPpacket[40] << 24) | (rxNTPpacket[41] << 16) | (rxNTPpacket[42] << 8) | (rxNTPpacket[43] << 0);
	if( *utc > (U32) 2208988800 )
	{
		*utc -= (U32) 2208988800; //Server startTime:1/1/1900 , device startTime:1/1/1970
	}
	else
	{
		ec = 4;
		goto NETSRVC_RQSTNTP_RETURN_EC;
	}
	return 0;
	NETSRVC_dbgPrt("\r\n %s:EC=%u", __func__, ec);
	NETSRVC_RQSTNTP_RETURN_EC:// Return error code
	return ec;
}
