//_____________________________________________________________________________
//
// Copyright 2013-4 Time Domain Corporation
//
//
// rcm.c
//
//   A collection of functions to communicate with RangeNet on an RCM.
//
//
//_____________________________________________________________________________


//_____________________________________________________________________________
//
// #includes 
//_____________________________________________________________________________

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef WIN32
#include "winsock2.h"
#else // linux
#include <arpa/inet.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include "pulson_ros/P4xx/RangeNet/rcmIf.h"
#include "pulson_ros/P4xx/RangeNet/rcm.h"
#include "pulson_ros/P4xx/RangeNet/ALOHA/rn.h"


//_____________________________________________________________________________
//
// #defines 
//_____________________________________________________________________________


//_____________________________________________________________________________
//
// typedefs
//_____________________________________________________________________________


//_____________________________________________________________________________
//
// static data
//_____________________________________________________________________________

static int msgIdCount;


//_____________________________________________________________________________
//
// Private function prototypes 
//_____________________________________________________________________________



//_____________________________________________________________________________
//
// rnConfigGet - get RangeNet base configuration from radio
//_____________________________________________________________________________

int rnConfigGet(rnConfiguration *config)
{
    rnMsg_GetConfigRequest request;
    rnMsg_GetConfigConfirm confirm;
    int retVal = ERR, numBytes;

    // create request message
	request.msgType = htons(RN_GET_CONFIG_REQUEST);
	request.msgId = htons(msgIdCount++);

    // make sure no pending messages
    rcmIfFlush();

    // send message to RCM
	rcmIfSendPacket(&request, sizeof(request));

    // wait for response
    numBytes = rcmIfGetPacket(&confirm, sizeof(rnMsg_GetConfigConfirm));

    // did we get a response from the RCM?
    if (numBytes == sizeof(rnMsg_GetConfigConfirm))
    {
        // Handle byte ordering
        confirm.msgType = ntohs(confirm.msgType);
        confirm.msgId = ntohs(confirm.msgId);

        // is this the correct message type?
        if (confirm.msgType == RN_GET_CONFIG_CONFIRM)
        {
            // copy config from message to caller's structure
            memcpy(config, &confirm.config, sizeof(*config));

            // Handle byte ordering
            config->maxNeighborAgeMs = ntohl(config->maxNeighborAgeMs);
            config->autosendNeighborDbUpdateIntervalMs = ntohs(config->autosendNeighborDbUpdateIntervalMs);
            config->rnFlags = ntohs(config->rnFlags);
            config->defaultIfAddr1 = ntohl(config->defaultIfAddr1);
            config->defaultIfAddr2 = ntohl(config->defaultIfAddr2);

            // milliseconds since radio boot
            confirm.timestamp = ntohl(confirm.timestamp);

            // status code
            confirm.status = ntohl(confirm.status);

            // only return OK if status is OK
            if (confirm.status == OK)
                retVal = OK;
        }
    }
    return retVal;
}


//_____________________________________________________________________________
//
// rnConfigSet - set RangeNet base configuration in radio
//_____________________________________________________________________________

int rnConfigSet(rnConfiguration *config)
{
    rnMsg_SetConfigRequest request;
    rnMsg_SetConfigConfirm confirm;
    int retVal = ERR, numBytes;

    // create request message
	request.msgType = htons(RN_SET_CONFIG_REQUEST);
	request.msgId = htons(msgIdCount++);
    memcpy(&request.config, config, sizeof(*config));

    // Handle byte ordering in config struct
    request.config.maxNeighborAgeMs = htonl(config->maxNeighborAgeMs);
    request.config.autosendNeighborDbUpdateIntervalMs = htons(config->autosendNeighborDbUpdateIntervalMs);
    request.config.rnFlags = htons(config->rnFlags);
    request.config.defaultIfAddr1 = htonl(config->defaultIfAddr1);
    request.config.defaultIfAddr2 = htonl(config->defaultIfAddr2);

    // make sure no pending messages
    rcmIfFlush();

    // send message to RCM
	rcmIfSendPacket(&request, sizeof(request));

    // wait for response
    numBytes = rcmIfGetPacket(&confirm, sizeof(confirm));

    // did we get a response from the RCM?
    if (numBytes == sizeof(confirm))
    {
        // Handle byte ordering
        confirm.msgType = ntohs(confirm.msgType);
        confirm.status = ntohl(confirm.status);

        // is this the correct message type and is status good?
        if (confirm.msgType == RN_SET_CONFIG_CONFIRM &&
                confirm.status == OK)
            retVal = OK;
    }
    return retVal;
}


//_____________________________________________________________________________
//
// rnAlohaConfigGet - get RangeNet ALOHA configuration from radio
//_____________________________________________________________________________

int rnAlohaConfigGet(rnALOHAConfiguration *config)
{
    rnMsg_GetALOHAConfigRequest request;
    rnMsg_GetALOHAConfigConfirm confirm;
    int retVal = ERR, numBytes;

    // create request message
	request.msgType = htons(RN_GET_ALOHA_CONFIG_REQUEST);
	request.msgId = htons(msgIdCount++);

    // make sure no pending messages
    rcmIfFlush();

    // send message to RCM
	rcmIfSendPacket(&request, sizeof(request));

    // wait for response
    numBytes = rcmIfGetPacket(&confirm, sizeof(rnMsg_GetALOHAConfigConfirm));

    // did we get a response from the RCM?
    if (numBytes == sizeof(rnMsg_GetALOHAConfigConfirm))
    {
        // Handle byte ordering
        confirm.msgType = ntohs(confirm.msgType);
        confirm.msgId = ntohs(confirm.msgId);

        // is this the correct message type?
        if (confirm.msgType == RN_GET_ALOHA_CONFIG_CONFIRM)
        {
            // copy config from message to caller's structure
            memcpy(config, &confirm.config, sizeof(*config));

            // Handle byte ordering
            config->minTimeBetweenTxMs = ntohs(config->minTimeBetweenTxMs);
            config->maxTimeBetweenTxMs = ntohs(config->maxTimeBetweenTxMs);
            config->maxRequestDataSize = ntohs(config->maxRequestDataSize);
            config->maxResponseDataSize = ntohs(config->maxResponseDataSize);
            config->flags = ntohs(config->flags);

            // status code
            confirm.status = ntohl(confirm.status);
            // only return OK if status is OK
            if (confirm.status == OK)
                retVal = OK;
        }
    }
    return retVal;
}


//_____________________________________________________________________________
//
// rnAlohaConfigSet - set RangeNet ALOHA configuration in radio
//_____________________________________________________________________________

int rnAlohaConfigSet(rnALOHAConfiguration *config)
{
    rnMsg_SetALOHAConfigRequest request;
    rnMsg_SetALOHAConfigConfirm confirm;
    int retVal = ERR, numBytes;

    // create request message
	request.msgType = htons(RN_SET_ALOHA_CONFIG_REQUEST);
	request.msgId = htons(msgIdCount++);
    memcpy(&request.config, config, sizeof(*config));

    // Handle byte ordering in config struct
    request.config.minTimeBetweenTxMs = htons(config->minTimeBetweenTxMs);
    request.config.maxTimeBetweenTxMs = htons(config->maxTimeBetweenTxMs);
    request.config.maxRequestDataSize = htons(config->maxRequestDataSize);
    request.config.maxResponseDataSize = htons(config->maxResponseDataSize);
    request.config.flags = htons(config->flags);

    // make sure no pending messages
    rcmIfFlush();

    // send message to RCM
	rcmIfSendPacket(&request, sizeof(request));

    // wait for response
    numBytes = rcmIfGetPacket(&confirm, sizeof(confirm));

    // did we get a response from the RCM?
    if (numBytes == sizeof(confirm))
    {
        // Handle byte ordering
        confirm.msgType = ntohs(confirm.msgType);
        confirm.status = ntohl(confirm.status);

        // is this the correct message type and is status good?
        if (confirm.msgType == RN_SET_ALOHA_CONFIG_CONFIRM &&
                confirm.status == OK)
            retVal = OK;
    }
    return retVal;
}
