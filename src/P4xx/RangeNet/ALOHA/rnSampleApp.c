//__________________________________________________________________________________________________
//
// Copyright 2013-4 Time Domain Corporation
//
//
// rnSampleApp.c
//
//   Sample code showing how to interface to P400 RCM module embedded RangeNet software.
//
//   This code uses the functions in rcm.c and rn.c to:
//      - make sure the RCM is awake and in the correct mode
//      - get the base, RangeNet, and ALOHA configurations from the RCM and print them
//      - get the status/info from the RCM and print it
//		- modify the base, RangeNet, and ALOHA configurations for sample app use (and print them)
//		- allow the user to choose between receiving Range Info or Neighbor Database messages
//		- display received messages until either a key is hit (Windows) or 10 sec elapses (linux)
//      - restore original configuration upon exit
//
// This sample can communicate with the RCM over Ethernet, the 3.3V serial port,
// or the USB interface (which acts like a serial port).
//
//__________________________________________________________________________________________________


//_____________________________________________________________________________
//
// #includes 
//_____________________________________________________________________________

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef WIN32
#include <conio.h>
#else // linux
#include <time.h>
#endif

#include "rnSampleApp.h"
#include "rcmIf.h"
#include "rcm.h"
#include "rn.h"

//_____________________________________________________________________________
//
// local function prototypes
//_____________________________________________________________________________

//_____________________________________________________________________________
//
// usage - display command line parameters when none are provided
//_____________________________________________________________________________
void usage(void)
{
#ifdef WIN32
	printf("usage: rnSampleApp -i <IP address> | -s <COM port> | -u <USB COM port>\n");
#else
	printf("usage: rnSampleApp -i <IP address> | -s <COM port> | -u <Node ID>\n");
#endif
    printf("\nTo connect to radio at IP address 192.168.1.100 via Ethernet:\n");
    printf("\trnSampleApp -i 192.168.1.100\n");
    printf("\nTo connect to radio's serial port using USB-to-TTL serial converter at COM3:\n");
    printf("\trnSampleApp -s COM3\n");
#ifdef WIN32
    printf("\nTo connect to radio's USB port at COM10:\n");
    printf("\trnSampleApp -u COM10\n");
#else
    printf("\nTo connect to Node ID via USB:\n");
    printf("\trnSampleApp -u 100\n");
#endif
	exit(0);
}

//_____________________________________________________________________________
//
// printRcmConfig - prints base configuration using tags from API doc
//_____________________________________________________________________________
void printRcmConfig(rcmConfiguration *rcmConfig)
{
    printf("\tnodeId: %d\n", rcmConfig->nodeId);
    printf("\tintegrationIndex: %d\n", rcmConfig->integrationIndex);
    printf("\tantennaMode: %d\n", rcmConfig->antennaMode);
    printf("\tcodeChannel: %d\n", rcmConfig->codeChannel);
    printf("\telectricalDelayPsA: %d\n", rcmConfig->electricalDelayPsA);
    printf("\telectricalDelayPsB: %d\n", rcmConfig->electricalDelayPsB);
    printf("\tflags: 0x%X\n", rcmConfig->flags);
	printf("\ttxGain: %d\n", rcmConfig->txGain);
}

//_____________________________________________________________________________
//
// printRnConfig - prints RangeNet configuration using tags from API doc
//_____________________________________________________________________________
void printRnConfig(rnConfiguration *rnConfig)
{
    printf("\tmaxNeighborAgeMs: %d\n", rnConfig->maxNeighborAgeMs);
    printf("\tautosendNeighborDbUpdateIntervalMs: %d\n", rnConfig->autosendNeighborDbUpdateIntervalMs);
    printf("\trnFlags: 0x%X\n", rnConfig->rnFlags);
    printf("\tnetworkSyncMode: %d\n", rnConfig->networkSyncMode);
    printf("\tautosendType: 0x%X\n", rnConfig->autosendType);
    printf("\tdefaultIf: %d\n", rnConfig->defaultIf);
    printf("\tdefaultIfAddr1: %d\n", rnConfig->defaultIfAddr1);
    printf("\tdefaultIfAddr2: %d\n", rnConfig->defaultIfAddr2);
}

//_____________________________________________________________________________
//
// printRnAlohaConfig - prints ALOHA configuration using tags from API doc
//_____________________________________________________________________________
void printRnAlohaConfig(rnALOHAConfiguration *rnAlohaConfig)
{
    printf("\tminTimeBetweenTxMs: %d\n", rnAlohaConfig->minTimeBetweenTxMs);
    printf("\tmaxTimeBetweenTxMs: %d\n", rnAlohaConfig->maxTimeBetweenTxMs);
    printf("\tmaxRequestDataSize: %d\n", rnAlohaConfig->maxRequestDataSize);
    printf("\tmaxResponseDataSize: %d\n", rnAlohaConfig->maxResponseDataSize);
    printf("\tFlags: 0x%X\n", rnAlohaConfig->flags);
    printf("\taccKfactor: %d\n", rnAlohaConfig->accKfactor);
}

//_____________________________________________________________________________
//
// main - sample app entry point
//_____________________________________________________________________________
int main(int argc, char *argv[])
{
#ifdef WIN32
	char keyHit;
#endif
    char msgType[100];
    int status;
	int valid = 0;
	int i;
	int initOpMode;
    rcmIfType rcmIf;

	// Configuration structures
    rcmConfiguration rcmInitConfig, rcmConfig;
	rnConfiguration rnInitConfig, rnConfig;
	rnALOHAConfiguration rnInitAlohaConfig, rnAlohaConfig;

	// Info message structures
    rcrmMsg_GetStatusInfoConfirm statusInfo;
    rcmMsg_FullRangeInfo rangeInfo;
	rnMsg_GetFullNeighborDatabaseConfirm ndbInfo;

	printf("RangeNet Sample App\n\n");

    // check command line arguments
    if (argc != 3)
        usage();

	// check for interface type
    if (!strcmp(argv[1], "-i"))
        rcmIf = rcmIfIp;
    else if (!strcmp(argv[1], "-s"))
        rcmIf = rcmIfSerial;
    else if (!strcmp(argv[1], "-u"))
        rcmIf = rcmIfUsb;
    else
        usage();

    // initialize the interface to the RCM
    if (rcmIfInit(rcmIf, argv[2]) != OK)
    {
        printf("Initialization failed.\n");
        exit(0);
    }

	// Put radio in RCM mode and set Sleep mode to Idle to keep extraneous
	//      messages from coming in while getting and setting configuration

    // Put radio in Idle
    if (rcmSleepModeSet(RCRM_SLEEP_MODE_IDLE) != 0)
    {
        printf("Time out waiting for sleep mode set.\n");
        exit(0);
    }

    // Make sure opmode is RCM
    if (rcmOpModeGet(&initOpMode) != 0)
    {
        printf("Time out waiting for opmode get.\n");
        exit(0);
    }
	if (initOpMode != RCRM_OPMODE_RCM)
		if (rcmOpModeSet(RCRM_OPMODE_RCM) != 0)
		{
			printf("Time out waiting for opmode set.\n");
			exit(0);
		}

    // execute Built-In Test - verify that radio is healthy
    if (rcmBit(&status) != 0)
    {
        printf("Time out waiting for BIT.\n");
        exit(0);
    }

    if (status != OK)
    {
        printf("Built-in test failed - status %d.\n", status);
        exit(0);
    }
    else
    {
        printf("Radio passes built-in test.\n\n");
    }

    // retrieve status info from RCM
    if (rcmStatusInfoGet(&statusInfo) != 0)
    {
        printf("Time out waiting for status info confirm.\n");
        exit(0);
    }

    // print out status info
    printf("\nStatus Info:\n");
	printf("\tPackage ID: %s\n", statusInfo.packageVersionStr);
    printf("\tRCM version: %d.%d build %d\n", statusInfo.appVersionMajor,
            statusInfo.appVersionMinor, statusInfo.appVersionBuild);
    printf("\tUWB Kernel version: %d.%d build %d\n", statusInfo.uwbKernelVersionMajor,
            statusInfo.uwbKernelVersionMinor, statusInfo.uwbKernelVersionBuild);
    printf("\tFirmware version: %x/%x/%x ver %X\n", statusInfo.firmwareMonth,
            statusInfo.firmwareDay, statusInfo.firmwareYear,
            statusInfo.firmwareVersion);
    printf("\tSerial number: %08X\n", statusInfo.serialNum);
    printf("\tBoard revision: %c\n", statusInfo.boardRev);
    printf("\tTemperature: %.2f degC\n\n", statusInfo.temperature/4.0);

    // retrieve RCM config
    if (rcmConfigGet(&rcmConfig) != 0)
    {
        printf("Time out waiting for rcmConfig confirm.\n");
        exit(0);
    }

    // print out configuration
    printf("Initial RCM Configuration:\n");
	printRcmConfig(&rcmConfig);

    // retrieve RN config
    if (rnConfigGet(&rnConfig) != 0)
    {
        printf("Time out waiting for rnConfig confirm.\n");
        exit(0);
    }

    // print out configuration
    printf("\nInitial RN Configuration:\n");
	printRnConfig(&rnConfig);

    // retrieve ALOHA config
    if (rnAlohaConfigGet(&rnAlohaConfig) != 0)
    {
        printf("Time out waiting for rnAlohaConfig confirm.\n");
        exit(0);
    }

    // print out configuration
    printf("\nInitial RN ALOHA Configuration:\n");
	printRnAlohaConfig(&rnAlohaConfig);

	// Configure radio for sample app use
	//
	printf("\n-------------------------------------------------------------------------\n");
	printf("Modifying Configuration for Sample App use...\n");

	// Start with RCM configuration
	//
	rcmInitConfig = rcmConfig;	// Save initial configuration to restore when done
	// Clear ELR
	rcmConfig.flags &= ~(1 << 7);
	// Clear Scans
	rcmConfig.flags &= ~(1 << 0);
	// Clear Full Scans
	rcmConfig.flags &= ~(1 << 1);
	// Disable CRE Ranges (note: setting the bit disables the sending of CREs)
	rcmConfig.flags |= (1 << 4);

	// Set RCM config
    if (rcmConfigSet(&rcmConfig) != 0)
    {
        printf("Time out waiting for rcmConfig confirm.\n");
        exit(0);
    }

	// Print out modified RCM Config
	printf("\nModified RCM Configuration\n");
	printRcmConfig(&rcmConfig);

	// Moving on to RangeNet configuration
	//
	rnInitConfig = rnConfig;	// Save initial configuration to restore when done
	// Clear range info messages
	rnConfig.autosendType &= ~(3 << 0);
	// Disable sending NDB
	rnConfig.autosendType &= ~(3 << 2);
	// Ensure Do Not Range To Me flag is not set
	rnConfig.rnFlags &= ~(1 << 1);
	// Set the NDB Neighbor Age arbitrarily high to keep nodes from dropping out
	rnConfig.maxNeighborAgeMs = 999999;
	// Let's do a NDB update rate of 1 sec
	rnConfig.autosendNeighborDbUpdateIntervalMs = 1000;

    // Set RangeNet config
    if (rnConfigSet(&rnConfig) != 0)
    {
        printf("Time out waiting for rnConfig confirm.\n");
        exit(0);
    }

	// Print out modified RN Config
	printf("\nModified RN Configuration\n");
	printRnConfig(&rnConfig);

	// Finally set up ALOHA configuration
	//
	rnInitAlohaConfig = rnAlohaConfig;	// Save initial configuration to restore when done
	// Ensure Beacon mode is not set
	rnAlohaConfig.flags &= ~(1 << 0);
	// Turn off ACC
	rnAlohaConfig.flags &= ~(1 << 2);
	// With ACC off, set Min and Max TX
	rnAlohaConfig.minTimeBetweenTxMs = 250;
	rnAlohaConfig.maxTimeBetweenTxMs = 1250;
	// Let's not have any data this time around
	rnAlohaConfig.maxRequestDataSize = rnAlohaConfig.maxResponseDataSize = 0;

    // Set ALOHA config
    if (rnAlohaConfigSet(&rnAlohaConfig) != 0)
    {
        printf("Time out waiting for rnAlohaConfig confirm.\n");
        exit(0);
    }

	// Print out modified Aloha Config
	printf("\nModified RN ALOHA Configuration\n");
	printRnAlohaConfig(&rnAlohaConfig);

    // enter loop ranging to a node and broadcasting the resulting range
	printf("\n-------------------------------------------------------------------------\n");
    while (1)
    {
        // get message option from user (or quit)
		printf("\nHere are your Options:\n");
		printf("\tR = Receive Range Info Messages\n");
		printf("\tN = Receive Neighbor Database Messages\n");
		printf("\tQ = Quit\n");
#ifdef WIN32
		printf("\nNote: When receiving messages, press any key to stop.\n");
#else
		printf("\nNote: Messages will be processed for 10 seconds.\n");
#endif
		printf("\nYour Choice:");
        fgets(msgType, sizeof(msgType), stdin);
        if (*msgType == 'q' || *msgType == 'Q')
            break;

		// Setup configuration according to response
		switch (*msgType)
		{
			// Configure for Full Range Infos...
		case 'r':
		case 'R':
			rnConfig.autosendType |= (2 << 0);
			rnConfig.autosendType &= ~(3 << 2); // Clear sending NDBs
			valid = 1;
			break;

			// Configure for Full Neighbor DBs...
		case 'n':
		case 'N':
			rnConfig.autosendType |= (1 << 2);
			rnConfig.autosendType &= ~(3 << 0); // Clear sending Range Infos
			valid = 1;
			break;

		default:
			printf("\nPlease enter a valid response.\n");
			valid = 0;
		}

		if (valid) {
			// get the RCM up and running

			// Update configuration
			if (rnConfigSet(&rnConfig) != 0)
			{
				printf("Time out waiting for rnConfig confirm.\n");
				exit(0);
			}
			// Put in RangeNet Mode and set Sleep Mode to Active
			if (rcmSleepModeSet(RCRM_SLEEP_MODE_ACTIVE) != 0)
			{
				printf("Time out waiting for sleep mode set.\n");
				exit(0);
			}
			if (rcmOpModeSet(RCRM_OPMODE_RN) != 0)
			{
				printf("Time out waiting for opmode set.\n");
				exit(0);
			}

			// Flush any pending messages
			rcmIfFlush();

			// Section break for incoming messages
			printf("\n-------------------------------------------------------------------------\n");
			printf("Processing Messages...\n");
		}

		// Git 'er done!
#ifndef WIN32
		time_t startTime;
		startTime = time(NULL);
		while ( ((time(NULL) - startTime) <= 10) && (valid) ) {
#else
        while (valid) {
#endif
			// Look for message and process
			switch(rcmInfoGet(&rangeInfo, &ndbInfo))
			{
				case RANGEINFO:
				{
					printf("\nRANGE_INFO: Responder ID %d   Message ID %u\n", rangeInfo.responderId, rangeInfo.msgId);
					printf("Range Status %d   Stopwatch %d ms\n", rangeInfo.rangeStatus, rangeInfo.stopwatchTime);
					printf("Noise %d   vPeak %d   Measurement Type %d\n", rangeInfo.noise, rangeInfo.vPeak,
							rangeInfo.rangeMeasurementType);

					// The RANGE_INFO can provide multiple types of ranges
					if (rangeInfo.rangeMeasurementType & RCM_RANGE_TYPE_PRECISION)
					{
						printf("Precision Range: %d mm, Error Estimate %d mm\n",
								rangeInfo.precisionRangeMm, rangeInfo.precisionRangeErrEst);
					}

					if (rangeInfo.rangeMeasurementType & RCM_RANGE_TYPE_COARSE)
					{
						printf("Coarse Range: %d mm, Error Estimate %d mm\n",
								rangeInfo.coarseRangeMm, rangeInfo.coarseRangeErrEst);
					}

					if (rangeInfo.rangeMeasurementType & RCM_RANGE_TYPE_FILTERED)
					{
						printf("Filtered Range: %d mm, Error Estimate %d mm\n",
								rangeInfo.filteredRangeMm, rangeInfo.filteredRangeErrEst);
						printf("Filtered Velocity: %d mm/s, Error Estimate %d mm/s\n",
								rangeInfo.filteredRangeVel, rangeInfo.filteredRangeVelErrEst);
					}
					break;
				}
				case FULLNDB:
				{
					printf("\nFULL_NEIGHBOR_DATABASE: Message ID %u   Number of Neighbors: %u   Sort Type %u\n",
						ndbInfo.msgId, ndbInfo.numNeighborEntries, ndbInfo.sortType);
					for (i = 0; i < ndbInfo.numNeighborEntries; i++)
					{
						printf("Node %u: Range %d mm   LED Flags 0x%X   vPeak %d   Last Heard %d ms\n",
							ndbInfo.neighbors[i].nodeId, ndbInfo.neighbors[i].rangeMm,
							ndbInfo.neighbors[i].ledFlags, ndbInfo.neighbors[i].vPeak,
							ndbInfo.timestamp - ndbInfo.neighbors[i].lastHeardTimestampMs);
					}
					break;
				}
			} // switch

#ifdef WIN32
			if (_kbhit()) {
				keyHit = _getch();

				valid = 0;
			}
#endif
		} // while (valid)

		// Do some cleanup
		// Put in RCM Mode and set Sleep Mode to Idle
		if (rcmSleepModeSet(RCRM_SLEEP_MODE_IDLE) != 0)
		{
			printf("Time out waiting for sleep mode set.\n");
			exit(0);
		}
		if (rcmOpModeSet(RCRM_OPMODE_RCM) != 0)
		{
			printf("Time out waiting for opmode set.\n");
			exit(0);
		}

		// Flush any pending messages
		rcmIfFlush();

    } // while (1)

	// All done. Restore original configuration
	printf("\nAll done. Restoring original configuration...\n");

	// Set RCM config
    if (rcmConfigSet(&rcmInitConfig) != 0)
    {
        printf("Time out waiting for rcmConfig confirm.\n");
        exit(0);
    }
    // Set RangeNet config
    if (rnConfigSet(&rnInitConfig) != 0)
    {
        printf("Time out waiting for rnConfig confirm.\n");
        exit(0);
    }
    // Set ALOHA config
    if (rnAlohaConfigSet(&rnInitAlohaConfig) != 0)
    {
        printf("Time out waiting for rnAlohaConfig confirm.\n");
        exit(0);
    }
	// Set opMode
	if (initOpMode != RCRM_OPMODE_RCM)
		if (rcmOpModeSet(initOpMode) != 0)
		{
			printf("Time out waiting for opmode set.\n");
			exit(0);
		}
    // Finally, put radio back into Active mode
    if (rcmSleepModeSet(RCRM_SLEEP_MODE_ACTIVE) != 0)
    {
        printf("Time out waiting for sleep mode set.\n");
        exit(0);
    }
	// perform cleanup
    rcmIfClose();
    return 0;
}

