//__________________________________________________________________________________________________
//
// Copyright 2013-4 Time Domain Corporation
//
//
// rnTdmaApp.c
//
//   Sample code showing how to interface to P4xx RCM module embedded RangeNet software. This
//      code demonstrates how to setup and run 2 - 4 radios for TDMA operation.
//
//   This code uses the functions in rcm.c and rn.c to:
//      - Connect to radios and put in idle state to easily change configurations
//      - Get the base and RangeNet configurations as well as the TDMA Slotmap to use as a baseline
//      - Modify base configurations for sample app use
//		- Build homongenous slot map
//		- Load modified configurations and slotmap onto radios
//      - Allow user to start network or quit
//		- Display received neighbor database until either a key is hit (Windows) or 10 sec elapses (linux)
//      - Restore original configuration upon exit
//
//   This sample can communicate with the RCM over Ethernet, the 3.3V serial port,
//   or the USB interface (which acts like a serial port).
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
#include <Windows.h>
#else // linux
#include <time.h>
#include <unistd.h>
#endif

#include "rnTdmaApp.h"
#include "rcmIf.h"
#include "rcm.h"
#include "rn.h"

//_____________________________________________________________________________
//
// #includes 
//_____________________________________________________________________________

#define MAX_RADIOS	4

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
	printf("This sample app demonstrates configuring and running a TDMA slot map on 2 - 4 radios.\n");
	printf("  USAGE: rnTdmaApp -i <IP address 1> <IP address 2> [<IP address 3> <IP address 4>] |\n");
	printf("                   -s <COM port 1> <COM port 2> [<COM port 3> <COM port 4>] |\n");
#ifdef WIN32
	printf("                   -u <USB COM port 1> <USB COM port 2> [<USB COM port 3> <USB COM port 4>]\n");
#else
	printf("                   -u <USB Node ID 1> <USB Node ID 2> [<USB Node ID 3> <USB Node ID 4>]\n");
#endif
    printf("\nTo use radios connected by Ethernet:\n");
    printf("\trnTdmaApp -i 192.168.1.100 192.168.1.101 192.168.1.102\n");
    printf("\nTo connect to radios using serial port USB-to-TTL converters:\n");
    printf("\trnTdmaApp -s COM3 COM4\n");
    printf("\nTo connect to radios via USB ports:\n");
#ifdef WIN32
    printf("\trnTdmaApp -u COM3 COM4 COM7 COM8\n");
#else
    printf("\trnTdmaApp -u 100 101 104 105\n");
#endif
	exit(0);
}

//_____________________________________________________________________________
//
// printRcmConfig - prints base configuration
//_____________________________________________________________________________

void printRcmConfig(rcmConfiguration *rcmConfig)
{
    printf("       Pii : %d\n", rcmConfig->integrationIndex);
    printf("   Antenna : %d\n", rcmConfig->antennaMode);
    printf("   Channel : %d\n", rcmConfig->codeChannel);
    printf("     Flags : 0x%X\n", rcmConfig->flags);
	printf("   TX Gain : %d\n", rcmConfig->txGain);
}

//_____________________________________________________________________________
//
// printRnConfig - prints RangeNet configuration
//_____________________________________________________________________________

void printRnConfig(rnConfiguration *rnConfig)
{
    printf("    Neighbor Age : %d ms\n", rnConfig->maxNeighborAgeMs);
    printf("    NDB Interval : %d ms\n", rnConfig->autosendNeighborDbUpdateIntervalMs);
    printf("           Flags : 0x%X\n", rnConfig->rnFlags);
    printf("    Network Mode : %d\n", rnConfig->networkSyncMode);
    printf("   Autosend Type : 0x%X\n", rnConfig->autosendType);
}

//_____________________________________________________________________________
//
// printRnSlotmap - prints a TDMA Slotmap
//_____________________________________________________________________________

void printRnSlotmap(rnMsg_SetTDMASlotmapRequest *rnSlotmap)
{
	int i;

	for (i = 0; i < rnSlotmap->numSlots; i++)
	{
		printf("SLOT %02d: ", rnSlotmap->slots[i].slotNumber);
		printf("Req ID:%u   ", rnSlotmap->slots[i].requesterId);
		printf("Rsp ID:%u   ", rnSlotmap->slots[i].responderId);
		printf("Pii:%u   ", rnSlotmap->slots[i].integrationIndex);
		printf("Channel:%u   ", rnSlotmap->slots[i].codeChannel);
		printf("Antenna:%u   ", rnSlotmap->slots[i].antennaMode);
		printf("Flags:0x%X   ", rnSlotmap->slots[i].flags);
		printf("Type:%u   ", rnSlotmap->slots[i].slotType);
		printf("Man Time:%u\n", rnSlotmap->slots[i].requestedDurationMicroseconds);
	}
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
	int valid = 0;
	int i, j;
	int initOpMode[MAX_RADIOS];
	int numRadios = 0;
	int slotNum = 0;
    rcmIfType rcmIf;

	// Configuration structures
    rcmConfiguration rcmInitConfig[MAX_RADIOS], rcmConfig[MAX_RADIOS];
	rnConfiguration rnInitConfig[MAX_RADIOS], rnConfig[MAX_RADIOS];
	rnMsg_GetTDMASlotmapConfirm rnInitTdmaSlotMap[MAX_RADIOS];
	rnMsg_SetTDMASlotmapRequest rnTdmaSlotMap[MAX_RADIOS];

	// Info message structures
    rcmMsg_FullRangeInfo rangeInfo;
	rnMsg_GetFullNeighborDatabaseConfirm ndbInfo;

	printf("\n\nRangeNet TDMA Sample App\n\n");

    // Check command line arguments
    if ( (argc < 4) || (argc > 6) )
        usage();

	// Get number of radios on command line
	numRadios = argc - 2;

	// check for interface type
    if (!strcmp(argv[1], "-i"))
        rcmIf = rcmIfIp;
    else if (!strcmp(argv[1], "-s"))
        rcmIf = rcmIfSerial;
    else if (!strcmp(argv[1], "-u"))
        rcmIf = rcmIfUsb;
    else
        usage();

	// Put each radio in IDLE and RCM mode and get their configurations
	for (i = 0; i < numRadios; i++) {

		// Connect (open port) to the radio
		if (rcmIfInit(rcmIf, argv[i+2]) != OK)
		{
			printf("Could not connect to radio (%s).\n", argv[i+2]);
			exit(0);
		}
#ifndef WIN32
		sleep(1);	// Need delay due to slow port (USB) handling from LINUX
#endif
		// Put radio in RCM mode and set Sleep mode to Idle to keep extraneous
		//      messages from coming in while getting and setting configuration

		// Put radio in Idle
		if (rcmSleepModeSet(RCRM_SLEEP_MODE_IDLE) != 0)
		{
			printf("Time out waiting for sleep mode set.\n");
			exit(0);
		}

		// Make sure opmode is RCM
		if (rcmOpModeGet(&initOpMode[i]) != 0)
		{
			printf("Time out waiting for opmode get.\n");
			exit(0);
		}
		if (initOpMode[i] != RCRM_OPMODE_RCM)
			if (rcmOpModeSet(RCRM_OPMODE_RCM) != 0)
			{
				printf("Time out waiting for opmode set.\n");
				exit(0);
			}

		// Retrieve all the initial configurations
		//   Will use these as a baseline for TDMA configuration.
		//   Initial configurations will be reset when sample app
		//   is done.
		//

		// Retrieve RCM config
		if (rcmConfigGet(&rcmInitConfig[i]) != 0)
		{
			printf("Time out waiting for rcmConfig confirm.\n");
			exit(0);
		}

		// Retrieve RN config
		if (rnConfigGet(&rnInitConfig[i]) != 0)
		{
			printf("Time out waiting for rnConfig confirm.\n");
			exit(0);
		}

		// Retrieve TDMA slot map
		if (rnTdmaSlotMapGet(&rnInitTdmaSlotMap[i]) != 0)
		{
			printf("Time out waiting for rnTdmaSlotMap confirm.\n");
			exit(0);
		}

		// Give some feedback
		printf("Retrieved configuration from Node ID %u\n", rcmInitConfig[i].nodeId);

		rcmIfClose();
	}

	// Modify baseline configuration for sample app use
	//
	printf("\n-------------------------------------------------------------------------\n");
	printf("Modifying Configuration for Sample App use...\n");

	// Just change one config and update others later

	// Start with RCM configuration
	//
	rcmConfig[0] = rcmInitConfig[0];	// Copy initial config to use as baseline
	// Clear ELR
	rcmConfig[0].flags &= ~(1 << 7);
	// Clear Scans
	rcmConfig[0].flags &= ~(1 << 0);
	// Clear Full Scans
	rcmConfig[0].flags &= ~(1 << 1);
	// Disable CRE Ranges (note: setting the bit disables the sending of CREs)
	rcmConfig[0].flags |= (1 << 4);

	// Print out modified RCM Config
	printf("\nModified RCM Configuration\n");
	printRcmConfig(&rcmConfig[0]);

	// Moving on to RangeNet configuration
	//
	rnConfig[0] = rnInitConfig[0];	// Copy initial config to use as baseline
	// Set network mode to TDMA
	rnConfig[0].networkSyncMode = 1;
	// Clear range info messages
	rnConfig[0].autosendType &= ~(3 << 0);
	// Disable sending NDB
	rnConfig[0].autosendType &= ~(3 << 2);
	// Ensure Do Not Range To Me flag is not set
	rnConfig[0].rnFlags &= ~(1 << 1);
	// Set the NDB Neighbor Age arbitrarily high to keep nodes from dropping out
	rnConfig[0].maxNeighborAgeMs = 999999;
	// Let's do a NDB update rate of 1 sec
	rnConfig[0].autosendNeighborDbUpdateIntervalMs = 1000;

	// Print out modified RN Config
	printf("\nModified RN Configuration\n");
	printRnConfig(&rnConfig[0]);

	// Finally build the slot map where every radio ranges to all other radios
	//
	rnTdmaSlotMap[0].numSlots = numRadios * (numRadios - 1);
	slotNum = 0;
	for (i = 0; i < numRadios; i++)
		for (j = 0; j < numRadios; j++)
			if (i != j) {
				rnTdmaSlotMap[0].slots[slotNum].slotNumber = slotNum;
				rnTdmaSlotMap[0].slots[slotNum].slotType = 1;
				rnTdmaSlotMap[0].slots[slotNum].requesterId = rcmInitConfig[i].nodeId;
				rnTdmaSlotMap[0].slots[slotNum].responderId = rcmInitConfig[j].nodeId;
				rnTdmaSlotMap[0].slots[slotNum].antennaMode = 0;
				rnTdmaSlotMap[0].slots[slotNum].integrationIndex = 7;
				rnTdmaSlotMap[0].slots[slotNum].codeChannel = 4;
				rnTdmaSlotMap[0].slots[slotNum].flags = 0;
				rnTdmaSlotMap[0].slots[slotNum].requestedDurationMicroseconds = 0;
				slotNum++;
			}

	// We want to configure only the clock master to send Full NDB messages
	rnConfig[0].autosendType |= (1 << 2);

	// Copy the config to all radios
	for (i = 1; i < numRadios; i++)
	{
		rcmConfig[i] = rcmConfig[0];
		rcmConfig[i].nodeId = rcmInitConfig[i].nodeId;	// Keep from setting Node ID all the same
		rnConfig[i] = rnConfig[0];
		rnTdmaSlotMap[i] = rnTdmaSlotMap[0];
	}

	// Print slot map
	printf("\n-------------------------------------------------------------------------\n");
	printf("Modified slotmap. Number of Slots = %d\n\n", rnTdmaSlotMap[0].numSlots);
	printRnSlotmap(&rnTdmaSlotMap[0]);

	// Update all the radios and put in RangeNet and Active mode
	//   except for the clock master (first radio)
	for (i = 0; i < numRadios; i++)
	{
		// Connect (open port) to the radio
		if (rcmIfInit(rcmIf, argv[i+2]) != OK)
		{
			printf("Could not connect to radio (%s).\n", argv[i+2]);
			exit(0);
		}

		// Set RCM config
	    if (rcmConfigSet(&rcmConfig[i]) != 0)
	    {
	        printf("Time out waiting for rcmConfig confirm.\n");
	        exit(0);
	    }

	    // Set RangeNet config
	    if (rnConfigSet(&rnConfig[i]) != 0)
	    {
	        printf("Time out waiting for rnConfig confirm.\n");
	        exit(0);
	    }

		// Set TDMA Slotmap config
	    if (rnTdmaSlotMapSet(&(rnTdmaSlotMap[i])) != 0)
	    {
	        printf("Time out waiting for rnTdmaSlotMapSet confirm.\n");
	        exit(0);
	    }

		if (i != 0)
		{
			// Put radio in Idle
			if (rcmSleepModeSet(RCRM_SLEEP_MODE_ACTIVE) != 0)
			{
				printf("Time out waiting for sleep mode set.\n");
				exit(0);
			}

			// Make sure opmode is RangeNet
			if (rcmOpModeSet(RCRM_OPMODE_RN) != 0)
			{
				printf("Time out waiting for opmode set.\n");
				exit(0);
			}
		}

		// Close connection to radio
		rcmIfClose();
	}

	// Connect to clock master and leave open until user decides to quit.
	//
	if (rcmIfInit(rcmIf, argv[2]) != OK)
	{
		printf("Could not connect to clock master (%s).\n", argv[2]);
		exit(0);
	}

    // This is user input control loop
	printf("\n-------------------------------------------------------------------------\n");
	printf("\nThe radios have been configured and slotmap loaded.\n");
    while (1)
    {
        // Get option from user (or quit)
		printf("\nHere are your Options:\n");
		printf("\tS = Start Network and Receive Neighbor Database Messages\n");
		printf("\tQ = Quit\n");
#ifdef WIN32
		printf("\n   NOTE: When receiving messages, press any key to stop.\n");
#else
		printf("\n   NOTE: Messages will be processed for 10 seconds.\n");
#endif
		printf("\nYour Choice:");
        fgets(msgType, sizeof(msgType), stdin);
        if (*msgType == 'q' || *msgType == 'Q')
            break;

		// Setup configuration according to response
		switch (*msgType)
		{
			// Start Network...
		case 's':
		case 'S':
			valid = 1;
			break;

		default:
			printf("\nPlease enter a valid response.\n");
			valid = 0;
		}

		// This puts the clock master into RangeNet and Active modes effectively starting the network
		if (valid) {

			// Put in RangeNet Mode and set Sleep Mode to Active
			if (rcmOpModeSet(RCRM_OPMODE_RN) != 0)
			{
				printf("Time out waiting for opmode set.\n");
				exit(0);
			}
			if (rcmSleepModeSet(RCRM_SLEEP_MODE_ACTIVE) != 0)
			{
				printf("Time out waiting for sleep mode set.\n");
				exit(0);
			}

#ifdef WIN32
			Sleep(250);
#else
			sleep(1);
#endif
			// Reset neighbor database stats to get a clean set
			if (rnResetNDBStats() != 0)
			{
				printf("Time out waiting for ResetNDBStats.\n");
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
				case FULLNDB:
				{
					printf("\nFULL_NEIGHBOR_DATABASE: Message ID %u   Number of Neighbors: %u\n",
						ndbInfo.msgId, ndbInfo.numNeighborEntries);
					for (i = 0; i < ndbInfo.numNeighborEntries; i++)
					{
						printf("Node %u: Range %d mm   Range Attempts %d   Rate %.2f Hz  Num Successful %d   Success %.1f%%\n",
							ndbInfo.neighbors[i].nodeId, ndbInfo.neighbors[i].rangeMm,
							ndbInfo.neighbors[i].statsNumRangeAttempts,
							(double)ndbInfo.neighbors[i].statsNumRangeAttempts / ((double)ndbInfo.neighbors[i].statsAgeMs / 1000.0),
							ndbInfo.neighbors[i].statsNumRangeSuccesses,
							(ndbInfo.neighbors[i].statsNumRangeAttempts == 0) ?
								0 : ((double)ndbInfo.neighbors[i].statsNumRangeSuccesses / (double)ndbInfo.neighbors[i].statsNumRangeAttempts) * 100.0);
					}
					break;
				}

				default:
					break;

			} // switch

#ifdef WIN32
			if (_kbhit()) {
				keyHit = _getch();

				valid = 0;
			}
#endif
		} // while (valid)

		// Put in RCM Mode and set Sleep Mode to Idle before going back to user options
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

	// Close connection to clock master
    rcmIfClose();

	// All done. Restore original configuration
	printf("\nRestoring original configuration...\n\n");

	for (i = 0; i < numRadios; i++)
	{
		printf("Restoring Node ID %u\n", rcmInitConfig[i].nodeId);
		
		// Connect (open port) to the radio
		if (rcmIfInit(rcmIf, argv[i+2]) != OK)
		{
			printf("Could not connect to radio (%s).\n", argv[i+2]);
			exit(0);
		}
#ifndef WIN32
		sleep(1);	// Need delay due to slow port (USB) handling from LINUX
#endif
		// Flush any pending messages
		rcmIfFlush();

		// Set opMode to RCM to perform updates
		if (rcmOpModeSet(RCRM_OPMODE_RCM) != 0)
		{
			printf("Time out waiting for opmode set to RCM mode.\n");
			exit(0);
		}

		// Put radio back in Idle mode to perform updates
		if (rcmSleepModeSet(RCRM_SLEEP_MODE_IDLE) != 0)
		{
			printf("Time out waiting for sleep mode set.\n");
			exit(0);
		}

		// Set RCM config
		if (rcmConfigSet(&rcmInitConfig[i]) != 0)
		{
			printf("Time out waiting for rcmConfig confirm.\n");
			exit(0);
		}

		// Set RangeNet config
		if (rnConfigSet(&rnInitConfig[i]) != 0)
		{
			printf("Time out waiting for rnConfig confirm.\n");
			exit(0);
		}

		// Restore slot map
		//   Have to convert rnInitTdmaSlotMap from a request structure to a confirm structure
		//   Reuse the rnTdmaSlotMap structure array
		rnTdmaSlotMap[i].numSlots = rnInitTdmaSlotMap[i].numSlots;
		for (j = 0; j < rnTdmaSlotMap[i].numSlots; j++) 
			rnTdmaSlotMap[i].slots[j] = rnInitTdmaSlotMap[i].slots[j].slot;
	    if (rnTdmaSlotMapSet(&(rnTdmaSlotMap[i])) != 0)
	    {
	        printf("Time out waiting for rnTdmaSlotMapSet confirm.\n");
	        exit(0);
	    }

		// Restore opMode
		if (initOpMode[i] != RCRM_OPMODE_RCM)
			if (rcmOpModeSet(initOpMode[i]) != 0)
			{
				printf("Time out waiting to restore opmode.\n");
				exit(0);
			}

		// Finally, put radio back into Active mode
		if (rcmSleepModeSet(RCRM_SLEEP_MODE_ACTIVE) != 0)
		{
			printf("Time out waiting for sleep mode set.\n");
			exit(0);
		}
		
		// Close connection to radio
		rcmIfClose();
	}
	
	printf("\nAll done.\n\n");
	return 0;
}

