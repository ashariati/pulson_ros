//_____________________________________________________________________________
//
// Copyright 2011-2 Time Domain Corporation
//
//
// rnSampleApp.h
//
//   Declarations for RCM communications functions.
//
//_____________________________________________________________________________

#ifndef __rnSampleApp_h
#define __rnSampleApp_h

#ifdef __cplusplus
    extern "C" {
#endif

//_____________________________________________________________________________
//
// #includes
//_____________________________________________________________________________

// pull in message structure declarations
#include "hostInterfaceCommon.h"
#include "hostInterfaceRCM.h"
#include "hostInterfaceRN.h"

//_____________________________________________________________________________
//
// #defines
//_____________________________________________________________________________

#ifndef OK
#define OK 0
#define ERR (-1)
#endif

//_____________________________________________________________________________
//
// typedefs
//_____________________________________________________________________________



//_____________________________________________________________________________
//
//  Function prototypes
//_____________________________________________________________________________


//
//  usage
//
//  Parameters:  void
//  Return:      void
//
//  Prints out the command line parameters for the sample app.
//
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
void usage(void);


//
//  printRcmConfig
//
//  Parameters:  rcmConfiguration *rcmConfig
//  Return:      void
//
//  Prints out the RCM configuration to the console. Only the parameters
//  relevant to the sample app are printed.
//
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
void printRcmConfig(rcmConfiguration *rcmConfig);


//
//  printRnConfig
//
//  Parameters:  rnConfiguration *rnConfig
//  Return:      void
//
//  Prints out the RangeNet configuration to the console. Only the parameters
//  relevant to the sample app are printed.
//
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
void printRnConfig(rnConfiguration *rnConfig);


//
//  printRnSlotmap
//
//  Parameters:  rnMsg_SetTDMASlotmapRequest *rnSlotmap
//  Return:      void
//
//  Prints out the TDMA slotmap to the console.
//
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
void printRnSlotmap(rnMsg_SetTDMASlotmapRequest *rnSlotmap);

#ifdef __cplusplus
    }
#endif


#endif
