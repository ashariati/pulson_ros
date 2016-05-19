///////////////////////////////////////
//
// hostInterface.h
//
// Definitions for the interface between a host computer and the embedded RCM.
//
// Copyright (c) 2010-2015 Time Domain
//

#ifndef __rcmHostInterfaceCommon_h
#define __rcmHostInterfaceCommon_h

// Portability

// The section "#ifdef _MSC_VER" was copied from http://msinttypes.googlecode.com/svn/trunk/stdint.h,
// an implementation of stdint.h for Microsoft Visual C/C++ versions earlier than Visual Studio 2010.
#ifdef _MSC_VER

// Visual Studio 6 and Embedded Visual C++ 4 doesn't
// realize that, e.g. char has the same size as __int8
// so we give up on __intX for them.
#if (_MSC_VER < 1300)
typedef signed char       rcm_int8_t;
typedef signed short      rcm_int16_t;
typedef signed int        rcm_int32_t;
typedef unsigned char     rcm_uint8_t;
typedef unsigned short    rcm_uint16_t;
typedef unsigned int      rcm_uint32_t;
#else
typedef signed __int8     rcm_int8_t;
typedef signed __int16    rcm_int16_t;
typedef signed __int32    rcm_int32_t;
typedef unsigned __int8   rcm_uint8_t;
typedef unsigned __int16  rcm_uint16_t;
typedef unsigned __int32  rcm_uint32_t;
#endif

typedef signed __int64       rcm_int64_t;
typedef unsigned __int64     rcm_uint64_t;

#else

typedef	__signed char			rcm_int8_t;
typedef	unsigned char			rcm_uint8_t;
typedef	short					rcm_int16_t;
typedef	unsigned short			rcm_uint16_t;
typedef	int						rcm_int32_t;
typedef	unsigned int			rcm_uint32_t;
typedef	long long				rcm_int64_t;
typedef	unsigned long long		rcm_uint64_t;

#endif


// Socket defines
#define RCM_SOCKET_PORT_NUM  21210


// Internal modes of operation - not all are supported
#define RCM_OPMODE_RCM		0
#define RCM_OPMODE_DEFAULT	RCM_OPMODE_RCM


// P400 sleep modes
#define RCM_SLEEP_MODE_ACTIVE          0
#define RCM_SLEEP_MODE_IDLE            1
#define RCM_SLEEP_MODE_STANDBY_ETH     2 // wakeup via ethernet or serial
#define RCM_SLEEP_MODE_STANDBY_SER     3 // wakeup via serial only
#define RCM_SLEEP_MODE_SLEEP           4 // wakeup via GPIO only

///////////////////////////////////////
//
// Message types
//

// REQUEST messages are sent by the host to the embedded applicaion.
// CONFIRM messages are sent by the embedded application to the host in response to REQUEST messages.
// INFO messages are sent automatically by the embedded application to the host when various events occur.
#define RCRM_MSG_TYPE_REQUEST			(0xF000)
#define RCRM_MSG_TYPE_CONFIRM			(0xF100)
#define RCRM_MSG_TYPE_INFO				(0xF200)


///////////////////////////////////////
//
// Host <-> Embedded conversation messages
//

// Get version and temperature info
#define RCM_GET_STATUS_INFO_REQUEST	(RCRM_MSG_TYPE_REQUEST + 1)
#define RCM_GET_STATUS_INFO_CONFIRM	(RCRM_MSG_TYPE_CONFIRM + 1)


// Reboot P400
#define RCM_REBOOT_REQUEST				(RCRM_MSG_TYPE_REQUEST + 2)
#define RCM_REBOOT_CONFIRM				(RCRM_MSG_TYPE_CONFIRM + 2)

// Set opmode
#define RCM_SET_OPMODE_REQUEST			(RCRM_MSG_TYPE_REQUEST + 3)
#define RCM_SET_OPMODE_CONFIRM			(RCRM_MSG_TYPE_CONFIRM + 3)

// Get opmode
#define RCM_GET_OPMODE_REQUEST			(RCRM_MSG_TYPE_REQUEST + 4)
#define RCM_GET_OPMODE_CONFIRM			(RCRM_MSG_TYPE_CONFIRM + 4)

// Set sleep mode
#define RCM_SET_SLEEP_MODE_REQUEST		(RCRM_MSG_TYPE_REQUEST + 5)
#define RCM_SET_SLEEP_MODE_CONFIRM		(RCRM_MSG_TYPE_CONFIRM + 5)

// Get sleep mode
#define RCM_GET_SLEEP_MODE_REQUEST		(RCRM_MSG_TYPE_REQUEST + 6)
#define RCM_GET_SLEEP_MODE_CONFIRM		(RCRM_MSG_TYPE_CONFIRM + 6)

// Get ambient samples
#define RCM_GET_AMBIENT_SAMPLES_REQUEST	(RCRM_MSG_TYPE_REQUEST + 7)
#define RCM_GET_AMBIENT_SAMPLES_CONFIRM	(RCRM_MSG_TYPE_CONFIRM + 7)

// Execute Built-In Test
#define RCM_BIT_REQUEST	            (RCRM_MSG_TYPE_REQUEST + 8)
#define RCM_BIT_CONFIRM	            (RCRM_MSG_TYPE_CONFIRM + 8)

// Get cause of last boot
#define RCM_GET_LAST_BOOT_CAUSE_REQUEST	(RCRM_MSG_TYPE_REQUEST + 9)
#define RCM_GET_LAST_BOOT_CAUSE_CONFIRM	(RCRM_MSG_TYPE_CONFIRM + 9)

// Get serial baud rate
#define RCM_GET_SERIAL_BAUD_RATE_REQUEST	(RCRM_MSG_TYPE_REQUEST + 10)
#define RCM_GET_SERIAL_BAUD_RATE_CONFIRM	(RCRM_MSG_TYPE_CONFIRM + 10)

// Set serial baud rate
#define RCM_SET_SERIAL_BAUD_RATE_REQUEST	(RCRM_MSG_TYPE_REQUEST + 11)
#define RCM_SET_SERIAL_BAUD_RATE_CONFIRM	(RCRM_MSG_TYPE_CONFIRM + 11)

// Unrecognized message type was received, or the message was the wrong size for the message type
#define RCM_INVALID_MESSAGE_CONFIRM		(RCRM_MSG_TYPE_CONFIRM + 12)

///////////////////////////////////////
//
// Common INFO messages to the host
//

// Sent when a waveform scan is completed.
// If the complete scan doesn't fit in a single packet, multiple packets are sent which can be combined to form the complete scan.
#define RCM_FULL_SCAN_INFO				(RCRM_MSG_TYPE_INFO + 1)

// Sent when the radio is ready to receive commands.
// Sent after a power up, reboot, or wakeup from sleep event is complete.
#define RCM_READY_INFO				    (RCRM_MSG_TYPE_INFO + 2)



///////////////////////////////////////
//
// Constants and flags
//


// *_CONFIRM message status codes
// Success
#define RCM_CONFIRM_MSG_STATUS_SUCCESS						0
// Catch-all for all uncategorized failures
#define RCM_CONFIRM_MSG_STATUS_GENERICFAILURE				1
// The REQUEST message cannot be acted upon in the current opmode
#define RCM_CONFIRM_MSG_STATUS_WRONGOPMODE					2
// The REQUEST message contained an invalid or unsupported value in one or more of its fields
#define RCM_CONFIRM_MSG_STATUS_UNSUPPORTEDVALUE			3
// The REQUEST message cannot be acted upon in the current sleep mode
#define RCM_CONFIRM_MSG_STATUS_INVALIDDURINGSLEEP			4
// The number of bytes in the REQUEST message did not match the expected number of bytes for the message type
#define RCM_CONFIRM_MSG_STATUS_WRONGMESSAGESIZE			5
// The feature used by the REQUEST message is currently disabled
#define RCM_CONFIRM_MSG_STATUS_NOTENABLED					6
// The specified size of a buffer in the REQUEST message, or the size of the buffer itself, did not make sense
#define RCM_CONFIRM_MSG_STATUS_WRONGBUFFERSIZE				7
// The REQUEST message type was not recognized
#define RCM_CONFIRM_MSG_STATUS_UNRECOGNIZEDMESSAGETYPE		8
// An internal error code was generated. This status is or'ed with the internal error code itself.
// Internal error codes are dependent on what activity the REQUEST message intended to have performed.
#define RCM_CONFIRM_MSG_STATUS_INTERNALERRORCODE			0x80000000

// Definitions of the antennaMode field in various messages

// Send RCM_ANTENNAMODE_DEFAULT in a message requiring antennaMode to use the default configured antenna
#define RCM_ANTENNAMODE_DEFAULT		0xff

#define RCM_ANTENNAMODE_TXA_RXA		0
#define RCM_ANTENNAMODE_TXB_RXB		1
#define RCM_ANTENNAMODE_TXA_RXB		2
#define RCM_ANTENNAMODE_TXB_RXA		3

#define RCM_ANTENNAMODE_TOGGLE_FLAG	(0x80)


#define RCM_MAX_SCAN_SAMPLES			(350)

// TX gain levels (inverses of the actual attenuation levels)
#define RCM_TXGAIN_MIN		0
#define RCM_TXGAIN_MAX		63


// Setting/saving configuration Persist flags.

// Update the active configuration with this new configuration info.
// The active configuration is not written to flash.
#define RCM_PERSIST_NONE				(0)

// Update the active configuration with this new configuration info.
// Write entire active configuration to flash so that it persists across reboots,
// including any not-written-to-flash changes that may have occurred (i.e. earlier set-config messages with RCM_PERSIST_NONE).
#define RCM_PERSIST_ALL				(1)

// Update the active configuration with this new configuration info.
// Write configuration to flash with the new changes.
// Earlier set config messsages with RCM_PERSIST_NONE will NOT be written to flash when this flag is set.
#define RCM_PERSIST_THIS_MSG			(2)

// (Experimental/Debug at this point)
// Update the active configuration with this new configuration info.
// Do not write the configuration to flash at this point,
// but any later set-config message with RCM_PERSIST_ALL or RCM_PERSIST_THIS_MSG will write these changes to flash.
// Intended when a series of set-config messages will be sent, but only one actual flash write is needed.
#define RCM_PERSIST_THIS_MSG_NO_WRITE	(3)

///////////////////////////////////////
//
// Message struct definitions
//



typedef struct
{
	// set to RCM_GET_STATUS_INFO_REQUEST
	rcm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcm_uint16_t msgId;
} rcmMsg_GetStatusInfoRequest;

typedef struct
{
	// set to RCM_GET_STATUS_INFO_CONFIRM
	rcm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcm_uint16_t msgId;

	rcm_uint8_t appVersionMajor;
	rcm_uint8_t appVersionMinor;
	rcm_uint16_t appVersionBuild;

	rcm_uint8_t uwbKernelVersionMajor;
	rcm_uint8_t uwbKernelVersionMinor;
	rcm_uint16_t uwbKernelVersionBuild;

	rcm_uint8_t firmwareVersion;
	rcm_uint8_t firmwareYear;
	rcm_uint8_t firmwareMonth;
	rcm_uint8_t firmwareDay;

	rcm_uint32_t serialNum;
	
	rcm_uint8_t boardRev;
	rcm_uint8_t bitResults;
	rcm_uint8_t model;
	rcm_uint8_t pulserConfig;

	// Divide this by 4 to get temperature in degrees C.
	rcm_int32_t temperature;

	char packageVersionStr[32];

	// status code
	rcm_uint32_t status;
} rcmMsg_GetStatusInfoConfirm;


typedef struct
{
	// set to RCM_REBOOT_REQUEST
	rcm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcm_uint16_t msgId;
} rcmMsg_RebootRequest;

typedef struct
{
	// set to RCM_REBOOT_CONFIRM
	rcm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcm_uint16_t msgId;
} rcmMsg_RebootConfirm;

typedef struct
{
	// set to RCM_SET_OPMODE_REQUEST
	rcm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcm_uint16_t msgId;
	
	// Requested operational mode of the P400.
	rcm_uint32_t opMode;
} rcmMsg_SetOpmodeRequest;

typedef struct
{
	// set to RCM_SET_OPMODE_CONFIRM
	rcm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcm_uint16_t msgId;
	
	// Opmode of the radio
	rcm_uint32_t opMode;

	rcm_uint32_t status;
} rcmMsg_SetOpmodeConfirm;

typedef struct
{
	// set to RCM_GET_OPMODE_REQUEST
	rcm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcm_uint16_t msgId;
} rcmMsg_GetOpmodeRequest;

typedef struct
{
	// set to RCM_GET_OPMODE_CONFIRM
	rcm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcm_uint16_t msgId;
	
	// Current operational mode of the P400.
	rcm_uint32_t opMode;
} rcmMsg_GetOpmodeConfirm;

typedef struct
{
	// set to RCM_SET_SLEEP_MODE_REQUEST
	rcm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcm_uint16_t msgId;
	
	// Requested sleep mode of the P400.
	rcm_uint32_t sleepMode;
} rcmMsg_SetSleepModeRequest;

typedef struct
{
	// set to RCM_SET_SLEEP_MODE_CONFIRM
	rcm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcm_uint16_t msgId;
	
	rcm_uint32_t status;
} rcmMsg_SetSleepModeConfirm;

typedef struct
{
	// set to RCM_GET_SLEEP_MODE_REQUEST
	rcm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcm_uint16_t msgId;
} rcmMsg_GetSleepModeRequest;

typedef struct
{
	// set to RCM_GET_SLEEP_MODE_CONFIRM
	rcm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcm_uint16_t msgId;
	
	// Current sleep mode of the P400.
	rcm_uint32_t sleepMode;
} rcmMsg_GetSleepModeConfirm;

typedef struct
{
	// set to RCM_GET_AMBIENT_SAMPLES_REQUEST
	rcm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcm_uint16_t msgId;

    // PII of samples - generally, the currently selected PII should be used
    // The radio can collect true "samples" (as opposed to integrated ramp
    // values) by setting this to 0.
	rcm_uint16_t integrationIndex;

} rcmMsg_GetAmbientSamplesRequest;

typedef struct
{
	// set to RCM_GET_AMBIENT_SAMPLES_CONFIRM
	rcm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcm_uint16_t msgId;
	
    // the ambient samples
	rcm_int32_t samples[RCM_MAX_SCAN_SAMPLES];
	
	rcm_uint32_t status;
} rcmMsg_GetAmbientSamplesConfirm;

typedef struct
{
	// set to RCM_BIT_REQUEST
	rcm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcm_uint16_t msgId;
} rcmMsg_BitRequest;

typedef struct
{
	// set to RCM_BIT_CONFIRM
	rcm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcm_uint16_t msgId;
	
    // BIT status - 0 is OK, anything else is an error
	rcm_uint32_t status;
} rcmMsg_BitConfirm;

typedef struct
{
	// set to RCM_GET_LAST_BOOT_CAUSE_REQUEST
	rcm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcm_uint16_t msgId;
} rcmMsg_GetLastBootCauseRequest;

typedef struct
{
	// set to RCM_GET_LAST_BOOT_CAUSE_CONFIRM
	rcm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcm_uint16_t msgId;
	
	// Cause of last boot
	// 0 - Normal powerup
	// 1 - Watchdog reset
	rcm_uint32_t lastBootCause;
} rcmMsg_GetLastBootCauseConfirm;


typedef struct
{
	// set to RCM_GET_SERIAL_BAUD_RATE_REQUEST
	rcm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcm_uint16_t msgId;
} rcmMsg_GetSerialBaudRateRequest;

typedef struct
{
	// set to RCM_GET_SERIAL_BAUD_RATE_CONFIRM
	rcm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcm_uint16_t msgId;
	
	// Baud rate
	rcm_uint32_t baudRate;
} rcmMsg_GetSerialBaudRateConfirm;


typedef struct
{
	// set to RCM_SET_SERIAL_BAUD_RATE_REQUEST
	rcm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcm_uint16_t msgId;

	// Set to non-zero to indicate settings should persist across radio reboots
	rcm_uint8_t persistFlag;
	// Reserved for aligment and future growth
    rcm_uint8_t reserved1;
    rcm_uint16_t reserved2;

    // baud rate
	rcm_uint32_t baudRate;
} rcmMsg_SetSerialBaudRateRequest;

typedef struct
{
	// set to RCM_SET_SERIAL_BAUD_RATE_CONFIRM
	rcm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcm_uint16_t msgId;
	
	rcm_uint32_t status;
} rcmMsg_SetSerialBaudRateConfirm;

typedef struct
{
	// set to RCM_INVALID_MESSAGE_CONFIRM
	rcm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcm_uint16_t msgId;
	
	// Message type of the unrecognized/malformed message.
	rcm_uint16_t invalidMsgType;
	// Message ID of the unrecognized/malformed message
	rcm_uint16_t invalidMsgId;

	// Indicates whether this is in response to an unrecognized message (RCM_CONFIRM_MSG_STATUS_UNRECOGNIZEDMESSAGETYPE)
	// or the request's size was incorrect (RCM_CONFIRM_MSG_STATUS_WRONGMESSAGESIZE)
	rcm_uint32_t status;
} rcmMsg_InvalidMessageConfirm;


typedef struct
{
	// set to RCM_READY_INFO
	rcm_uint16_t msgType;
	// identifier to correlate requests with info messages
	rcm_uint16_t msgId;
} rcmMsg_ReadyInfo;

typedef struct
{
	// set to RCM_FULL_SCAN_INFO
	rcm_uint16_t msgType;
	// identifier to correlate range requests with info messages
	rcm_uint16_t msgId;

	// ID of the transmitting radio
	rcm_uint32_t sourceId;

	// Milliseconds since radio boot at the time the scan was completed
	rcm_uint32_t timestamp;

	// noise
	rcm_uint16_t noise;
	// Vpeak
	rcm_uint16_t vPeak;
	
	rcm_uint32_t reserved1;	// inserted with RCM 2.0 to avoid breaking MRM 1.1

	// These are indices within the assembled scan.
	rcm_int32_t ledIndex;
	rcm_int32_t lockspotOffset;
	
	rcm_int32_t scanStartPs;
	rcm_int32_t scanStopPs;
	
	rcm_uint16_t scanStepBins;
	
	// Raw, fast time, motion, etc.
	rcm_uint8_t scanFiltering;
	
	rcm_uint8_t reserved2;	// alignment

	// Antenna the scan was received on
	rcm_uint8_t antennaId;
	
	// The type of operation behind this scan (ranging, BSR, MSR)
	rcm_uint8_t operationMode;
	
	// Number of scan samples in this message
	rcm_uint16_t numSamplesInMessage;

	// Number of samples in the entire scan
	rcm_uint32_t numSamplesTotal;

	// Index of the message in the scan
	rcm_uint16_t messageIndex;

	// Total number of RCM_FULL_SCAN_INFO messages to expect for this particular scan.
	rcm_uint16_t numMessagesTotal;

	// Scan samples.
	// Note that, unlike RCM_SCAN_INFO, this is NOT a variable-sized packet.
	rcm_int32_t scan[RCM_MAX_SCAN_SAMPLES];
} rcmMsg_FullScanInfo;

#endif	// #ifdef __rcmHostInterfaceCommon_h
