/*
**             Copyright 2017 by Kvaser AB, Molndal, Sweden
**                         http://www.kvaser.com
**
** This software is dual licensed under the following two licenses:
** BSD-new and GPLv2. You may use either one. See the included
** COPYING file for details.
**
** License: BSD-new
** ==============================================================================
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**     * Redistributions of source code must retain the above copyright
**       notice, this list of conditions and the following disclaimer.
**     * Redistributions in binary form must reproduce the above copyright
**       notice, this list of conditions and the following disclaimer in the
**       documentation and/or other materials provided with the distribution.
**     * Neither the name of the <organization> nor the
**       names of its contributors may be used to endorse or promote products
**       derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
** AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
** ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
** LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
** CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
** SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
** BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
** IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
** ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
** POSSIBILITY OF SUCH DAMAGE.
**
**
** License: GPLv2
** ==============================================================================
** This program is free software; you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation; either version 2 of the License, or
** (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
**
**
** IMPORTANT NOTICE:
** ==============================================================================
** This source code is made available for free, as an open license, by Kvaser AB,
** for use with its applications. Kvaser AB does not accept any liability
** whatsoever for any third party patent or other immaterial property rights
** violations that may result from any usage of this source code, regardless of
** the combination of source code and various applications that it can be used
** in, or with.
**
** -----------------------------------------------------------------------------
*/

/*
** Description:
**   Definitions for the config file (PARAM.LIF) on the flash disk.
**
** -----------------------------------------------------------------------------
*/

#ifndef _LOGGER_CONFIG_FILE_H_
#define _LOGGER_CONFIG_FILE_H_

#include "compilerassert.h"

#include <stdint.h>

#define LOG_CONFIG_MAJOR_VERSION 6     // Must fit in one byte
#define LOG_CONFIG_MINOR_VERSION 0     // Must fit in one byte

#define PARAM_FILE_SIZE 320 * 32 * 1024 //10MB

/*
 * Number of trigVars, actions and statements etc.
 * Keep numbers down, affects the performance of the Memorator.
 */
#define MAX_NR_OF_TRIGVARS               32
#define MAX_ACTIONS_PER_STATEMENT        8
CompilerAssert(MAX_ACTIONS_PER_STATEMENT % 4 == 0);   // Alignment!
#define MAX_NR_OF_STATEMENTS             32
#define FILTER_WORDS                     6   // Do not change!
#define MAX_NR_OF_FILTERS                192 // Total nof pass/stop filter arrays for each channel.
CompilerAssert(MAX_NR_OF_FILTERS <= FILTER_WORDS * 32);
#define MAX_NR_OF_AUTO_TRANSMIT_MSGS     128
#define MAX_NR_OF_AUTO_TRANSMIT_CHAINS   16

#define MAX_NR_OF_XCP_CROS               1321
#define MAX_NR_OF_XCP_LISTS              8

/*
 * Trigger types (ConfigFileTrigger.trigType)
 * It is a bit mask. If the TRIG_LOG_ALL bit is set, logging is started at
 * once.
 * Setting TRIG_LOG_FIFO for other modes than TRIG_LOG_ALL is usually not
 * useful (maybe if one have a very large postTrigger value and want to
 * continue until memorator is shut down once trigger is started).
 */
#define TRIG_LOG_ALL               1
#define TRIG_ON_EVENT              2 // isn't this obsolete?
#define TRIG_SCRIPTED              4
#define TRIG_LOG_FIFO              0x80

/*
 *  Definitions for the transTableEntries.
 */
#define DE_FORMAT_SIGN_BIT         0x80
#define DE_FORMAT_BIGENDIAN_BIT    0x40
#define DE_FORMAT_FLOAT            0x20

#define HLP_TYPE_MASK              0x07
#define HLP_TYPE_NONE              0
#define HLP_TYPE_J1939             1

#define HLP_SPEC_MASK              0xF8
#define J1939_ID_DEST_ADDRESS      0x08
#define J1939_ID_SOURCE_ADDRESS    0x10
#define J1939_ID_PGN_NO            0x20

// Block identifiers. 0 and 0xFFFFFFFF are reserved; both are invalid.
#define BLOCK_ID_VERSION           1
#define BLOCK_ID_BUSPARAMS         2
#define BLOCK_ID_TRIGGER           3
#define BLOCK_ID_FILTER            4
#define BLOCK_ID_IDENT             5
#define BLOCK_ID_LINX              6
#define BLOCK_ID_AUTO_TRANSMIT     7
#define BLOCK_ID_XCP               8
#define BLOCK_ID_AUTOBAUD          9
#define BLOCK_ID_SCRIPT           10
#define BLOCK_ID_AFTERBURNER      11
#define BLOCK_ID_BUSPARAMS_FD     12
#define BLOCK_ID_TRIGGER_FD       13
#define BLOCK_ID_FILTER_FD        14
#define BLOCK_ID_AUTO_TRANSMIT_FD 15
#define BLOCK_ID_SCRIPT_SD        16
#define BLOCK_ID_END              0x72648AC5

#define LINX_J1587_PCI_TYPE_NONE        1
#define LINX_J1587_PCI_TYPE_SINGLE      0
#define LINX_J1587_PCI_TYPE_FIRST       5
#define LINX_J1587_PCI_TYPE_CONSECUTIVE 6

// Could be set to 37 depending on
// it should be ok to trig and filter on special network
// with j1587 messages that could be up to 255 bytes.
#define LINX_MAX_NR_J1587_CAN_MSG       5

#define LINX_TRIG_J1587_MID             1
#define LINX_TRIG_J1587_MID_PID         2
#define LINX_TRIG_J1587_MID_SID         3
#define LINX_TRIG_J1587_PID             4
#define LINX_TRIG_J1587_SID             5

// Linx types
#define LINX_TYPE_NONE   0
#define LINX_TYPE_LIN    1
#define LINX_TYPE_J1708  2
#define LINX_TYPE_K_LINE 3
#define LINX_TYPE_SWC    4
#define LINX_TYPE_LS     5

// Linx: modes
#define LINX_MODE_NONE 0

// LINX_MODE_J1708_NORMAL has a higher resistance
// between J1708_A och B
// and is the normal diagnostic mode.
// LINX_MODE_J1708_NODE has a lower resistance
// and simulates a node.
#define LINX_MODE_J1708_NORMAL 1
#define LINX_MODE_J1708_NODE   2

#define LINX_MODE_LIN_MASTER 1
#define LINX_MODE_LIN_SLAVE  2

// Linx: flags
#define LINX_FLAG_LIN_ENHANCED_CHECKSUM  0x01
#define LINX_FLAG_LIN_VARIABLE_DLC       0x02

typedef struct
{
  uint8_t  channel;   // CAN channel 0 or 1
  uint8_t  mode;      // LINX_MODE_XXX
  uint8_t  linxType;  // LINX_TYPE_XXX
  uint8_t  flags;     // LINX_FLAG_XXX
  uint32_t bitrate;   // Bitrate in bps
} ConfigFileLinx;

// CAN bit timing.
typedef struct
{
  uint8_t   used;
  int8_t    channel;
  uint8_t   psc;
  uint8_t   prSeg;
  uint8_t   phSeg1;
  uint8_t   phSeg2;
  uint8_t   sjw;
  uint8_t   samples;
  uint8_t   silent;
  uint8_t   highSpeed;
  uint16_t  padding;
} ConfigFileBusParams;
CompilerAssert(sizeof(ConfigFileBusParams) == 12);

// Values for can_mode
#define OPEN_AS_CAN           0
#define OPEN_AS_CANFD_ISO     1
#define OPEN_AS_CANFD_NONISO  2

typedef struct
{
  uint8_t   used;
  int8_t    channel;
  uint8_t   can_mode;
  uint8_t   reserved1;
  uint32_t  bitrate;
  uint32_t  bitrate_brs;
  uint16_t  tseg1;
  uint16_t  tseg2;
  uint16_t  tseg1_brs;
  uint16_t  tseg2_brs;
  uint8_t   sjw;
  uint8_t   sjw_brs;
  uint8_t   silent;
  uint8_t   reserved2;
/*  uint8_t   samples; obsolete */
/*  uint8_t   highSpeed; obsolete???*/
/* any other flags or formats needed? */
  uint32_t flags;
} ConfigFileBusParamsFd;
CompilerAssert(sizeof(ConfigFileBusParamsFd) == 28);

typedef struct {
  uint8_t   used;
  int8_t    enabled;
  int8_t    sjw;
  int8_t    sample_point;
  int32_t   padding;
  uint32_t  list[16];
} ConfigFileAutoBaudCfg;
CompilerAssert(sizeof(ConfigFileAutoBaudCfg) == 72);

// Identity information for Dispatcher
typedef struct
{
  uint8_t   baseIdent[32];
  uint32_t  identifier;
  uint16_t  bitCode;
  uint16_t  flags;
  uint8_t   used;
  uint8_t   padding;
  uint16_t  padding2;
} ConfigFileIdentInfo;
CompilerAssert(sizeof(ConfigFileIdentInfo) == 44);

#define AUTO_TRANSMIT_FLAG_CYCLIC       0x01
#define AUTO_TRANSMIT_FLAG_AUTOSTART    0x02
#define AUTO_TRANSMIT_FLAG_CHAIN_IN_USE 0x8000

typedef struct {
  uint16_t         flags;
  uint8_t          first;
  uint8_t          last;
  uint32_t         msgDelay;
  uint32_t         cycleDelay;
} autoTransmitChainConfig;
CompilerAssert(sizeof(autoTransmitChainConfig) == 12);

#ifndef MSGFLAG_ERROR_FRAME
#define MSGFLAG_ERROR_FRAME         0x01        // Msg is a bus error
#define MSGFLAG_REMOTE_FRAME        0x10        // Msg is a remote frame
#endif

typedef struct {
  uint32_t  id;  // CAN_IDENT_IS_EXTENDED flag denotes extended frame
  uint8_t   dlc;
  uint8_t   flags;
  uint8_t   channel;
  uint8_t   padding;
  uint8_t   data[8];
} autoTransmitMessage;
CompilerAssert(sizeof(autoTransmitMessage) == 16);

typedef struct {
  uint32_t  id;  // CAN_IDENT_IS_EXTENDED flag denotes extended frame
  uint32_t  flags;
  uint8_t   dlc;
  uint8_t   channel;
  uint8_t   databytes;
  uint8_t   data[64];
  uint8_t   padding;
} autoTransmitMessageFd;
CompilerAssert(sizeof(autoTransmitMessageFd) == 76);

typedef struct {
  uint8_t                   used;
  uint8_t                   padding;
  uint16_t                  padding2;
  autoTransmitMessage       msgs[MAX_NR_OF_AUTO_TRANSMIT_MSGS];
  autoTransmitChainConfig   chains[MAX_NR_OF_AUTO_TRANSMIT_CHAINS];
} ConfigFileAutoTransmit;
CompilerAssert(sizeof(ConfigFileAutoTransmit) == 4 + (16 * MAX_NR_OF_AUTO_TRANSMIT_MSGS + 12 * MAX_NR_OF_AUTO_TRANSMIT_CHAINS));

typedef struct {
  uint8_t                   used;
  uint8_t                   padding;
  uint16_t                  padding2;
  autoTransmitMessageFd     msgs[MAX_NR_OF_AUTO_TRANSMIT_MSGS];
  autoTransmitChainConfig   chains[MAX_NR_OF_AUTO_TRANSMIT_CHAINS];
} ConfigFileAutoTransmitFd;
CompilerAssert(sizeof(ConfigFileAutoTransmitFd) == 4 + (76 * MAX_NR_OF_AUTO_TRANSMIT_MSGS + 12 * MAX_NR_OF_AUTO_TRANSMIT_CHAINS));

#define MAX_EXPR_LEN  32
CompilerAssert(MAX_EXPR_LEN % 4 == 0);    // Alignment.


#define EXPR_OP_OFFSET 100

#define EXPR_OP_NONE   (EXPR_OP_OFFSET + 0)
#define EXPR_OP_AND    (EXPR_OP_OFFSET + 1)
#define EXPR_OP_OR     (EXPR_OP_OFFSET + 2)

//Should correspond to the numbers of expressions.
#define EXPR_OP_MAX   3

//the variables are indices between 0 and 31
//x should be unsigned
#define IS_VARIABLE(x) ((x) < MAX_NR_OF_TRIGVARS)

//The EXPR_OP_NONE is not considered an operator. It is used to
//mark the end of an expression.
#define IS_OPERATOR(x) (((x) > EXPR_OP_OFFSET) && ((x) < (EXPR_OP_OFFSET + EXPR_OP_MAX)))

//
// Definitions of all TrigVar types
//

typedef struct {             // timer, trig when current RTC time - last RTC time >= delay
  uint32_t last;             // init to current time when memorator has power
  uint32_t offset;           // add this to rtc when trigging [s]
  uint32_t delay;            // delay = offset*1000 is set in logger.c
  uint8_t  activated;        // user can set timer to activated/not activated at start up
  uint8_t  repeat;           // true if repeat timer, false otherwise
  uint16_t padding;
} TimerTrigVar;
CompilerAssert(sizeof(TimerTrigVar) == 14 + 2);

typedef struct {             // trig on id/id range
  uint32_t id;
  uint32_t id_min;
  uint32_t id_mask;
  uint8_t  HLP;
  uint8_t  padding;
  uint16_t padding2;
} MsgIdTrigVar;
CompilerAssert(sizeof(MsgIdTrigVar) == 14 + 2);

// placed in flags in SigValTrigVar
#define TRIGVAR_FLAG_IGNORE_DLC   1

typedef struct {             // trig on signal
  uint32_t id;
  uint8_t  dlc;
  uint8_t  startbit;
  uint8_t  length;
  uint8_t  dataType;         // intel, motorola, unsigned, signed
  int32_t  data;
  int32_t  data_min;
  uint8_t  condition;        // ON_DATA_XXX
  uint8_t  HLP;
  uint8_t  oldDataValueKnown;
  uint8_t  flags;            // TRIGVAR_FLAG_IGNORE_DLC
  int32_t  oldDataValue;     // oldDataValue contains last data value
} SigValTrigVar;
CompilerAssert(sizeof(SigValTrigVar) == 24);

typedef struct {             // trig on signal
  uint32_t id;
  uint8_t  dlc;
  uint8_t  dataType;         // intel, motorola, unsigned, signed
  uint16_t startbit;
  uint16_t length;
  uint16_t reserved;
  int32_t  data;
  int32_t  data_min;
  uint8_t  condition;        // ON_DATA_XXX
  uint8_t  HLP;
  uint8_t  oldDataValueKnown;
  uint8_t  flags;            // TRIGVAR_FLAG_IGNORE_DLC
  int32_t  oldDataValue;     // oldDataValue contains last data value
} SigValTrigVarFd;
CompilerAssert(sizeof(SigValTrigVarFd) == 28);

typedef struct {             // trig on specific dlc/dlc range
  uint8_t dlc;
  uint8_t dlc_min;
} DLCTrigVar;
CompilerAssert(sizeof(DLCTrigVar) == 2);

typedef struct {            // trig on external signal
  uint8_t level;            // TRIG_EXTERNAL_LEVEL_ flags
  uint8_t debounce;         // on/off, 
} ExternalTrigVar;
CompilerAssert(sizeof(ExternalTrigVar) == 2);

typedef struct {            // trig on message flags (error frame, wakeup etc)
  uint32_t flag;            // TRIGVAR_MSG_FLAG_XXX // make smaller?
} MsgFlagTrigVar;
CompilerAssert(sizeof(MsgFlagTrigVar) == 4);

typedef struct {            // trig on full disk, if using fifo, it will never trig
  uint16_t padding;
} DiskFullTrigVar;
CompilerAssert(sizeof(DiskFullTrigVar) == 2);

/*
typedef struct {      // trig on larger file size than maxSize
  //uint32_t maxSize; // maxSize in [kB]?
  uint16_t maxSize;   // maxSize in [kB]?
} FileSizeTrigVar;
CompilerAssert(sizeof(FileSizeTrigVar) == 2);
*/

// below: not implemented yet
/*
typedef struct {      // trig when current RTC time >= rtc
  uint32_t rtc;       // time to trig [s]
  uint32_t offset;    // add this to rtc when trigging [s]
} RTCTrigVar;
CompilerAssert(sizeof(RTCTrigVar) == 8);

typedef struct {      // trig on busstatus (error active/passive, bus off etc)
  uint8_t busStatus;
} BusStatusTrigVar;
CompilerAssert(sizeof(BusStatusTrigVar) == 1);


//typedef struct {      // trig on a usb packet
//} USBPacketTrigVar;

typedef struct {        // trig on a busload range
  uint8_t load;
  uint8_t load_min;
} BusLoadTrigVar;
CompilerAssert(sizeof(BusLoadTrigVar) == 2);
*/

// trigvar types
#define TRIGVAR_TYPE_DISABLED       0x80

#define TRIGVAR_TYPE_MSG_ID         0
#define TRIGVAR_TYPE_MSG_DLC        1
#define TRIGVAR_TYPE_MSG_FLAG       2
#define TRIGVAR_TYPE_SIGVAL         3
#define TRIGVAR_TYPE_EXTERNAL       4
#define TRIGVAR_TYPE_TIMER          5
#define TRIGVAR_TYPE_DISK_FULL      6
#define TRIGVAR_TYPE_LINX_J1587     7
#define TRIGVAR_TYPE_LINX_J1587_VAL 8
#define TRIGVAR_TYPE_STARTUP        9
// not implemented yet:
//#define TRIGVAR_TYPE_USB_PACKET
//#define TRIGVAR_TYPE_RTC
//#define TRIGVAR_TYPE_BUS_STATUS
//#define TRIGVAR_TYPE_BUS_LOAD

// MsgFlagTrigVar flags
#define TRIGVAR_MSG_FLAG_NERR                0x01
#define TRIGVAR_MSG_FLAG_WAKE_UP             0x02
#define TRIGVAR_MSG_FLAG_ERROR_FRAME         0x04
// not implemented yet:
//#define TRIGVAR_MSG_FLAG_OVERRUN             0x08
//#define TRIGVAR_MSG_FLAG_HW_ERROR            0x10

// When the timeOut variable in TriggerVar is
// equal to TRIGVAR_LIVE_FOREVER, then the trigger
// will not time out
#define TRIGVAR_LIVE_FOREVER       ((uint32_t)-1)

// ExternalTrigVar level
#define TRIG_EXTERNAL_LEVEL_LO_HI   0x01
#define TRIG_EXTERNAL_LEVEL_HI_LO   0x02

//  condition for sigVal TrigVar
#define ON_DATA_EQUAL_TO           0
#define ON_DATA_NOT_EQUAL_TO       1
#define ON_DATA_LARGER_THAN        2
#define ON_DATA_SMALLER_THAN       3
#define ON_DATA_CHANGE_TO          4
#define ON_DATA_CHANGE_FROM        5
#define ON_DATA_CHANGE             6  // each time the data is changed
#define ON_DATA_LARGER_THAN_OR_EQUAL  7
#define ON_DATA_SMALLER_THAN_OR_EQUAL 8

//
// TriggerVar
//
typedef struct {
  uint8_t  type;                     // TRIGVAR_TYPE_XXX
  uint8_t  channel;
  uint16_t padding;
  uint32_t timeOut;                  // Time in ms that a trigVar can be active

  union {
    MsgIdTrigVar        msgId;        // 10
    SigValTrigVar       sigVal;       // 24
    MsgFlagTrigVar      msgFlag;      //  4
    ExternalTrigVar     extSign;      //  2
    DLCTrigVar          msgDlc;       //  2
    TimerTrigVar        timer;        // 14
    DiskFullTrigVar     diskFull;     //  2
    //BusLoadTrigVar   busLoad;
    //RTCTrigVar       rtc;
    //USBPacketTrigVar usbPacket;
    //BusStatusTrigVar busStatus;
  } t;
} TriggerVar;
CompilerAssert(sizeof(TriggerVar) == 6 + 2 + 24);

//
// TriggerVarFd
//
typedef struct {
  uint8_t  type;                     // TRIGVAR_TYPE_XXX
  uint8_t  channel;
  uint16_t padding;
  uint32_t timeOut;                  // Time in ms that a trigVar can be active

  union {
    MsgIdTrigVar        msgId;        // 10
    SigValTrigVarFd     sigVal;       // 28
    MsgFlagTrigVar      msgFlag;      //  4
    ExternalTrigVar     extSign;      //  2
    DLCTrigVar          msgDlc;       //  2
    TimerTrigVar        timer;        // 14
    DiskFullTrigVar     diskFull;     //  2
  } t;
} TriggerVarFd;
CompilerAssert(sizeof(TriggerVarFd) == 6 + 2 + 28);


typedef struct TriggerStatement_struct TriggerStatement;

// This is only used in the trigger code.
typedef struct {
  uint32_t timeOut;               // Copied from TriggerStatement.
  uint32_t timeOfLastOccurance;   // Set in trigger code.
} TriggerTimeout;


#if defined(__x86_64__) || defined (__aarch64__)
// Dummy function pointer for 64-bit. This is not used in the dll.
// We do not check for __arm64__ as AArch64 is the official back-end from ARM.
typedef uint32_t actionFunction;
#else
typedef void (*actionFunction)(TriggerStatement *, uint32_t);
#endif


typedef struct {
  actionFunction action;
  uint32_t       actionParam;  // Parameter to the function.
} actionCall;

//
// actionList defines                               // actionParam
//
#define ACTION_START_LOG                        1        // ACTION_LOG
#define ACTION_STOP_LOG                         2        // ACTION_LOG_DIRECT/ACTION_LOG
#define ACTION_STOP_LOG_COMPLETELY              3        // no parameter for now
#define ACTION_ACTIVATE_TIMER                   4        // trigVar index
#define ACTION_DEACTIVATE_TIMER                 5        // trigVar index
#define ACTION_ACTIVATE_AUTO_TRANSMIT_CHAIN     6        // chain no
#define ACTION_DEACTIVATE_AUTO_TRANSMIT_CHAIN   7        // chain no
#define ACTION_ACTIVATE_FILTERS                 8        // channel, word, mask
#define ACTION_DEACTIVATE_FILTERS               9        // channel, word, mask
#define ACTION_FLASH_LEDS                       10       // ACTION_FLASH_XXX
#define ACTION_EXTERNAL_PULSE                   11       //

#define ACTION_SET_ACTIVE_FILTER_MASK_HIGH_CH0  12       // mask
#define ACTION_SET_ACTIVE_FILTER_MASK_HIGH_CH1  13       // mask
#define ACTION_START_XCP                        14       // listNr
#define ACTION_SET_ACTIVE_FILTER_MASK_LOW_CH0   15        // mask
#define ACTION_SET_ACTIVE_FILTER_MASK_LOW_CH1   16        // mask
// The following were removed since they had unnecessary limitations.
//#define ACTION_SET_ACTIVE_FILTER_MASK_LOW/HIGH_CH0/1   // mask

//
// actionParam defines
//

// for ACTION_START_LOG/STOP_LOG
#define ACTION_LOG_DIRECT  0 // only applicable on stop_log
#define ACTION_LOG         1

// for ACTION_FLASH_LEDS
#define ACTION_FLASH_CAN1_RX_LED  0x01    // doesn't seems to be used anywhere, dump50 in configure ?!
#define ACTION_FLASH_CAN2_RX_LED  0x02
#define ACTION_FLASH_ERROR_LED    0x04

// for ACTION_EXTERNAL_PULSE
#define ACTION_5_SEK_PULSE 5

// add more here


//
// Contains expression and a list of actions that will occur when expression is true
//
struct TriggerStatement_struct {
  uint8_t        noOfActions;                    // number of actions
  uint8_t        padding;
  uint16_t       padding1;

  uint32_t       XXXreactionDelay;

  // A list of actions for this expression, ACTION_XXX
  uint8_t        actionList[MAX_ACTIONS_PER_STATEMENT];

  // Action below is set in logger code to point at correct action,
  // set in actionList (ACTION_LOG_XXX). Same indexing as actionList.
  actionCall     call[MAX_ACTIONS_PER_STATEMENT];

  // The expression. An array of indices to either 1) retval bits
  // (0 to 31), 2) operator index AND and OR see EXPR_OFFSET macro etc.
  uint8_t        postFixExpr[MAX_EXPR_LEN];

  // Post-trigger time in RTC units (ms). 0 == infinite time
  uint32_t       preTrigger;                     // pre-trigger time in RTC units (ms)
  uint32_t       postTrigger;                    // post-trigger time in RTC units (ms)
  uint32_t       padding2;
  uint32_t       padding3;
};
CompilerAssert(sizeof(struct TriggerStatement_struct) == 8 + MAX_EXPR_LEN +
                                                         MAX_ACTIONS_PER_STATEMENT * (8 + 1) +
                                                         16);
CompilerAssert(sizeof(struct TriggerStatement_struct) % 32 == 0);


//
// All trigger information
//
typedef struct
{
  uint8_t           logMode;                          // log all, fifo, trimdisk, event
  uint8_t           noOfTriggers;                     // number of trigvars
  uint8_t           noOfStatements;                   // number of listOfTrigStatements
  uint8_t           noOfEnabledTriggers;
  uint32_t          preTriggerSectors;
  uint32_t          padding[6];
  TriggerStatement  listOfTrigStatements[MAX_NR_OF_STATEMENTS];
  TriggerVar        listOfTrigVars[MAX_NR_OF_TRIGVARS];
} ConfigFileTrigger;
CompilerAssert(sizeof(ConfigFileTrigger) == MAX_NR_OF_STATEMENTS * 128 +
                                            MAX_NR_OF_TRIGVARS * 32 + 32);

//
// All Fd trigger information
//
typedef struct
{
  uint8_t           logMode;                          // log all, fifo, trimdisk, event
  uint8_t           noOfTriggers;                     // number of trigvars
  uint8_t           noOfStatements;                   // number of listOfTrigStatements
  uint8_t           noOfEnabledTriggers;
  uint32_t          preTriggerSectors;
  uint32_t          padding[6];
  TriggerStatement  listOfTrigStatements[MAX_NR_OF_STATEMENTS];
  TriggerVarFd      listOfTrigVars[MAX_NR_OF_TRIGVARS];
} ConfigFileTriggerFd;
CompilerAssert(sizeof(ConfigFileTriggerFd) == MAX_NR_OF_STATEMENTS * 128 +
                                              MAX_NR_OF_TRIGVARS * 36 + 32);

typedef struct {
  uint8_t        exprPos[MAX_NR_OF_STATEMENTS];         // Ptr into exprs below
  uint8_t        postFixExpr[256 - 40];                 // Short expressions
  uint32_t       trigVarMask[MAX_NR_OF_STATEMENTS];     // Vars used in statement
  uint32_t       timeoutActive;                         // Mask of infinite TO:s
  TriggerTimeout timeouts[MAX_NR_OF_TRIGVARS];
} TriggerHelp;

/*
 * Filter Types
 * -----------
 * ID, DLC and BYTEVALUE can all be intervals.
 * Each ID is HLP aware.
 * Each filter can be used together with FILTER_FLAG
 *
 * - Just ID
 * - ID and DLC
 * - Just DLC
 * - Signal
 * - ID and Counter
 * - Signal and Counter
 * - Just flags (e.g. filter out every extended message)
 *
 * Errorframes ougth to pass through every filter as default
 */

// placed in type
#define FILTER_INTERNAL_FLAG_ID     1
#define FILTER_INTERNAL_FLAG_DLC    2
#define FILTER_INTERNAL_FLAG_SIGNAL 4
#define FILTER_INTERNAL_FLAG_CNTR   8
#define FILTER_INTERNAL_FLAG_PASS  16
#define FILTER_INTERNAL_FLAG_STOP  32


#define FILTER_TYPE_ONLY_ID         (FILTER_INTERNAL_FLAG_ID)
#define FILTER_TYPE_ID_AND_DLC      (FILTER_INTERNAL_FLAG_ID | FILTER_INTERNAL_FLAG_DLC)
#define FILTER_TYPE_ONLY_DLC        (FILTER_INTERNAL_FLAG_DLC)
#define FILTER_TYPE_SIGNAL          (FILTER_INTERNAL_FLAG_SIGNAL) // has its own id & dlc check
#define FILTER_TYPE_ID_COUNTER      (FILTER_INTERNAL_FLAG_ID | FILTER_INTERNAL_FLAG_CNTR)
#define FILTER_TYPE_SIGNAL_COUNTER  (FILTER_INTERNAL_FLAG_SIGNAL | FILTER_INTERNAL_FLAG_CNTR)
#define FILTER_TYPE_JUST_FLAGS      (0)  // All filters care about flags

// placed in flags
#define FILTER_FLAG_STD         1
#define FILTER_FLAG_EXT         2
#define FILTER_FLAG_ERRORFRAME  4
#define FILTER_FLAG_IGNORE_DLC  8

typedef struct {
  uint32_t id;      // including flags, FILTER_FLAG
  uint32_t id_min;  // used when id range [id_min, id]
  uint32_t id_mask; // used when id masked
  uint8_t  dlc;     // used when dlc range [dlc_min, dlc]
  uint8_t  dlc_min;
  uint16_t padding;
} idMsg;
CompilerAssert(sizeof(idMsg) == 16);

typedef struct {
  uint32_t id;      // including flags, FILTER_FLAG
  int32_t  data;
  uint8_t  dlc;
  uint8_t  dataType;
  uint8_t  startbit;
  uint8_t  length;
  uint32_t padding;
  uint32_t padding2;
} idSign;
CompilerAssert(sizeof(idSign) == 20);

typedef struct {
  uint32_t id;      // including flags, FILTER_FLAG
  int32_t  data;
  uint8_t  dlc;
  uint8_t  dataType;
  uint16_t startbit;
  uint16_t length;
  uint16_t padding;
  uint32_t padding2;
} idSignFd;
CompilerAssert(sizeof(idSignFd) == 20);

typedef struct {
  uint8_t  type;        // type of filter, FILTER_TYPE
  uint8_t  HLPTypeSpec; // j1939?
  uint16_t flags;       // standard, extended, errorframe, remote, FILTER_FLAG_IGNORE_DLC

  // counter to be able to pick 5 out of 1000 messages
  // counter goes from 0 to counter_max-1 and if counter < counter_threshold
  // then the filter is active, otherwise inactive
  uint16_t counter_max;
  uint16_t counter_threshold;
  uint16_t counter;
  uint16_t padding;

  union {
    idMsg     msg;    // to be able to check intervals in id and dlc
    idSign    sign;   // to be able to check a signal in a message
  } ident;
} FilterVar;
CompilerAssert(sizeof(FilterVar) == 20 + 10 + 2);

typedef struct {
  uint8_t  type;        // type of filter, FILTER_TYPE
  uint8_t  HLPTypeSpec; // j1939?
  uint16_t flags;       // standard, extended, errorframe, remote, FILTER_FLAG_IGNORE_DLC

  // counter to be able to pick 5 out of 1000 messages
  // counter goes from 0 to counter_max-1 and if counter < counter_threshold
  // then the filter is active, otherwise inactive
  uint16_t counter_max;
  uint16_t counter_threshold;
  uint16_t counter;
  uint16_t padding;

  union {
    idMsg     msg;    // to be able to check intervals in id and dlc
    idSignFd  sign;   // to be able to check a signal in a message
  } ident;
} FilterVarFd;
CompilerAssert(sizeof(FilterVarFd) == 20 + 10 + 2);


#define PROTOCOL_CCP 0
#define PROTOCOL_XCP 1

typedef struct s_XCPListT {
  uint16_t    size;       // Not really the size but rather an index to the first
                          // CRO of next list, hence true size of list is:
                          // ("size" of this list - "size" of previous list)
  uint8_t     canChannel;
  uint8_t     protocol;
  uint32_t    masterId;
  uint32_t    slaveId;
  uint16_t    timeout1;
  uint16_t    timeout7;
  uint16_t    timeoutNoData; // If no data with slaveID is seen in x sec, list is restarted.
  uint8_t     stopOfList; // If != 0xFF, this is the stop list of list with index x.
  uint8_t     padding; // Not used, set to zero.
} XCPListT;

typedef struct {
  XCPListT    xcpLists[MAX_NR_OF_XCP_LISTS];
  uint8_t     cros[8*MAX_NR_OF_XCP_CROS];
} ConfigFileXCP;


//
// Message flags.
//
// The CAN-message is in extended mode.
#define   CAN_IDENT_IS_EXTENDED           0x80000000

//
// Filters
//
typedef struct {
  int8_t             channel;
  uint8_t            used;
  uint8_t            totalNumberOfFilters;
  uint8_t            beginPassFilters;
  uint8_t            numberOfPassFilters;
  uint8_t            beginStopFilters;
  uint8_t            numberOfStopFilters;
  uint8_t            padding;
  uint32_t           activeFiltersMask[FILTER_WORDS];  // 1 - on
  FilterVar          filterVariables[MAX_NR_OF_FILTERS];
} ConfigFileFilter;
CompilerAssert(sizeof(ConfigFileFilter) == 8 + 6 * 4 + MAX_NR_OF_FILTERS * 32);

typedef struct {
  int8_t             channel;
  uint8_t            used;
  uint8_t            totalNumberOfFilters;
  uint8_t            beginPassFilters;
  uint8_t            numberOfPassFilters;
  uint8_t            beginStopFilters;
  uint8_t            numberOfStopFilters;
  uint8_t            padding;
  uint32_t           activeFiltersMask[FILTER_WORDS];  // 1 - on
  FilterVarFd        filterVariables[MAX_NR_OF_FILTERS];
} ConfigFileFilterFd;
CompilerAssert(sizeof(ConfigFileFilterFd) == 8 + 6 * 4 + MAX_NR_OF_FILTERS * 32);


//
// afterburner information.
//
//
typedef struct {
  uint32_t           canPowerTimeout; // 32 bit number in [ms]
  uint32_t           padding[15];
} ConfigFileAfterburner;
CompilerAssert(sizeof(ConfigFileAfterburner) == 64);


//
// File version information.
// The checksum is the CRC-32 of all bytes in the file including the BLOCK_ID_END
// The checksum is then appended directly after BLOCK_ID_END.
// Note that the BLOCK_ID_END block doesn't have a length.
//
typedef struct {
  uint32_t           magic;
  uint16_t           version;   // Upper byte: major, lower byte: minor
  uint16_t           padding;
} ConfigFileVersion;
CompilerAssert(sizeof(ConfigFileVersion) == 8);


#define CONFIG_MAGIC    0xEA1DADDE

#define MAX_SCRIPT_CODE_SIZE 65536
typedef struct {
  uint8_t     used;
  uint8_t     primary;
  int8_t      defaultChannel;
  uint8_t     script_ext;
  uint32_t    scriptLen;
  uint8_t     scriptCode[MAX_SCRIPT_CODE_SIZE];
} TMhydraScriptConfig;
CompilerAssert(sizeof(TMhydraScriptConfig) == MAX_SCRIPT_CODE_SIZE + 8);


typedef struct {
  uint8_t     used;
  uint8_t     primary;
  int8_t      defaultChannel;
  uint8_t     script_ext;
  uint32_t    scriptLen;
  char        scriptName[40];
} TMhydraScriptConfigSd;

typedef struct {
  uint32_t  id;       // BLOCK_ID_xxx (or 0 for end of file)
  uint32_t  len;      // Length including this blockhead
} BlockHead;
CompilerAssert(sizeof(BlockHead) == 8);

#endif // _LOGGER_CONFIG_FILE_H_
