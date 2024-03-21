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
**   Constants for converting XML settings to a binary param.lif
**
** -----------------------------------------------------------------------------
*/

#ifndef KVAMEMOLIBXMLCONST_H
#define KVAMEMOLIBXMLCONST_H
#include <stdint.h>
#include "logger_config_file.h"

#ifdef __cplusplus
extern "C" {
#endif

#define KVASER_MAX_BITRATE 1000000
#define KVASER_MIN_BITRATE 50000

#define KVASER_MAX_BITRATE_FD 10000000
#define KVASER_MIN_BITRATE_FD 500000

#define MAX_NR_OF_CAN_CHANNELS           5

#define KVASER_XML_VERSION_ELEMENT_MAJOR  2
#define KVASER_XML_VERSION_ELEMENT_MINOR  0
#define KVASER_XML_VERSIONS_STRING "2.0"

#define KVASER_BINARY_VERSION_ELEMENT_MAJOR 6
#define KVASER_BINARY_VERSION_ELEMENT_MINOR 0
#define KVASER_BINARY_VERSION_ELEMENT_MAJOR_OLD 5
#define KVASER_BINARY_VERSION_ELEMENT_MINOR_OLD 0
#define KVASER_BINARY_VERSIONS_STRING "5.0, 6.0"

#define EAGLE_MAX_NR_OF_SCRIPTS 4

#define KVASER_MAX_AFTERBURNER_TIME 30000

#define GENERIC_MESSAGE_LENGTH 256

#define MAX_NR_OF_WARNINGS 128
#define MAX_NR_OF_ERRORS 128

// --------------------------------------------------------------------------
// Limits and constants; might depend on EAN
// --------------------------------------------------------------------------
const unsigned long CAN_SYS_FREQ = 16000000L; // Should depend on firmware
const unsigned long PSC_FACTOR = 2;           // Needed to match firmware
const uint8_t TSEG1_MIN = 2;
const uint8_t TSEG1_MAX = 16;
const uint8_t TSEG2_MIN = 2;
const uint8_t TSEG2_MAX = 8;
const uint8_t SJW_MIN = 1;
const uint8_t SJW_MAX = 4;
const uint8_t PSC_MIN = 1;
const uint8_t PSC_MAX = 255;

// Bitrate, arbitration phase CAN FD
const uint8_t TSEG1_ARB_MIN = 2;
const uint8_t TSEG1_ARB_MAX = 255;  // Could be 511 but FW only supports uint8
const uint8_t TSEG2_ARB_MIN = 2;
const uint8_t TSEG2_ARB_MAX = 32;
const uint8_t SJW_ARB_MIN = 1;
const uint8_t SJW_ARB_MAX = 16;

// Bitrate, data phase CAN FD
const uint8_t TSEG1_DATA_MIN = 1;
const uint8_t TSEG1_DATA_MAX = 255; // Could be 511 but FW only supports uint8
const uint8_t TSEG2_DATA_MIN = 2;
const uint8_t TSEG2_DATA_MAX = 32;
const uint8_t SJW_DATA_MIN = 1;
const uint8_t SJW_DATA_MAX = 16;
const uint8_t BRP_MIN = 1;
const uint8_t BRP_MAX = 2;

// --------------------------------------------------------------------------
// XML tags
// --------------------------------------------------------------------------
const char XML_KVASER_ROOT_NODE[] = "KVASER";
const char XML_KVASER_VERSION[] = "VERSION";
const char XML_KVASER_BIN_VERSION[] = "BINARY_VERSION";

const char XML_BUSPARAM_BLOCK[] = "CAN_BUS";
const char XML_BUSPARAM[]  = "PARAMETERS";
const char XML_BUSPARAM_ATTR_CHAN[]    = "channel";
const char XML_BUSPARAM_ATTR_SJW[]     = "sjw";
const char XML_BUSPARAM_ATTR_SILENT[]  = "silent";
const char XML_BUSPARAM_ATTR_BITRATE[] = "bitrate";
const char XML_BUSPARAM_ATTR_TSEG1[]   = "tseg1";
const char XML_BUSPARAM_ATTR_TSEG2[]   = "tseg2";

const char XML_BUSPARAM_ATTR_BITRATE_BRS[] = "bitrate_brs";
const char XML_BUSPARAM_ATTR_TSEG1_BRS[]   = "tseg1_brs";
const char XML_BUSPARAM_ATTR_TSEG2_BRS[]   = "tseg2_brs";
const char XML_BUSPARAM_ATTR_SJW_BRS[]     = "sjw_brs";
const char XML_BUSPARAM_ATTR_ISO_BRS[]     = "iso";
const char XML_BUSPARAM_ATTR_BRP_BRS[]     = "brp";

const char XML_TRIGGER_BLOCK[] = "TRIGGERBLOCK";
const char XML_TRIGGERS[]      = "TRIGGERS";

const char XML_TRIGGERS_ATTR_CHAN[]     = "channel";
const char XML_TRIGGERS_ATTR_NAME[]     = "name";
const char XML_TRIGGERS_ATTR_TMO[]      = "timeout";

const char XML_TRIGGERS_ATTR_ID[]       = "msgid";
const char XML_TRIGGERS_ATTR_ID_MIN[]   = "msgid_min";
const char XML_TRIGGERS_ATTR_ID_EXT[]   = "can_ext";
const char XML_TRIGGERS_ATTR_ID_MSK[]   = "msg_field";
const char XML_TRIGGERS_ATTR_ID_PROT[]  = "protocol";

const char XML_TRIGGERS_ATTR_DLC[]      = "dlc";
const char XML_TRIGGERS_ATTR_DLC_MIN[]  = "dlc_min";

const char XML_TRIGGERS_ATTR_STARTBIT[]  = "startbit";
const char XML_TRIGGERS_ATTR_BITLENGTH[] = "length";
const char XML_TRIGGERS_ATTR_DATATYPE[]  = "datatype";
const char XML_TRIGGERS_ATTR_ENDIAN[]    = "byteorder";
const char XML_TRIGGERS_ATTR_DATA[]      = "data";
const char XML_TRIGGERS_ATTR_DATA_MIN[]  = "data_min";
const char XML_TRIGGERS_ATTR_DATA_COND[] = "condition";

const char XML_TRIGGERS_ATTR_EXTLEVEL[] = "level";
const char XML_TRIGGERS_ATTR_OFFSET[]   = "offset";
const char XML_TRIGGERS_ATTR_REPEAT[]   = "repeat";

const char XML_STATEMENT_BLOCK[]     = "STATEMENTS";
const char XML_EXPRESSION[]          = "EXPRESSION";
const char XML_POSTFIXEXPR[]         = "POSTFIXEXPR";
const char XML_ACTIONS[]             = "ACTIONS";
const char XML_ACTIONS_ATTR_TRANSMITLIST[]  = "name";
const char XML_ACTIONS_ATTR_DURATION[]  = "duration";
const char XML_STATEMENT[]           = "STATEMENT";
const char XML_STATEMENT_ATTR_PRE[]  = "pretrigger";
const char XML_STATEMENT_ATTR_POST[] = "posttrigger";

const char XML_SETTINGS[]              = "SETTINGS";
const char XML_SETTINGS_ATTR_TIMEOUT[] = "timeout";
const char XML_MODE[]                  = "MODE";
const char XML_MODE_ATTR_LOGALL[]      = "log_all";
const char XML_MODE_ATTR_FIFO[]        = "fifo_mode";
const char XML_AFTERBURNER[]           = "CANPOWER";
const char XML_TARGET_EAN[]            = "TARGET_EAN";

const char XML_SCRIPT_BLOCK[]       = "SCRIPTS";
const char XML_SCRIPT[]             = "SCRIPT";
const char XML_SCRIPT_ATTR_PRIM[]   = "primary";
const char XML_SCRIPT_ATTR_EXT[]    = "script_external";
const char XML_SCRIPT_ATTR_CHAN[]   = "default_channel";
const char XML_SCRIPT_FILE[]        = "FILENAME";
const char XML_SCRIPT_PATH[]        = "PATH";



const char XML_TRANSMIT_LIST_BLOCK[]             = "TRANSMIT_LISTS";
const char XML_TRANSMIT_LIST[]                   = "TRANSMIT_LIST";
const char XML_TRANSMIT_LIST_ATTR_NAME[]         = "name";
const char XML_TRANSMIT_LIST_ATTR_MSG_DELAY[]    = "msg_delay";
const char XML_TRANSMIT_LIST_ATTR_CYCLIC_DELAY[] = "cycle_delay";
const char XML_TRANSMIT_LIST_ATTR_CYCLIC[]       = "cyclic";
const char XML_TRANSMIT_LIST_ATTR_AUTO_START[]   = "autostart";

const char XML_TRANSMIT_MSG[]      = "TRANSMIT_MESSAGE";
const char XML_TRANSMIT_MSG_NAME[] = "name";
const char XML_TRANSMIT_CHAN[]     = "channel";

const char XML_MSG_BLOCK[]        = "MESSAGES";
const char XML_MSG[]              = "MESSAGE";
const char XML_MSG_ATTR_NAME[]    = "name";
const char XML_MSG_ATTR_ID[]      = "msgid";
const char XML_MSG_ATTR_ID_EXT[]  = "can_ext";
const char XML_MSG_ATTR_DLC[]     = "dlc";
const char XML_MSG_ATTR_FLAG_EF[] = "error_frame";
const char XML_MSG_ATTR_FLAG_RF[] = "remote_frame";
const char XML_MSG_ATTR_FLAG_FD[]  = "can_fd";
const char XML_MSG_ATTR_FLAG_BRS[] = "can_fd_brs";

const char XML_FILTER_BLOCK[] = "FILTERS";
const char XML_FILTER_CHAN[] = "CHANNEL";

const char XML_FILTER_ATTR_ID[] = "msgid";
const char XML_FILTER_ATTR_ID_MIN[] = "msgid_min";
const char XML_FILTER_ATTR_DLC[] = "dlc";
const char XML_FILTER_ATTR_ID_EXT[]   = "can_ext";
const char XML_FILTER_ATTR_FLAG_STD[] = "flag_std";
const char XML_FILTER_ATTR_FLAG_EXT[] = "flag_ext";
const char XML_FILTER_ATTR_FLAG_EF[]  = "flag_errorframe";

const char XML_FILTER_ATTR_STARTBIT[]  = "startbit";
const char XML_FILTER_ATTR_BITLENGTH[] = "length";
const char XML_FILTER_ATTR_DATATYPE[]  = "datatype";
const char XML_FILTER_ATTR_ENDIAN[]    = "byteorder";
const char XML_FILTER_ATTR_DATA[]      = "data";

const char XML_FILTER_ATTR_COUNTER_THRESHOLD[] = "counter_threshold";
const char XML_FILTER_ATTR_COUNTER_MAX[] = "counter_max";

const char XML_FILTER_ATTR_ID_MSK[]   = "msg_field";
const char XML_FILTER_ATTR_ID_PROT[]  = "protocol";


const char XML_REF_MAX_ELEMENT_COUNT_ATTR[] = "KVASER_XML_MAX_NUMBER_OF_ALLOWED_ELEMENTS";
const char XML_REF_MIN_ELEMENT_COUNT_ATTR[] = "KVASER_XML_MIN_NUMBER_OF_ALLOWED_ELEMENTS";

// --------------------------------------------------------------------------
// String constants that matches the defines in logger_config_file.h
// --------------------------------------------------------------------------
const char PROTOCOL_NONE[] = "NONE";
const char PROTOCOL_J1939[] = "J1939";
const char ID_MSK_SRC[] = "SRC";
const char ID_MSK_DST[] = "DST";
const char ID_MSK_PGN[] = "PGN";

const char DATATYPE_UNSIGNED[] = "UNSIGNED";
const char DATATYPE_SIGNED[] = "SIGNED";
const char ENDIAN_BIG[] = "BIG_ENDIAN";
const char ENDIAN_LITTLE[] = "LITTLE_ENDIAN";

char const * const FilterTypeStrings[] = {
"MESSAGE_PASS",
"MESSAGE_STOP",
"MESSAGE_COUNTING_PASS",
"FLAG_PASS",
"FLAG_STOP",
"SIGNAL_PASS",
"SIGNAL_STOP",
"SIGNAL_COUNTING_PASS",
"FLAG_COUNTING_PASS"
};

#define MESSAGE_PASS_TYPE           0
#define MESSAGE_STOP_TYPE           1
#define MESSAGE_COUNTING_PASS_TYPE  2
#define FLAGS_PASS_TYPE             3
#define FLAGS_STOP_TYPE             4
#define SIGNAL_PASS_TYPE            5
#define SIGNAL_STOP_TYPE            6
#define SIGNAL_COUNTING_PASS_TYPE   7
#define FLAGS_COUNTING_PASS_TYPE    8

const int  FilterTypeValues[] = {
MESSAGE_PASS_TYPE,
MESSAGE_STOP_TYPE,
MESSAGE_COUNTING_PASS_TYPE,
FLAGS_PASS_TYPE,
FLAGS_STOP_TYPE,
SIGNAL_PASS_TYPE,
SIGNAL_STOP_TYPE,
SIGNAL_COUNTING_PASS_TYPE,
FLAGS_COUNTING_PASS_TYPE
};

const unsigned int FilterTypeLength = sizeof(FilterTypeValues) / sizeof(int);

char const * const TrigvarTypeStrings[] = {
"TRIGVAR_TYPE_DISABLED",
"TRIGGER_MSG_ID",
"TRIGGER_MSG_DLC",
"TRIGGER_MSG_ERROR_FRAME",
"TRIGGER_SIGVAL",
"TRIGGER_EXTERNAL",
"TRIGGER_TIMER",
"TRIGGER_DISK_FULL",
"TRIGGER_STARTUP"};


const int TrigvarTypeValues[] = {
TRIGVAR_TYPE_DISABLED,
TRIGVAR_TYPE_MSG_ID,
TRIGVAR_TYPE_MSG_DLC,
TRIGVAR_TYPE_MSG_FLAG,
TRIGVAR_TYPE_SIGVAL,
TRIGVAR_TYPE_EXTERNAL,
TRIGVAR_TYPE_TIMER,
TRIGVAR_TYPE_DISK_FULL,
TRIGVAR_TYPE_STARTUP};

const unsigned int TrigvarTypeLength = sizeof(TrigvarTypeValues) / sizeof(int);

char const * const ActionTypeStrings[] = {
"ACTION_START_LOG",
"ACTION_STOP_LOG",
"ACTION_SET_ACTIVE_FILTER_MASK_LOW_CH0",
"ACTION_SET_ACTIVE_FILTER_MASK_LOW_CH1",
"ACTION_ACTIVATE_TIMER",
"ACTION_DEACTIVATE_TIMER",
"ACTION_STOP_LOG_COMPLETELY",
"ACTION_FLASH_LEDS",
"ACTION_EXTERNAL_PULSE",
"ACTION_SET_ACTIVE_FILTER_MASK_HIGH_CH0",
"ACTION_SET_ACTIVE_FILTER_MASK_HIGH_CH1",
"ACTION_ACTIVATE_AUTO_TRANSMIT_LIST",
"ACTION_DEACTIVATE_AUTO_TRANSMIT_LIST",
"ACTION_START_XCP"};

const int ActionTypeValues[] = {
ACTION_START_LOG,
ACTION_STOP_LOG,
ACTION_SET_ACTIVE_FILTER_MASK_LOW_CH0,
ACTION_SET_ACTIVE_FILTER_MASK_LOW_CH1,
ACTION_ACTIVATE_TIMER,
ACTION_DEACTIVATE_TIMER,
ACTION_STOP_LOG_COMPLETELY,
ACTION_FLASH_LEDS,
ACTION_EXTERNAL_PULSE,
ACTION_SET_ACTIVE_FILTER_MASK_HIGH_CH0,
ACTION_SET_ACTIVE_FILTER_MASK_HIGH_CH1,
ACTION_ACTIVATE_AUTO_TRANSMIT_CHAIN,
ACTION_DEACTIVATE_AUTO_TRANSMIT_CHAIN,
ACTION_START_XCP};

const unsigned int ActionTypeLength = sizeof(ActionTypeValues) / sizeof(int);

// ExternalTrigVar level
char const * const TrigLevelStrings[] = {
"TRIG_EXTERNAL_LEVEL_LO_HI",
"TRIG_EXTERNAL_LEVEL_HI_LO"};

const int TrigLevelValues[] = {
TRIG_EXTERNAL_LEVEL_LO_HI,
TRIG_EXTERNAL_LEVEL_HI_LO};

const unsigned int TrigLevelLength = sizeof(TrigLevelValues) / sizeof(int);

char const * const OnDataTypeStrings[] = {
"ON_DATA_EQUAL_TO",
"ON_DATA_NOT_EQUAL_TO",
"ON_DATA_LARGER_THAN",
"ON_DATA_SMALLER_THAN",
"ON_DATA_CHANGE_TO",
"ON_DATA_CHANGE_FROM",
"ON_DATA_CHANGE",
"ON_DATA_LARGER_THAN_OR_EQUAL",
"ON_DATA_SMALLER_THAN_OR_EQUAL"};

const int OnDataTypeValues[] = {
ON_DATA_EQUAL_TO,
ON_DATA_NOT_EQUAL_TO,
ON_DATA_LARGER_THAN,
ON_DATA_SMALLER_THAN,
ON_DATA_CHANGE_TO,
ON_DATA_CHANGE_FROM,
ON_DATA_CHANGE,
ON_DATA_LARGER_THAN_OR_EQUAL,
ON_DATA_SMALLER_THAN_OR_EQUAL};

const unsigned int OnDataTypeLength = sizeof(OnDataTypeValues) / sizeof(int);

char const * const XCPProtocolStrings[] = {
"PROTOCOL_CCP",
"PROTOCOL_XCP"};

const int XCPProtocolValues[] = {
PROTOCOL_CCP,
PROTOCOL_XCP};

const unsigned int XCPProtocolLength = sizeof(XCPProtocolValues) / sizeof(int);


char const * const BlockStrings[] = {
"BLOCK_ID_VERSION",
"BLOCK_ID_BUSPARAMS",
"BLOCK_ID_TRIGGER",
"BLOCK_ID_FILTER",
"BLOCK_ID_IDENT",
"BLOCK_ID_AUTO_TRANSMIT",
"BLOCK_ID_XCP",
"BLOCK_ID_AUTOBAUD",
"BLOCK_ID_SCRIPT",
"BLOCK_ID_AFTERBURNER",
"BLOCK_ID_END",
"BLOCK_ID_BUSPARAMS_FD",
"BLOCK_ID_TRIGGER_FD",
"BLOCK_ID_FILTER_FD",
"BLOCK_ID_AUTO_TRANSMIT_FD",
"BLOCK_ID_SCRIPT_SD"
};

const int BlockValues[] = {
BLOCK_ID_VERSION,
BLOCK_ID_BUSPARAMS,
BLOCK_ID_TRIGGER,
BLOCK_ID_FILTER,
BLOCK_ID_IDENT,
BLOCK_ID_AUTO_TRANSMIT,
BLOCK_ID_XCP,
BLOCK_ID_AUTOBAUD,
BLOCK_ID_SCRIPT,
BLOCK_ID_AFTERBURNER,
BLOCK_ID_END,
BLOCK_ID_BUSPARAMS_FD,
BLOCK_ID_TRIGGER_FD,
BLOCK_ID_FILTER_FD,
BLOCK_ID_AUTO_TRANSMIT_FD,
BLOCK_ID_SCRIPT_SD
};

const unsigned int BlockLength = sizeof(BlockValues) / sizeof(int);



#ifdef __cplusplus
}
#endif

#endif //KVAMEMOLIBXMLCONST_H
