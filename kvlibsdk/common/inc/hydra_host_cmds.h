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

#ifndef HYDRA_HOST_CMDS_H_
#define HYDRA_HOST_CMDS_H_

# include <linux/types.h>


#ifdef HYDRA_PRIVATE
#   include "hydra_host_private_cmds.h"
#endif

#define CMD_RX_STD_MESSAGE                12
#define CMD_TX_CAN_MESSAGE                33
#define CMD_RX_EXT_MESSAGE                14
#define CMD_SET_BUSPARAMS_REQ             16
#define CMD_GET_BUSPARAMS_REQ             17
#define CMD_GET_BUSPARAMS_RESP            18
#define CMD_GET_CHIP_STATE_REQ            19
#define CMD_CHIP_STATE_EVENT              20
#define CMD_SET_DRIVERMODE_REQ            21
#define CMD_GET_DRIVERMODE_REQ            22
#define CMD_GET_DRIVERMODE_RESP           23
#define CMD_RESET_CHIP_REQ                24
#define CMD_RESET_CARD_REQ                25
#define CMD_START_CHIP_REQ                26
#define CMD_START_CHIP_RESP               27
#define CMD_STOP_CHIP_REQ                 28
#define CMD_STOP_CHIP_RESP                29
#define CMD_READ_CLOCK_REQ                30
#define CMD_READ_CLOCK_RESP               31
#define CMD_GET_CARD_INFO_2               32
// 33 may be used - NOT see CMD_TX_CAN_MESSAGE
#define CMD_GET_CARD_INFO_REQ             34
#define CMD_GET_CARD_INFO_RESP            35
#define CMD_GET_INTERFACE_INFO_REQ        36
#define CMD_GET_INTERFACE_INFO_RESP       37
#define CMD_GET_SOFTWARE_INFO_REQ         38
#define CMD_GET_SOFTWARE_INFO_RESP        39
#define CMD_GET_BUSLOAD_REQ               40
#define CMD_GET_BUSLOAD_RESP              41
#define CMD_RESET_STATISTICS              42
#define CMD_CHECK_LICENSE_REQ             43
#define CMD_CHECK_LICENSE_RESP            44
#define CMD_ERROR_EVENT                   45
// 46, 47 reserved
#define CMD_FLUSH_QUEUE                   48
#define CMD_RESET_ERROR_COUNTER           49
#define CMD_TX_ACKNOWLEDGE                50
#define CMD_CAN_ERROR_EVENT               51

#define CMD_MEMO_GET_DATA                 52
#define CMD_MEMO_PUT_DATA                 53
#define CMD_MEMO_PUT_DATA_START           54
#define CMD_MEMO_ASYNCOP_START            55
#define CMD_MEMO_ASYNCOP_GET_DATA         56
#define CMD_MEMO_ASYNCOP_CANCEL           57
#define CMD_MEMO_ASYNCOP_FINISHED         58
#define CMD_DISK_FULL_INFO                59
#define CMD_TX_REQUEST                               60
#define CMD_SET_HEARTBEAT_RATE_REQ                   61
#define CMD_HEARTBEAT_RESP                           62

#define CMD_SET_AUTO_TX_BUFFER                       63

#define CMD_GET_EXTENDED_INFO                        64
#define CMD_TCP_KEEPALIVE                            65
#define CMD_FLUSH_QUEUE_RESP                         66
#define CMD_HYDRA_TX_INTERVAL_REQ                    67
#define CMD_HYDRA_TX_INTERVAL_RESP                   68
#define CMD_SET_BUSPARAMS_FD_REQ                     69
#define CMD_SET_BUSPARAMS_FD_RESP                    70
#define CMD_GET_BUSPARAMS_TQ_REQ                     71
#define CMD_AUTO_TX_BUFFER_REQ                       72
#define CMD_AUTO_TX_BUFFER_RESP                      73
#define CMD_SET_TRANSCEIVER_MODE_REQ                 74
#define CMD_TREF_SOFNR                               75
#define CMD_SOFTSYNC_ONOFF                           76
#define CMD_USB_THROTTLE                             77
#define CMD_SOUND                                    78
#define CMD_LOG_TRIG_STARTUP                         79
#define CMD_SELF_TEST_REQ                            80
#define CMD_SELF_TEST_RESP                           81
#define CMD_SET_BUSPARAMS_TQ_RESP                    84
#define CMD_SET_BUSPARAMS_RESP                       85

#define CMD_SET_IO_PORTS_REQ                         86
#define CMD_GET_IO_PORTS_REQ                         87
#define CMD_GET_IO_PORTS_RESP                        88

#define CMD_TRANSPORT_REQ                            91
#define CMD_TRANSPORT_RESP                           92
#define CMD_KDI                                      93

#define CMD_STORAGE                                  94

#define CMD_GET_CAPABILITIES_REQ                     95
#define CMD_GET_CAPABILITIES_RESP                    96
#define CMD_GET_TRANSCEIVER_INFO_REQ                 97
#define CMD_GET_TRANSCEIVER_INFO_RESP                98
#define CMD_MEMO_CONFIG_MODE                         99
#define CMD_GET_BUSPARAMS_TQ_RESP                    100
#define CMD_LED_ACTION_REQ                          101
#define CMD_LED_ACTION_RESP                         102
#define CMD_INTERNAL_DUMMY                          103
#define CMD_READ_USER_PARAMETER                     104
#define CMD_MEMO_CPLD_PRG                           105
#define CMD_LOG_MESSAGE                             106
#define CMD_LOG_TRIG                                107
#define CMD_LOG_RTC_TIME                            108

#define CMD_SCRIPT_ENVVAR_CTRL_REQ                  109
#define CMD_SCRIPT_ENVVAR_CTRL_RESP                 110
#define CMD_SCRIPT_ENVVAR_TRANSFER_CTRL_REQ         111 // PC wants to set value in VM
#define CMD_SCRIPT_ENVVAR_TRANSFER_CTRL_RESP        112 // PC wants to set value in VM
#define CMD_SCRIPT_ENVVAR_TRANSFER_BULK             113 // PC wants to set value in VM


#define CMD_SCRIPT_CTRL_REQ                         116
#define CMD_SCRIPT_CTRL_RESP                        117

// 118 reserved

#define CMD_IMP_KEY                                 119

#define CMD_PRINTF                                  120
#define RES_PRINTF                                  (CMD_PRINTF + 128)

#define TRP_DATA                                    121

#define CMD_REGISTER_HE_REQ                         122
#define CMD_REGISTER_HE_RESP                        123
#define CMD_QUERY_ADDR_HE_REQ                       124
#define CMD_QUERY_ADDR_HE_RESP                      125
#define CMD_LISTEN_TO_HE_REQ                        126
#define CMD_LISTEN_TO_HE_RESP                       127
#define CMD_QUERY_NEXT_HE_REQ                       128
#define CMD_QUERY_NEXT_HE_RESP                      129

#define CMD_MEMORY_READ_REQ                         130
#define CMD_MEMORY_READ_RESP                        131
#define CMD_MEMORY_WRITE_REQ                        132
#define CMD_MEMORY_WRITE_RESP                       133
#define CMD_MEMORY_SEARCH_REQ                       134
#define CMD_MEMORY_SEARCH_RESP                      135

#define CMD_MEASURE                                 136

#define CMD_FATAL_ERROR                             137

#define CMD_LOG_ACTION                              138

#define CMD_SET_BUSPARAMS_TQ_REQ                    139

#define CMD_IO_TRIG_REQ                             148
#define CMD_IO_TRIG_RESP                            149
#define CMD_IO_TRIG_MSG                             150

#define CMD_IO_PORT_INFO_REQ                        151
#define CMD_IO_PORT_INFO_RESP                       152
#define CMD_IO_PORT_CTRL_REQ                        153
#define CMD_IO_PORT_CTRL_RESP                       154

#define CMD_GET_FILE_COUNT_REQ                      158
#define CMD_GET_FILE_COUNT_RESP                     159
#define CMD_GET_FILE_NAME_REQ                       160
#define CMD_GET_FILE_NAME_RESP                      161

#define CMD_GET_NETWORK_DEVICE_NAME_REQ             162
#define CMD_GET_NETWORK_DEVICE_NAME_RESP            163

#define CMD_DEVICE_PING_REQ                         164
#define CMD_DEVICE_PING_RESP                        165

#define CMD_LOG_TRIG_FD                             172
#define CMD_LOG_TRIG_STARTUP_FD                     173
#define CMD_LOG_RTC_TIME_FD                         174
#define CMD_LOG_VERSION_FD                          175

#define CMD_IO_PIN_CMD                              190

#define CMD_MAP_CHANNEL_REQ                         200
#define CMD_MAP_CHANNEL_RESP                        201
#define CMD_GET_SOFTWARE_DETAILS_REQ                202
#define CMD_GET_SOFTWARE_DETAILS_RESP               203

#define CMD_SET_DEVICE_MODE                         204
#define CMD_GET_DEVICE_MODE                         205

#define CMD_UNKNOWN_COMMAND                         209


#define FD_CMD_RANGE                                0xE0
#define FD_CMD_MASK                                 0xF0
#define CMD_HAS_REFPOOL(X) (((X) & FD_CMD_MASK) == FD_CMD_RANGE)

#define CMD_TX_CAN_MESSAGE_FD                       224 //0xE0
#define CMD_TX_ACKNOWLEDGE_FD                       225
#define CMD_RX_MESSAGE_FD                           226
#define CMD_AUTOTX_MESSAGE_FD                       227
#define CMD_RESERVED_FD_4                           228
#define CMD_RESERVED_FD_5                           229
#define CMD_RESERVED_FD_6                           230
#define CMD_RESERVED_FD_7                           231
#define CMD_RESERVED_FD_8                           232
#define CMD_RESERVED_FD_9                           233
#define CMD_RESERVED_FD_A                           234
#define CMD_RESERVED_FD_B                           235
#define CMD_RESERVED_FD_C                           236
#define CMD_RESERVED_FD_D                           237
#define CMD_RESERVED_FD_E                           238
#define CMD_RESERVED_FD_F                           239
#define CMD_EXTENDED                                255


#define MEMO_STATUS_SUCCESS         0
#define MEMO_STATUS_MORE_DATA       1
#define MEMO_STATUS_UNKNOWN_COMMAND 2
#define MEMO_STATUS_FAILED          3
#define MEMO_STATUS_EOF             4



typedef struct {
  uint8_t     d[20];
} hdata20_t;

// These are the diagnostics data sent via CMD_TCP_KEEPALIVE.
typedef struct {
  uint8_t     version;
  union {
    struct {
      uint8_t     medium;
      uint8_t     tx_queue_max;
      int8_t      rssi_min;
      uint16_t    rx_wnd_min;
      uint16_t    tx_wnd_min;
      uint16_t    rtt_max;
    } v1;
    struct {
      uint8_t     d[15];
    } diagRawData16_t;
  };
} hDiagnostics;

#define DIAGNOSTICS_VERSION 1

#define SCRIPT_CTRL_ERR_SUCCESS               0
#define SCRIPT_CTRL_ERR_NO_MORE_PROCESSES     1
#define SCRIPT_CTRL_ERR_FILE_NOT_FOUND        2
#define SCRIPT_CTRL_ERR_OPEN_FILE_ERR         3
#define SCRIPT_CTRL_ERR_OPEN_FILE_NO_MEM      4
#define SCRIPT_CTRL_ERR_FILE_READ_ERR         5
#define SCRIPT_CTRL_ERR_LOAD_FILE_ERR         6
#define SCRIPT_CTRL_ERR_OUT_OF_CODE_MEM       7
#define SCRIPT_CTRL_ERR_FILE_REWIND_FAIL      8
#define SCRIPT_CTRL_ERR_LOAD_FAIL             9
#define SCRIPT_CTRL_ERR_SETUP_FAIL           10
#define SCRIPT_CTRL_ERR_SETUP_FUN_TABLE_FAIL 11
#define SCRIPT_CTRL_ERR_SETUP_PARAMS_FAIL    12
#define SCRIPT_CTRL_ERR_PROCESSES_NOT_FOUND  13
#define SCRIPT_CTRL_ERR_START_FAILED         14
#define SCRIPT_CTRL_ERR_STOP_FAILED          15
#define SCRIPT_CTRL_ERR_SPI_BUSY             16
#define SCRIPT_CTRL_ERR_PROCESS_NOT_STOPPED  17
#define SCRIPT_CTRL_ERR_PROCESS_NOT_RUNNING  18
#define SCRIPT_CTRL_ERR_ENVVAR_NOT_FOUND     19

#define SCRIPT_CTRL_ERR_UNKNOWN_COMMAND      20
#define SCRIPT_CTRL_ERR_PROCESS_NOT_LOADED   21

#define SCRIPT_CTRL_ERR_COMPILER_VERSION     22
#define SCRIPT_CTRL_ERR_INVALID_PARAMETER    23
#define SCRIPT_CTRL_ERR_NOT_IMPLEMENTED      43


// cmdScriptCtrlReXX payload union
typedef union
{
  struct {                          // SCRIPT_CMD_SCRIPT_EVENT
    int32_t     eventType;
    int32_t     eventNo;
    uint32_t    eventData;
  } cmdScriptEvent;

  struct {                          //  SCRIPT_CMD_SCRIPT_LOAD_REMOTE_DATA
    char length;
    char data[15];
  } cmdScriptLoad;

  struct {                          // SCRIPT_CMD_SCRIPT_LOAD
    uint8_t     d[16];
  } cmdRawData16_t;

  struct {                          // SCRIPT_CMD_SCRIPT_QUERY_STATUS
    uint32_t    scriptStatus;
  } cmdScriptInfo;

  struct {                          // SCRIPT_CMD_SCRIPT_STOP
    int8_t      mode;
  } cmdScriptStop;

} hscriptCtrlPayload;


typedef struct {
  uint8_t     scriptNo;       // which script to deliver command to
  uint8_t     channel;
  uint8_t     reserved[2];
  uint32_t     subCmd;        // command specifys what the data contains
  hscriptCtrlPayload  payload;       // data associated with event
} hcmdScriptCtrlReq;

typedef struct {
  uint8_t     scriptNo;
  uint8_t     reserved[3];
  uint32_t     subCmd;
  uint32_t     status;
  hscriptCtrlPayload  payload;
} hcmdScriptCtrlResp;


#define M32_ENVVAR_GET_INFO      1
#define M32_ENVVAR_GET_MAX_SIZE  2

#define M32_ENVVAR_TYPE_UNKNOWN  -1
#define M32_ENVVAR_TYPE_INT       1
#define M32_ENVVAR_TYPE_FLOAT     2
#define M32_ENVVAR_TYPE_STRING    3

// union to transfer data to/from device in cmdEnvvarCtrlReXXX
typedef union {
  struct {
    uint32_t    hash;
    int32_t     type;
    int32_t     length;
  } cmdGetInfo;   // M32_ENVVAR_GET_INFO
  struct {
    int32_t     maxLength;
  } cmdMaxLength; // M32_ENVVAR_GET_MAX_SIZE
} henvvarData;


typedef struct {
  uint8_t     channel;
  uint8_t     reserved[3];
  uint32_t    subCmd;         // command
  uint8_t     data[16];
} hcmdEnvvarCtrlReq;

typedef struct {
  uint8_t     channel;
  uint8_t     reserved[3];
  uint32_t    subCmd;         // command
  uint32_t    status;
  uint8_t     data[16];
} hcmdEnvvarCtrlResp;

typedef struct {
  uint8_t     subCmd;
  uint8_t     reserved;
  uint16_t    data2;           // Sector count for disk reads
  uint32_t    data1;           // Sector number for disk reads
  uint8_t     data[20];
} hcmdMemoGetDataReq;

typedef struct {
  uint8_t     subCmd;
  uint8_t     dataLen;
  uint16_t    offset;
  uint8_t     data[20];        // Data, or status (dioStat in 0 and lioStat in 1)
  uint8_t     status;          // MEMO_STATUS_xxx
  uint8_t     reserved;
} hcmdMemoGetDataResp;

typedef struct {
  uint8_t     subCmd;
  uint8_t     reserved[3];
  uint32_t    data1;           // Sector number for disk reads
  uint16_t    data2;           // Sector count for disk reads
  uint8_t     reserved2[14];
} hcmdMemoPutDataStartReq;

typedef struct {
  uint8_t     subCmd;
  uint8_t     dataLen;
  uint16_t    offset;
  uint8_t     data[20];
  uint16_t    reserved;
} hcmdMemoPutDataReq;

typedef struct {
  uint8_t     subCmd;
  uint8_t     dataLen;
  uint8_t     status;
  uint8_t     reserved;
  uint8_t     data[20];
} hcmdMemoPutDataResp;

typedef struct {
  uint8_t     subCmd;
  uint8_t     reserved[3];
} hcmdMemoAsyncopStartReq;

typedef struct {
  uint8_t     subCmd;
  uint8_t     status;
  uint16_t    reserved;
} hcmdMemoAsyncopStartResp;

typedef struct {
  uint8_t     subCmd;
  uint8_t     status;
  uint16_t    reserved;
} hcmdMemoAsyncopFinishedResp;

typedef struct {
  uint8_t     subCmd;
  uint8_t     reserved[3];
} hcmdMemoAsyncopGetDataReq;

typedef struct {
  uint8_t     subCmd;
  uint8_t     status;
  uint8_t     reserved[2];
  uint8_t     data[20];
} hcmdMemoAsyncopGetDataResp;

typedef struct {
  uint8_t     subCmd;
  uint8_t     reserved[3];
} hcmdMemoAsyncopCancelReq;


// *All* subcommands to be in one number series
#define MEMO_SUBCMD_GET_FS_INFO       1   // Get DOS filesys info; for get_data
#define MEMO_SUBCMD_GET_DISK_INFO_A   2   // Get disk info; for get_data
#define MEMO_SUBCMD_GET_DISK_INFO_B   3   // Get logio info; for get_data

#define MEMO_SUBCMD_READ_PHYSICAL_SECTOR         4
#define MEMO_SUBCMD_WRITE_PHYSICAL_SECTOR        5
#define MEMO_SUBCMD_ERASE_PHYSICAL_SECTOR        6
#define MEMO_SUBCMD_READ_LOGICAL_SECTOR          7
#define MEMO_SUBCMD_WRITE_LOGICAL_SECTOR         8
#define MEMO_SUBCMD_ERASE_LOGICAL_SECTOR         9

#define MEMO_SUBCMD_FORMAT_DISK      10   // Format disk (FAT16 or -32) asyncop
#define MEMO_SUBCMD_INIT_DISK        11   // Create logdata.kmf, asyncop
#define MEMO_SUBCMD_CLEAR_DATA       12   // Clear logdata.kmf, asyncop

#define MEMO_SUBCMD_GET_MISC_INFO    13   // for get_data
#define MEMO_SUBCMD_GET_RTC_INFO     14   // for get_data
#define MEMO_SUBCMD_PUT_RTC_INFO     15   // for put_data

#define MEMO_SUBCMD_GET_FS_INFO_B    16   // Get various filesystem info

#define MEMO_SUBCMD_FASTREAD_PHYSICAL_SECTOR         17
#define MEMO_SUBCMD_FASTREAD_LOGICAL_SECTOR          18

#define MEMO_SUBCMD_OPEN_FILE           19
#define MEMO_SUBCMD_READ_FILE           20
#define MEMO_SUBCMD_CLOSE_FILE          21
#define MEMO_SUBCMD_WRITE_FILE          22
#define MEMO_SUBCMD_DELETE_FILE         23


typedef struct {
  uint16_t    fat_size;                // Size of the FAT, in sectors
  uint16_t    fat_type;                // 12 or 16 depending on FAT type.
  uint16_t    dir_entries;             // Number of directory entries in the root dir
  uint16_t    cluster_size;            // Two-logarithm of the cluster size in sectors
  uint32_t    fat1_start;              // First FAT starts in this sector
  uint32_t    first_data_sector;       // First sector available for data
  uint32_t    last_data_sector;        // Last sector available for data
} hMemoDataFsInfo;

typedef struct {
  uint32_t    logfile_sectors;          // Number of sectors in log file
  uint32_t    first_param_sector;       // First sector for param.lif
  uint32_t    last_param_sector;        // Last sector for param.lif
  uint32_t    first_dbase_sector;       // First sector for databases (if any)
  uint32_t    last_dbase_sector;        // Last sector for databases (if any)
} hMemoDataFsInfoB;

typedef struct {
  uint8_t     productRev;
  uint8_t     oemId[2];
  uint8_t     reserved1;
  uint32_t    mfgId;
  char        productName[10];
  uint16_t    dateCode;
} hMemoDataDiskInfoA;

typedef struct {
  uint32_t    serialNumber;
  uint8_t     disk_type;
  uint8_t     version;
  uint8_t     read_time;
  uint8_t     wr_factor;
  uint8_t     file_format;
  uint8_t     erase_value;
  uint16_t    read_blk_size;
  uint16_t    wr_blk_size;
  uint16_t    trans_speed;
  uint32_t    data_size;
} hMemoDataDiskInfoB;

#define MEMO_POWER_BAT_FAULT      0x01    // Battery fault of some kind
#define MEMO_POWER_BAT_CHARGING   0x02    // Battery is charging
#define MEMO_POWER_BAT_POWER_OK   0x04    // Battery power OK
#define MEMO_POWER_EXTPOWER_OK    0x08    // External power OK
#define MEMO_POWER_USBPOWER_OK    0x10    // USB power OK
#define MEMO_POWER_BAT_FAULT_NTC  0x20    // NTC Battery fault

#define MEMO_TEMPERATURE_FAILURE  0       // Failed measuring temperature
#define MEMO_TEMPERATURE_MEMO2    1       // Temperature is encoded as in Memo2 (NTC 10K connected to 3.3V)
#define MEMO_TEMPERATURE_MHYDRA   2
#define MEMO_TEMPERATURE_MEMO5    3

typedef struct {
  uint8_t     diskPresent;
  uint8_t     configMode;
  uint8_t     diskWriteProtected;
  uint8_t     temperatureEncoding;    // Encoding of 'temperature' field
  uint16_t    powerStatus;            // MEMO_POWER_xxx flags
  uint16_t    temperature;            // Temperature, raw value
  uint8_t     cpldVersion;            // CPLD version number
  uint8_t     reserved[1];
  uint16_t    reserved1;
  uint16_t    battery;                // Battery voltage, raw value
  uint8_t     reserved2[6];
} hMemoDataMiscInfo;

typedef struct {
  uint8_t     second;
  uint8_t     minute;
  uint8_t     hour;
  uint8_t     day;
  uint8_t     month;
  uint8_t     year;
  uint16_t    padding2;
} hMemoDataRtcInfo;

typedef struct {
  uint32_t    maxDataSize;
  uint32_t    dbaseSpace;
  uint32_t    reserveSpace;
  uint8_t     fileSystem;
  uint8_t     reserved[7];
} hMemoInitDiskReq;

typedef struct {
  uint8_t     dioStatus;
  uint8_t     lioStatus;
  uint8_t     reserved[10];
} hMemoInitDiskResp;

typedef struct hcanErrorFrameData_s {
  uint8_t     busStatus;
  uint8_t     errorFactor;
  uint8_t     txErrorCounter;
  uint8_t     rxErrorCounter;
} hcanErrorFrameData_t;





#define SOUND_SUBCOMMAND_INIT         0
#define SOUND_SUBCOMMAND_BEEP         1
#define SOUND_SUBCOMMAND_NOTE         2
#define SOUND_SUBCOMMAND_DISABLE      3

#define LED_SUBCOMMAND_ALL_LEDS_ON    0
#define LED_SUBCOMMAND_ALL_LEDS_OFF   1
#define LED_SUBCOMMAND_LED_0_ON       2
#define LED_SUBCOMMAND_LED_0_OFF      3
#define LED_SUBCOMMAND_LED_1_ON       4
#define LED_SUBCOMMAND_LED_1_OFF      5
#define LED_SUBCOMMAND_LED_2_ON       6
#define LED_SUBCOMMAND_LED_2_OFF      7
#define LED_SUBCOMMAND_LED_3_ON       8
#define LED_SUBCOMMAND_LED_3_OFF      9
#define LED_SUBCOMMAND_LED_4_ON       10
#define LED_SUBCOMMAND_LED_4_OFF      11
#define LED_SUBCOMMAND_LED_5_ON       12
#define LED_SUBCOMMAND_LED_5_OFF      13
#define LED_SUBCOMMAND_LED_6_ON       14
#define LED_SUBCOMMAND_LED_6_OFF      15
#define LED_SUBCOMMAND_LED_7_ON       16
#define LED_SUBCOMMAND_LED_7_OFF      17
#define LED_SUBCOMMAND_LED_8_ON       18
#define LED_SUBCOMMAND_LED_8_OFF      19
#define LED_SUBCOMMAND_LED_9_ON       20
#define LED_SUBCOMMAND_LED_9_OFF      21
#define LED_SUBCOMMAND_LED_10_ON      22
#define LED_SUBCOMMAND_LED_10_OFF     23
#define LED_SUBCOMMAND_LED_11_ON      24
#define LED_SUBCOMMAND_LED_11_OFF     25

#define CONFIG_DATA_CHUNK                             24


//===========================================================================
// Flags
//===========================================================================

//////////////////////////
// CAN message flags
#define MSGFLAG_ERROR_FRAME         0x01        // Msg is a bus error
#define MSGFLAG_OVERRUN             0x02        // Some kind of overrun occured
#define MSGFLAG_NERR                0x04        // NERR active during this msg
#define MSGFLAG_WAKEUP              0x08        // Msg rcv'd in wakeup mode
#define MSGFLAG_REMOTE_FRAME        0x10        // Msg is a remote frame
#define MSGFLAG_EXTENDED_ID         0x20        // Extended id.
#define MSGFLAG_TX                  0x40        // TX acknowledge
#define MSGFLAG_TXRQ                0x80        // TX request
#define MSGFLAG_SSM_NACK        0x001000        // Single shot transmission failed.
#define MSGFLAG_ABL             0x002000        // Single shot transmission failed due to ArBitration Loss.
#define MSGFLAG_EDL             0x010000        // Obsolete, use MSGFLAG_FDF instead
#define MSGFLAG_FDF             0x010000        // Message is an FD message (CAN FD)
#define MSGFLAG_BRS             0x020000        // Message is sent/received with bit rate switch (CAN FD)
#define MSGFLAG_ESI             0x040000        // Sender of the message is in error passive mode (CAN FD)

#define MSGFLAG_EXT                 0x20        // Extended message


// This one is added to the identifier to mean an extended CAN identifier
// #define DRIVER_EXT_FLAG             0x80000000


/////////////////////////
// Chip status flags
#define BUSSTAT_FLAG_ERR_ACTIVE  0x00
#define BUSSTAT_FLAG_RESET       0x01
#define BUSSTAT_FLAG_BUS_ERROR   0x10
#define BUSSTAT_FLAG_ERR_PASSIVE 0x20
#define BUSSTAT_FLAG_BUSOFF      0x40


////////////////////////
// Driver modes
#define DRIVERMODE_NORMAL           0x01
#define DRIVERMODE_SILENT           0x02
#define DRIVERMODE_SELFRECEPTION    0x03
#define DRIVERMODE_OFF              0x04

////////////////////////
// Transceiver (logical) types. Must be the same as TRANSCEIVER_xxx in e.g. canlib.h.
#define HYDRA_TRANSCEIVER_TYPE_UNKNOWN         0
#define HYDRA_TRANSCEIVER_TYPE_251             1
#define HYDRA_TRANSCEIVER_TYPE_252             2 // 252/1053/1054 w/o opto
#define HYDRA_TRANSCEIVER_TYPE_SWC             6 // J2411 type
#define HYDRA_TRANSCEIVER_TYPE_1054_OPTO      11 // 1054 with optical isolation
#define HYDRA_TRANSCEIVER_TYPE_SWC_OPTO       12 // J2411 type with optical isolation
#define HYDRA_TRANSCEIVER_TYPE_1050           14 // TJA1050
#define HYDRA_TRANSCEIVER_TYPE_1050_OPTO      15 // TJA1050 with optical isolation
#define HYDRA_TRANSCEIVER_TYPE_LIN            19 // LIN

// Transceiver line modes. Must be the same as in canlib.h
#define HYDRA_TRANSCEIVER_LINEMODE_NA          0  // Not Affected/Not available.
#define HYDRA_TRANSCEIVER_LINEMODE_SWC_SLEEP   4  // SWC Sleep Mode.
#define HYDRA_TRANSCEIVER_LINEMODE_SWC_NORMAL  5  // SWC Normal Mode.
#define HYDRA_TRANSCEIVER_LINEMODE_SWC_FAST    6  // SWC High-Speed Mode.
#define HYDRA_TRANSCEIVER_LINEMODE_SWC_WAKEUP  7  // SWC Wakeup Mode.
#define HYDRA_TRANSCEIVER_LINEMODE_SLEEP       8  // Sleep mode for those supporting it.
#define HYDRA_TRANSCEIVER_LINEMODE_NORMAL      9  // Normal mode (the inverse of sleep mode) for those supporting it.
#define HYDRA_TRANSCEIVER_LINEMODE_STDBY      10  // Standby for those who support it
// Transceiver resnet modes. Not supported.
#define HYDRA_TRANSCEIVER_RESNET_NA            0


////////////////////////////
// Error codes.
// Used in CMD_ERROR_EVENT.
#define FIRMWARE_ERR_OK               0     // No error.
#define FIRMWARE_ERR_CAN              1     // CAN error, addInfo1 contains error code.
#define FIRMWARE_ERR_NVRAM_ERROR      2     // Flash error
#define FIRMWARE_ERR_NOPRIV           3     // No privilege for attempted operation
#define FIRMWARE_ERR_ILLEGAL_ADDRESS  4     // Illegal RAM/ROM address specified
#define FIRMWARE_ERR_UNKNOWN_CMD      5     // Unknown command or subcommand
#define FIRMWARE_ERR_FATAL            6     // A severe error. addInfo1 contains error code.
#define FIRMWARE_ERR_CHECKSUM_ERROR   7     // Downloaded code checksum mismatch
#define FIRMWARE_ERR_QUEUE_LEVEL      8     // Tx queue levels (probably driver error)
#define FIRMWARE_ERR_PARAMETER        9     // Parameter error, addInfo1 contains offending command

// Maximum length of a command. Do not change this.
#define MAX_CMD_LEN                   32


// For CMD_READ_CLOCK_REQ
#define READ_CLOCK_NOW                0x01  // Provide a fast, unsynchronized response.

// For CMD_GET_SOFTWARE_OPTIONS (max 32 flags here)
#define SWOPTION_CONFIG_MODE          0x01L // Memorator in config mode.
#define SWOPTION_AUTO_TX_BUFFER       0x02L // Firmware has auto tx buffers
#define SWOPTION_BETA                 0x04L // Firmware is a beta release
#define SWOPTION_RC                   0x08L // Firmware is a release candidate
#define SWOPTION_BAD_MOOD             0x10L // Firmware detected config error or the like
#define SWOPTION_CPU_FQ_MASK          0x60L
#define SWOPTION_80_MHZ_CLK           0x20L // hires timers run at 80 MHZ
#define SWOPTION_24_MHZ_CLK           0x40L // hires timers run at 24 MHZ
#define SWOPTION_DELAY_MSGS          0x100L // Firmware supports delay messages.
#define SWOPTION_USE_HYDRA_EXT       0x200L // Firmware supports extended Hydra commands
#define SWOPTION_CANFD_CAP           0x400L // Software supports CAN-FD.
#define SWOPTION_NONISO_CAP          0x800L // Software supports NON-ISO.
#define SWOPTION_CAP_REQ            0x1000L // Software supporte CMD_GET_CAPABILITIES_REQ
#define SWOPTION_80_MHZ_CAN_CLK     0x2000L // CAN controller run at 80 MHz
#define SWOPTION_24_MHZ_CAN_CLK     0x4000L // CAN controller run tat 24 MHz
#define SWOPTION_CAN_CLK_MASK       0x6000L

// CMD_SET_AUTO_TX_REQ and _RESP enum values
#define AUTOTXBUFFER_CMD_GET_INFO     1     // Get implementation information
#define AUTOTXBUFFER_CMD_CLEAR_ALL    2     // Clear all buffers on a channel
#define AUTOTXBUFFER_CMD_ACTIVATE     3     // Activate a specific buffer
#define AUTOTXBUFFER_CMD_DEACTIVATE   4     // Dectivate a specific buffer
#define AUTOTXBUFFER_CMD_SET_INTERVAL 5     // Set tx buffer transmission interval
#define AUTOTXBUFFER_CMD_GENERATE_BURST 6   // Generate a burst of messages
#define AUTOTXBUFFER_CMD_SET_MSG_COUNT 7    // Set tx buffer message count
#define AUTOTXBUFFER_CMD_SET_BUFFER    8    // Set tx buffer message

// CMD_SET_AUTO_TX_RESP bit values for automatic tx buffer capabilities
#define AUTOTXBUFFER_CAP_TIMED_TX         0x01    // Periodic transmission
#define AUTOTXBUFFER_CAP_AUTO_RESP_DATA   0x02    // Auto response to data frames
#define AUTOTXBUFFER_CAP_AUTO_RESP_RTR    0x04    // Auto response to RTR
#define AUTOTXBUFFER_CAP_CANFD            0x08    // FD capable

// Use these message flags with cmdSetAutoTxBuffer.flags
#define AUTOTXBUFFER_MSG_REMOTE_FRAME     0x10    // Msg is a remote frame
#define AUTOTXBUFFER_MSG_SINGLE_SHOT      0x40    // single shot capable
#define AUTOTXBUFFER_MSG_EXT              0x80    // Extended identifier
#define AUTOTXBUFFER_MSG_FDF              0x100   // msg is canfd
#define AUTOTXBUFFER_MSG_BRS              0x200   // msg is canfd and brs


// For CMD_SOFTSYNC_ONOFF
#define SOFTSYNC_OFF          0
#define SOFTSYNC_ON           1
#define SOFTSYNC_NOT_STARTED  2

// For canTimeStampRef in cmdGetCardInfoResp
#define CAN_TIME_STAMP_REF_ACK        0
#define CAN_TIME_STAMP_REF_SOF        1
#define CAN_TIME_STAMP_REF_INTERRUPT  2
#define CAN_TIME_STAMP_REF_CANTIMER   3


// for script control
#define SCRIPT_CMD_SCRIPT_START                 0x1
#define SCRIPT_CMD_SCRIPT_STOP                  0x2
#define SCRIPT_CMD_SCRIPT_EVENT                 0x4
#define SCRIPT_CMD_SCRIPT_LOAD                  0x5
#define SCRIPT_CMD_SCRIPT_QUERY_STATUS          0x6
#define SCRIPT_CMD_SCRIPT_LOAD_REMOTE_START     0x7
#define SCRIPT_CMD_SCRIPT_LOAD_REMOTE_DATA      0x8
#define SCRIPT_CMD_SCRIPT_LOAD_REMOTE_FINISH    0x9
#define SCRIPT_CMD_SCRIPT_UNLOAD                0xA

// script status info
// SCRIPT_CMD_SCRIPT_QUERY_STATUS
#define SCRIPT_STATUS_LOADED    0x1
#define SCRIPT_STATUS_RUNNING   0x2

// For CMD_SET_DEVICE_MODE et al. Bitmask values.
#define DEVICEMODE_INTERFACE    0x00
#define DEVICEMODE_LOGGER       0x01


/*
** Command Structs
*/

typedef struct {
  uint8_t     cmdLen;  // The length of the whole packet (i.e. including this byte)
  uint8_t     cmdNo;
} hcmdLogHead;



typedef struct {
  uint32_t    timeL;
  uint32_t    timeH;
  uint8_t     key;
  uint8_t     padding[19];
} hcmdImpKey;

typedef struct {
  uint32_t    timeL;
  uint32_t    timeH;
  uint16_t    len;        //  2 byte
  uint8_t     flags;      //  1 byte, 0: tpFollows, 1: Res first, 2: Res last
  uint8_t     reserved;   //  1 byte unused
  char        string[16]; // 16 byte
} hcmdPrintf;

typedef struct {
  uint32_t    addr;     //  4 byte
  uint32_t    addr2;    //  4 byte
  uint16_t    len;      //  2 byte
  uint8_t     flags;    //  1 byte, 0: tpFollows, 1: Res first, 2: Res last
  uint8_t     reserved; //  1 byte unused
  uint8_t     data[16]; // 16 byte
} hcmdMemory;

typedef struct {
  char           text[28];
} hcmdFatalError;

typedef struct {
  uint32_t    data[7];  // 28 bytes
} hcmdMeasure;

#define CMDPRINTF_TPFOLLOW_FLAG  0x01
#define CMDPRINTF_RESFIRST_FLAG  0x02
#define CMDPRINTF_RESLAST_FLAG   0x04

typedef struct {
  uint32_t    timeL;
  uint32_t    timeH;
  uint8_t     status; //  1 byte
  uint8_t     padding[19];
} hresPrintf;

#define HM_STATUS_OK 1
#define HM_STATUS_FAILED -1

#define TRPDATA_EOF_FLAG          0x01
#define TRPDATA_ASCII_FLAG        0x02
#define TRPDATA_DIT_FLAG          0x04
#define TRPDATA_RESWANTED_FLAG    0x08
#define TRPDATA_TRUNCATED_FLAG    0x10
#define TRPDATA_RSP_FLAG          0x20  // This TRP is a Response

#define TRPDATA_STATUS_OK          0
#define TRPDATA_STATUS_BUSY        1  // Busy, ask me later with an empty TRPDATA
#define TRPDATA_STATUS_RESEND      2  // Resend all data from last respons
#define TRPDATA_STATUS_ABORT       3  // Error, abort the complete transfer

typedef struct {
  uint8_t     len;
  uint8_t     flags;   /* 1: EOF, 2: ASCII, 4: DIT, 8: RESW */
  uint8_t     status;
  uint8_t     padding[1];
  uint8_t     data[24];
} htrpData;



typedef union {
  struct {
    uint32_t    options;  // any TRP_OPT_XXX
    uint32_t    transaction_len;
    uint8_t     padding[16];
  } start;
  struct {
    uint32_t    mtu;
    uint32_t    status;
    uint8_t     padding[16];
  } start_resp;
  struct {
    uint8_t     payload[24];
  } data;
  struct {
    uint32_t    checksum;
    uint8_t     padding[20];
  } end_req;
  struct {
    uint32_t    status;
    uint8_t     padding[20];
  } end_resp;
  uint8_t       raw[24];
} htrpDataEx;


#define TRP_START_TRANSACTION_REQ          1
#define TRP_START_TRANSACTION_RESP         2
#define TRP_END_TRANSACTION_REQ            3
#define TRP_END_TRANSACTION_RESP           4
#define TRP_DATA_TRANSACTION               5
#define TRP_DATA_TRANSACTION_RESP          6
#define TRP_DATA_TRANSACTION_MTU_DONE      7
#define TRP_DATA_TRANSACTION_MTU_DONE_RESP 8


#define HYDRA_TRP_PIPE_DIAG_TRIGGER    1
#define HYDRA_TRP_PIPE_DIAG_PG         2
#define HYDRA_TRP_PIPE_LAST_ENTRY      HYDRA_TRP_PIPE_DIAG_PG // update this if you add more pipes!!


#define TRP_OK 0
#define TRP_PIPE_NOT_IMPLEMENTED 1
#define TRP_BUFFER_OVERFLOW      2  // trying to transfer a larger buffer in one chunk than fw can handle.




typedef struct {
  uint8_t     cmd;
  uint8_t     len;
  uint16_t    seqNo;
  htrpDataEx  trp;
} htrpDataExtended;



typedef struct {
  uint8_t     he;        // Set to zero if not known
  uint8_t     channels;
  uint16_t    reserved;
  char        name[16];
} hcmdRegisterHeReq;

typedef struct {
  uint8_t     he;
} hcmdRegisterHeResp;

struct hydraHostCmd;

typedef struct {
  uint8_t     enable;
  uint8_t     he;
  uint8_t     channel;
} hcmdListenToHeReq;

typedef struct {
  uint8_t     status;
} hcmdListenToHeResp;

typedef struct {
  uint8_t     he;
} hcmdQueryNextHeReq;

typedef struct {
  uint8_t     he;
  uint8_t     channels;
  uint8_t     reserved[2];
  char        name[16];
} hcmdQueryNextHeResp;


#define MAX_LOGGED_CAN_DATABYTES 8

typedef struct {
  uint8_t     cmdLen;
  uint8_t     cmdNo;
  uint8_t     channel;
  uint8_t     flags;
  uint16_t    time[3];
  uint8_t     dlc;
  uint8_t     padding;
  uint32_t    id;        // incl. CAN_IDENT_IS_EXTENDED
  uint8_t     data[MAX_LOGGED_CAN_DATABYTES];
} hcmdLogMessage;

typedef struct {
  uint8_t     cmdLen;
  uint8_t     cmdNo;
  uint8_t     channel;

  uint8_t     action_type;
  uint32_t    action_param;

  uint16_t    statement_idx;
  uint16_t    action_idx;
} hcmdLogAction;

typedef struct
{
  uint8_t     cmdLen;
  uint8_t     cmdNo;
  uint8_t     type;
  uint8_t     padding;
  uint16_t    trigNo;
  uint16_t    time[3];
  uint32_t    preTrigger;
  uint32_t    postTrigger;
} hcmdLogTrig;

typedef struct
{
  uint8_t     cmdLen;
  uint8_t     cmdNo;
  uint8_t     verMajor;            // File version number
  uint8_t     verMinor;            // File version number
  unsigned int   unixTime;            // Seconds since 1970
  uint8_t     hiresTimerFqMHz;     // High-resolution timer frequency, in MHz
#ifdef __GNUC__
  // Remember to change the file version number when adding stuff
  uint8_t     padding[15];
#else
  uint8_t     padding[11];
#endif
} hcmdLogRtcTime;

typedef struct {
  uint32_t    id;
  uint8_t     data[8];
  uint8_t     dlc;
  uint8_t     flags;
  uint16_t    transId;
  uint8_t     channel;
  uint8_t     padding[11];
} hcmdTxCanMessage;

typedef struct {
  uint32_t    id;
  uint8_t     data[8];
  uint8_t     dlc;
  uint8_t     flags;
  uint16_t    time[3];
  uint8_t     padding[8];
} hcmdTxAck;

typedef struct {
  uint8_t     reserved1;
  uint8_t     reserved2;
  uint16_t    time[3];
  uint16_t    padding;
} hcmdTxRequest;

typedef struct {
  uint8_t     cmdLen;
  uint8_t     cmdNo;
  uint8_t     channel;
  uint8_t     status;
  uint32_t    interval;
} hcmdTxInterval;

typedef struct {
  uint32_t    bitRate;
  uint8_t     tseg1;
  uint8_t     tseg2;
  uint8_t     sjw;
  uint8_t     noSamp;
  uint8_t     reserved;
  uint8_t     padding[3];
  uint32_t    bitRateFd;
  uint8_t     tseg1Fd;
  uint8_t     tseg2Fd;
  uint8_t     sjwFd;
  uint8_t     noSampFd;
  uint8_t     open_as_canfd;
  uint8_t     padding2[7];
} hcmdSetBusparamsReq;

typedef struct {
  uint16_t  prop;
  uint16_t  phase1;
  uint16_t  phase2;
  uint16_t  sjw;
  uint16_t  brp;
  uint16_t  propFd;
  uint16_t  phase1Fd;
  uint16_t  phase2Fd;
  uint16_t  sjwFd;
  uint16_t  brpFd;
  uint8_t   open_as_canfd;
  uint8_t   res1;
  uint8_t   padding[6];
} hcmdSetBusparamsTqReq;

typedef struct {
  uint8_t  status;
  uint8_t  reserved[27];
} hcmdSetBusparamsTqResp;

#define BUSPARAM_FLAG_CANFD  0x01

typedef struct {
  uint8_t     param_type;
  uint8_t     reserved[27];
} hcmdGetBusparamsReq;

typedef struct {
  uint32_t    bitRate;
  uint8_t     tseg1;
  uint8_t     tseg2;
  uint8_t     sjw;
  uint8_t     noSamp;
  uint8_t     reserved[20];
} hcmdGetBusparamsResp;

typedef struct {
  uint8_t     param_type;
  uint8_t     reserved[27];
} hcmdGetBusparamsTqReq;

typedef struct {
  uint16_t  prop;
  uint16_t  phase1;
  uint16_t  phase2;
  uint16_t  sjw;
  uint16_t  brp;
  uint16_t  propFd;
  uint16_t  phase1Fd;
  uint16_t  phase2Fd;
  uint16_t  sjwFd;
  uint16_t  brpFd;
  uint8_t   open_as_canfd;
  uint8_t   status;
  uint8_t   padding[6];
} hcmdGetBusparamsTqResp;

typedef struct {
  uint8_t     reserved;
} hcmdGetChipStateReq;

typedef struct {
  uint16_t     time[3];
  uint8_t     txErrorCounter;
  uint8_t     rxErrorCounter;
  uint8_t     busStatus;
  uint8_t     reserved[19];
} hcmdChipStateEvent;

typedef struct {
  uint8_t     driverMode;
  uint8_t     reserved;
} hcmdSetDrivermodeReq;

typedef struct {
  uint8_t     reserved;
} hcmdGetDrivermodeReq;

typedef struct {
  uint8_t     driverMode;
  uint8_t     reserved;
} hcmdGetDrivermodeResp;

typedef struct {
  uint8_t     reserved;
} hcmdResetChipReq;

typedef struct {
  uint8_t     reserved;
} hcmdResetCardReq;

typedef struct {
  uint8_t     reserved;
} hcmdStartChipReq;

typedef struct {
  uint8_t     reserved;
} hcmdStartChipResp;

typedef struct {
  uint8_t     reserved;
} hcmdStopChipReq;

typedef struct {
  uint8_t     reserved;
} hcmdStopChipResp;

typedef struct {
  uint8_t     flags;
} hcmdReadClockReq;

typedef struct {
  uint16_t    time[3];
  uint16_t    padding2;
} hcmdReadClockResp;

typedef struct {
  uint8_t     reserved;
} hcmdSelfTestReq;

typedef struct {
  uint32_t    results;
} hcmdSelfTestResp;

typedef struct {
  int8_t      dataLevel;
} hcmdGetCardInfo2Req;

typedef struct {
  uint8_t     pcb_id[24];
  uint32_t    oem_unlock_code;
} hcmdGetCardInfo2Resp;

typedef struct {
  int8_t      dataLevel;
} hcmdGetCardInfoReq;

typedef struct {
  uint32_t    serialNumber;
  uint32_t    clockResolution;
  uint32_t    mfgDate;
  uint8_t     EAN[8];
  uint8_t     hwRevision;
  uint8_t     usbHsMode;
  uint8_t     hwType;
  uint8_t     canTimeStampRef;
  uint8_t     channelCount;
} hcmdGetCardInfoResp;

typedef struct {
  uint8_t     reserved;
} hcmdGetInterfaceInfoReq;

typedef struct {
  uint32_t    channelCapabilities;
  uint8_t     canChipType;
  uint8_t     canChipSubType;
  uint16_t     padding;
} hcmdGetInterfaceInfoResp;

typedef struct {
  uint8_t     reserved[28];
} hcmdGetSoftwareInfoReq;

typedef struct {
  uint32_t    reserved1;
  uint32_t    reserved2;
  uint16_t    maxOutstandingTx;
  uint16_t    padding1;
  uint32_t    padding[4];
} hcmdGetSoftwareInfoResp;


typedef struct {
  uint8_t     useHydraExt;
  uint8_t     reserved[27];
} hcmdGetSoftwareDetailsReq;

typedef struct {
  uint32_t    swOptions;
  uint32_t    swVersion;
  uint32_t    swName;
  uint32_t    EAN[2];
  uint32_t    maxBitrate;
  uint32_t    padding[1];
} hcmdGetSoftwareDetailsResp;


typedef struct {
    uint8_t     reserved;
} hcmdGetBusLoadReq;

typedef struct {
  uint16_t    time[3];         // "Absolute" timestamp
  uint16_t    sample_interval; // Sampling interval in microseconds
  uint16_t    active_samples;  // Number of samples where tx or rx was active
  uint16_t    delta_t;         // Milliseconds since last response
  uint8_t     reserved;
} hcmdGetBusLoadResp;

typedef struct {
  uint8_t     reserved;
} hcmdResetStatisticsReq;

typedef struct {
  uint8_t     reserved;
} hcmdCheckLicenseReq;

typedef struct {
  uint32_t    licenseMask;
  uint32_t    kvaserLicenseMask;
} hcmdCheckLicenseResp;

typedef struct {
  uint16_t     time[3];
  uint8_t     reserved;
  uint8_t     errorCode;
  uint16_t     addInfo1;
  uint16_t     addInfo2;
} hcmdErrorEvent;


typedef struct {
  uint8_t     flags;
  uint8_t     reserved;
} hcmdFlushQueue;


typedef struct {
  uint8_t     reserved;
} hcmdNoCommand;


typedef struct {
  uint8_t     reserved;
} hcmdResetErrorCounter;

typedef struct {
  uint16_t    time[3];
  uint8_t     flags;
  uint8_t     reserved;
  uint8_t     txErrorCounter;
  uint8_t     rxErrorCounter;
  uint8_t     busStatus;
  uint8_t     errorFactor;
} hcmdCanErrorEvent;

typedef struct {
  uint8_t     lineMode;
  uint8_t     resistorNet;
  uint8_t     reserved;
} hcmdSetTransceiverModeReq;

typedef struct {
  uint16_t    time;
} hcmdInternalDummy;

typedef struct {
  uint8_t     status;
} hcmdMemoCpldPrgResp;

typedef struct {
  uint8_t     subCmd;
  uint8_t     padding;
  uint8_t     data[CONFIG_DATA_CHUNK];
} hcmdMemoCpldPrgReq;

typedef struct {
  uint8_t     subCmd;
  uint8_t     padding;
  int16_t       timeout;
} hcmdLedActionReq;

typedef struct {
    uint8_t     subCmd;
} hcmdLedActionResp;

typedef struct {
    uint8_t     info;
} hcmdDiskFullInfo;

typedef struct {
  uint8_t     userNo;
  uint8_t     paramNo;
  uint8_t     status;
  uint8_t     padding;
  uint8_t     data[8];
} hcmdReadUserParameter;


typedef struct {
  int32_t     interval;
  uint8_t     padding[4];
} hcmdMemoConfigModeReq;

typedef struct {
  uint8_t     diskStat;
  uint8_t     configMode;
  uint8_t     reserved[10];
} hcmdMemoConfigModeResp;

typedef struct {
  uint8_t     subCmd;
  uint8_t     padding;
  uint16_t    freq;
  uint16_t    duration;
} hcmdSound;

// port status
#define IO_PORT_STATUS_MISSING  0
#define IO_PORT_STATUS_ENABLED  1
#define IO_PORT_STATUS_DISABLED 2

// port type
#define IO_PORT_TYPE_UNKNOWN      0
#define IO_PORT_TYPE_TRIGGER_IN   1
#define IO_PORT_TYPE_DIGITAL_OUT  2


// Subcommands
#define PORT_DISABLE     0
#define PORT_ENABLE      1
#define PORT_CONFIG_GET  4
#define PORT_CONFIG_SET  5

typedef struct {
  uint32_t    pulseDur;
  uint32_t    activeVal;
  uint32_t    dormantVal;
  uint32_t    portVal;
  uint16_t    type;                // IO_PORT_TYPE_xxx
  uint8_t     status;              // IO_PORT_STATUS_xxx
  uint8_t     subCmd;              // PORT_CONFIG_SET, PORT_xxx
  uint8_t     portNo;
  uint8_t     padding[7];
} hcmdIoPortCtrl;

typedef struct {
  uint8_t     portNo;             // Hardware-specific port #
  uint8_t     padding[3];
  uint32_t    portVal;            // Hardware-specific port value
} hcmdSetIoPortsReq;

typedef struct {
  uint8_t     origReq[12];
  uint8_t     portNo;             // Hardware-specific port #
  uint8_t     padding[15];
} hcmdGetIoPortsReq;

typedef struct {
  uint32_t    portVal;            // Hardware-specific port value
  uint8_t     origReq[12];
  uint8_t     portNo;             // Hardware-specific port #
  uint8_t     status;
  uint8_t     padding[10];
} hcmdGetIoPortsResp;

typedef struct {
    uint8_t     mode;                 // Hardware-specific mode value
    uint8_t     padding[3];
} hcmdSetDeviceModeReq;

typedef struct {
    uint8_t     mode;                 // Hardware-specific mode value
    uint8_t     status;
    uint8_t     padding[2];
} hcmdGetDeviceModeResp;

typedef struct {
    uint8_t     reserved;
} hcmdGetTransceiverInfoReq;

typedef struct {
  uint32_t    transceiverCapabilities;
  uint8_t     transceiverStatus;
  uint8_t     transceiverType;
  uint8_t     reserved;
} hcmdGetTransceiverInfoResp;

typedef struct {
  uint16_t    rate;
  uint16_t    pad2;
} hcmdSetHeartbeatRateReq;

typedef struct {
  uint16_t    time[3];
} hcmdHeartbeatResp;

typedef struct {
  uint32_t    id;
  uint8_t     data[8];
  uint8_t     dlc;
  uint8_t     flags;
  uint8_t     bufNo;
  uint8_t     padding[13];
} hcmdSetAutoTxBuffer;

typedef struct {
  uint32_t    interval;
  uint8_t     requestType;
  uint8_t     bufNo;
  uint8_t     reserved;
  uint8_t     padding;
  uint32_t    id;
  uint8_t     data[8];
  uint8_t     dlc;
  uint8_t     flags;
  uint8_t     padding2[6];
} hcmdAutoTxBufferReq;

typedef struct {
  uint8_t     responseType;
  uint8_t     bufferCount;
  uint16_t    capabilities;
  uint32_t    timerResolution;
  uint32_t    status;
} hcmdAutoTxBufferResp;

typedef struct {
  uint16_t    time[3];
  uint16_t    sofNr;
} hcmdTrefSofSeq;

typedef struct {
  uint16_t    onOff;
} hcmdSoftSyncOnOff;

#define THROTTLE_FLAG_IS_REMOTE  0x01
#define THROTTLE_FLAG_DO_READ    0x02
typedef struct {
  uint16_t    throttle;
  uint8_t     flags;
} hcmdUsbThrottle;

typedef struct {
  uint8_t     subCmd;
  uint8_t     status;
  uint8_t     first;
  uint8_t     data[14];
  uint8_t     padding[11];
} hcmdGetExtendedInfoReq, hcmdGetExtendedInfoResp;

typedef struct {
  uint8_t     seqNo;
  uint8_t     reserved1;
  uint16_t    reserved2;
  unsigned int   time;
  uint8_t     data[16];
  uint8_t     padding[4];
} hcmdTcpKeepalive;


typedef struct {
  uint16_t    envvarLen;
  uint8_t     channel;
  uint8_t     padding2;
  uint32_t    hash;
} hcmdScriptEnvvarNotifyEvent;

typedef struct {
  uint8_t     index;
  uint8_t     data[16];
  uint8_t     padding[11];
} hcmdNetworkDeviceName;

typedef struct {
  uint32_t    time_in_us;
  uint8_t     padding[24];
} hcmdDevicePing;

#define SCRIPT_ENVVAR_SUBCMD_SET_START     1
#define SCRIPT_ENVVAR_SUBCMD_GET_START     2

#define SCRIPT_ENVVAR_RESP_OK                 0
#define SCRIPT_ENVVAR_RESP_UNKNOWN_VAR        1
#define SCRIPT_ENVVAR_RESP_WRONG_VAR_LEN      2
#define SCRIPT_ENVVAR_RESP_OUT_OF_MEMORY      3

typedef union {
  struct {
    uint16_t    bulkLen;
  } cmdStartSet;
  struct {
    uint16_t    startOffset;
    uint16_t    payloadLengthWanted;
  } cmdStartGet;
} hcommandData;

typedef struct {
  uint8_t     subCommand;
  uint8_t     origin;
  uint16_t    reserved;
  uint32_t    hash;
  hcommandData    subCmdData;
} hcmdScriptEnvvarTransferCtrlReq;

typedef struct {
  uint8_t     subCommand;
  uint8_t     resp;
  uint16_t    reserved;
  uint32_t    hash;
} hcmdScriptEnvvarTransferCtrlResp;

#define HYDRA_SCRIPT_ENVVAR_BULKSIZE 18

typedef struct {
  uint16_t    offset;
  uint16_t    length;
  uint32_t    hash;
  uint8_t     origin;
  uint8_t     bulkDone;
  uint8_t     bulkData[HYDRA_SCRIPT_ENVVAR_BULKSIZE];
} hcmdScriptEnvvarTransferBulk;

typedef struct {
  char    name[16];           // CAN, LIN, WHATEVER
  uint8_t channel;            // 0, 1,... (combine with name)
  uint8_t reserved[11];
} hcmdMapChannelReq;


typedef struct {
  uint8_t  heAddress;         // Opaque for the driver.
  uint8_t  position;          // Physical "position" in device
  uint16_t flags;             // Various flags, to be defined
  uint8_t  reserved1[24];
} hcmdMapChannelResp;

typedef struct {
  uint16_t    fileNo;            // File number
  uint16_t    fileCount;         // Number of files on SD card
  uint32_t    fileSize;
  uint32_t    fileTime;
  char        filename[14];        // Only 8.3
  int16_t     status;
} hcmdFileInfo;

//Sub commands in CMD_GET_CAPABILITIES_REQ
#define CAP_SUB_CMD_DUMMY_NOT_IMPLEMENTED    0
#define CAP_SUB_CMD_DUMMY_UNAVAILABLE        1
#define CAP_SUB_CMD_SILENT_MODE              2
#define CAP_SUB_CMD_ERRFRAME                 3
#define CAP_SUB_CMD_BUS_STATS                4
#define CAP_SUB_CMD_ERRCOUNT_READ            5
#define CAP_SUB_CMD_SINGLE_SHOT              6
#define CAP_SUB_CMD_SYNC_TX_FLUSH            7 // Used in filo
#define CAP_SUB_CMD_HAS_LOGGER               8
#define CAP_SUB_CMD_HAS_REMOTE               9
#define CAP_SUB_CMD_HAS_SCRIPT               10
#define CAP_SUB_CMD_LIN_HYBRID               11
#define CAP_SUB_CMD_KDI_INFO                 12
#define CAP_SUB_CMD_HAS_KDI                  13
#define CAP_SUB_CMD_HAS_IO_API               14
#define CAP_SUB_CMD_HAS_BUSPARAMS_TQ         15
#define CAP_SUB_CMD_HAS_ALWAYS_SILENT        16

// the following are not capabilities/bits
#define CAP_SUB_CMD_DATA_START               1024
#define CAP_SUB_CMD_GET_LOGGER_INFO          CAP_SUB_CMD_DATA_START+1
#define CAP_SUB_CMD_REMOTE_INFO              CAP_SUB_CMD_DATA_START+2
#define CAP_SUB_CMD_HW_STATUS                CAP_SUB_CMD_DATA_START+3
#define CAP_SUB_CMD_FEATURE_EAN              CAP_SUB_CMD_DATA_START+4

// CAP_SUB_CMD_GET_LOGGER_TYPE
#define LOGGERTYPE_NOT_A_LOGGER 0
#define LOGGERTYPE_V1 1
#define LOGGERTYPE_V2 2

// CAP_SUB_CMD_REMOTE_TYPE
#define REMOTE_TYPE_NOT_REMOTE  0
#define REMOTE_TYPE_WLAN 1
#define REMOTE_TYPE_LAN  2

typedef union {
  uint32_t padding[6];
  uint32_t channel;
  uint8_t analyzerNo;
} hcapExtraInfo_u;

typedef struct {
  uint16_t  subCmdNo;
  uint16_t  unused;
  hcapExtraInfo_u  subData;
} hcmdCapabilitiesReq;

typedef struct
{
  uint32_t mask;
  uint32_t value;
  uint32_t padding[4];
} hchannelCap32_t;

typedef struct
{
  uint32_t data;
} hInfo_t;

typedef struct
{
  unsigned int     webServer;
  unsigned int     remoteType;
} hRemoteInfo_t;

typedef struct
{
  uint32_t codes[6];
} hhwStatus_t;

typedef struct
{
  uint32_t eanLo;
  uint32_t eanHi;
} hfeatureEan_t;

typedef struct
{
  uint32_t version;
  uint8_t  numAnalyzers;
  uint8_t  analyzerNo;
  uint8_t  analyzerType;
  uint8_t  analyzerChanMask;
  uint32_t reserved2[4];
} hKdiInfo_t;

//Status codes in CMD_GET_CAPABILITIES_RESP
#define CAP_STATUS_OK 0
#define CAP_STATUS_NOT_IMPLEMENTED 1
#define CAP_STATUS_UNAVAILABLE 2

typedef struct {
  uint16_t  subCmdNo;
  uint16_t  status;
  union {
    hchannelCap32_t silentMode;    // CAP_SUB_CMD_SILENT_MODE
    hchannelCap32_t errframeCap;   // CAP_SUB_CMD_ERRFRAME
    hchannelCap32_t busstatCap;    // CAP_SUB_CMD_BUS_STATS
    hchannelCap32_t errcountCap;   // CAP_SUB_CMD_ERRCOUNT_READ
    hchannelCap32_t singleshotCap; // CAP_SUB_CMD_SINGLE_SHOT
    hchannelCap32_t loggerCap;     // CAP_SUB_CMD_HAS_LOGGER
    hchannelCap32_t remoteCap;     // CAP_SUB_CMD_HAS_REMOTE
    hchannelCap32_t scriptCap;     // CAP_SUB_CMD_HAS_SCRIPT
    hchannelCap32_t linHybridCap;  // CAP_SUB_CMD_LIN_HYBRID
    hchannelCap32_t kdiCap;        // CAP_SUB_CMD_HAS_KDI
    hchannelCap32_t ioApiCap;      // CAP_SUB_CMD_HAS_IO_API
    hchannelCap32_t busparamsTqCap;// CAP_SUB_CMD_HAS_BUSPARAMS_TQ
    hchannelCap32_t alwaysSilentMode; // CAP_SUB_CMD_HAS_ALWAYS_SILENT
    hInfo_t loggerType;            // CAP_SUB_CMD_GET_LOGGER_TYPE
    hRemoteInfo_t remoteInfo;      // CAP_SUB_CMD_REMOTE_TYPE
    hhwStatus_t   hwStatus;        // CAP_SUB_CMD_HW_STATUS
    hfeatureEan_t featureEan;      // CAP_SUB_CMD_FEATURE_EAN
    hKdiInfo_t kdiInfo;            // CAP_SUB_CMD_KDI_INFO
  };
} hcmdCapabilitiesResp;

// Union for all logged messages.
typedef union {
  hcmdLogHead                  logHead;
  hcmdLogMessage               logMessage;
  hcmdLogTrig                  logTrig;
  hcmdLogRtcTime               logRtcTime;
  hcmdLogAction                logAction;
} hydraHostCmdLog;

// CAN FD messages
#define MAX_LOGGED_DATABYTES 64

typedef struct {
  uint8_t     len;
  uint8_t     logCmdNo;
  uint8_t     channel;
  uint8_t     dlc;
  uint32_t    flags;
  uint32_t    id;
  uint32_t    time[2];
  uint8_t     data[MAX_LOGGED_DATABYTES];
} hcmdLogMessageFD;

typedef struct
{
  uint8_t     len;
  uint8_t     logCmdNo;
  uint8_t     type;
  uint8_t     channel;
  uint32_t    time[2];
  uint32_t    preTrigger;
  uint32_t    postTrigger;
  uint16_t    trigNo; // Bit mask showing which trigger was activated
  uint8_t     padding2[2];
} hcmdLogTrigFD;


typedef struct
{
  uint8_t     len;
  uint8_t     logCmdNo;
  uint16_t    hiresTimerFqMHz;
  uint32_t    unixTime;            // Seconds since 1970
  uint32_t    unixTimeSubsecond;
  uint32_t    time[2];             // sync with unixTime
  uint32_t    padding2;
} hcmdLogRtcTimeFD;

typedef struct
{
  uint8_t     len;
  uint8_t     logCmdNo;
  uint8_t     lioMajor; // Lio major version
  uint8_t     lioMinor; // Lio minor version
  uint8_t     fwMajor;  // Firmware major version
  uint8_t     fwMinor;  // Firmware major version
  uint16_t    fwBuild;       // Firmware build version
  uint32_t    serialNumber;  // Serial
  uint32_t    eanHi;         // EANHI
  uint32_t    eanLo;         // EANLO
} hcmdLogVerFD;


// Union for all logged CAN FD messages.
typedef union {
  hcmdLogHead                  logHead;
  hcmdLogMessageFD             logMessageFD;
  hcmdLogTrigFD                logTrigFD;
  hcmdLogRtcTimeFD             logRtcTimeFD;
  hcmdLogVerFD                 logVerFD;
} hydraHostCmdLogFD;


typedef struct {
  uint8_t   unknownCmd;
} hcmdUnknownCommandResp;

#define KDI_SUBCMD_TRIGGER_START     1
#define KDI_SUBCMD_TRIGGER_STOP      2
// #define KDI_SUBCMD_TRIGGER_DOWNLOAD  3
#define KDI_SUBCMD_TRIGGER_RUNNING   6

#define KDI_SUBCMD_PG_START         21
#define KDI_SUBCMD_PG_STOP          22
#define KDI_SUBCMD_PG_PLEN          23
#define KDI_SUBCMD_PG_BURST_LENGTH  24
#define KDI_SUBCMD_PG_IDLE_LENGTH   25
#define KDI_SUBCMD_PG_SP_POS        26
#define KDI_SUBCMD_PG_ADJUST_PLEN   27
// #define KDI_SUBCMD_PG_PATTERN       28
#define KDI_SUBCMD_PG_STAT_IDLE     29
#define KDI_SUBCMD_PG_RUNNING       31

#define KDI_SUBCMD_ATTACH   40
#define KDI_SUBCMD_DETACH   41

#define SUBCMD_LOOPBACK_GET    50
#define SUBCMD_LOOPBACK_SET    51
#define SUBCMD_LOOPBACK_RESP   52

#define KDI_SUBCMD_POWER_UP_ANALYZER    53
#define KDI_SUBCMD_POWER_DOWN_ANALYZER  54

#define KDI_STATUS_OK                 1
#define KDI_STATUS_NOT_IMPLEMENTED    2
#define KDI_LOOPBACK_ERROR            3
#define KDI_STATUS_ERROR              4

#define KDI_ANALYZER_TYPE_DEFAULT 0
typedef union {
  uint32_t             pg_ctrl;
  uint32_t             trigger_running;
  uint32_t             pg_running;
  uint8_t              buffer[24];
  struct {
    uint8_t block;
    uint8_t rx_bus;
    uint8_t tx_bus;
  } loopback_set;
  struct {
    uint8_t block;
  } loopback_get;
  struct {
    uint8_t rx_bus;
    uint8_t tx_bus;
  } loopback_resp;
  struct {
    uint32_t type;
  } analyzer;
} hkdiCtrlData;

typedef struct {
  uint8_t subCmdNo; //start, stop
  uint8_t status;
  uint8_t padding[2];
  hkdiCtrlData data;
} hcmdKDICmd;

// Use with CMD_IO_PIN_CMD
#define IO_SUBCMD_GET_COUNT                       1
#define IO_SUBCMD_GET_PIN_INFO                    2
#define IO_SUBCMD_SET_PIN_INFO                    3

#define IO_SUBCMD_GET_DIGITAL                    10
#define IO_SUBCMD_SET_DIGITAL                    11
#define IO_SUBCMD_GET_ANALOG                     12
#define IO_SUBCMD_SET_ANALOG                     13
#define IO_SUBCMD_SET_RELAY                      16

#define IO_SUBCMD_EVENT_SETUP                    20
#define IO_SUBCMD_EVENT_DISABLE                  21
#define IO_SUBCMD_CONFIRM_CONFIG                 22

#define IO_SUBCMD_GET_OUTPUT_RELAY               23
#define IO_SUBCMD_GET_OUTPUT_ANALOG              24
#define IO_SUBCMD_GET_OUTPUT_DIGITAL             25
#define IO_SUBCMD_GET_MODULE_PORTS               26
#define IO_SUBCMD_SET_MODULE_PORTS               27

#define IO_PIN_INFO_UNKNOWN                       0
#define IO_PIN_INFO_MODULE_TYPE                   1
#define IO_PIN_INFO_DIRECTION                     2

#define IO_PIN_INFO_TYPE                          4
#define IO_PIN_INFO_NUMBER_OF_BITS                5
#define IO_PIN_INFO_RANGE_MIN                     6
#define IO_PIN_INFO_RANGE_MAX                     7
#define IO_PIN_INFO_DIGITAL_LOW_HIGH_FILTER       8
#define IO_PIN_INFO_DIGITAL_HIGH_LOW_FILTER       9
#define IO_PIN_INFO_ANALOG_LOWPASS_FILTER_ORDER  10
#define IO_PIN_INFO_ANALOG_HYSTERESIS            11
#define IO_PIN_INFO_ANALOG_VALUE_ABOVE           12
#define IO_PIN_INFO_ANALOG_VALUE_BELOW           13
#define IO_PIN_INFO_MODULE_NUMBER                14
#define IO_PIN_INFO_SERIAL_NUMBER                15
#define IO_PIN_INFO_FW_VERSION                   16

#define IO_PIN_TYPE_UNKNOWN                       0
#define IO_PIN_TYPE_DIGITAL                       1
#define IO_PIN_TYPE_ANALOG                        2
#define IO_PIN_TYPE_RELAY                         3

#define IO_PIN_DIRECTION_UNKNOWN                  0
#define IO_PIN_DIRECTION_IN                       4
#define IO_PIN_DIRECTION_OUT                      8

#define IO_MODULE_TYPE_UNKNOWN                    0
#define IO_MODULE_TYPE_DIGITAL                    1
#define IO_MODULE_TYPE_ANALOG                     2
#define IO_MODULE_TYPE_RELAY                      3
#define IO_MODULE_TYPE_INTERNAL                   4

#define IO_EVENT_UNKNOWN                          0
#define IO_EVENT_CONFIG_CHANGED                   2
#define IO_EVENT_VALUE_CHANGED                    3
#define IO_EVENT_VALUE_ABOVE                      4
#define IO_EVENT_VALUE_BELOW                      5
#define IO_EVENT_POSITIVE_FLANK                   6
#define IO_EVENT_NEGATIVE_FLANK                   7

#define IO_API_OK                                 0
#define IO_API_ERROR_CONFIG_CHANGED              -1
#define IO_API_ERROR_NO_CONFIG                   -2
#define IO_API_ERROR_PIN_NOT_FOUND_IN_CONFIG     -3
#define IO_API_ERROR_WRONG_PIN_MODULE_TYPE       -4
#define IO_API_ERROR_PIN_INFO_ITEM_UNKNOWN       -5
#define IO_API_ERROR_PIN_INFO_ITEM_READ_ONLY     -6
#define IO_API_ERROR_PIN_PENDING                 -7
#define IO_API_ERROR_UNKNOWN_EVENT               -8
#define IO_API_ERROR_TOO_MANY_EVENTS             -9
#define IO_API_ERROR_COMMUNICATION              -10
#define IO_API_ERROR_CONFIG_NOT_CONFIRMED       -11
#define IO_API_ERROR_RX_CRC                     -12
#define IO_API_ERROR_HW_NOT_READY               -13
#define IO_API_ERROR_RX_TIMEOUT                 -14
#define IO_API_ERROR_INDEX_TOO_HIGH             -15
#define IO_API_ERROR_VALUE_OUT_OF_RANGE         -16

typedef struct {
  uint8_t subCmdNo;
  int8_t status;
  uint16_t dummy; // Needed for alignment
  uint16_t pinNo;
  uint16_t item;

  union {
    uint32_t int_value;
    float float_value;
    uint8_t buffer[20];
  };
} hcmdIOCmd;


// Use with CMD_STORAGE
#define STORAGE_SUBCMD_FORMAT_FAT32 1

#define STORAGE_STATUS_OK                 1
#define STORAGE_STATUS_NOT_IMPLEMENTED    2
#define STORAGE_STATUS_ERROR              3

typedef union {
  uint8_t              buffer[24];
} hstorageData;

typedef struct {
  uint8_t subCmdNo;       // STORAGE_SUBCMD_FORMAT etc
  uint8_t status;         // STORAGE_STATUS...
  uint8_t dioStatus;      // When applicable
  uint8_t lioStatus;      // When applicable
  hstorageData data;       // When applicable
} hcmdStorageCmd;

// Well-known HEs
#define BROADCAST         0x0f
#define BROADCAST_DEBUG   0x1f
#define BROADCAST_ERROR   0x2f
#define ROUTER_HE         0x00
#define DYNAMIC_HE        ROUTER_HE
#define ILLEGAL_HE        0x3e

#define HYDRA_CMD_SIZE          32

typedef struct hydraHostCmd {
  uint8_t     cmdNo;
  union {
    struct {
      uint8_t     dstAddr    : 6;
      uint8_t     srcChannel : 2;
    } cmdIOP;
    uint8_t     heAddress;    // 0..5: dstHE, 6..7: srcHE-msb-part
  };
  union {
    struct {
      uint16_t    transId : 12; //  0..11: Seq
      uint16_t    srcHE   : 4;  // 12..15: srcHe-lsb-part
    } cmdIOPSeq;
    uint16_t    transId;     // 0..11 transId; 12..15 don't care
  };

  union {
    hcmdFatalError            fatalError;
    hcmdImpKey                keyMsg;
    hcmdPrintf                printfMsg;
    hresPrintf                printfRes;
    htrpData                  trpDataMsg;
    htrpDataExtended          trpDataMsgExt;

    hcmdMemory                memory;
    hcmdMeasure               measure;

    hcmdLogMessage            logMessage;
    hcmdLogTrig               logTrig;
    hcmdLogRtcTime            logRtcTime;
    hcmdLogAction             logAction;
    hcmdTxCanMessage          txCanMessage;
    hcmdTxAck                 txAck;
    hcmdTxRequest             txRequest;
    hcmdTxInterval            txInterval;
    hcmdSetBusparamsReq       setBusparamsReq;

    hcmdGetBusparamsReq       getBusparamsReq;
    hcmdGetBusparamsResp      getBusparamsResp;

    hcmdSetBusparamsTqReq     setBusparamsTqReq;
    hcmdSetBusparamsTqResp    setBusparamsTqResp;

    hcmdGetBusparamsTqReq     getBusparamsTqReq;
    hcmdGetBusparamsTqResp    getBusparamsTqResp;

    hcmdGetChipStateReq       getChipStateReq;
    hcmdChipStateEvent        chipStateEvent;
    hcmdSetDrivermodeReq      setDrivermodeReq;
    hcmdGetDrivermodeReq      getDrivermodeReq;
    hcmdGetDrivermodeResp     getDrivermodeResp;

    hcmdResetChipReq          resetChipReq;
    hcmdResetCardReq          resetCardReq;
    hcmdStartChipReq          startChipReq;
    hcmdStartChipResp         startChipResp;
    hcmdStopChipReq           stopChipReq;
    hcmdStopChipResp          stopChipResp;

    hcmdGetBusLoadReq         getBusLoadReq;
    hcmdGetBusLoadResp        getBusLoadResp;
    hcmdCanErrorEvent         canErrorEvent;
    hcmdFlushQueue            flushQueue;
    hcmdResetErrorCounter     resetErrorCounter;

    hcmdGetCardInfo2Req          getCardInfo2Req;
    hcmdGetCardInfo2Resp         getCardInfo2Resp;
    hcmdGetCardInfoReq           getCardInfoReq;
    hcmdGetCardInfoResp          getCardInfoResp;
    hcmdGetInterfaceInfoReq      getInterfaceInfoReq;
    hcmdGetInterfaceInfoResp     getInterfaceInfoResp;
    hcmdGetSoftwareInfoReq       getSoftwareInfoReq;
    hcmdGetSoftwareInfoResp      getSoftwareInfoResp;

    hcmdResetStatisticsReq       resetStatisticsReq;
    hcmdErrorEvent               errorEvent;
    hcmdNoCommand                noCommand;
    hcmdCheckLicenseReq          checkLicenseReq;
    hcmdCheckLicenseResp         checkLicenseResp;
    hcmdReadClockReq             readClockReq;
    hcmdReadClockResp            readClockResp;
    hcmdSelfTestReq              selfTestReq;
    hcmdSelfTestResp             selfTestResp;
    hcmdSetTransceiverModeReq    setTransceiverModeReq;
    hcmdInternalDummy            internalDummy;

    hcmdRegisterHeReq            registerHeReq;
    hcmdRegisterHeResp           registerHeResp;
    hcmdListenToHeReq            listenToHeReq;
    hcmdListenToHeResp           listenToHeResp;
    hcmdQueryNextHeReq           queryNextHeReq;
    hcmdQueryNextHeResp          queryNextHeResp;

    hcmdSetDeviceModeReq         setDeviceModeReq;
    hcmdGetDeviceModeResp        getDeviceModeResp;

    hcmdLedActionReq             ledActionReq;
    hcmdLedActionResp            ledActionResp;

    hcmdIoPortCtrl               ioPortCtrl;

    hcmdSetIoPortsReq            setIoPortsReq;
    hcmdGetIoPortsReq            getIoPortsReq;
    hcmdGetIoPortsResp           getIoPortsResp;

    hcmdGetTransceiverInfoReq    getTransceiverInfoReq;
    hcmdGetTransceiverInfoResp   getTransceiverInfoResp;

    hcmdSetHeartbeatRateReq      setHeartbeatRateReq;
    hcmdHeartbeatResp            heartbeatResp;

    hcmdSetAutoTxBuffer          setAutoTxBuffer;
    hcmdAutoTxBufferReq          autoTxBufferReq;
    hcmdAutoTxBufferResp         autoTxBufferResp;

    hcmdTrefSofSeq               trefSofSeq;
    hcmdSoftSyncOnOff            softSyncOnOff;
    hcmdUsbThrottle              usbThrottle;
    hcmdSound                    sound;

    hcmdMemoCpldPrgReq           memoCpldPrgReq;
    hcmdMemoCpldPrgResp          memoCpldPrgResp;
    hcmdMemoConfigModeReq        memoConfigModeReq;
    hcmdMemoConfigModeResp       memoConfigModeResp;

    hcmdMemoGetDataReq           memoGetDataReq;
    hcmdMemoGetDataResp          memoGetDataResp;
    hcmdMemoPutDataStartReq      memoPutDataStartReq;
    hcmdMemoPutDataReq           memoPutDataReq;
    hcmdMemoPutDataResp          memoPutDataResp;
    hcmdMemoAsyncopStartReq      memoAsyncopStartReq;
    hcmdMemoAsyncopStartResp     memoAsyncopStartResp;
    hcmdMemoAsyncopGetDataReq    memoAsyncopGetDataReq;
    hcmdMemoAsyncopGetDataResp   memoAsyncopGetDataResp;
    hcmdMemoAsyncopCancelReq     memoAsyncopCancelReq;
    hcmdMemoAsyncopFinishedResp  memoAsyncopFinishedResp;
    hcmdDiskFullInfo             diskFullInfo;

    hcmdReadUserParameter        readUserParameter;

    hcmdGetExtendedInfoReq       getExtendedInfoReq;
    hcmdGetExtendedInfoResp      getExtendedInfoResp;

    hcmdTcpKeepalive             tcpKeepalive;

    hcmdScriptCtrlReq            scriptCtrlReq;
    hcmdScriptCtrlResp           scriptCtrlResp;

    hcmdEnvvarCtrlReq            envvarCtrlReq;
    hcmdEnvvarCtrlResp           envvarCtrlResp;

    hcmdNetworkDeviceName        networkDeviceNameReq;
    hcmdNetworkDeviceName        networkDeviceNameResp;
    hcmdDevicePing               devicePingReq;
    hcmdDevicePing               devicePingResp;

    hcmdScriptEnvvarTransferCtrlReq  scriptEnvvarTransferCtrlReq;
    hcmdScriptEnvvarTransferCtrlResp scriptEnvvarTransferCtrlResp;
    hcmdScriptEnvvarTransferBulk     scriptEnvvarTransferBulk;
    hcmdScriptEnvvarNotifyEvent      scriptEnvvarNotifyEvent;

    hcmdMapChannelReq             mapChannelReq;
    hcmdMapChannelResp            mapChannelResp;
    hcmdGetSoftwareDetailsReq     getSoftwareDetailsReq;
    hcmdGetSoftwareDetailsResp    getSoftwareDetailsResp;
    hcmdFileInfo                  fileInfo;

    hcmdUnknownCommandResp        unknownCommandResp;
    hcmdCapabilitiesReq           capabilitiesReq;
    hcmdCapabilitiesResp          capabilitiesResp;

    hcmdKDICmd                    kdiCmd;
    hcmdIOCmd                     ioCmd;
    
    hcmdStorageCmd               storageCmd;

#ifdef HYDRA_PRIVATE
    hcmdHydraOtherCommand        o;
#endif

  } ;
} hydraHostCmd;

#ifndef getSRC
#define HE_BITS    4
#define CH_BITS    2
#define SEQ_BITS   12
#if (2 * CH_BITS + HE_BITS) > 8
# error "Too many HE or CH bits!"
#endif
#if (HE_BITS + SEQ_BITS) > 16
# error "Too many HE or SEQ bits!"
#endif
#define ADDR_BITS  (HE_BITS + CH_BITS)
#define CH_COUNT   (1 << CH_BITS)
#define HE_COUNT   (1 << HE_BITS)
#define ADDR_COUNT (1 << (HE_BITS + CH_BITS))
#define SEQ_COUNT  (1 << SEQ_BITS)
#define KLI_COUNT  8
#define CH_MASK    ((CH_COUNT - 1) << HE_BITS)
#define CH_HI_MASK ((CH_COUNT - 1) << (HE_BITS + CH_BITS))
#define HE_MASK    (HE_COUNT - 1)
#define ADDR_MASK  (ADDR_COUNT - 1)
#define SEQ_MASK   (SEQ_COUNT - 1)
#define KLI_MASK   (KLI_COUNT - 1)

#define getSRC(cmd)     (((((cmd)->heAddress) & CH_HI_MASK) >> CH_BITS) |  \
                         (((cmd)->transId) >> SEQ_BITS))
#define getSEQ(cmd)     ((cmd)->transId & SEQ_MASK)
#define setDST(cmd,dst)                                          \
  do { (cmd)->heAddress = ((cmd)->heAddress    & CH_HI_MASK) |   \
                          ((dst)               & ADDR_MASK);     \
  } while (0)
#define setSRC(cmd,src)                                          \
  do { (cmd)->heAddress = ((cmd)->heAddress    & ADDR_MASK)  |   \
                          (((src) << CH_BITS)  & CH_HI_MASK);    \
       (cmd)->transId   = ((cmd)->transId      & SEQ_MASK)   |   \
                          ((src) << SEQ_BITS);                   \
                         } while (0)
#define setSEQ(cmd,seq)                                          \
  do { (cmd)->transId = ((cmd)->transId        & ~SEQ_MASK)  |   \
                         ((seq)                & SEQ_MASK);      \
  } while (0)
#endif


// hydraHostCmd must be 32 bytes long!
extern char xx[(sizeof(hydraHostCmd) == 32) ? 1 : -1];
extern char xy[(sizeof(hydraHostCmd) % 4) == 0 ? 1 : -1];


typedef struct {
  uint32_t    flags;
  uint32_t    id;
  uint32_t    fpga_id;
  uint32_t    fpga_control;
  uint64_t    fpga_timestamp;
  uint8_t     fpga_payload[64];
} hcmdRxCanMessageFd;

typedef struct {
  uint32_t    flags;
  uint32_t    id;
  uint32_t    fpga_id;
  uint32_t    fpga_control;
  uint8_t     databytes;
  uint8_t     dlc;
  uint8_t     reserved4[6];
  uint8_t     fpga_payload[64];
} hcmdTxCanMessageFd;

typedef struct {
  uint8_t     requestType;
  uint8_t     bufNo;
  uint8_t     databytes;
  uint8_t     dlc;
  uint32_t    interval;
  uint32_t    flags;
  uint32_t    id;
  uint8_t     data[64];
} hcmdAutoTxMessageFd;

typedef struct {
  uint32_t    flags;
  uint32_t    padding1;
  uint64_t    fpga_timestamp;
  uint32_t    padding2[2];
} hcmdTxAckFd;

//Used when cmdNo is 0xFF (CMD_EXTENDED)
typedef struct {
  uint8_t     cmdNo;
  union {
    struct {
      uint8_t     dstAddr    : 6;
      uint8_t     srcChannel : 2;
    } cmdIOP;
    uint8_t     heAddress;
  };
  union {
    struct {
      uint16_t    transId : 12;
      uint16_t    srcHE   : 4;
    } cmdIOPSeq;
    uint16_t    transId;
  };

  uint16_t    cmdLen;
  uint8_t     cmdNoExt;
  uint8_t     Reserved;

  union {
    hcmdRxCanMessageFd        rxCanMessageFd;
    hcmdTxCanMessageFd        txCanMessageFd;
    hcmdAutoTxMessageFd       autoTxMessageFd;
    hcmdTxAckFd               txAckFd;
  };
} hydraHostCmdExt;




#endif //_HYDRA_HOST_CMDS_H_
