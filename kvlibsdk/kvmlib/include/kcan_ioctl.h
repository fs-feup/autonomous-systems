/*
**             Copyright 2023 by Kvaser AB, Molndal, Sweden
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

/* kcan_ioctl.h: ioctls()'s specific for Kvasers CAN drivers */

#ifndef _KCAN_IOCTL_H
#define _KCAN_IOCTL_H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif /* __KERNEL__ */

//#   include <linux/ioctl.h>
#include <asm/ioctl.h>
#include "compilerassert.h"

#define KCAN_IOC_MAGIC 'k'

// For compatibility with Windows #define:s below.
#define VCAN_DEVICE      0 // dummy
#define KCAN_IOCTL_START 0
#define METHOD_BUFFERED  0 // dummy
#define FILE_ANY_ACCESS  0

#define CANIO_MAX_FILE_NAME (8 + 1 + 3) // Used by MEMO_SUBCMD_OPEN_FILE in FileCopyXxx command
#define CANIO_DFS_READ      0x01
#define CANIO_DFS_WRITE     0x02

// request's
#define KCAN_SCRIPT_REQUEST_TEXT_UNSUBSCRIBE 0x1
#define KCAN_SCRIPT_REQUEST_TEXT_SUBSCRIBE   0x2

// slots: secret for internal use
#define KCAN_SCRIPT_ERROR_SLOT 0x30000000
#define KCAN_SCRIPT_INFO_SLOT  0x20000000


#define CTL_CODE(x, i, y, z) _IO(KCAN_IOC_MAGIC, (i))

#define KCAN_IOCTL_OBJBUF_FREE_ALL \
    CTL_CODE(VCAN_DEVICE, KCAN_IOCTL_START + 6, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCAN_IOCTL_OBJBUF_ALLOCATE \
    CTL_CODE(VCAN_DEVICE, KCAN_IOCTL_START + 7, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCAN_IOCTL_OBJBUF_FREE \
    CTL_CODE(VCAN_DEVICE, KCAN_IOCTL_START + 8, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCAN_IOCTL_OBJBUF_WRITE \
    CTL_CODE(VCAN_DEVICE, KCAN_IOCTL_START + 9, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCAN_IOCTL_OBJBUF_SET_FILTER \
    CTL_CODE(VCAN_DEVICE, KCAN_IOCTL_START + 10, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCAN_IOCTL_OBJBUF_SET_FLAGS \
    CTL_CODE(VCAN_DEVICE, KCAN_IOCTL_START + 11, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCAN_IOCTL_OBJBUF_ENABLE \
    CTL_CODE(VCAN_DEVICE, KCAN_IOCTL_START + 12, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCAN_IOCTL_OBJBUF_DISABLE \
    CTL_CODE(VCAN_DEVICE, KCAN_IOCTL_START + 13, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define KCAN_IOCTL_OBJBUF_SET_PERIOD \
    CTL_CODE(VCAN_DEVICE, KCAN_IOCTL_START + 22, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCAN_IOCTL_OBJBUF_SEND_BURST \
    CTL_CODE(VCAN_DEVICE, KCAN_IOCTL_START + 23, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define KCAN_IOCTL_OBJBUF_SET_MSG_COUNT \
    CTL_CODE(VCAN_DEVICE, KCAN_IOCTL_START + 34, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define KCAN_IOCTL_READ_TREF_LIST \
    CTL_CODE(VCAN_DEVICE, KCAN_IOCTL_START + 67, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define KCAN_IOCTL_TX_INTERVAL \
    CTL_CODE(VCAN_DEVICE, KCAN_IOCTL_START + 68, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define KCAN_IOCTL_OPEN_MODE \
    CTL_CODE(VCAN_DEVICE, KCAN_IOCTL_START + 69, METHOD_BUFFERED, FILE_ANY_ACCESS)
// Windows code has this in vcanio.h
#define VCAN_IOCTL_GET_CARD_INFO \
    CTL_CODE(VCAN_DEVICE, KCAN_IOCTL_START + 70, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCAN_IOCTL_GET_CARD_INFO_2 \
    CTL_CODE(VCAN_DEVICE, KCAN_IOCTL_START + 71, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define KCAN_IOCTL_SET_BRLIMIT \
    CTL_CODE(VCAN_DEVICE, KCAN_IOCTL_START + 72, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCAN_IOCTL_GET_CUST_CHANNEL_NAME \
    CTL_CODE(VCAN_DEVICE, KCAN_IOCTL_START + 73, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCAN_IOCTL_GET_CARD_INFO_MISC \
    CTL_CODE(VCAN_DEVICE, KCAN_IOCTL_START + 74, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define KCANY_IOCTL_MEMO_CONFIG_MODE \
    CTL_CODE(VCAN_DEVICE, KCAN_IOCTL_START + 75, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCANY_IOCTL_MEMO_GET_DATA \
    CTL_CODE(VCAN_DEVICE, KCAN_IOCTL_START + 76, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCANY_IOCTL_MEMO_PUT_DATA \
    CTL_CODE(VCAN_DEVICE, KCAN_IOCTL_START + 77, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCANY_IOCTL_MEMO_DISK_IO \
    CTL_CODE(VCAN_DEVICE, KCAN_IOCTL_START + 78, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCANY_IOCTL_MEMO_DISK_IO_FAST \
    CTL_CODE(VCAN_DEVICE, KCAN_IOCTL_START + 79, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define KCAN_IOCTL_SCRIPT_CONTROL \
    CTL_CODE(VCAN_DEVICE, KCAN_IOCTL_START + 80, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCAN_IOCTL_LIN_MODE \
    CTL_CODE(VCAN_DEVICE, KCAN_IOCTL_START + 81, METHOD_BUFFERED, FILE_ANY_ACCESS)


#define KCAN_IOCTL_DEVICE_MESSAGES_SUBSCRIPTION \
    CTL_CODE(VCAN_DEVICE, KCAN_IOCTL_START + 89, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCAN_IOCTL_SCRIPT_GET_TEXT \
    CTL_CODE(VCAN_DEVICE, KCAN_IOCTL_START + 90, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCAN_IOCTL_SCRIPT_ENVVAR_CONTROL \
    CTL_CODE(VCAN_DEVICE, KCAN_IOCTL_START + 91, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCAN_IOCTL_SCRIPT_SET_ENVVAR \
    CTL_CODE(VCAN_DEVICE, KCAN_IOCTL_START + 92, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCAN_IOCTL_SCRIPT_GET_ENVVAR \
    CTL_CODE(VCAN_DEVICE, KCAN_IOCTL_START + 93, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define KCAN_IOCTL_FLASH_PROG \
    CTL_CODE(VCAN_DEVICE, KCAN_IOCTL_START + 94, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define KCAN_CARDFLAG_FIRMWARE_BETA     0x01 // Firmware is beta
#define KCAN_CARDFLAG_FIRMWARE_RC       0x02 // Firmware is release candidate
#define KCAN_CARDFLAG_AUTO_RESP_OBJBUFS 0x04 // Firmware supports auto-response object buffers
#define KCAN_CARDFLAG_REFUSE_TO_RUN     0x08 // Major problem detected
#define KCAN_CARDFLAG_REFUSE_TO_USE_CAN 0x10 // Major problem detected
#define KCAN_CARDFLAG_AUTO_TX_OBJBUFS   0x20 // Firmware supports periodic transmit object buffers


#define KCAN_DRVFLAG_BETA 0x01 // Driver is beta

#if defined(DEVHND_DRIVER_IS_BETA)
CompilerAssert(KCAN_DRVFLAG_BETA == DEVHND_DRIVER_IS_BETA);
#endif

typedef struct s_kcan_ioctl_read_tref_list {
    int64_t tReflist[16 * 2];
} KCAN_IOCTL_READ_TREF_LIST_VALUE;

// ---------------------------------------------------------------------------
// status
#define CAN_OPEN_MODE_SUCCESS         0
#define CAN_OPEN_MODE_MISMATCH        -1
#define CAN_OPEN_MODE_NOT_IMPLEMENTED -2
#define CAN_OPEN_MODE_FAILURE         -3

//fd
#define OPEN_AS_CAN          0
#define OPEN_AS_CANFD_ISO    1
#define OPEN_AS_CANFD_NONISO 2
#define OPEN_AS_LIN          3

// action
#define CAN_MODE_SET          1
#define CAN_MODE_READ         2
#define CAN_MODE_READ_VERSION 3

typedef struct {
    unsigned int mode; // CANFD_NONISO, CANFD_ISO, CAN
    unsigned int action; // CAN_CANFD_SET, CAN_CANFD_READ
    unsigned int reply; // reply from read?
    int status; // CAN_CANFD_MATCHING, CAN_CANFD_MISMATCH
    unsigned int unused[8];
} KCAN_IOCTL_OPEN_MODE_T;

typedef struct {
    unsigned int status; // !0 => no name found for this channel.
    unsigned char data[64]; // Transfered bytes between user space and kernel space
} KCAN_IOCTL_GET_CUST_CHANNEL_NAME_T;

/***************************************************************************/
// for vCanScriptXxx
// same script control command numbers as defined in hydra_host_cmds.h
// Translation done in mhydraHWIf.c
#define CMD_SCRIPT_START 0x1
#define CMD_SCRIPT_STOP  0x2

#define CMD_SCRIPT_EVENT        0x4
#define CMD_SCRIPT_LOAD         0x5
#define CMD_SCRIPT_QUERY_STATUS 0x6

#define CMD_SCRIPT_LOAD_REMOTE_START  0x7
#define CMD_SCRIPT_LOAD_REMOTE_DATA   0x8
#define CMD_SCRIPT_LOAD_REMOTE_FINISH 0x9
#define CMD_SCRIPT_UNLOAD             0xA

typedef struct s_kcan_ioctl_script_control {
    unsigned int scriptNo; /* which script to deliver command to aka slotNo*/
    unsigned int command;
    unsigned int channel;
    unsigned int script_control_status;
    union {
        struct {
            long type;
            long number;
            unsigned long data;
        } event;
        struct {
            char length;
            char data[11];
        } script;
        char data[12];
        signed char stopMode;
        unsigned int scriptStatus;
    };
} KCAN_IOCTL_SCRIPT_CONTROL_T;

typedef struct {
    uint32_t slot;
    uint32_t requestFlags;
} KCAN_IOCTL_DEVICE_MESSAGES_SUBSCRIPTION_T;

#define CMD_ENVVAR_GET_INFO     1
#define CMD_ENVVAR_GET_MAX_SIZE 2

typedef union {
    struct { // CMD_ENVVAR_GET_INFO
        unsigned int hash;
        int type;
        int length;
    } envvar_info;

    struct { // CMD_ENVVAR_GET_MAX_SIZE
        unsigned int maxsize;
    } envvar_maxsize;

    struct {
        char data[24]; // just make sure we have space for new stuff
    } raw_data_access;
} envvar_payload;

typedef struct s_kcan_ioctl_envvar_get_info {
    unsigned int subcommand;
    unsigned int channel;
    unsigned int hash;
    envvar_payload payload;
    int payloadLen;
    unsigned int envvar_status;
} KCAN_IOCTL_ENVVAR_GET_INFO_T;

typedef struct s_kcan_ioctl_set_envvar {
    unsigned int hash;
    char data[4096];
    int dataLen;
    unsigned int envvar_status;
} KCAN_IOCTL_SCRIPT_SET_ENVVAR_T;

typedef struct s_kcan_ioctl_get_envvar {
    unsigned int hash;
    char data[4096];
    int dataLen;
    int offset;
    unsigned int envvar_status;
} KCAN_IOCTL_SCRIPT_GET_ENVVAR_T;

// Get whatever info from a driver/card
#define KCAN_IOCTL_MISC_INFO_SUBCMD_CHANNEL_REMOTE_INFO 1
#define KCAN_IOCTL_MISC_INFO_SUBCMD_CHANNEL_LOGGER_INFO 2
#define KCAN_IOCTL_MISC_INFO_SUBCMD_HW_STATUS           3
#define KCAN_IOCTL_MISC_INFO_SUBCMD_FEATURE_EAN         4

#define KCAN_IOCTL_MISC_INFO_RETCODE_SUCCESS 0
#define KCAN_IOCTL_MISC_INFO_NOT_IMPLEMENTED 1

#define KCAN_IOCTL_MISC_INFO_REMOTE_NO_WEBSERVER 0
#define KCAN_IOCTL_MISC_INFO_REMOTE_WEBSERVER_V1 1

#define KCAN_IOCTL_MISC_INFO_REMOTE_TYPE_NOT_REMOTE 0
#define KCAN_IOCTL_MISC_INFO_REMOTE_TYPE_WLAN       1
#define KCAN_IOCTL_MISC_INFO_REMOTE_TYPE_LAN        2

#define KCAN_IOCTL_MISC_INFO_LOGGER_TYPE_NOT_A_LOGGER 0
#define KCAN_IOCTL_MISC_INFO_LOGGER_TYPE_V1           1
#define KCAN_IOCTL_MISC_INFO_LOGGER_TYPE_V2           2

typedef struct {
    unsigned int isRemote;
    unsigned int webServer;
    unsigned int remoteType;
} miscSubCmdRemoteInfo;

typedef struct {
    unsigned int loggerType;
} miscSubCmdLoggerInfo;

typedef struct {
    unsigned int codes[6];
} miscSubCmdHwStatus;

typedef struct {
    unsigned int eanLo;
    unsigned int eanHi;
} miscSubCmdFeatureEan;

union subcmd_payload {
    unsigned char payload[120];
    miscSubCmdRemoteInfo remoteInfo;
    miscSubCmdLoggerInfo loggerInfo;
    miscSubCmdHwStatus hwStatus;
    miscSubCmdFeatureEan featureEan;
};

typedef struct s_kcan_ioctl_misc_info {
    unsigned int subcmd;
    unsigned int retcode; // Status code from device
    union subcmd_payload payload;
} KCAN_IOCTL_MISC_INFO;
CompilerAssert(sizeof(KCAN_IOCTL_MISC_INFO) == 128);

//===========================================================================
// for KCAN_IOCTL_LED_ACTION_I
#define KCAN_LED_SUBCOMMAND_ALL_LEDS_ON  0
#define KCAN_LED_SUBCOMMAND_ALL_LEDS_OFF 1
#define KCAN_LED_SUBCOMMAND_LED_0_ON     2
#define KCAN_LED_SUBCOMMAND_LED_0_OFF    3
#define KCAN_LED_SUBCOMMAND_LED_1_ON     4
#define KCAN_LED_SUBCOMMAND_LED_1_OFF    5
#define KCAN_LED_SUBCOMMAND_LED_2_ON     6
#define KCAN_LED_SUBCOMMAND_LED_2_OFF    7
#define KCAN_LED_SUBCOMMAND_LED_3_ON     8
#define KCAN_LED_SUBCOMMAND_LED_3_OFF    9

typedef struct s_kcan_ioctl_led_action {
    unsigned long sub_command; // One of KCAN_LED_SUBCOMMAND_xxx
    int timeout;
} KCAN_IOCTL_LED_ACTION_I;

// Arguments from canFileGetName()
typedef struct {
    int fileNo;
    char *name;
    int namelen;
} VCAN_FILE_GET_NAME_T;

typedef struct {
    int interval;
    int padding[10]; // for future usage.
} KCANY_CONFIG_MODE;

typedef struct {
    unsigned int subcommand;
    int status; // From the driver: MEMO_STATUS_xxx
    int dio_status; // From the driver: DioResult
    int lio_status; // From the driver: LioResult
    unsigned int buflen;
    unsigned int timeout; // Timeout in ms
    unsigned char buffer[1000]; // Contents & usage dependent on subcommand
} KCANY_MEMO_INFO;

typedef struct {
    unsigned int subcommand;
    unsigned int first_sector;
    unsigned int count;
    unsigned char buffer[512];
    int status; // From the driver: MEMO_STATUS_...
    int dio_status; // From the driver: DioResult
    int lio_status; // From the driver: LioResult
} KCANY_MEMO_DISK_IO;

typedef struct {
    unsigned int subcommand; // To the driver: ..IO_FASTREAD..
    unsigned int first_sector; // To the driver: first sector no
    unsigned int count; // To the driver: sector count
    int status; // From the driver: MEMO_STATUS_...
    int dio_status; // From the driver: DioResult
    int lio_status; // From the driver: LioResult
    unsigned char buffer[16][512];
} KCANY_MEMO_DISK_IO_FAST;

// Must agree with MEMO_SUBCMD_xxx in filo_cmd.h
#define KCANY_MEMO_DISK_IO_READ_PHYSICAL_SECTOR     4
#define KCANY_MEMO_DISK_IO_WRITE_PHYSICAL_SECTOR    5
#define KCANY_MEMO_DISK_IO_ERASE_PHYSICAL_SECTOR    6
#define KCANY_MEMO_DISK_IO_READ_LOGICAL_SECTOR      7
#define KCANY_MEMO_DISK_IO_WRITE_LOGICAL_SECTOR     8
#define KCANY_MEMO_DISK_IO_ERASE_LOGICAL_SECTOR     9
#define KCANY_MEMO_DISK_IO_FASTREAD_PHYSICAL_SECTOR 17
#define KCANY_MEMO_DISK_IO_FASTREAD_LOGICAL_SECTOR  18


#define KCAN_USBSPEED_NOT_AVAILABLE 0
#define KCAN_USBSPEED_FULLSPEED     1
#define KCAN_USBSPEED_HISPEED       2

#define MAX_IOCTL_CARD_NAME      31
#define MAX_IOCTL_DRIVER_NAME    31
#define MAX_IOCTL_VENDOR_NAME    31
#define MAX_IOCTL_CHANNEL_PREFIX 31

typedef struct s_kcan_ioctl_card_info {
    char card_name[MAX_IOCTL_CARD_NAME + 1], driver_name[MAX_IOCTL_DRIVER_NAME + 1];
    int hardware_type, channel_count;
    unsigned int driver_version_major, driver_version_minor, driver_version_build,
        firmware_version_major, firmware_version_minor, firmware_version_build, hardware_rev_major,
        hardware_rev_minor;
    unsigned int license_mask1, license_mask2, card_number;
    unsigned int serial_number;
    unsigned int timer_rate;
    char vendor_name[MAX_IOCTL_VENDOR_NAME + 1];
    char channel_prefix[MAX_IOCTL_CHANNEL_PREFIX + 1];
    unsigned int product_version_major, product_version_minor, product_version_minor_letter;
    unsigned long max_bitrate;
    unsigned int reserved[43];
} VCAN_IOCTL_CARD_INFO;

typedef struct s_kcan_ioctl_card_info_2 {
    unsigned char ean[8];
    unsigned long hardware_address;
    unsigned long ui_number;
    unsigned long usb_speed; // KCAN_USBSPEED_xxx
    unsigned long softsync_running;
    long softsync_instab;
    long softsync_instab_max;
    long softsync_instab_min;
    unsigned long card_flags; // KCAN_CARDFLAG_xxx
    unsigned long driver_flags; // KCAN_DRVFLAG_xxx
    char pcb_id[32]; // e.g. P023B002V1-2 (see doc Q023-059)
    unsigned long mfgdate; // Seconds since 1970-01-01
    unsigned long usb_host_id; // Checksum of USB host controller
    unsigned int usb_throttle; // Enforced delay between transmission of commands.
    unsigned char reserved[40];
} KCAN_IOCTL_CARD_INFO_2;


#include "kcan_ioctl_flash.h"
#endif /* KCANIO_H */
