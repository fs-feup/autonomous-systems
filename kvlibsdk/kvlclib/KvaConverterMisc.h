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

#ifndef KVACONVERTERMISC_H_
#define KVACONVERTERMISC_H_

#include "canstat.h"
#include "kvlclib.h"
#include "common_defs.h"

#ifndef MIN
#  define MIN(x, y) (((x) < (y)) ? (x) : (y))
#endif
#ifndef MAX
#  define MAX(x, y) (((x) > (y)) ? (x) : (y))
#endif

#define KVACONV_MAX_PATH_LEN   (1024 - 1) // Space for null-termniator
#define KVACONV_ID_MASK        0x1FFFFFFF
#define MAX_CHANNELS           16
#define N_BYTES_MAX_IN_CAN_MSG 64

#define ILOG_TYPE_TRIGGER   0
#define ILOG_TYPE_MESSAGE   1
#define ILOG_TYPE_CANOTHER  2
#define ILOG_TYPE_RTC       3
#define ILOG_TYPE_VERSION   4
#define ILOG_TYPE_EXTERNAL  5  // Non-kvaser format may contain events we have no translation for. If set as external, the event will be flushed.

// flags for char transceiver_type[MAX_CHANNELS]
#define TRANSCEIVER_TYPE_HS             0
#define TRANSCEIVER_TYPE_SWC            6 // from loggerDefinitions.h

typedef struct {
  char          type;
  bool          new_data;
  uint64        event_counter;
  char          transceiver_type[MAX_CHANNELS];
  time_uint64   nanos_since_1970;
  time_uint64   time64;  // Timestamp at SOF or undefined timestamp of msg
}imLogData_common;

typedef struct {
  imLogData_common common;
  unsigned char    type;
  long             preTrigger;
  unsigned long    postTrigger;
  unsigned char    trigNo; // A bit mask showing which trigger was activated
  bool             active; // is trigger active?
} imLogData_trigger;

typedef struct {
  imLogData_common common;
  char             dlc;                           // 4 bit i CAN
  uint32_t         id;                            // 11/29 bit i CAN, 4? bit i LIN
  unsigned int     flags;                         // SWC, LS, NERR, WAKEUP, OVERFLOW, ERRORFRAME...
  unsigned char    channel;                       // which CAN interface received the msg, counts from 0
  unsigned long    frame_counter;                 // message counter
  char             data[N_BYTES_MAX_IN_CAN_MSG];  // always last!!!
} imLogData_canMessage;

typedef struct {
  imLogData_common common;
  unsigned int     status; // bitmask with buson/off, active/passive, etc, etc
} imLogData_canStatus;

typedef struct
{
  imLogData_common common;
  unsigned char    lioMajor;       // Lio major version
  unsigned char    lioMinor;       // Lio minor version
  unsigned char    fwMajor;        // Firmware major version
  unsigned char    fwMinor;        // Firmware major version
  unsigned short   fwBuild;        // Firmware build version
  unsigned long    serialNumber;   // Serial
  unsigned long    eanHi;          // EANHI
  unsigned long    eanLo;          // EANLO
} imLogData_version;

// Intermediate logdata
typedef union {
  imLogData_common       common;
  imLogData_trigger      trig;
  imLogData_canMessage   msg;
  imLogData_canStatus    canother;
  imLogData_version      ver;
} imLogData;

unsigned int dlcToNumBytesFD (unsigned int dlc);
char numBytesToDLC(unsigned int numBytes);
FILE* utf_fopen(const char* filename, const char* type);


#endif /*KVACONVERTERMISC_H_*/
