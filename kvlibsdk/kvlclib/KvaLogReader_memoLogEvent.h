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

#ifndef KVALOGREADER_MEMOLOGEVENT_H_
#define KVALOGREADER_MEMOLOGEVENT_H_

#include "common_defs.h"

typedef uint8_t        uint8;
typedef uint32_t       uint32;
typedef int32_t        int32;
typedef int64_t        int64;

#include <pshpack1.h>

// Several small spieces of small furry structs gathered together from
// kvaMemoLib.h and grooved with a pict; could be kvaMemolib.h

// For memoLogEventEx
#define MEMOLOG_TYPE_INVALID    0
#define MEMOLOG_TYPE_CLOCK      1
#define MEMOLOG_TYPE_MSG        2
#define MEMOLOG_TYPE_TRIGGER    3
#define MEMOLOG_TYPE_VERSION    4


typedef struct {
  int32   type;
  int32   preTrigger;
  int32   postTrigger;
  uint32  trigNo;         // Bitmask with the activated trigger(s)
  int64   timeStamp;      // timestamp in units of 1 nanoseconds
} memoLogTriggerEx;

typedef struct {
  uint32 id;            // The identifier
  int64  timeStamp;     // timestamp in units of 1 nanoseconds
  uint32 channel;       // The channel on which the message arrived, 0,1,...
  uint32 dlc;           // The length of the message
  uint32 flags;         // Message flags
  uint8  data[64];       // Message data (8 bytes)
} memoLogMsgEx;

typedef struct {
  uint32 calendarTime;   // RTC date (unix format)
  int64  timeStamp;
} memoLogRtcClockEx;

typedef struct
{
  uint32 lioMajor;     // Lio major version
  uint32 lioMinor;     // Lio minor version
  uint32 fwMajor;      // Firmware major version
  uint32 fwMinor;      // Firmware major version
  uint32 fwBuild;      // Firmware build version
  uint32 serialNumber; // Serial
  uint32 eanHi;        // EANHI
  uint32 eanLo;        // EANLO
} memoLogVersionEx;

typedef struct {
  uint32                type;   // MEMOLOG_TYPE_xxx
  union {
    memoLogMsgEx        msg;
    memoLogRtcClockEx   rtc;
    memoLogTriggerEx    trig;
    memoLogVersionEx    ver;
    uint8               raw[128];
  } x;
} memoLogEventEx;


#include <poppack.h>

class KvaLogReader_memoLogEvent : public KvaLogReader {
  private:
    time_int64      time_of_last_clock_event; // nanoseconds since 1970-01-01 00:00
    uint64          current_eventno;
    unsigned long   current_frameno;
    memoLogEventEx  current_memoLogEvent;
    KvlcStatus interpret_event(memoLogEventEx *me,
                                   imLogData *le);
  bool time_gap;

public:
  KvaLogReader_memoLogEvent();
  ~KvaLogReader_memoLogEvent();

  KvlcStatus interpret_event(void *event,
                                 imLogData *logEvent)
  {
    return interpret_event((memoLogEventEx *)event, logEvent);
  }
  KvlcStatus read_row(imLogData *logEvent);
  uint64 event_count();
  bool isBinary() { return true; }
  KvlcStatus next_file();
};

#endif
