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
**  Reader for logevents produced by memolib (that is, this is no file
**  format, the events are made by memolib)
** ---------------------------------------------------------------------------
*/

#include <stdio.h>
#include <string.h>
#include "common_defs.h"
#include "kvaConverter.h"
#include "KvaLogReader_memoLogEvent.h"
#include "TimeConv.h"
#include "kvdebug.h"

//===========================================================================
KvlcStatus KvaLogReader_memoLogEvent::interpret_event(
      memoLogEventEx *me,
      imLogData *le)
{
  int i;

  memset(le->common.transceiver_type, TRANSCEIVER_TYPE_HS, MAX_CHANNELS*sizeof(char));
  le->common.event_counter = ++current_eventno;

  switch (me->type) {

    case MEMOLOG_TYPE_INVALID:
    {
      PRINTF(("Invalid memoLogEvent\n"));
      return kvlcERR_INVALID_LOG_EVENT;
    }

    case MEMOLOG_TYPE_CLOCK:
    {
      unsigned int year, month, day, hour, minute, second;
      time_uint64 caltim;

      caltim = (time_uint64) me->x.rtc.calendarTime;
      last_clock_event = caltim * ONE_BILLION;
      unix_time(caltim, &year, &month, &day, &hour, &minute, &second);

      le->common.new_data          = true;
      le->common.type              = ILOG_TYPE_RTC;

      le->common.nanos_since_1970 = ONE_BILLION * me->x.rtc.calendarTime;

      if (start_of_measurement64 == 0) {
        if (first_timestamp) {
          PRINTF(("Start of measurement read (memoLogEvent) = %d - %lu\n",
                  (int)caltim, first_timestamp));
          start_of_measurement64 = caltim * ONE_BILLION - first_timestamp;
          time_gap = false;
          PRINTF(("Deals with next file by updating start_of_measurement64"));
        }
      }
      else if (time_gap) {
        if (first_timestamp) {
          PRINTF(("RTC read (memoLogEvent) = %lu\n", caltim * ONE_BILLION));
          // (start - calendar)
          time_gap = false;

          time_of_last_clock_event = caltim * ONE_BILLION - start_of_measurement64;
          time_of_last_clock_event -= first_timestamp;
          PRINTF(("time_of_last_clock_event %lu", time_of_last_clock_event));

          if (time_of_last_clock_event >= -4 * ONE_BILLION) {
            if (time_of_last_clock_event <=  4 * ONE_BILLION) {
              time_of_last_clock_event = 0;
            }
          } else {
            PRINTF(("Decreasing time between files!!!"));
            return kvlcERR_TIME_DECREASING;
          }
        }
      }
      le->common.time64 = time_of_last_clock_event + (time_uint64)me->x.rtc.timeStamp;

      return kvlcOK;
    }

    case MEMOLOG_TYPE_MSG:
    {
      le->msg.frame_counter = ++current_frameno;
      le->common.new_data          = true;
      le->common.type              = ILOG_TYPE_MESSAGE;
      le->msg.flags   = me->x.msg.flags;
      le->msg.id      = me->x.msg.id;
      le->msg.channel = (unsigned char)me->x.msg.channel;
      le->msg.dlc     = numBytesToDLC(me->x.msg.dlc);
      for (i = 0; i < 64; i++) {
        le->msg.data[i] = (char)me->x.msg.data[i];
      }

      if (!first_timestamp) {
        first_timestamp = me->x.msg.timeStamp;
        if (start_of_measurement64 && time_gap && last_clock_event) {
          PRINTF(("RTC read (memoLogEvent) = %lu\n", last_clock_event));
          // (start - calendar)
          time_gap = false;

          time_of_last_clock_event = last_clock_event - start_of_measurement64;
          time_of_last_clock_event -= first_timestamp;
          PRINTF(("time_of_last_clock_event %lu", time_of_last_clock_event));

          if (time_of_last_clock_event >= -4 * ONE_BILLION) {
            if (time_of_last_clock_event <=  4 * ONE_BILLION) {
              time_of_last_clock_event = 0;
            }
          } else {
            PRINTF(("Decreasing time between files!!!"));
            return kvlcERR_TIME_DECREASING;
          }
        }
      }
      le->common.time64 = time_of_last_clock_event +
                             ((time_uint64)(me->x.msg.timeStamp));
      return kvlcOK;
    }

    case MEMOLOG_TYPE_TRIGGER:
    {
      if (!first_timestamp) {
        first_timestamp = me->x.trig.timeStamp;
        if (start_of_measurement64 && time_gap && last_clock_event) {
             PRINTF(("RTC read (memoLogTrigger) = %lu\n", last_clock_event));
          // (start - calendar)
          time_gap = false;

          time_of_last_clock_event = last_clock_event - start_of_measurement64;
          time_of_last_clock_event -= first_timestamp;
          PRINTF(("time_of_last_clock_event %lu", time_of_last_clock_event));

          if (time_of_last_clock_event >= -4 * ONE_BILLION) {
            if (time_of_last_clock_event <=  4 * ONE_BILLION) {
              time_of_last_clock_event = 0;
            }
          } else {
            PRINTF(("Decreasing time between files!!!"));
            return kvlcERR_TIME_DECREASING;
          }
        }
      }

      le->common.new_data            = true;
      le->common.type                = ILOG_TYPE_TRIGGER;
      le->trig.active   = true;
      le->trig.trigNo   = (unsigned char)me->x.trig.trigNo;
      le->trig.postTrigger  = me->x.trig.postTrigger;
      le->trig.preTrigger   = me->x.trig.preTrigger;
      le->common.time64       = time_of_last_clock_event +
                                    (time_uint64)me->x.trig.timeStamp;
      le->trig.type     = (unsigned char)me->x.trig.type;
      return kvlcOK;
    }

    case MEMOLOG_TYPE_VERSION:
    {
      le->common.new_data = true;
      le->common.type = ILOG_TYPE_VERSION;
      le->ver.lioMajor = me->x.ver.lioMajor;
      le->ver.lioMinor = me->x.ver.lioMinor;
      le->ver.fwMajor = me->x.ver.fwMajor;
      le->ver.fwMinor = me->x.ver.fwMinor;
      le->ver.fwBuild = me->x.ver.fwBuild;
      le->ver.serialNumber = me->x.ver.serialNumber;
      le->ver.eanHi = me->x.ver.eanHi;
      le->ver.eanLo = me->x.ver.eanLo;
      le->common.time64 = 0;
      return kvlcOK;
    }

    default:
    {
      PRINTF(("Unknown type of memo log event (%d)\n",me->type));
      return kvlcERR_INVALID_LOG_EVENT;
      break;
    }
  }
  return kvlcOK;
}


//===========================================================================
KvlcStatus KvaLogReader_memoLogEvent::read_row(imLogData *logEvent)
{
  if (!isOpened) {
    PRINTF(("KvaLogReader_memoLogEvent::read_row, file not open\n"));
    return kvlcERR_FILE_ERROR;
  }

  read_file((char *)&current_memoLogEvent, sizeof(memoLogEventEx));

  return interpret_event(&current_memoLogEvent, logEvent);
}


//===========================================================================
uint64 KvaLogReader_memoLogEvent::event_count()
{
  return (uint64) ((file_size-file_position) / sizeof(memoLogEventEx));
}


//===========================================================================
KvaLogReader_memoLogEvent::KvaLogReader_memoLogEvent()
{
  PRINTF(("KvaLogReader_memoLogEvent::KvaLogReader_memoLogEvent()\n"));
  time_of_last_clock_event = 0;
  current_eventno = 0;
  current_frameno = 0;
  time_gap = true;
}

//===========================================================================
KvaLogReader_memoLogEvent::~KvaLogReader_memoLogEvent()
{ 
  PRINTF(("~KvaLogReader_memoLogEvent\n"));
}


//===========================================================================
KvlcStatus KvaLogReader_memoLogEvent::next_file()
{
  PRINTF(("KvaLogReader_memoLogEvent::next_file()"));
  time_gap = true;
  last_clock_event = 0;
  first_timestamp = 0;
  return kvlcOK;
}

class KvaReaderMaker_MemoLog : public KvaReaderMaker
{
  public:
    KvaReaderMaker_MemoLog() : KvaReaderMaker(KVLC_FILE_FORMAT_MEMO_LOG) {}
    int getName(char *str) { return sprintf(str, "%s", "Kvaser Memorator"); }
    int getExtension(char *str) { return sprintf(str, "%s", "memo"); }
    int getDescription(char *str) { return sprintf(str, "%s", "Kvaser Memorator"); }

  private:
    KvaLogReader *OnCreateReader() {
      return new KvaLogReader_memoLogEvent();
    }
}  registerKvaLogReader_MemoLog;
