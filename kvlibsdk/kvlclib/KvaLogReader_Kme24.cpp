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

#include "common_defs.h"
#include "KvaLogReader_Kme24.h"
#include "kme_24_format.h"
#include "TimeConv.h"
#include "kvdebug.h"
#include "kvaConverter.h"
#include "kvlclib.h"

KvlcStatus KvaLogReader_Kme24::interpret_event(
      kme24_logStruct *kmeEvent,
      imLogData *logEvent
    )
{
  memset(logEvent->common.transceiver_type, TRANSCEIVER_TYPE_HS, MAX_CHANNELS*sizeof(char));
  logEvent->common.event_counter = ++current_eventno;

  switch (kmeEvent->type) {

    case KME24_LOG_TYPE_CAN_CH_0:
    case KME24_LOG_TYPE_CAN_CH_1:
    case KME24_LOG_TYPE_CAN_CH_2:
    case KME24_LOG_TYPE_CAN_CH_3:
    case KME24_LOG_TYPE_CAN_CH_4:
    {
      logEvent->msg.frame_counter = ++current_frameno;
      logEvent->common.new_data = true;
      logEvent->common.type = ILOG_TYPE_MESSAGE;
      logEvent->msg.flags = 0;

      logEvent->msg.id =
          kmeEvent->payload.frame.id & 0x1fffffff; // max 29 bit
      if (kmeEvent->payload.frame.id & KME24_LOG_MSG_EXT_FLAG) {
        logEvent->msg.flags |= canMSG_EXT;
      }
      else {
        logEvent->msg.flags |= canMSG_STD;
      }

      logEvent->msg.dlc = kmeEvent->payload.frame.dlc & 0x0f;
      memcpy(logEvent->msg.data, kmeEvent->payload.frame.data, 8);
      logEvent->msg.channel = event_to_channel(kmeEvent->type);
      logEvent->common.time64 = (time_of_last_clock_event)
        + ((time_uint64)(kmeEvent->payload.frame.time)) * (time_uint64)10000;

      if (kmeEvent->payload.frame.id & KME24_LOG_MSG_NERR_FLAG) {
          logEvent->msg.flags |= canMSG_NERR;
       }

      if (kmeEvent->payload.frame.dlc & KME24_LOG_MSG_RTR_FLAG) {
        logEvent->msg.flags |= canMSG_RTR;
      }
      if (kmeEvent->payload.frame.dlc & KME24_LOG_MSG_ERROR_FLAG) {
        logEvent->msg.flags |= canMSG_ERROR_FRAME;
      }
      if (kmeEvent->payload.frame.dlc & KME24_LOG_MSG_OVR_FLAG) {
        logEvent->msg.flags |= canMSGERR_HW_OVERRUN;
        logEvent->msg.flags |= canMSGERR_SW_OVERRUN;
      }
      break;
    }

    case KME24_LOG_TYPE_TRIGGER:
    {
      logEvent->common.new_data = true;
      logEvent->common.type = ILOG_TYPE_TRIGGER;
      logEvent->trig.active = true;
      logEvent->trig.postTrigger = kmeEvent->payload.trig.postTrigger;
      logEvent->trig.preTrigger = kmeEvent->payload.trig.preTrigger;
      logEvent->common.time64 = time_of_last_clock_event
        + (time_uint64)kmeEvent->payload.trig.time * (time_uint64)10000;
      logEvent->trig.type = kmeEvent->payload.trig.type;
      logEvent->trig.trigNo = kmeEvent->payload.trig.trigNo;
      break;
    }

    case KME24_LOG_TYPE_TIME:
    {

      time_uint64 overflow = ((time_uint64) kmeEvent->payload.clock.hiResTimeStampHi) << 16;
      time_of_last_clock_event = 10000 * overflow;

      logEvent->common.new_data = true;
      logEvent->common.type = ILOG_TYPE_RTC;

      logEvent->common.nanos_since_1970 = ONE_BILLION * convert_time(
        (kmeEvent->payload.clock.date >> 9)+1980,
        (kmeEvent->payload.clock.date >> 5) & 0x0F,
        kmeEvent->payload.clock.date & 0x1F,
        kmeEvent->payload.clock.time >> 11,
        (kmeEvent->payload.clock.time >> 5) & 0x3F,
        (kmeEvent->payload.clock.time & 0x1F) * 2);

      logEvent->common.time64 = ((((time_uint64) kmeEvent->payload.clock.hiResTimeStampHi) << 16) + ((time_uint64) kmeEvent->payload.clock.hiResTimeStampLo))*10000;
      logEvent->common.nanos_since_1970 += (((time_uint64) kmeEvent->payload.clock.hiResTimeStampHi) << 16) + ((time_uint64) kmeEvent->payload.clock.hiResTimeStampLo)*10000;

      if (start_of_measurement64 == 0) {
        start_of_measurement64 = logEvent->common.nanos_since_1970;
        PRINTF(("Start of measurement read  = %lu.%lu\n", start_of_measurement64 / ONE_BILLION, start_of_measurement64 % ONE_BILLION));
      }
      break;
    }

    default:
    {
      PRINTF(("Unknown type of kme event: %d\n", kmeEvent->type));
      return kvlcERR_INVALID_LOG_EVENT;
      break;
    }
  }
  return kvlcOK;
}


KvlcStatus KvaLogReader_Kme24::read_row(imLogData *logEvent)
{
  KvlcStatus status;

  if (!isOpened) {
    PRINTF(("KvaLogReader_Kme24::read_row, file not open\n"));
    return kvlcERR_FILE_ERROR;
  }

  status = read_file((char *)&current_kmeEvent, 16);
  if (kvlcOK != status) {
    PRINTF(("KvaLogReader_Kme24::read_row, read_file() failed with %d\n", status));
    return status;
  }

  return interpret_event(&current_kmeEvent, logEvent);
}

uint64 KvaLogReader_Kme24::event_count()
{
  return (uint64) ((file_size-file_position) / sizeof(kme24_logStruct));
}

unsigned char KvaLogReader_Kme24::event_to_channel(unsigned char event)
{
  int channel = 0;

  switch (event) {
  case KME24_LOG_TYPE_CAN_CH_0:
    channel = 0;
    break;
  case KME24_LOG_TYPE_CAN_CH_1:
    channel = 1;
    break;
  case KME24_LOG_TYPE_CAN_CH_2:
    channel = 2;
    break;
  case KME24_LOG_TYPE_CAN_CH_3:
    channel = 3;
    break;
  case KME24_LOG_TYPE_CAN_CH_4:
    channel = 4;
    break;
  default:
    PRINTF(("Error: Unsupported event = %d\n", event));
  }

  return channel;
}


KvaLogReader_Kme24::KvaLogReader_Kme24()
{
  PRINTF(("KvaLogReader_Kme24::KvaLogReader_Kme24()\n"));
  time_of_last_clock_event = 0;
  current_eventno = 0;
  current_frameno = 0;
}

KvaLogReader_Kme24::~KvaLogReader_Kme24() 
{ 
  PRINTF(("~KvaLogReader_Kme24\n"));
}


class KvaReaderMaker_Kme24 : public KvaReaderMaker
{
  public:
    KvaReaderMaker_Kme24() : KvaReaderMaker(KVLC_FILE_FORMAT_KME24) {}
    int getName(char *str) { return sprintf(str, "%s", "KME 2.4"); }
    int getExtension(char *str) { return sprintf(str, "%s", "kme"); }
    int getDescription(char *str) { return sprintf(str, "%s", "Kvaser binary format (KME 2.4)"); }

  private:
    KvaLogReader *OnCreateReader() {
      return new KvaLogReader_Kme24();
    }
}  registerKvaLogReader_Kme24;
