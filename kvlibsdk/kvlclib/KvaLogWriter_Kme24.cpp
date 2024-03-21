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
#include "KvaLogWriter_Kme24.h"
#include "kme_24_format.h"
#include "KvaLogWriter.h"
#include "TimeConv.h"
#include "kvdebug.h"

KvaLogWriter_Kme24::KvaLogWriter_Kme24()
{
  PRINTF(("KvaLogWriter_Kme24::KvaLogWriter_Kme24()\n"));
  timeLo = 0;
  timeHi = 0;
  frameCounter = 0;
  memset(&KmeEventOld, 0, sizeof(KmeEventOld));
}

KvlcStatus KvaLogWriter_Kme24::write_header()
{
  timeLo = 0;
  timeHi = 0;
  frameCounter = 0;
  memset(&KmeEventOld, 0, sizeof(KmeEventOld));
  return kvlcOK;
}

KvlcStatus KvaLogWriter_Kme24::write_row(imLogData *logEvent)
{
  kme24_logStruct KmeEvent;
  KvlcStatus kvstatus = kvlcOK;

  if (!isOpened) {
    PRINTF(("KvaLogWriter_Kme24::write_row, file was not opened\n"));
    return kvlcERR_FILE_ERROR;
  }

  if (!start_of_logging) {
    setStartOfLogging(logEvent);
    if (start_of_logging) {
      // Generate a time event first
      unsigned int year, month, day, hour, minute, second;

      KmeEvent.type = KME24_LOG_TYPE_TIME;
      KmeEvent.payload.clock.hiResTimeStampHi = 0;
      KmeEvent.payload.clock.hiResTimeStampLo = 0;
      KmeEvent.payload.clock.res = 0; // Seams to be 0 in old logfiles... unused?

      unix_time(start_of_logging / ONE_BILLION, &year, &month, &day, &hour, &minute, &second);

      KmeEvent.payload.clock.frameCounter = frameCounter;
      KmeEvent.payload.clock.date = (year-1980) << 9;
      KmeEvent.payload.clock.date |= month << 5;
      KmeEvent.payload.clock.date |= day;
      KmeEvent.payload.clock.time = hour << 11;
      KmeEvent.payload.clock.time |= minute << 5;
      KmeEvent.payload.clock.time |= second /2;
      KmeEvent.payload.clock.loResTimeStamp = 0;

      kvstatus = write_file(&KmeEvent, sizeof(KmeEvent));
      if (kvstatus != kvlcOK) {
        logEvent->common.new_data = false;
        return kvstatus;
      }
    }
  }

  memset(&KmeEvent, 0, sizeof(KmeEvent));
  switch (logEvent->common.type) {

    case ILOG_TYPE_MESSAGE:
    {
      if (!((1<<logEvent->msg.channel) & get_property_channel_mask())) {
        // Don't use this channel so bail out
        return kvlcOK;
      }

      unsigned short lastTimeHi;
      frameCounter++;

      lastTimeHi = timeHi;
      timeLo = (unsigned short)(logEvent->common.time64 / 10000);
      timeHi = (unsigned short)((logEvent->common.time64 / 10000)/ 65536);

      kvstatus = add_periodic_events(lastTimeHi, logEvent->common.time64);
      if (kvstatus != kvlcOK) {
        break;
      }

      // Channel/type
      KmeEvent.type = channel_to_event(logEvent->msg.channel);

      // Id and flags in id field
      KmeEvent.payload.frame.id = logEvent->msg.id;
      if (logEvent->msg.flags & canMSG_EXT) {
        KmeEvent.payload.frame.id |= KME24_LOG_MSG_EXT_FLAG;
      }
      if (logEvent->msg.flags & canMSG_NERR) {
        KmeEvent.payload.frame.id |= KME24_LOG_MSG_NERR_FLAG;
      }
      // dlc and flags in the dlc field
      KmeEvent.payload.frame.dlc = logEvent->msg.dlc;
      if (logEvent->msg.flags & canMSG_RTR) {
        KmeEvent.payload.frame.dlc |= KME24_LOG_MSG_RTR_FLAG;
      }
      if (logEvent->msg.flags & canMSG_ERROR_FRAME) {
        KmeEvent.payload.frame.dlc |= KME24_LOG_MSG_ERROR_FLAG;
      }
      if (logEvent->msg.flags & canMSGERR_OVERRUN) {
        KmeEvent.payload.frame.dlc |= KME24_LOG_MSG_OVR_FLAG;
        overrun_occurred = true;
      }
      if (logEvent->msg.flags & canFDMSG_FDF) {
        // There are no flag bits left for CANFD flags in KME24
        if (dlcToNumBytesFD(logEvent->msg.dlc) > 8) {
          data_truncation_occurred = true;
        }
      }

      // The msg timestamp divided by 10 000 to get the timestamp in 10
      // microsecs

      KmeEvent.payload.frame.time = (unsigned short)(logEvent->common.time64 / 10000);

      memcpy(KmeEvent.payload.frame.data, logEvent->msg.data, 8);
      kvstatus = write_file(&KmeEvent, sizeof(KmeEvent));
      break;
    }

    case ILOG_TYPE_TRIGGER:
    {
      unsigned short lastTimeHi = timeHi;
      timeLo = (unsigned short)(logEvent->common.time64 / 10000);
      timeHi = (unsigned short)((logEvent->common.time64 / 10000)/ 65536);

      kvstatus = add_periodic_events(lastTimeHi, logEvent->common.time64);
      if (kvstatus != kvlcOK) {
        break;
      }

      KmeEvent.payload.trig.postTrigger = logEvent->trig.postTrigger;
      KmeEvent.payload.trig.preTrigger = logEvent->trig.preTrigger;
      KmeEvent.payload.trig.time = (unsigned short) (logEvent->common.time64 /10000);
      KmeEvent.payload.trig.type = logEvent->trig.type;
      KmeEvent.type = KME24_LOG_TYPE_TRIGGER;
      KmeEvent.payload.trig.trigNo = logEvent->trig.trigNo;
      kvstatus = write_file(&KmeEvent, sizeof(KmeEvent));
      break;
    }

    case ILOG_TYPE_RTC:
    {
      // Is it the first RTC message after the 2 second firmware update?
      if (KmeEventOld.payload.clock.time != KmeEvent.payload.clock.time ||
          KmeEventOld.payload.clock.date != KmeEvent.payload.clock.date) {
          memcpy(&KmeEventOld, &KmeEvent, sizeof(KmeEventOld));

          // Is it time to create a new file?
          if (mSplitPending) {
            open_new_file();
            kvstatus = write_file(&KmeEvent, sizeof(KmeEvent));
            kvstatus = write_file(&KmeEventOld, sizeof(KmeEventOld));
          }
      }

      // We make our own when we need them
      break;
    }

    case ILOG_TYPE_VERSION:
    {
      // Version is not implemented in this format.
      break;
    }

    default:
      PRINTF(("logEvent->type = %d, frame_counter = %lu\n\n\n",
              logEvent->common.type, logEvent->common.event_counter));
      kvstatus = kvlcERR_INVALID_LOG_EVENT;
      break;

  }
  logEvent->common.new_data = false;
  return kvstatus;
}

unsigned char KvaLogWriter_Kme24::channel_to_event(unsigned char channel)
{
  int event = KME24_LOG_TYPE_CAN_CH_0;

  switch (channel) {
  case 0:
    event = KME24_LOG_TYPE_CAN_CH_0;
    break;
  case 1:
    event = KME24_LOG_TYPE_CAN_CH_1;
    break;
  case 2:
    event = KME24_LOG_TYPE_CAN_CH_2;
    break;
  case 3:
    event = KME24_LOG_TYPE_CAN_CH_3;
    break;
  case 4:
    event = KME24_LOG_TYPE_CAN_CH_4;
    break;
  default:
    PRINTF(("Error: Unsupported channel = %d\n", channel));
  }

  return event;
}

KvlcStatus KvaLogWriter_Kme24::add_periodic_events(unsigned short lastTimeHi,
                                                       time_uint64 currentTime)
{
  KvlcStatus kvstatus = kvlcOK;
  kme24_logStruct KmeEvent;
  // Add periodic event(s) if we have had silence for a while

  if (lastTimeHi < timeHi) {
    int noOfOverflows, j;
    unsigned int year, month, day, hours, minute, second;

    noOfOverflows = timeHi-lastTimeHi;
    // Adds one TimeEvent every 0,65 second(OverflowEvents in kme25).
    // is it ok to submit a TIME event with the same time as the msg?
    for(j=1; j<=noOfOverflows; j++) {
      memset(&KmeEvent, 0, sizeof(KmeEvent));
      KmeEvent.type = KME24_LOG_TYPE_TIME;

      KmeEvent.payload.clock.hiResTimeStampHi = lastTimeHi + j;
      KmeEvent.payload.clock.hiResTimeStampLo = timeLo;

      KmeEvent.payload.clock.res = 0;
      KmeEvent.payload.clock.frameCounter = frameCounter;

      unix_time(((start_of_logging + (currentTime + (time_uint64)
        (1-noOfOverflows+j)*(time_uint64)655360000)) / ONE_BILLION),
          &year, &month, &day, &hours, &minute, &second);


      KmeEvent.payload.clock.date = (year-1980) <<9;
      KmeEvent.payload.clock.date |= month << 5;
      KmeEvent.payload.clock.date |= day;
      KmeEvent.payload.clock.time = hours << 11;
      KmeEvent.payload.clock.time |= minute << 5;
      KmeEvent.payload.clock.time |= second /2;
      KmeEvent.payload.clock.loResTimeStamp = 0;
      kvstatus = write_file(&KmeEvent, sizeof(KmeEvent));
      if (kvstatus != kvlcOK) {
        break;
      }
    }
  }
  return kvstatus;
}
#define NAME        "KME 2.4"
#define EXTENSION   "kme"
#define DESCRIPTION "Kvaser binary format (KME 2.4) - used for Vector CANalyzer"

class KvaWriterMaker_Kme24 : public KvaWriterMaker
{
  public:
    KvaWriterMaker_Kme24() : KvaWriterMaker(KVLC_FILE_FORMAT_KME24) {
      propertyList[KVLC_PROPERTY_START_OF_MEASUREMENT] = true;
      propertyList[KVLC_PROPERTY_FIRST_TRIGGER]        = true;
      propertyList[KVLC_PROPERTY_CHANNEL_MASK]         = true;
      propertyList[KVLC_PROPERTY_CROP_PRETRIGGER]      = true;
      propertyList[KVLC_PROPERTY_SIZE_LIMIT]           = true;
      propertyList[KVLC_PROPERTY_TIME_LIMIT]           = true;
      propertyList[KVLC_PROPERTY_OVERWRITE]            = true;
    }
    int getName(char *str) { return sprintf(str, "%s", NAME); }
    int getExtension(char *str) { return sprintf(str, "%s", EXTENSION); }
    int getDescription(char *str) { return sprintf(str, "%s", DESCRIPTION); }

  private:
    KvaLogWriter *OnCreateWriter() {
      return new KvaLogWriter_Kme24();
    }
}  registerKvaLogWriter_Kme24;
