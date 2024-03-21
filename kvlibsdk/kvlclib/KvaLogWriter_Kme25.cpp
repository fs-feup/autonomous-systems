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
#include "KvaLogWriter_Kme25.h"
#include "loggerDefinitions.h"
#include "KvaLogWriter.h"
#include "TimeConv.h"
#include "kvdebug.h"

KvaLogWriter_Kme25::KvaLogWriter_Kme25()
{
  PRINTF(("KvaLogWriter_Kme25::KvaLogWriter_Kme25()\n"));
  timeLo = 0;
  timeHi = 0;
  memset(&eOld, 0, sizeof(eOld));
}

KvlcStatus KvaLogWriter_Kme25::write_header()
{
  timeLo = 0;
  timeHi = 0;
  memset(&eOld, 0, sizeof(eOld));
  return kvlcOK;
}

void createLogTypeTime(imLogData *logEvent, logStruct *e)
{
  unsigned int year;
  unsigned int month;
  unsigned int day;
  unsigned int hour;
  unsigned int minute;
  unsigned int second;

  memset(e, 0, sizeof(logStruct));
  e->payload.rtc.verMajor       = LOG_FILE_MAJOR_VERSION;
  e->payload.rtc.verMinor       = LOG_FILE_MINOR_VERSION;

  unix_time(logEvent->common.nanos_since_1970 / ONE_BILLION,
         &year,
         &month,
         &day,
         &hour,
         &minute,
         &second);

  e->payload.rtc.date = (year-1980) << 9;
  e->payload.rtc.date |= month << 5;
  e->payload.rtc.date |= day;
  e->payload.rtc.time = hour << 11;
  e->payload.rtc.time |= minute << 5;
  e->payload.rtc.time |= second /2;

  // Only ch1 here, since it's only possible to define transceiver on ch1
  e->payload.rtc.canDriverType1 = logEvent->common.transceiver_type[1];
  e->type                       = LOG_TYPE_TIME;
}

KvlcStatus KvaLogWriter_Kme25::write_row(imLogData *logEvent)
{
  KvlcStatus kvstatus = kvlcOK;

  if (!isOpened) {
    PRINTF(("KvaLogWriter_Kme25::write_row, file was not opened\n"));
    return kvlcERR_FILE_ERROR;
  }

  if (!start_of_logging) {
    if (setStartOfLogging(logEvent)) {
      // Create a RTC first in the file
      logStruct e;
      createLogTypeTime(logEvent, &e);
      kvstatus = write_file(&e, sizeof(e));
      if (kvstatus != kvlcOK) {
        logEvent->common.new_data = false;
        return kvstatus;
      }
    }
  }

  switch (logEvent->common.type) {

    case ILOG_TYPE_MESSAGE:
    {
      unsigned short lastTimeHi;
      logStruct e;

      if (!((1<<logEvent->msg.channel) & get_property_channel_mask())) {
        // Don't use this channel so bail out
        return kvlcOK;
      }

      memset(&e, 0, sizeof(e));
      lastTimeHi = timeHi;
      timeLo = (unsigned short)(logEvent->common.time64 / 10000);
      timeHi = (unsigned short)((logEvent->common.time64 / 10000) / 65536);

      // add periodic event if we have had silence for a while
      kvstatus = add_periodic_events(lastTimeHi);
      if (kvstatus != kvlcOK) {
        break;
      }

      memset(&e, 0, sizeof(e));
      e.type = channel_to_event(logEvent->msg.channel);

      //id
      e.payload.frame.id = logEvent->msg.id  & 0x1fffffff;
      if (logEvent->msg.flags & canMSG_EXT) {
        e.payload.frame.id |= LOG_MSG_EXT_FLAG;
      }
      // time
      timeLo = (unsigned short)(logEvent->common.time64 / 10000);
      e.payload.frame.time = timeLo;

      //dlc
      e.payload.frame.dlc = 0x0f & logEvent->msg.dlc;

      //data
      memcpy(e.payload.frame.data, logEvent->msg.data, 8);

      // nerr flag
      if (logEvent->msg.flags & canMSG_NERR) {
        e.payload.frame.id |= LOG_MSG_NERR_FLAG;
      }
      else if (logEvent->msg.flags & canMSG_WAKEUP) {
        // transtype = TRANSCEIVER_TYPE_SWC
        e.payload.frame.id |= LOG_MSG_NERR_FLAG;
      }
      //more flags
      if (logEvent->msg.flags & canMSG_RTR) {
        e.payload.frame.dlc |= LOG_MSG_RTR_FLAG;
      }
      if (logEvent->msg.flags & canMSG_ERROR_FRAME) {
        e.payload.frame.dlc |= LOG_MSG_ERROR_FLAG;
      }
      if (logEvent->msg.flags & canMSGERR_OVERRUN) {
        e.payload.frame.dlc |= LOG_MSG_OVR_FLAG;
        overrun_occurred = true;
      }
      if (logEvent->msg.flags & canFDMSG_FDF) {
        // There are no flag bits left for CANFD flags in KME25
        if (dlcToNumBytesFD(logEvent->msg.dlc) > 8) {
          data_truncation_occurred = true;
        }
      }

      kvstatus = write_file(&e, sizeof(e));
      break;
    }

    case ILOG_TYPE_TRIGGER:
    {
      logStruct e;
      unsigned short lastTimeHi = timeHi;
      timeLo = (unsigned short)(logEvent->common.time64 / 10000);
      timeHi = (unsigned short)((logEvent->common.time64 / 10000)/ 65536);

      // add periodic event if we have has silence for a while
      kvstatus = add_periodic_events(lastTimeHi);
      if (kvstatus != kvlcOK) {
        break;
      }

      memset(&e, 0, sizeof(e));
      e.type = LOG_TYPE_TRIGGER;
      e.payload.trig.postTrigger = logEvent->trig.postTrigger;
      e.payload.trig.preTrigger  = logEvent->trig.preTrigger;
      e.payload.trig.type        = logEvent->trig.type;
      e.payload.trig.trigNo      = logEvent->trig.trigNo;
      e.payload.trig.time        = timeLo;
      kvstatus = write_file(&e, sizeof(e));
      break;
    }

    //KvaConverter.cpp is responsible for that the first event that comes is a ILOG_TYPE_RTC
    case ILOG_TYPE_RTC:
    {
      logStruct e;

      createLogTypeTime(logEvent, &e);
      // Is it the first RTC message after the 2 second firmware update?
      if (eOld.payload.rtc.time != e.payload.rtc.time ||
          eOld.payload.rtc.date != e.payload.rtc.date)
      {
        memcpy(&eOld, &e, sizeof(logStruct));
        // Is it time to create a new file?
        if (mSplitPending) {
          open_new_file();
          kvstatus = write_file(&e, sizeof(e));

          // Clock overflow
          memset(&e, 0, sizeof(e));
          e.type = LOG_TYPE_CLOCK_OVERFLOW;
          e.payload.clockOverflow.currentTime = (((unsigned long)(timeHi + 1)) << 16);
          kvstatus = write_file(&e, sizeof(e));

          // Trigger
          memset(&e, 0, sizeof(e));
          e.type = LOG_TYPE_TRIGGER;
          e.payload.trig.postTrigger = 0;
          e.payload.trig.preTrigger  = 0;
          e.payload.trig.type        = 6; //TRIGVAR_TYPE_DISK_FULL
          e.payload.trig.trigNo      = 0;
          e.payload.trig.time        = 0; //timeLo;
          kvstatus = write_file(&e, sizeof(e));

          // Second RTC message
          kvstatus = write_file(&eOld, sizeof(eOld));
        }
      }
      kvstatus = write_file(&e, sizeof(e));
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

unsigned char KvaLogWriter_Kme25::channel_to_event(unsigned char channel)
{
  int event = LOG_TYPE_CAN_CH_0;

  switch (channel) {
  case 0:
    event = LOG_TYPE_CAN_CH_0;
    break;
  case 1:
    event = LOG_TYPE_CAN_CH_1;
    break;
  case 2:
    event = LOG_TYPE_CAN_CH_2;
    break;
  case 3:
    event = LOG_TYPE_CAN_CH_3;
    break;
  case 4:
    event = LOG_TYPE_CAN_CH_4;
    break;
  default:
    PRINTF(("Error: Unsupported channel = %d\n", channel));
  }

  return event;
}

KvlcStatus KvaLogWriter_Kme25::add_periodic_events(unsigned short lastTimeHi)
{
  KvlcStatus kvstatus = kvlcOK;
  // Add periodic event(s) if we have had silence for a while
  if (lastTimeHi < timeHi) {
    int noOfOverflows, j;
    logStruct e;

    noOfOverflows = timeHi-lastTimeHi;
    // Adds one TimeEvent per clock turnover
    for (j=0; j<noOfOverflows; j++) {
      memset(&e, 0, sizeof(e));
      e.type = LOG_TYPE_CLOCK_PERIODIC;

      // this will giva a new event every 0xffff ticks,
      // we could add the with approx. 1 sec in between
      // but i see no reason
      e.payload.clockDummy.timeHi = lastTimeHi + j;
      e.payload.clockDummy.timeLo = timeLo;
      kvstatus = write_file(&e, sizeof(e));
      if (kvstatus != kvlcOK) {
        break;
      }
    }

    // clock overflow
    memset(&e, 0, sizeof(e));
    e.type = LOG_TYPE_CLOCK_OVERFLOW;
    e.payload.clockOverflow.currentTime = ((unsigned long) timeHi)<<16;
    kvstatus = write_file(&e, sizeof(e));
  }
  return kvstatus;
}


#define NAME        "KME 2.5"
#define EXTENSION   "kme25"
#define DESCRIPTION "Kvaser binary format (KME 2.5)"

class KvaWriterMaker_Kme25: public KvaWriterMaker
{
  public:
    KvaWriterMaker_Kme25() : KvaWriterMaker(KVLC_FILE_FORMAT_KME25) {
      propertyList[KVLC_PROPERTY_START_OF_MEASUREMENT] = true;
      propertyList[KVLC_PROPERTY_FIRST_TRIGGER] = true;
      propertyList[KVLC_PROPERTY_CHANNEL_MASK] = true;
      propertyList[KVLC_PROPERTY_CROP_PRETRIGGER] = true;
      propertyList[KVLC_PROPERTY_SIZE_LIMIT] = true;
      propertyList[KVLC_PROPERTY_TIME_LIMIT] = true;
      propertyList[KVLC_PROPERTY_OVERWRITE] = true;
    }
    int getName(char *str) { return sprintf(str, "%s", NAME); }
    int getExtension(char *str) { return sprintf(str, "%s", EXTENSION); }
    int getDescription(char *str) { return sprintf(str, "%s", DESCRIPTION); }

  private:
    KvaLogWriter *OnCreateWriter() {
      return new KvaLogWriter_Kme25();
    }
}  registerKvaLogWriter_Kme25;
