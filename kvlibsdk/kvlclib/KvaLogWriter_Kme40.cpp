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

#include "kvdebug.h"
#include "TimeConv.h"
#include "common_defs.h"
#include "KvaLogWriter_Kme40.h"
#include "filo_cmds.h"
#include "KvaLogWriter.h"

KvaLogWriter_Kme40::KvaLogWriter_Kme40()
{
  PRINTF(("KvaLogWriter_Kme40::KvaLogWriter_Kme40()\n"));
  memset(&L_old, 0, sizeof(L_old));
  mSplitPending = false;
}

KvlcStatus KvaLogWriter_Kme40::write_header()
{
  return kvlcOK;
}

KvlcStatus KvaLogWriter_Kme40::split_file()
{
  mSplitPending = true;
  return kvlcOK;
}

void createLogRtcTime(imLogData *logEvent, filoResponse *L) {
  unsigned int year;
  unsigned int month;
  unsigned int day;
  unsigned int hour;
  unsigned int minute;
  unsigned int second;

  // set everything to zero
  memset(L, 0, sizeof(filoResponse));

  L->logRtcTime.cmdNo = CMD_LOG_RTC_TIME;
  L->logRtcTime.cmdLen = sizeof(cmdLogRtcTime);

  unix_time(logEvent->common.nanos_since_1970 / ONE_BILLION,
         &year,
         &month,
         &day,
         &hour,
         &minute,
         &second);

  L->logRtcTime.verMajor       = 4;
  L->logRtcTime.verMinor       = 0;
  L->logRtcTime.date           =  (year-1980) << 9;
  L->logRtcTime.date           |= month << 5;
  L->logRtcTime.date           |= day;
  L->logRtcTime.time           =  hour << 11;
  L->logRtcTime.time           |= minute << 5;
  L->logRtcTime.time           |= second/2;
  L->logRtcTime.canDriverType0 = logEvent->common.transceiver_type[0];
  L->logRtcTime.canDriverType1 = logEvent->common.transceiver_type[1];
  L->logRtcTime.hardwareType   = 54;

}

KvlcStatus KvaLogWriter_Kme40::write_row(imLogData *logEvent)
{
  KvlcStatus kvstatus = kvlcOK;
  filoResponse L;

  if (!isOpened) {
    PRINTF(("KvaLogWriter_Kme40::write_row, file was not opened\n"));
    return kvlcERR_FILE_ERROR;
  }

  if (!start_of_logging) {
    if (setStartOfLogging(logEvent)) {
      // Create a RTC first in the file
      createLogRtcTime(logEvent, &L);
      kvstatus = write_file(&L, sizeof(L));
      if (kvstatus != kvlcOK) {
        logEvent->common.new_data = false;
        return kvstatus;
      }
    }
  }

  switch (logEvent->common.type) {

    case ILOG_TYPE_MESSAGE:
    {
      uint64_t time;

      if (!((1<<logEvent->msg.channel) & get_property_channel_mask())) {
        // Don't use this channel so bail out
        return kvlcOK;
      }

      // set everything to zero
      memset(&L, 0, sizeof(L));

      L.logMessage.cmdNo = CMD_LOG_MESSAGE;
      L.logMessage.cmdLen = sizeof(cmdLogMessage);

      time = logEvent->common.time64; // in nano seconds
      // below, 24 is standard Freq for Memorator Pro (
      time = time * 24 / 1000;
      L.logMessage.time[0] = time         & 0xffff;
      L.logMessage.time[1] = (time >> 16) & 0xffff;;
      L.logMessage.time[2] = (time >> 32) & 0xffff;
      L.logMessage.timeOffset = 0;

      L.logMessage.channel = logEvent->msg.channel;

      //id
      L.logMessage.id = logEvent->msg.id  & 0x1fffffff;
      if (logEvent->msg.flags & canMSG_EXT) {
        L.logMessage.id |= 0x80000000;
      }

      //dlc
      L.logMessage.dlc = 0x0f & logEvent->msg.dlc;

      //data
      memcpy(L.logMessage.data, logEvent->msg.data, 8);

      // nerr flag
      if (logEvent->msg.flags & canMSG_NERR) {
        L.logMessage.flags |= MSGFLAG_NERR;
      }
      else if (logEvent->msg.flags & canMSG_WAKEUP) {
        // transtype = TRANSCEIVER_TYPE_SWC
        L.logMessage.flags |= MSGFLAG_WAKEUP;
      }
      //more flags
      if (logEvent->msg.flags & canMSG_RTR) {
        L.logMessage.flags |= MSGFLAG_REMOTE_FRAME;
      }
      if (logEvent->msg.flags & canMSG_ERROR_FRAME) {
        L.logMessage.flags |= MSGFLAG_ERROR_FRAME;
      }
      if (logEvent->msg.flags & canMSGERR_OVERRUN) {
        L.logMessage.flags |= MSGFLAG_OVERRUN;
        overrun_occurred = true;
      }
      if(logEvent->msg.flags & canMSG_TXACK) {
        L.logMessage.flags |= MSGFLAG_TX;
      }
      if (logEvent->msg.flags & canFDMSG_FDF) {
        // There are no flag bits left for CANFD flags in KME40
        if (dlcToNumBytesFD(logEvent->msg.dlc) > 8) {
          data_truncation_occurred = true;
        }
      }

      kvstatus = write_file(&L, sizeof(L));
      break;
    }

    case ILOG_TYPE_TRIGGER:
    {
      uint64_t time;

      // set everything to zero
      memset(&L, 0, sizeof(L));

      L.logTrig.cmdNo = CMD_LOG_TRIG;
      L.logTrig.cmdLen = sizeof(cmdLogTrig);

      time = logEvent->common.time64; // in nano seconds
      // below, 24 is standard Freq for Memorator Pro (
      time = time  * 24 / 1000;
      L.logTrig.time[0] = time         & 0xffff;
      L.logTrig.time[1] = (time >> 16) & 0xffff;
      L.logTrig.time[2] = (time >> 32) & 0xffff;

      L.logTrig.postTrigger = logEvent->trig.postTrigger;
      L.logTrig.preTrigger  = logEvent->trig.preTrigger;
      L.logTrig.type        = logEvent->trig.type;
      L.logTrig.trigNo      = logEvent->trig.trigNo;

      kvstatus = write_file(&L, sizeof(L));
      break;
    }

    //KvaConverter.cpp is responsible for that the first event that comes is a ILOG_TYPE_RTC
    case ILOG_TYPE_RTC:
    {
      createLogRtcTime(logEvent, &L);

      // Is it the first RTC message after the 2 second firmware update?
      if (L_old.logRtcTime.time != L.logRtcTime.time ||
          L_old.logRtcTime.date != L.logRtcTime.date)
      {
        memcpy(&L_old, &L, sizeof(L_old));

        // Is it time to create a new file?
        if (mSplitPending) {
          kvstatus = open_new_file();
          if (kvstatus != kvlcOK) return kvstatus;
        }
      }

      kvstatus = write_file(&L, sizeof(L));
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

#define NAME        "KME 4.0"
#define EXTENSION   "kme40"
#define DESCRIPTION "Kvaser binary format (KME 4.0)"

class KvaWriterMaker_Kme40 : public KvaWriterMaker
{
  public:
    KvaWriterMaker_Kme40() : KvaWriterMaker(KVLC_FILE_FORMAT_KME40) {
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
      return new KvaLogWriter_Kme40();
    }
}  registerKvaLogWriter_Kme40;

