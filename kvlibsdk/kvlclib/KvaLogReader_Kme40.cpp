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
#include "kvaConverter.h"
#include "KvaLogReader_Kme40.h"
#include "TimeConv.h"
#include "kvdebug.h"

KvlcStatus KvaLogReader_Kme40::interpret_event(
      filoResponse *kmeEvent,
      imLogData *logEvent
    )
{
  // just channel 1 can be != HS
  uint64_t x;
  unsigned int  freq = 24; // default value


  logEvent->common.transceiver_type[0] = transceiver_type0;
  logEvent->common.transceiver_type[1] = transceiver_type1;

  logEvent->common.event_counter = ++current_eventno;
  switch (kmeEvent->head.cmdNo) {

    case CMD_LOG_MESSAGE:
    {
      logEvent->msg.frame_counter = ++current_frameno;
      logEvent->common.new_data = true;
      logEvent->common.type = ILOG_TYPE_MESSAGE;
      logEvent->msg.flags = 0;

      logEvent->msg.id =
          kmeEvent->logMessage.id & 0x1fffffff; // max 29 bit
      if (kmeEvent->logMessage.id & 0x80000000) {
        logEvent->msg.flags |= canMSG_EXT;
      }
      else {
        logEvent->msg.flags |= canMSG_STD;
      }

      logEvent->msg.dlc = kmeEvent->logMessage.dlc & 0x0f;
      memcpy(logEvent->msg.data, kmeEvent->logMessage.data, 8);
      logEvent->msg.channel = kmeEvent->logMessage.channel;

      x  = kmeEvent->logMessage.time[0] + (((uint64_t)kmeEvent->logMessage.time[1]) << 16);
      x += ((uint64_t) kmeEvent->logMessage.time[2]) << 32;
      logEvent->common.time64 = x * 1000 / freq; // in nano secods

      if (kmeEvent->logMessage.flags & MSGFLAG_OVERRUN) {
        logEvent->msg.flags |= canMSGERR_HW_OVERRUN;
        logEvent->msg.flags |= canMSGERR_SW_OVERRUN;
      }
      if (kmeEvent->logMessage.flags & MSGFLAG_NERR) {
          logEvent->msg.flags |= canMSG_NERR;
      }
      if (kmeEvent->logMessage.flags & MSGFLAG_WAKEUP) {
          logEvent->msg.flags |= canMSG_WAKEUP;
      }
      if (kmeEvent->logMessage.flags & MSGFLAG_REMOTE_FRAME) {
        logEvent->msg.flags |= canMSG_RTR;
      }
      if (kmeEvent->logMessage.flags & MSGFLAG_ERROR_FRAME) {
        logEvent->msg.flags |= canMSG_ERROR_FRAME;
      }
      if (kmeEvent->logMessage.flags & MSGFLAG_TX) {
        logEvent->msg.flags |= canMSG_TXACK;
      }
      // logEvent.event.msg.time64_ack =
      break;
    }

    case CMD_LOG_RTC_TIME:
    {
      logEvent->common.new_data = true;
      logEvent->common.type = ILOG_TYPE_RTC;
      transceiver_type0 = kmeEvent->logRtcTime.canDriverType0;
      transceiver_type1 = kmeEvent->logRtcTime.canDriverType1;

      // we have no hires rtc-time in kme40

      logEvent->common.nanos_since_1970 = ONE_BILLION * convert_time(
        1980+(kmeEvent->logRtcTime.date >> 9),
        (kmeEvent->logRtcTime.date >> 5) & 0x0F,
        kmeEvent->logRtcTime.date & 0x1F,
        kmeEvent->logRtcTime.time >> 11,
        (kmeEvent->logRtcTime.time >> 5) & 0x3F,
        (kmeEvent->logRtcTime.time & 0x1F) * 2);

      if (start_of_measurement64 == 0) {
        start_of_measurement64 = logEvent->common.nanos_since_1970;
        PRINTF(("Start of measurement read  = %lu", start_of_measurement64));
      }
      logEvent->common.time64 = 0;
      break;
    }

    case CMD_LOG_TRIG:
    {
      logEvent->common.new_data = true;
      logEvent->common.type = ILOG_TYPE_TRIGGER;
      logEvent->trig.active = true;
      logEvent->trig.postTrigger = kmeEvent->logTrig.postTrigger;
      logEvent->trig.preTrigger = kmeEvent->logTrig.preTrigger;

      x  = kmeEvent->logTrig.time[0] + (((uint64_t)kmeEvent->logTrig.time[1]) << 16);
      x += ((uint64_t) kmeEvent->logTrig.time[2]) << 32;
      logEvent->common.time64 = x * 1000 / freq; // in nano secods

      logEvent->trig.type = kmeEvent->logTrig.type;
      logEvent->trig.trigNo = (unsigned char)kmeEvent->logTrig.trigNo;
      break;
    }

    default:
    {
      PRINTF(("Unknown type of kme event %d", kmeEvent->head.cmdNo));
      return kvlcERR_INVALID_LOG_EVENT;
      break;
    }
  }
  return kvlcOK;
}

KvlcStatus KvaLogReader_Kme40::read_row(imLogData *logEvent)
{
  KvlcStatus status;
  if (!isOpened) {
    PRINTF(("KvaLogReader_Kme40::read_row, file not open"));
    return kvlcERR_FILE_ERROR;
  }

  status = read_file((char *)&current_kmeEvent, sizeof(filoResponse));
  if (kvlcOK != status) {
    return status;
  }

  return interpret_event(&current_kmeEvent, logEvent);
}

uint64 KvaLogReader_Kme40::event_count()
{
  return (uint64) ((file_size-file_position) / sizeof(filoResponse));
}


KvaLogReader_Kme40::KvaLogReader_Kme40()
{
  PRINTF(("KvaLogReader_Kme40::KvaLogReader_Kme40()"));
  current_eventno = 0;
  current_frameno = 0;
  transceiver_type0 = 0;
  transceiver_type1 = 0;
}

KvaLogReader_Kme40::~KvaLogReader_Kme40()
{ 
  PRINTF(("~KvaLogReader_Kme40"));
}

class KvaReaderMaker_Kme40 : public KvaReaderMaker
{
  public:
    KvaReaderMaker_Kme40() : KvaReaderMaker(KVLC_FILE_FORMAT_KME40) {}
    int getName(char *str) { return sprintf(str, "%s", "KME 4.0"); }
    int getExtension(char *str) { return sprintf(str, "%s", "kme40"); }
    int getDescription(char *str) { return sprintf(str, "%s", "Kvaser binary format (KME 4.0)"); }

  private:
    KvaLogReader *OnCreateReader() {
      return new KvaLogReader_Kme40();
    }
}  registerKvaLogReader_Kme40;
