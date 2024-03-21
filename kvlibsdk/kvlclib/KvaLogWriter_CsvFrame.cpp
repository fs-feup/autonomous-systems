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
**   Writer for CSV files.
** ---------------------------------------------------------------------------
*/

#include "common_defs.h"
#include "KvaLogWriter_CsvFrame.h"
#include "TimeConv.h"
#include "kvdebug.h"
#include "os_util.h"

KvaLogWriter_CsvFrame::KvaLogWriter_CsvFrame()
{
  PRINTF(("KvaLogWriter_CsvFrame::KvaLogWriter_CsvFrame()\n"));
  currentSize = 0;
  mMaxDataBytes = get_property_limit_databytes();
}

KvaLogWriter_CsvFrame::~KvaLogWriter_CsvFrame()
{
  PRINTF(("~KvaLogWriter_CsvFrame\n"));
}


KvlcStatus KvaLogWriter_CsvFrame::write_header()
{
  char outbuffer[2000];
  char *p = outbuffer;
  char timerow[80];
  time_t aclock;
  struct tm newtime;
  int i;

  outbuffer[0] = '\0';
  mMaxDataBytes = get_property_limit_databytes();

  if (!get_property_write_header()) {
    PRINTF(("write_header turned off by user option"));
    return kvlcOK;
  }

  if (!isOpened) {
    PRINTF(("KvaLogWriter_CsvFrame::write_header, file is not opened\n"));
    return kvlcERR_FILE_ERROR;
  }

  aclock = get_wallclock_time();
  get_calendar_time(aclock, &newtime);

  os_rewind(outfile);

  p += sprintf(p, "CSV CAN data\n");
  p += sprintf(p, "Converted from Kvaser Memorator at%c", get_property_separator_char());
  p += printCalendarDate(p, newtime);
  p += sprintf(p, "\n");

  if (overrun_occurred) {
    p += sprintf(p, CSV_OVERRUN_STRING"\n");
  } else {
    for (i = (int)strlen(CSV_OVERRUN_STRING); i > 0; i--) {
      p += sprintf(p, " ");
    } p += sprintf(p, "\n");
  }
  p += sprintf(p,
          "Format of data field%c%s\n",
          get_property_separator_char(),
          get_property_data_in_hex()?"HEX":"DEC");

  p += sprintf(p,
          "Format of %s field%c%s\n",
          get_property_hlp_j1939()?"PGN":"id",
          get_property_separator_char(),
          get_property_id_in_hex()?"HEX":"DEC");


  printDecimalNumber(timerow, get_property_offset(), get_property_time_decimals());

  p += sprintf(p, "Timestamp Offset%c%20s%cseconds\n",
          get_property_separator_char(),
          timerow,
          get_property_separator_char()
         );

  p += sprintf(p, "Logging started at%c", get_property_separator_char());
  p += printCalendarDate(p, start_of_logging);
  p += sprintf(p, "\n");

  p += sprintf(p, "Time%cChannel%c%s%cFlags%cDLC%c",
               get_property_separator_char(), get_property_separator_char(),
               get_property_hlp_j1939()?"PGN":"id",
               get_property_separator_char(), get_property_separator_char(), get_property_separator_char());

  for (unsigned int i = 0; i < mMaxDataBytes; i++) {
    p += sprintf(p, "Data%d%c", i, get_property_separator_char());
  }
  p += sprintf(p, "Counter");

  if (get_property_calendar_time_stamps()) {
    p += sprintf(p, "%cAbsTime", get_property_separator_char());
  }

  p += sprintf(p, "\n");
  currentSize += p - outbuffer;
  return write_file(outbuffer, p - outbuffer);
}

KvlcStatus KvaLogWriter_CsvFrame::write_row(imLogData *logEvent)
{
//Time;Channel;Id;Flags;DLC;Data0;Data1;
//Data2;Data3;Data4;Data5;Data6;Data7;Counter;AbsTime
  char outbuffer[1000];
  char *p = outbuffer;
  unsigned int i;

  KvlcStatus kvstatus = kvlcOK;

  // Only used for checking that time is increasing
  time_uint64 lastTime = 0;
  outbuffer[0] = '\0';
  if (!isOpened) {
    PRINTF(("KvaLogWriter_CsvFrame::write_row, file is not opened\n"));
    return kvlcERR_FILE_ERROR;
  }

  mMaxDataBytes = get_property_limit_databytes();

  if (!start_of_logging) setStartOfLogging(logEvent);

  switch (logEvent->common.type) {

    case ILOG_TYPE_MESSAGE:
    {
      if (!((1<<logEvent->msg.channel) & get_property_channel_mask())) {
        // dont use this channel so bail out
        return kvlcOK;
      }

      if (lastTime > logEvent->common.time64) {
        PRINTF(("WARNING, decreasing time\n"));
      }
      lastTime = logEvent->common.time64;

      time_int64 msgTime = ((time_int64) logEvent->common.time64 +
                           (time_int64) get_property_offset());
      bool isNegative = msgTime < 0;
      if (isNegative) msgTime = -msgTime;
      time_uint64 t1 = msgTime / ONE_BILLION;
      time_uint64 t2 = msgTime % ONE_BILLION;

      // time and channel

      p += printDecimalNumber(p, t1, t2, isNegative, get_property_time_decimals());

      p += sprintf(p, "%c%u%c",
                   get_property_separator_char(),
                   (unsigned int)logEvent->msg.channel+1,
                   get_property_separator_char());

      if (logEvent->msg.flags & canMSGERR_OVERRUN) {
        overrun_occurred = true;
      }

      // if error frame - don't show id, dlc and data..
      // Need to indicate Error Frame somehow..Now the only indication
      // for EF is that column flags will have value 34..
      // Perhaps write Error Frame in text in the ID column?
      if (logEvent->msg.flags & canMSG_ERROR_FRAME) {
        p += sprintf(p, "%c", get_property_separator_char()); // no id
        p += sprintf(p, // show flags
                    "%u%c",
                    (unsigned char)logEvent->msg.flags,
                    get_property_separator_char());
        p += sprintf(p, // no dlc
                    "%c",
                    get_property_separator_char());
        for (i = 0; i < mMaxDataBytes; i++) { // MemoPro had status code in Error frame's, this is ignored
          p += sprintf(p, // no data
                    "%c",
                    get_property_separator_char());
        }
      }
      else {
        unsigned int id = (unsigned int) logEvent->msg.id;

        // make sure the id has the right format
        if ((logEvent->msg.flags & canMSG_EXT) != 0) {
          id |= 0x80000000;
        }
        if (get_property_hlp_j1939()) {
          id = get_J1939_id(id);
        }

        p += sprintf(p,  // id
                    get_property_id_in_hex()?"%x%c":"%u%c",
                    id & KVACONV_ID_MASK,
                    get_property_separator_char());
        p += sprintf(p, // flags
                    "%u%c",
                    (unsigned int)logEvent->msg.flags,
                    get_property_separator_char());
        p += sprintf(p, // dlc
                    "%u%c",
                    (unsigned int)logEvent->msg.dlc & 0x0f,
                    get_property_separator_char());

        unsigned int num_bytes = logEvent->msg.dlc;
        if (num_bytes > 8) {
          num_bytes = 8;
        }
        if (logEvent->msg.flags & canFDMSG_FDF) {
          num_bytes = dlcToNumBytesFD(logEvent->msg.dlc);
        }

        if (num_bytes > mMaxDataBytes) {
          data_truncation_occurred = true;
        }
        for (i = 0; i < mMaxDataBytes; i++) {
          if (i < num_bytes) {
            p += sprintf(p,
                      get_property_data_in_hex()?"%02x%c":"%u%c",
                      (unsigned char)logEvent->msg.data[i],
                      get_property_separator_char());
          }
          else // Print only separator outside of message
          {
            p += sprintf(p,"%c", get_property_separator_char());
          }
        }
      }
      p += sprintf(p, "%u",
                   (unsigned int)logEvent->msg.frame_counter);

      if (get_property_calendar_time_stamps()) {
        p += sprintf(p, "%c", get_property_separator_char());
        p += printCalendarDate(p, logEvent->common.nanos_since_1970);
      }

      p += sprintf(p, "\n");

      kvstatus = write_file(outbuffer, p - outbuffer);
      break;
    }

    case ILOG_TYPE_RTC:
    {
      break;
    }

    case ILOG_TYPE_VERSION:
    {
      // Version is not implemented in this format.
      break;
    }

    case ILOG_TYPE_TRIGGER:
    {
      break;
    }

    default:
    {
      PRINTF(("logEvent->type = %d, frame_counter = %lu\n", logEvent->common.type, logEvent->common.event_counter));
      kvstatus = kvlcERR_INVALID_LOG_EVENT;
      break;
    }
  }
  logEvent->common.new_data = false;
  currentSize += p - outbuffer;
  return kvstatus;
}

#define NAME        "CSV Frame"
#define EXTENSION   "csv"
#define DESCRIPTION "CAN frames in CSV format"

class KvaWriterMaker_CsvFrame : public KvaWriterMaker
{
  public:
    KvaWriterMaker_CsvFrame() : KvaWriterMaker(KVLC_FILE_FORMAT_CSV) {
      propertyList[KVLC_PROPERTY_START_OF_MEASUREMENT] = true;
      propertyList[KVLC_PROPERTY_FIRST_TRIGGER] = true;
      propertyList[KVLC_PROPERTY_CHANNEL_MASK] = true;
      propertyList[KVLC_PROPERTY_HLP_J1939] = true;
      propertyList[KVLC_PROPERTY_CALENDAR_TIME_STAMPS] = true;
      propertyList[KVLC_PROPERTY_WRITE_HEADER] = true;
      propertyList[KVLC_PROPERTY_SEPARATOR_CHAR] = true;
      propertyList[KVLC_PROPERTY_DECIMAL_CHAR] = true;
      propertyList[KVLC_PROPERTY_ID_IN_HEX] = true;
      propertyList[KVLC_PROPERTY_DATA_IN_HEX] = true;
      propertyList[KVLC_PROPERTY_NUMBER_OF_TIME_DECIMALS ] = true;
      propertyList[KVLC_PROPERTY_ISO8601_DECIMALS] = true;
      propertyList[KVLC_PROPERTY_CROP_PRETRIGGER] = true;
      propertyList[KVLC_PROPERTY_SIZE_LIMIT] = true;
      propertyList[KVLC_PROPERTY_TIME_LIMIT] = true;
      propertyList[KVLC_PROPERTY_LIMIT_DATA_BYTES] = true;
      propertyList[KVLC_PROPERTY_OVERWRITE] = true;
      propertyList[KVLC_PROPERTY_TIMEZONE] = true;

      int defaultValue = 5;
      properties.set_property_and_default_value(KVLC_PROPERTY_NUMBER_OF_TIME_DECIMALS, &defaultValue, sizeof(defaultValue));
      defaultValue = 0;
      properties.set_property_and_default_value(KVLC_PROPERTY_ISO8601_DECIMALS, &defaultValue, sizeof(defaultValue));
    }
    int getName(char *str) { return sprintf(str, "%s", NAME); }
    int getExtension(char *str) { return sprintf(str, "%s", EXTENSION); }
    int getDescription(char *str) { return sprintf(str, "%s", DESCRIPTION); }

  private:
    KvaLogWriter *OnCreateWriter() {
      KvaLogWriter *writer = new KvaLogWriter_CsvFrame();
      writer->mProperties = properties;
      return writer;
    }
}  registerKvaLogWriter_CsvFrame;

