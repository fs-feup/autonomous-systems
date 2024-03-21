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
**  Reader for CSV frames. Used for tests only.
** ---------------------------------------------------------------------------
*/

#include "common_defs.h"
#include "TimeConv.h"
#include "kvdebug.h"
#include "kvaConverter.h"
#include "kvlclib.h"
#include "KvaLogReader_CsvFrame.h"
#include "KvaConverterMisc.h"
#include <string>
#include <sstream>
#include <algorithm>
#include "KvaProperties.h"

using namespace std;

static char buf[4096] = {0};
static vector<string> elements;
static char separator;

static void split_string_to_vector(vector<string>& items, char* str)
{
  char *p = strtok(str,&separator);
  items.clear();
  while(p != NULL)
  {
    items.push_back(p);
    p = strtok(NULL,&separator);
  }
}

static int toInt(const char *str)
{
  char *p = NULL;
  int tmp = strtol(str, &p, 0);
  if ((*p) != '\0') {
    //PRINTF(("Bytes left: '%s',", str));
  }
  return tmp;
}

static int toInt(string& str)
{
  size_t pos = str.find("=") + 1;
  return toInt(&str.c_str()[pos]);
}

KvlcStatus KvaLogReader_CsvFrame::interpret_event(
      char *line,
      imLogData *logEvent)
{
  double timestamp = 0;
  bool foundRTC = false;
  memset(logEvent, 0, sizeof(imLogData));
  logEvent->common.new_data = false;

  // Convert each line to a vector of strings
  if (!mUseSavedFrame) {
    split_string_to_vector(elements, line);
    if (!elements.size()) {
      PRINTF(("Empty string; skipping"));
      return kvlcOK;
    }
  }

  try {
    timestamp = stod(elements[0]);
  } catch (int err){
    (void) err;
    return kvlcOK;
  }

  switch (state) {
    case SEND_TRIGGER:
    {
      PRINTF(("ILOG_TYPE_TRIGGER"));
      memcpy(&mLogEvent, logEvent, sizeof(imLogData));
      mUseSavedFrame = true;
      logEvent->common.type = ILOG_TYPE_TRIGGER;
      logEvent->common.new_data = true;
      logEvent->trig.type = 1;
      logEvent->trig.trigNo = 0;
      logEvent->trig.preTrigger = 0;
      logEvent->trig.postTrigger = -1;
      logEvent->common.time64 = (time_uint64) (timestamp * ONE_BILLION);
      logEvent->common.new_data = true;
      logEvent->trig.active = true;
      state = SEND_CLOCK;
    }
    break;

    case SEND_CLOCK:
    {
      char sep, c;
      unsigned int year, month, day, hh, mm, ss;
      int64_t unix_seconds, fraction = 0;
      unsigned int cnt;
      size_t last = elements.size() - 1;

      // Date with fractions?
      cnt = sscanf(elements[last].c_str(),"%4u-%02u-%02uT%02u:%02u:%02u%c%04u%c",
                   &year, &month, &day, &hh, &mm, &ss, &sep, (int*)&fraction, &c);
      if (cnt == 8) {
        foundRTC = true;
      } else {
        // Date without fractions?
        cnt = sscanf(elements[last].c_str(),"%4u-%02u-%02u %02u:%02u:%02u%c",
        &year, &month, &day, &hh, &mm, &ss,&c);
        if (cnt == 6) {
          foundRTC = true;
        }
      }

      if (foundRTC) {
        PRINTF(("ILOG_TYPE_RTC"));
        PRINTF(("DateTime: %d-%02d-%02d %02d:%02d:%02d", year, month, day, hh, mm, ss));
        unix_seconds = convert_time(year, month, day, hh, mm, ss);
        logEvent->common.new_data = true;
        logEvent->common.type = ILOG_TYPE_RTC;
        logEvent->common.time64 = (time_uint64) (timestamp * ONE_BILLION);
        // From KvaLogWriter::printCalendarDate()
        logEvent->common.nanos_since_1970 = unix_seconds * ONE_BILLION +
        100000LL*fraction*ONE_BILLION;
        if (start_of_measurement64 == 0) {
          start_of_measurement64 = logEvent->common.nanos_since_1970;
          PRINTF(("Start of measurement read  = %lu", start_of_measurement64));
        }
      }
      state = SEND_FRAMES;
    }
    break;

    case SEND_FRAMES:
    {
      logEvent->msg.frame_counter = ++current_frameno;
      logEvent->common.new_data = true;
      logEvent->common.type = ILOG_TYPE_MESSAGE;
      logEvent->msg.channel = toInt(elements[1]) - 1;
      if (elements.size() < 6) {
        PRINTF(("ERROR FRAME"));
        logEvent->msg.flags = toInt(elements[2]);
      } else {
        PRINTF(("ILOG_TYPE_MESSAGE"));
        logEvent->msg.id = toInt(elements[2]);
        logEvent->msg.flags = toInt(elements[3]);

        logEvent->msg.dlc = toInt(elements[4]);
        for (unsigned int k = 5; k < elements.size() - 1; k++) {
          logEvent->msg.data[k-5] = toInt(elements[k]);
        }
      }
      logEvent->common.time64 = (time_uint64) (timestamp * ONE_BILLION);
      mUseSavedFrame = false;
    }
    break;
  }

  logEvent->common.event_counter = ++current_eventno;
  return kvlcOK;
}

KvlcStatus KvaLogReader_CsvFrame::read_row(imLogData *logEvent)
{
  KvlcStatus status;
  if (!isOpened) {
    PRINTF(("KvaLogReader_CsvFrame::read_row, file not open"));
    return kvlcERR_FILE_ERROR;
  }

  if (!mUseSavedFrame) {
    status = read_line(buf, sizeof(buf));
    if (kvlcOK != status) {
      return status;
    }
  }
  return interpret_event(buf, logEvent);
}

uint64 KvaLogReader_CsvFrame::event_count()
{
  // Counting lines would be more appropriate
  return (uint64) ((file_size-file_position) / 50);
}


KvaLogReader_CsvFrame::KvaLogReader_CsvFrame()
{
  PRINTF(("KvaLogReader_CsvFrame::KvaLogReader_CsvFrame()"));
  current_eventno = 0;
  current_frameno = 0;
  state = SEND_TRIGGER;
  mUseSavedFrame = false;

  // Use default writer properties; no properties for readers exist (yet).
  KvlcStatus stat = kvlcGetWriterPropertyDefault(KVLC_FILE_FORMAT_CSV,
                                      KVLC_PROPERTY_SEPARATOR_CHAR,
                                      &separator, sizeof(separator));
  if (stat != kvlcOK ) {
    separator = KvaProperties::DEFAULT_CSV_SEPARATOR_CHAR;
  }
}

KvaLogReader_CsvFrame::~KvaLogReader_CsvFrame() {
  PRINTF(("~KvaLogReader_CsvFrame")); 
}


class KvaReaderMaker_CsvFrame : public KvaReaderMaker
{
  public:
    KvaReaderMaker_CsvFrame() : KvaReaderMaker(KVLC_FILE_FORMAT_CSV) {}
    int getName(char *str) { return sprintf(str, "%s", "CSV Frame"); }
    int getExtension(char *str) { return sprintf(str, "%s", "csv"); }
    int getDescription(char *str) { return sprintf(str, "%s", "CAN frames in CSV format"); }

  private:
    KvaLogReader *OnCreateReader() {
      return new KvaLogReader_CsvFrame();
    }
}  registerKvaLogReader_CsvFrame;
