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
**  Reader for Plain ASCII. Used for tests only.
** ---------------------------------------------------------------------------
*/

#include "common_defs.h"
#include "TimeConv.h"
#include "kvdebug.h"
#include "kvaConverter.h"
#include "kvlclib.h"
#include "KvaLogReader_PlainAsc.h"
#include "KvaConverterMisc.h"
#include <string>
#include <sstream>
#include <algorithm>

using namespace std;

static char buf[4096] = {0};
static vector<string> elements;

static void strip_extra_spaces(char* str) {
  int i,x;
  for(i=x=1; str[i]; ++i)
    if(!isspace(str[i]) || (i>0 && !isspace(str[i-1])))
      str[x++] = str[i];
  str[x] = '\0';
}

static void split_string_to_vector(vector<string>& items, char* str)
{
  char *p = strtok(str," ");
  items.clear();
  while(p != NULL)
  {
    items.push_back(p);
    p = strtok(NULL," ");
  }
}

static int toInt(const char *str, int radix = 0)
{
  char *p = NULL;
  int tmp = strtol(str, &p, radix);
  if ((*p) != '\0') {
    //PRINTF(("Bytes left: '%s',", str));
  }
  return tmp;
}

static int toInt(string& str, int radix = 0)
{
  size_t pos = str.find("=") + 1;
  return toInt(&str.c_str()[pos], radix);
}

static int isExtendedId(string& str)
{
  unsigned int flag = 0;
  if (str.find("x") != string::npos) {
    flag = canMSG_EXT;
  }
  return flag;
}

static int isErrorFrame(string& str)
{
  unsigned int flag = 0;
  if (str.find("ERROR") != string::npos) {
    flag = canMSG_ERROR_FRAME;
  }
  return flag;
}

static unsigned int getFlags(string& str)
{
  unsigned int flags = 0;

  if (str.find("ERROR") != string::npos)
    return 0;

  if (str.find("TxRq") != string::npos) {
    flags |= canMSG_TXRQ;
  }
  else if (str.find("Tx") != string::npos) {
    flags |= canMSG_TXACK;
  }
  else if (str.find("Rx") != string::npos) {
    // Rx is deafult
  }
  else if (str.find("R") != string::npos) {
    flags |= canMSG_RTR;
  }

  if (str.find("E") != string::npos) {
    flags |= canMSG_NERR;
  }
  if (str.find("F") != string::npos) {
    flags |= canFDMSG_FDF;
  }
  if (str.find("B") != string::npos) {
    flags |= canFDMSG_BRS;
  }
  if (str.find("P") != string::npos) {
    flags |= canFDMSG_ESI;
  }
  if (str.find("W") != string::npos) {
    flags |= canMSG_WAKEUP;
  }
  if (str.find("#") != string::npos) {
    flags |= canMSGERR_OVERRUN;
  }
  return flags;
}

KvlcStatus KvaLogReader_PlainAsc::interpret_event(
      char *line,
      imLogData *logEvent)
{
  double timestamp = 0;
  memset(logEvent, 0, sizeof(imLogData));
  logEvent->common.new_data = false;

  // Convert each line to a vector of strings
  strip_extra_spaces(line);
  split_string_to_vector(elements, line);

  size_t size = elements.size();

  if (size == 3 && elements[0].find("Kvaser") != string::npos
      && elements[1].find("Memorator") != string::npos
      && elements[2].find("Log") != string::npos) {
    maybe_header = true;
  }

  if (maybe_header && size >= 5
      && elements[0].find("Format") != string::npos) {
    int radix;
       
    if (elements[4].find("DEC") != string::npos) {
      radix = 10;
    } else if (elements[4].find("HEX") != string::npos) {
      radix = 16;
    } else {
      return kvlcERR_INVALID_LOG_EVENT;
    }

    if (elements[2].find("data") != string::npos) {
      data_radix = radix;
    } else if (elements[2].find("id") != string::npos) {
      id_radix = radix;
    } else {
      return kvlcERR_INVALID_LOG_EVENT;
    }
    return kvlcOK;
  }

  if (size < 3) {
    PRINTF(("Empty or short line: '%s'", line));
    return kvlcOK;
  }

  if (elements[0].find("DateTime") != string::npos) {
    PRINTF(("ILOG_TYPE_RTC"));
    char c;
    unsigned int year, month, day, hh,mm,ss;
    uint64_t unix_seconds;

    sscanf(elements[1].c_str(),"%d-%d-%d%c",&year, &month, &day, &c);
    sscanf(elements[2].c_str(),"%d:%d:%d%c",&hh, &mm, &ss, &c);
    PRINTF(("DateTime: %d-%02d-%02d %02d:%02d:%02d", year, month, day, hh, mm, ss));
    unix_seconds = convert_time(year, month, day, hh, mm, ss);
    logEvent->common.new_data = true;
    logEvent->common.type = ILOG_TYPE_RTC;
    logEvent->common.nanos_since_1970 = unix_seconds * ONE_BILLION;
    if (start_of_measurement64 == 0) {
      start_of_measurement64 = logEvent->common.nanos_since_1970;
      PRINTF(("Start of measurement read  = %lu", start_of_measurement64));
    }
    logEvent->common.event_counter = ++current_eventno;
    return kvlcOK;
  }

  try {
    timestamp = stod(elements[0]);
    not_header();
  } catch (const std::exception &e){
    (void) e;
    if (maybe_header)
      return kvlcOK;
    else
      return kvlcERR_INVALID_LOG_EVENT;
  }

  if (elements[1].find("Trigger") != string::npos) {
    PRINTF(("ILOG_TYPE_TRIGGER"));
    logEvent->trig.type = toInt(elements[2]);
    logEvent->trig.trigNo = toInt(elements[3]);
    logEvent->trig.preTrigger = toInt(elements[4]);
    logEvent->trig.postTrigger = toInt(elements[5]);
    logEvent->common.time64 = (time_uint64) (timestamp * ONE_BILLION);
    logEvent->common.new_data = true;
    logEvent->common.type = ILOG_TYPE_TRIGGER;
    logEvent->trig.active = true;
    not_header();
  }
  else {
    PRINTF(("ILOG_TYPE_MESSAGE"));
    unsigned int ind = 1;
    logEvent->msg.frame_counter = ++current_frameno;
    logEvent->common.new_data = true;
    logEvent->common.type = ILOG_TYPE_MESSAGE;
    logEvent->msg.channel = toInt(elements[ind++]) - 1;
    logEvent->msg.id = toInt(elements[ind], id_radix);
    logEvent->msg.flags = isExtendedId(elements[ind++]);

    if (elements[2].find("ErrorFrame") != string::npos)
      logEvent->msg.flags |= canMSG_ERROR_FRAME;

    logEvent->msg.flags |= getFlags(elements[ind++]);
    if (getFlags(elements[ind])) {
      logEvent->msg.flags |= getFlags(elements[ind++]);
    }
    logEvent->msg.flags |= isErrorFrame(elements[ind]);
    logEvent->msg.dlc = toInt(elements[ind++]);
    for (unsigned int k = ind; k < elements.size() - 1; k++) {
      logEvent->msg.data[k-ind] = toInt(elements[k], data_radix);
    }
    logEvent->common.time64 = (time_uint64) (timestamp * ONE_BILLION);
    not_header();
  }
  logEvent->common.event_counter = ++current_eventno;
  return kvlcOK;
}

KvlcStatus KvaLogReader_PlainAsc::read_row(imLogData *logEvent)
{
  KvlcStatus status;
  if (!isOpened) {
    PRINTF(("KvaLogReader_PlainAsc::read_row, file not open"));
    return kvlcERR_FILE_ERROR;
  }
  status = read_line(buf, sizeof(buf));
  if (kvlcOK != status) {
    return status;
  }

  if (strncmp(buf, "====================", 20) == 0) {
    not_header();
  }

  return interpret_event(buf, logEvent);
}

uint64 KvaLogReader_PlainAsc::event_count()
{
  // Counting lines would be more appropriate
  return (uint64) ((file_size-file_position) / 50);
}

void KvaLogReader_PlainAsc::not_header()
{
    maybe_header = false;
}

KvaLogReader_PlainAsc::KvaLogReader_PlainAsc()
{
  PRINTF(("KvaLogReader_PlainAsc::KvaLogReader_PlainAsc()"));
  current_eventno = 0;
  current_frameno = 0;
  current_line = 0;
  id_radix = 0;
  data_radix = 0;
  maybe_header = true;
}

class KvaReaderMaker_PlainAsc : public KvaReaderMaker
{
  public:
    KvaReaderMaker_PlainAsc() : KvaReaderMaker(KVLC_FILE_FORMAT_PLAIN_ASC) {}
    int getName(char *str) { return sprintf(str, "%s", "Plain text"); }
    int getExtension(char *str) { return sprintf(str, "%s", "txt"); }
    int getDescription(char *str) { return sprintf(str, "%s", "CAN frames in plain text format"); }

  private:
    KvaLogReader *OnCreateReader() {
      return new KvaLogReader_PlainAsc();
    }
}  registerKvaLogReader_PlainAsc;
