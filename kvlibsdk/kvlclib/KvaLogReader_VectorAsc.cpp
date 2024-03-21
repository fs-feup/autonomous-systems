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
**  Reader for Vector ASCII. Used for tests only.
** ---------------------------------------------------------------------------
*/

#include "common_defs.h"
#include "TimeConv.h"
#include "kvdebug.h"
#include "kvaConverter.h"
#include "kvlclib.h"
#include "KvaLogReader_VectorAsc.h"
#include "KvaConverterMisc.h"
#include <string>
#include <sstream>
#include <algorithm>

using namespace std;

static const int format_classic = 0;
static const int format_canfd   = 1;

static char buf[4096] = {0};
static vector<string> elements;

//remove leading and trailing whitespace
//replace multiple whitespace with a single one
static void strip_extra_spaces(char* str) {
  int i,n,x;
  for (i = n = x = 0; str[i]; ++i) {
    if (isspace(str[i])) {
      n++;
    } else {
      if (n > 0) {
        if (x > 0) {
          str[x++] = ' ';
        }
        n = 0;
      }
      str[x++] = str[i];
    }
  }
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

static unsigned int getFlags(string& str)
{
  if (str.find("TxRq") != string::npos) {
    return canMSG_TXRQ;
  }
  else if (str.find("Tx") != string::npos) {
    return canMSG_TXACK;
  }
  else if (str.find("Rx") != string::npos) {
    return 0;
  }

  return 0;
}

static unsigned int getDirection(string& str)
{
  if (str.compare("d") == 0)
    return 0;
  else if (str.compare("r") == 0)
    return canMSG_RTR;
  else
    return 0;
}

KvlcStatus KvaLogReader_VectorAsc::interpret_event_classic (imLogData *logEvent)
{
  double timestamp = 0;

  size_t size = elements.size();

  if (size >= 5 && elements[0].find("date") != string::npos) {
    maybe_header = true;
  }

  if (maybe_header && size >= 4 && elements[0].find("base") != string::npos) {
    if (elements[1].find("dec") != string::npos) {
      radix = 10;
    } else if (elements[1].find("hex") != string::npos) {
      radix = 16;
    } else {
     return kvlcERR_INVALID_LOG_EVENT;
    }
    return kvlcOK;
  }

  if (size < 3) {
    return kvlcOK;
  }

  try {
    timestamp = stod(elements[0]);
    not_header();
  } catch (const std::exception &e){
    (void) e;
    if (maybe_header) {
      return kvlcOK;
    } else {
      return kvlcERR_INVALID_LOG_EVENT;
    }
  }

  if (size == 4) {
    if (strcmp(elements[1].c_str(), "Start") == 0 &&
        strcmp(elements[2].c_str(), "of") == 0 &&
        strcmp(elements[3].c_str(), "measurement") == 0) {

      start_of_measurement64 += (time_uint64)(timestamp * ONE_BILLION);
      return kvlcOK;
    }
  }
  if (elements[2].find("Trigger") != string::npos) {
    if (size < 8){
      return kvlcERR_INVALID_LOG_EVENT;
    }
    logEvent->trig.type = toInt(elements[4]);
    logEvent->trig.trigNo = toInt(elements[5]);
    logEvent->trig.preTrigger = toInt(elements[6]);
    logEvent->trig.postTrigger = toInt(elements[7]);
    logEvent->common.time64 = (time_uint64) (timestamp * ONE_BILLION);
    logEvent->common.new_data = true;
    logEvent->common.type = ILOG_TYPE_TRIGGER;
    logEvent->trig.active = true;
    not_header();
  }
  else {
    unsigned int ind = 1;

    if ((elements[0].find("//")    == string::npos) &&
        (elements[1].find("CANFD") != string::npos)) {
      format = format_canfd;
      return kvlcOK;
    }
    if ((elements[0].find("//") == string::npos)){
      if (size < 5){
        return kvlcERR_INVALID_LOG_EVENT;
      } else if ( (elements[3].find("--") != string::npos) &&
                  (elements[4].find("r")  != string::npos))
      {
        format = format_canfd;
        return kvlcOK;
      }
    }

    logEvent->msg.frame_counter = ++current_frameno;
    logEvent->common.new_data = true;
    logEvent->common.type = ILOG_TYPE_MESSAGE;
    logEvent->msg.channel = toInt(elements[ind++]) - 1;

    if ((elements[0].find("//")         == string::npos) &&
        (elements[2].find("ErrorFrame") != string::npos))
    {
      if (size == 3) {
        format = format_canfd;
        return kvlcOK;
      }

      logEvent->msg.flags |= canMSG_ERROR_FRAME;
      logEvent->msg.dlc = 4;
      ind = 3;
    } else {
      if (size < ind+4){
        return kvlcERR_INVALID_LOG_EVENT;
      }
      logEvent->msg.id = toInt(elements[ind], radix);
      logEvent->msg.flags = isExtendedId(elements[ind++]);
      logEvent->msg.flags |= getFlags(elements[ind++]);
      logEvent->msg.flags |= getDirection(elements[ind++]);
      logEvent->msg.dlc = toInt(elements[ind++]);
    }
    for (unsigned int k = ind; k < elements.size(); k++) {
      logEvent->msg.data[k-ind] = toInt(elements[k], radix);
    }
    logEvent->common.time64 = (time_uint64) (timestamp * ONE_BILLION);
    not_header();
  }
  logEvent->common.event_counter = ++current_eventno;
  return kvlcOK;
}

KvlcStatus KvaLogReader_VectorAsc::interpret_event_canfd (imLogData *logEvent)
{
  double timestamp = 0;
  size_t size      = elements.size();

  if (size >= 5 && elements[0].find("date") != string::npos) {
    maybe_header = true;
  }

  if (maybe_header && size >= 4 && elements[0].find("base") != string::npos) {
    if (elements[1].find("dec") != string::npos) {
      radix = 10;
    } else if (elements[1].find("hex") != string::npos) {
      radix = 16;
    } else {
     return kvlcERR_INVALID_LOG_EVENT;
    }
    return kvlcOK;
  }

  if (size < 3) {
    return kvlcOK;
  }

  try {
    timestamp = stod(elements[0]);
    not_header();
  } catch (const std::exception &e){
    (void) e;
    if (maybe_header) {
      return kvlcOK;
    } else {
      return kvlcERR_INVALID_LOG_EVENT;
    }
  }

  if (elements[2].find("Trigger") != string::npos) {
    if (size < 8){
      return kvlcERR_INVALID_LOG_EVENT;
    }
    logEvent->trig.type = toInt(elements[4]);
    logEvent->trig.trigNo = toInt(elements[5]);
    logEvent->trig.preTrigger = toInt(elements[6]);
    logEvent->trig.postTrigger = toInt(elements[7]);
    logEvent->common.time64 = (time_uint64) (timestamp * ONE_BILLION);
    logEvent->common.new_data = true;
    logEvent->common.type = ILOG_TYPE_TRIGGER;
    logEvent->trig.active = true;
    not_header();
  }
  else {
    logEvent->msg.frame_counter = ++current_frameno;
    logEvent->common.new_data   = true;
    logEvent->common.type       = ILOG_TYPE_MESSAGE;

    if (size >= 3 && elements[2].find("ErrorFrame") != string::npos) {
		  logEvent->msg.flags = canMSG_ERROR_FRAME;
		  logEvent->msg.channel = toInt(elements[1]) - 1;
	  } else if (size < 4) {
      return kvlcERR_INVALID_LOG_EVENT;
    } else if (elements[3].find("--") != string::npos) { //remote
      logEvent->msg.flags    = canMSG_RTR;
      logEvent->msg.channel  = toInt(elements[1]) - 1;
      logEvent->msg.id       = toInt(elements[2], radix);
      logEvent->msg.flags   |= isExtendedId(elements[2]);
      logEvent->msg.dlc      = toInt(elements[5], radix);
    } else {
      logEvent->msg.channel  = toInt(elements[2]) - 1;
      logEvent->msg.id       = toInt(elements[4], radix);
      logEvent->msg.flags    = isExtendedId(elements[4]);
      logEvent->msg.flags   |= getFlags(elements[3]);
      logEvent->msg.dlc      = toInt(elements[7], 16);

      if (elements[size - 6][2] == '1') {
        logEvent->msg.flags |= canFDMSG_FDF;
      } else if (elements[size - 6][2] == '3') {
        logEvent->msg.flags |= canFDMSG_FDF;
        logEvent->msg.flags |= canFDMSG_BRS;
      }

      size_t nBytes = (size_t)dlcToNumBytesFD(logEvent->msg.dlc);

      for (size_t k = 0; k < nBytes; k++) {
        logEvent->msg.data[k] = toInt(elements[k + size - nBytes - 8], radix);
      }
    }
    logEvent->common.time64 = (time_uint64) (timestamp * ONE_BILLION);
    not_header();
  }
  logEvent->common.event_counter = ++current_eventno;
  return kvlcOK;
}

KvlcStatus KvaLogReader_VectorAsc::interpret_event (char      *line,
                                                    imLogData *logEvent)
{
  KvlcStatus stat;

  memset(logEvent, 0, sizeof(imLogData));
  logEvent->common.new_data = false;

  strip_extra_spaces(line);

  if (strncmp(line, "Begin Triggerblock", 18) == 0) {
    not_header();
    return kvlcOK;
  } else if (strcmp(line, "End Triggerblock") == 0) {
    return kvlcOK;
  }

  // Convert each line to a vector of strings
  split_string_to_vector(elements, line);
  //skip comments
  if (elements.size() > 0) {
    if (elements[0].find("//") == 0) {
      return kvlcOK;
    }
  }

  switch (format) {
    case format_classic:

      stat = interpret_event_classic (logEvent);
      if ((stat != kvlcOK) || (format == format_classic)) {
        return stat;
      }

      if (format == format_canfd) {
        return interpret_event_canfd (logEvent);
      }

      return kvlcFail;

    case format_canfd:
      return interpret_event_canfd (logEvent);

    default:
      return kvlcFail;
  }
}

KvlcStatus KvaLogReader_VectorAsc::read_row(imLogData *logEvent)
{
  KvlcStatus status;
  if (!isOpened) {
    return kvlcERR_FILE_ERROR;
  }
  status = read_line(buf, sizeof(buf));
  if (kvlcOK != status) {
    return status;
  }

  return interpret_event(buf, logEvent);
}

uint64 KvaLogReader_VectorAsc::event_count()
{
  // Counting lines would be more appropriate
  return (uint64) ((file_size-file_position) / 50);
}

void KvaLogReader_VectorAsc::not_header()
{
    maybe_header = false;
}

KvaLogReader_VectorAsc::KvaLogReader_VectorAsc()
{
  current_eventno = 0;
  current_frameno = 0;
  current_line = 0;
  radix = 0;
  maybe_header = true;
  format = format_classic;
}

class KvaReaderMaker_VectorAsc : public KvaReaderMaker
{
  public:
    KvaReaderMaker_VectorAsc() : KvaReaderMaker(KVLC_FILE_FORMAT_VECTOR_ASC) {}
    int getName(char *str) { return sprintf(str, "%s", "Vector ASCII"); }
    int getExtension(char *str) { return sprintf(str, "%s", "asc"); }
    int getDescription(char *str) { return sprintf(str, "%s", "CAN frames in Vector ASCII format"); }

  private:
    KvaLogReader *OnCreateReader() {
      return new KvaLogReader_VectorAsc();
    }
}  registerKvaLogReader_VectorAsc;
