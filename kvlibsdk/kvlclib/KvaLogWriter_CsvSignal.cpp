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
#include "KvaLogWriter_CsvSignal.h"
#include "TimeConv.h"
#include "kvdebug.h"
#include "os_util.h"

//===========================================================================
KvaLogWriter_CsvSignal::KvaLogWriter_CsvSignal()
{
  PRINTF(("KvaLogWriter_CsvSignal::KvaLogWriter_CsvSignal()\n"));
  currentSize = 0;
}

//===========================================================================
KvaLogWriter_CsvSignal::~KvaLogWriter_CsvSignal()
{
  PRINTF(("KvaLogWriter_CsvSignal::~KvaLogWriter_CsvSignal()\n"));
}


//===========================================================================
KvlcStatus KvaLogWriter_CsvSignal::write_signal_header()
{
  char *p = outbuffer;
  char timerow[80];
  time_t aclock;
  struct tm newtime;

  KvlcStatus kvstatus = kvlcOK;
  outbuffer[0] = '\0';

  if (!get_property_write_header()) {
    PRINTF(("write_header turned off by user option"));
    return kvlcOK;
  }

  if (!isOpened) {
    PRINTF(("KvaLogWriter_CsvSignal::write_header, file is not opened\n"));
    return kvlcERR_FILE_ERROR;
  }

  aclock = get_wallclock_time();
  get_calendar_time(aclock, &newtime);

  os_rewind(outfile);

  p += sprintf(p, "CSV signals\n");
  p += sprintf(p, "Converted from Kvaser Memorator at%c", get_property_separator_char());
  p += printCalendarDate(p, newtime);
  p += sprintf(p, "\n");

  if (overrun_occurred) {
    p += sprintf(p, CSV_OVERRUN_STRING"\n");
  }
  else {
    for (int i = (int)strlen(CSV_OVERRUN_STRING); i > 0; i--) {
      p += sprintf(p, " ");
    }
    p += sprintf(p, "\n");
  }
  p += sprintf(p,
                   "Format of data field%c%s\n",
                   get_property_separator_char(),
                   get_property_data_in_hex()?"HEX":"DEC");

  printDecimalNumber(timerow, get_property_offset(), get_property_time_decimals());

  p += sprintf(p, "Timestamp Offset%c%20s%cseconds\n",
          get_property_separator_char(),
          timerow,
          get_property_separator_char()
         );

  p += sprintf(p, "Logging started at%c", get_property_separator_char());

  p += printCalendarDate(p, start_of_logging);
  p += sprintf(p, "\n");

  kvstatus = write_file(outbuffer, p - outbuffer);
  if (kvstatus != kvlcOK) {
    return kvstatus;
  }
  currentSize += p - outbuffer;
  p = outbuffer;

  // Signal Names
  if (true /*printTimeStamp*/) {
    p += sprintf(p, "Timestamp%c", get_property_separator_char());
  }

  for (int i = 0; i < sr->getSize(); i++) {
    if(get_property_full_names()){
      p += sr->getFullName(p, i);
    }else{
      p += sr->getName(p, i);
    }
    if (i < sr->getSize() - 1) {
      p += sprintf(p, "%c", get_property_separator_char());
    }
  }

  if (get_property_calendar_time_stamps()) {
    p += sprintf(p, "%cAbsTime", get_property_separator_char());
  }

  p += sprintf(p, "%cChannel ID", get_property_separator_char());

  // add end of line to line of signal names
  p += sprintf(p, "\n");

  kvstatus = write_file(outbuffer, p - outbuffer);
  if (kvstatus != kvlcOK) {
    return kvstatus;
  }

  currentSize += p - outbuffer;
  if (kvstatus == kvlcOK && get_property_show_units()) {
    p = outbuffer;
    if (true /*printTimeStamp*/) {
      p += sprintf(p, "s%c", get_property_separator_char());
    }

    for (int i = 0; i < sr->getSize(); i++) {
      p += sr->getUnit(p, i);
      if (i < sr->getSize() - 1) {
        p += sprintf(p, "%c", get_property_separator_char());
      }
    }

    if (get_property_calendar_time_stamps()) {
      if (get_property_iso_8601_decimals()) {
        p += sprintf(p, "%cYYYY-MM-DDTHH:MM:SS.ssss", get_property_separator_char());
      }
      else {
        p += sprintf(p, "%cYYYY-MM-DD HH:MM:SS", get_property_separator_char());
      }
    }
    // Add None as unit for channel ID
    p += sprintf(p, "%cNone", get_property_separator_char());

    // add end of line to line of units
    p += sprintf(p, "\n");

    kvstatus = write_file(outbuffer, p - outbuffer);
    if (kvstatus != kvlcOK) {
      return kvstatus;
    }
    currentSize += p - outbuffer;
  }
  return kvstatus;
}

#define KV_UNDEF_STR ""

KvlcStatus KvaLogWriter_CsvSignal::write_signals() {
  KvlcStatus kvstatus = kvlcOK;
  char *p = outbuffer;
  int i;

  if (true /* printTimeStamp */) {
    p += printDecimalNumber(p, sr->getTimeStamp(), 4);
    p += sprintf(p, "%c", get_property_separator_char());
  }

  for (i = 0; i < sr->getSize(); i++) {
    if (sr->isSignalDefined(i)) {
      if (get_property_enum_values() && sr->isSignalEnumeration(i)) {
        p += printEnumString(p, sr->getSignalString(i));
      }
      else {
        p += printDoubleNumber(p, sr->getSignalValue(i), sr->getNumDecimals(i));
      }
    }
    else {
      p += sprintf(p, "%s", KV_UNDEF_STR);
    }
    if (i < sr->getSize() - 1) {
      p += sprintf(p, "%c", get_property_separator_char());
    }
  }

  if (get_property_calendar_time_stamps()) {
    p += sprintf(p, "%c", get_property_separator_char());
    p += printCalendarDate(p, sr->getAbsTime());
  }

  p += sprintf(p, "%c%d", get_property_separator_char(), sr->getChannel());

  p += sprintf(p, "\n");

  if (!get_property_fill_blanks()) {
    sr->clearRow();
  }
  currentSize += p - outbuffer;
  return kvstatus = write_file(outbuffer, p - outbuffer);
}

#define NAME        "CSV Signal"
#define EXTENSION   "csv"
#define DESCRIPTION "Selected signals in CSV format"

class KvaWriterMaker_CsvSignal : public KvaWriterMaker
{
  public:
    KvaWriterMaker_CsvSignal() : KvaWriterMaker(KVLC_FILE_FORMAT_CSV_SIGNAL) {
      propertyList[KVLC_PROPERTY_START_OF_MEASUREMENT] = true;
      propertyList[KVLC_PROPERTY_FIRST_TRIGGER] = true;
      propertyList[KVLC_PROPERTY_CHANNEL_MASK] = true;
      propertyList[KVLC_PROPERTY_HLP_J1939] = true;
      propertyList[KVLC_PROPERTY_CALENDAR_TIME_STAMPS] = true;
      propertyList[KVLC_PROPERTY_WRITE_HEADER] = true;
      propertyList[KVLC_PROPERTY_SEPARATOR_CHAR] = true;
      propertyList[KVLC_PROPERTY_DECIMAL_CHAR] = true;
      propertyList[KVLC_PROPERTY_SIGNAL_BASED] = true;
      propertyList[KVLC_PROPERTY_NUMBER_OF_TIME_DECIMALS ] = true;
      propertyList[KVLC_PROPERTY_FILL_BLANKS] = true;
      propertyList[KVLC_PROPERTY_SHOW_UNITS] = true;
      propertyList[KVLC_PROPERTY_ISO8601_DECIMALS] = true;
      propertyList[KVLC_PROPERTY_SHOW_SIGNAL_SELECT] = true;
      propertyList[KVLC_PROPERTY_SHOW_COUNTER] = true;
      propertyList[KVLC_PROPERTY_CROP_PRETRIGGER] = true;
      propertyList[KVLC_PROPERTY_ENUM_VALUES] = true;
      propertyList[KVLC_PROPERTY_SIZE_LIMIT] = true;
      propertyList[KVLC_PROPERTY_TIME_LIMIT] = true;
      propertyList[KVLC_PROPERTY_OVERWRITE] = true;
      propertyList[KVLC_PROPERTY_TIMEZONE] = true;
      propertyList[KVLC_PROPERTY_FULLY_QUALIFIED_NAMES] = true;
      propertyList[KVLC_PROPERTY_NUMBER_OF_DATA_DECIMALS] = true;

      int defaultValue = 4;
      properties.set_property_and_default_value(KVLC_PROPERTY_NUMBER_OF_TIME_DECIMALS, &defaultValue, sizeof(defaultValue));
      defaultValue = 8;
      properties.set_property_and_default_value(KVLC_PROPERTY_ISO8601_DECIMALS, &defaultValue, sizeof(defaultValue));
    }
    int getName(char *str) { return sprintf(str, "%s", NAME); }
    int getExtension(char *str) { return sprintf(str, "%s", EXTENSION); }
    int getDescription(char *str) { return sprintf(str, "%s", DESCRIPTION); }

  private:
    KvaLogWriter *OnCreateWriter() {
      KvaLogWriter *writer = new KvaLogWriter_CsvSignal();
      writer->mProperties = properties;
      return writer;
    }
}  registerKvaLogWriter_CsvSignal;

