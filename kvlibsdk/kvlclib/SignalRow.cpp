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
**   Utility class that can contain a set of strings and print them as row
**   using a special separator between the elements.
**
** ---------------------------------------------------------------------------
*/


#include <stdio.h>
#include <string.h>
#include "SignalRow.h"
#include "kvdebug.h"
#include "kvassert.h"

void SignalRow::expandIfNecessary(int column)
{
  if (column >= numColumns) {
    for(; numColumns < column + 1; numColumns++) {
      values[column].name = new char[1];
      values[column].name[0] = '\0';
      values[column].fullname = new char[1];;
      values[column].fullname[0] = '\0';
      values[column].unit = new char[1];
      values[column].unit[0] = '\0';
      values[column].enumeration = NULL;
      clear(column);
    }

    ASSERT(numColumns == column+1);
  }
}

void SignalRow::setFullName(int column, const char *fullname)
{
  size_t len = strlen(fullname);

  expandIfNecessary(column);
  if (values[column].fullname) {
    delete(values[column].fullname);
  }
  values[column].fullname = new char[len+1];
  memcpy(values[column].fullname, fullname, len);
  values[column].fullname[len] = '\0';
}

int SignalRow::getFullName(char *buf, size_t column)
{
  if (values[column].fullname) {
    return sprintf(buf, "%s", values[column].fullname);
  }
  else {
    return 0;
  }
}

void SignalRow::setName(int column, const char *name)
{
  size_t len = strlen(name);

  expandIfNecessary(column);
  if (values[column].name) {
    delete(values[column].name);
  }
  values[column].name = new char[len+1];
  memcpy(values[column].name, name, len);
  values[column].name[len] = '\0';
}

int SignalRow::getName(char *buf, int column)
{
  if (values[column].name) {
    return sprintf(buf, "%s", values[column].name);
  }
  else {
    return 0;
  }
}

void SignalRow::setUnit(int column, const char *unit)
{
  PRINTF(("setUnit(%d, %s)\n", column, unit));
  size_t len = strlen(unit);

  expandIfNecessary(column);
  if (values[column].unit) {
    delete(values[column].unit);
  }
  values[column].unit = new char[len+1];
  memcpy(values[column].unit, unit, len);
  values[column].unit[len] = '\0';
}

int SignalRow::getUnit(char *buf, size_t column)
{
  if (values[column].unit) {
    PRINTF(("getUnit(%s,%lu)\n", values[column].unit, column));
    return sprintf(buf, "%s", values[column].unit);
  }
  else {
    PRINTF(("getUnit(%p,%lu) failed\n", buf, column));
    return 0;
  }
}

SignalRow::SignalRow(int _numColumns, char _separator)
{
  int i;
  setSeparatorChar(_separator);

  numColumns = _numColumns;
  if (numColumns > 0) {
    values = new SignalType[numColumns];
    for (i = 0; i < numColumns; i++) {
      values[i].defined = false;
      values[i].missing = false;
      values[i].accessed = false;
      values[i].factor = 1.0;
      values[i].offset = 0.0;
      values[i].lowerLimit = 0.0;
      values[i].upperLimit = 0.0;
      values[i].message = 0;
      values[i].muxch = NULL;
      values[i].enumeration = NULL;
      values[i].name = NULL;
      values[i].unit = NULL;
      values[i].fullname = NULL;
    }
  }
  else {
    values = NULL;
  }
}

void SignalRow::setSeparatorChar(char _separator)
{
  separator = _separator;
}

SignalRow::SignalRow(char _separator)
{
  int column;
  numColumns = 0;
  separator = _separator;

  values = new SignalType[MAX_COLUMNS];
  for (column = 0; column < MAX_COLUMNS; column++) {
    values[column].defined = false;
    values[column].missing = false;
    values[column].accessed = false;
    values[column].factor = 1.0;
    values[column].offset = 0.0;
    values[column].lowerLimit = 0.0;
    values[column].upperLimit = 0.0;
    values[column].message = 0;
    values[column].muxch = NULL;

    values[column].name = NULL;
    values[column].unit = NULL;
    values[column].fullname = NULL;
    values[column].enumeration = NULL;
  }
}

SignalRow::~SignalRow()
{
  for (int column = 0; column < numColumns; column++) {
    if (values[column].name) {
      delete values[column].name;
      values[column].name = NULL;
    }
    if (values[column].unit) {
      delete values[column].unit;
      values[column].unit = NULL;
    }
    if (values[column].fullname) {
      delete values[column].fullname;
      values[column].fullname = NULL;
    }
    if (values[column].muxch) {
      delete values[column].muxch;
      values[column].muxch = NULL;
    }
    if (values[column].enumeration) {
      delete values[column].enumeration;
      values[column].enumeration = NULL;
    }
  }
  delete values;
}

void SignalRow::copySignalValues(const SignalRow *other)
{
  if (!other) return;

  // Update size
  numColumns = other->numColumns;
  
  for (int i = 0; i < numColumns; i++) {
    // Misc values
    values[i].defined     = other->values[i].defined;
    values[i].missing     = other->values[i].missing;
    values[i].accessed    = other->values[i].accessed;
    values[i].factor      = other->values[i].factor;
    values[i].offset      = other->values[i].offset;
    values[i].message     = other->values[i].message;
    values[i].value       = other->values[i].value;
    values[i].factor      = other->values[i].factor;
    values[i].offset      = other->values[i].offset;
    values[i].lowerLimit  = other->values[i].lowerLimit;
    values[i].upperLimit  = other->values[i].upperLimit;

    values[i].numDecimals = other->values[i].numDecimals;
    values[i].dataType    = other->values[i].dataType;
    values[i].channel     = other->values[i].channel;

    // Name (never changes)
    if (other->values[i].name && !values[i].name) {
      size_t len = strlen(other->values[i].name);
      values[i].name = new char[len+1];
      memcpy(values[i].name, other->values[i].name, len);
      values[i].name[len] = '\0';
    }

    // Unit (never changes)
    if (other->values[i].unit && !values[i].unit) {
      size_t len = strlen(other->values[i].unit);
      values[i].unit = new char[len+1];
      memcpy(values[i].unit, other->values[i].unit, len);
      values[i].unit[len] = '\0';
    }
    
    // Fullname (never changes)
    if (other->values[i].fullname && !values[i].fullname) {
      size_t len = strlen(other->values[i].fullname);
      values[i].fullname = new char[len+1];
      memcpy(values[i].fullname, other->values[i].fullname, len);
      values[i].fullname[len] = '\0';
    }

    // Enumeration (always update)
    if (other->values[i].enumeration) {
      if (values[i].enumeration) delete values[i].enumeration;
      size_t len = strlen(other->values[i].enumeration);
      values[i].enumeration = new char[len+1];
      memcpy(values[i].enumeration, other->values[i].enumeration, len);
      values[i].enumeration[len] = '\0';
    }
  }
}



int SignalRow::getSize()
{
  return numColumns;
}

void SignalRow::clearRow()
{
  for (int column = 0; column < numColumns; column++) {
    clearVal(column);
  }
  resetUpdated();
  PRINTF(("clearRow: UPDATED = false\n"));
}


void SignalRow::clear(int column)
{
  PRINTF(("clear(%d)\n", column));
  if (column > numColumns) {
    return;
  }
  values[column].defined = false;
  values[column].missing = false;
  values[column].accessed = false;
  if (values[column].name) {
    delete values[column].name;
    values[column].name = NULL;
    delete values[column].unit;
    values[column].unit = NULL;
  }
}

void SignalRow::clearVal(int column)
{
  if (column > numColumns) {
    return;
  }
  values[column].defined = false;
  values[column].missing = false;
  values[column].factor = 1.0;
  values[column].offset = 0.0;
  values[column].lowerLimit  = 0.0;
  values[column].upperLimit  = 0.0;
  values[column].accessed = false;
  values[column].message = 0;
  if (values[column].enumeration) {
    delete values[column].enumeration;
    values[column].enumeration = NULL;
  }
}

// print column names to buffer
int SignalRow::printNames(char *p)
{
  int i;
  char *oldp = p;

  if (printTimeStamp) {
    p += sprintf(p, "TimeStamp");
    p += sprintf(p, "%c", separator);
  }

  for (i = 0; i < numColumns; i++) {
    p += getName(p, i);
    if (i < numColumns - 1) {
      p += sprintf(p, "%c", separator);
    }
  }

  if (printRTC) {
    p += sprintf(p, "%c", separator);
    p += sprintf(p,"RTC");
  }

  return (int)(p - oldp);
}

// print units to buffer
int SignalRow::printUnits(char *p)
{
  int i;
  char *oldp = p;

  if (printTimeStamp) {
    p += sprintf(p, "s");
    p += sprintf(p, "%c", separator);
  }

  for (i = 0; i < numColumns; i++) {
    p += getUnit(p, i);
    if (i < numColumns - 1) {
      p += sprintf(p, "%c", separator);
    }
  }

  if (printRTC) {
    p += sprintf(p, "%c", separator);
    p += sprintf(p,"RTC");
  }

  return (int)(p - oldp);
}

// will place a copy of the null-terminated string in the internal memory of SignalRow at specified column
bool SignalRow::setSignalValue(int column, double val)
{

  expandIfNecessary(column);

  // or should fabs() >= eps??

  // Use 1000*eps to make the definition of equal a little bit fuzzy...
  if (!updated && (!values[column].defined || fabs(values[column].value - val) > 1000*DBL_EPSILON)) {
    #ifdef DEBUG
      if (values[column].defined) {
        PRINTF(("UPDATED column (%d), old (%f) != new (%f)\n", column, values[column].value, val));
      }
      else {
        PRINTF(("UPDATED column (%d), old (undefined), new (%f)\n", column, val));
      }
    #endif
    updated = true;
  }
  values[column].accessed = true;

  values[column].value = val;
  values[column].defined = true;
  values[column].missing = false;
  if (values[column].enumeration) {
    delete values[column].enumeration;
    values[column].enumeration = NULL;
    updated = true;
  }
  return true;
}


// will place a copy of the null-terminated string in the internal memory of SignalRow at specified column
bool SignalRow::setSignalValue(int column, const char *str)
{
  bool copy = true;
  expandIfNecessary(column);

  if (values[column].enumeration) {
    if (!updated && (!values[column].defined || strcmp(str, values[column].enumeration) != 0)) {
      #ifdef DEBUG
        if (values[column].defined) {
          PRINTF(("UPDATED column (%d), old (%s) != new (%s)\n", column, values[column].enumeration, str));
        }
        else {
          PRINTF(("UPDATED column (%d), old (undefined), new (%s)\n", column, str));
        }
      #endif
      delete values[column].enumeration;
      values[column].enumeration = NULL;
      updated = true;
    } else {
      // We already got the correct string
      copy = false;
    }
  }

  if (copy) {
     size_t len = strlen(str);
     values[column].enumeration = new char[len+1];
     memcpy(values[column].enumeration, str, len);
     values[column].enumeration[len] = '\0';
     updated = true;
  }

  values[column].accessed = true;
  values[column].defined = true;
  values[column].missing = false;
  return true;
}


void SignalRow::setTimeStamp(int64_t ts)
{
  timestamp = ts;
}

int64_t SignalRow::getTimeStamp()
{
  return timestamp;
}

void SignalRow::setAbsTime(uint64_t abs)
{
  abstime = abs;
}

uint64_t SignalRow::getAbsTime()
{
  return abstime;
}

void SignalRow::setChannel(uint32_t ch){
  channel = ch;
}
uint32_t SignalRow::getChannel(){
  return channel;
}

void SignalRow::setPrintProperties(bool ts, bool rtc)
{
  printTimeStamp = ts;
  printRTC = rtc;
}

double SignalRow::getSignalValue(int column)
{
  if (column > numColumns) {
    PRINTF(("getSignalValue(%d) outside range [0..%d]\n", column, numColumns));
    return 0.0;
  }
  return values[column].value;
}

const char * SignalRow::getSignalString(int column)
{
  if (column > numColumns) {
    PRINTF(("getSignalValue(%d) outside range [0..%d]\n", column, numColumns));
    return NULL;
  }
  return values[column].enumeration;
}

bool SignalRow::isSignalDefined(int column)
{
  if (column > numColumns) {
    PRINTF(("isSignalDefined(%d) outside range [0..%d]\n", column, numColumns));
    return false;
  }

  return values[column].defined;
}

bool SignalRow::isSignalEnumeration(int column)
{

  if (column > numColumns) {
    PRINTF(("isSignalEnumeration(%d) outside range [0..%d]\n", column, numColumns));
    return false;
  }
  return (values[column].enumeration != NULL);
}

bool SignalRow::isSignalMissing(int column)
{
  if (column > numColumns) {
    PRINTF(("isSignalMissing(%d) outside range [0..%d]\n", column, numColumns));
    return false;
  }

  return values[column].missing;
}

void SignalRow::setSignalMissing(int column)
{
  if (column > numColumns) {
    PRINTF(("isSignalMissing(%d) outside range [0..%d]\n", column, numColumns));
    return;
  }

  values[column].missing = true;
}

uint8_t SignalRow::getNumDecimals(int column)
{
  if (column > numColumns) {
    PRINTF(("getNumDecimals(%d) outside range [0..%d]\n", column, numColumns));
    return 0;
  }
  return values[column].numDecimals;
}

void SignalRow::setNumDecimals(int column, uint8_t numdecimals)
{
  expandIfNecessary(column);

  values[column].numDecimals = numdecimals;
}

void SignalRow::setScaling(int column, double factor, double offset)
{
  expandIfNecessary(column);

  values[column].factor = factor;
  values[column].offset = offset;
}

double SignalRow::getFactor(int column)
{
  if (column > numColumns) {
    PRINTF(("getFactor(%d) outside range [0..%d]\n", column, numColumns));
    return 0;
  }
  return values[column].factor;
}

double SignalRow::getOffset(int column)
{
  if (column > numColumns) {
    PRINTF(("getOffset(%d) outside range [0..%d]\n", column, numColumns));
    return 0;
  }
  return values[column].offset;
}

void SignalRow::setMux(int column, int mode, void *mux_signal, MuxCallback mcb) {
  if (!values[column].muxch) {
    delete values[column].muxch;
  }
  values[column].muxch = new MuxChecker(mux_signal, mode, mcb);
}

bool SignalRow::inEvent(int column, unsigned char *event_data, unsigned int len) {
  if (!values[column].muxch) {
    return true;
  }
  return values[column].muxch->inEvent(event_data, len);
}

void SignalRow::setMessageId(int column, void* id)
{
  expandIfNecessary(column);

  values[column].message = (uint64_t) id;
}

uint64_t SignalRow::getMessageId(int column)
{
  if (column > numColumns) {
    PRINTF(("getMessageId(%d) outside range [0..%d]\n", column, numColumns));
    return 0;
  }
  return values[column].message;
}

// Resets updated-flag and all columns' accessed-flags
void SignalRow::resetUpdated()
{
  int i;
  updated = false;
  for (i = 0; i < numColumns; i++) {
    values[i].accessed = false;
  }
}

void SignalRow::resetDefined()
{
  int i;
  for (i = 0; i < numColumns; i++) {
    values[i].defined = false;
  }

}

bool SignalRow::isAccessed(int column)
{
  if (column > numColumns) {
    PRINTF(("isAccessed(%d) outside range [0..%d]\n", column, numColumns));
    return false;
  }
  return values[column].accessed;
}

void SignalRow::setLimits(int column, double lower, double upper)
{
  expandIfNecessary(column);

  values[column].lowerLimit = lower;
  values[column].upperLimit = upper;
}

double SignalRow::getLowerLimit(int column)
{
  if (column > numColumns) {
    PRINTF(("getLowerLimit(%d) outside range [0..%d]\n", column, numColumns));
    return 0;
  }
  return values[column].lowerLimit;
}

double SignalRow::getUpperLimit(int column)
{
  if (column > numColumns) {
    PRINTF(("getUpperLimit(%d) outside range [0..%d]\n", column, numColumns));
    return 0;
  }
  return values[column].upperLimit;
}



/*
int main() {
  SignalRow *d = new SignalRow(get_property_separator_char());
  char foo[1000];
  int64_t time;
  d->setColumnName(0, "First");
//  d->setColumnName(1, "Second");
  d->setColumnName(2, "Third");
  d->setColumnName(3, "Fourth");
  d->setColumnName(4, "Fifth");

  d->setUnit(0, "First");
//  d->setUnit(1, "Second");
  d->setUnit(2, "Third");
  d->setUnit(3, "Fourth");
  d->setUnit(4, "Fifth");

  d->setSignalValue(0,-1.2,0);
  d->setSignalValue(1,3.4,1);
  //d->setSignalValue(2,5.6,2);
  d->setSignalValue(3,7.8,3);
  d->setSignalValue(4,9.0,4);
  d->setSignalValue(2,5.6,2);
  d->setPrintProperties(true, true);
  d->clearVal(3);

  d->printColumnNames(foo);
  printf("%s\n", foo);
  for (time = ONE_BILLION; time < ((uint64_t)10*ONE_BILLION); time += ONE_BILLION) {
    d->setTimeStamp(time);
    d->printto(foo);
    printf("%s\n", foo);
  }
  d->clearRow();
  d->printColumnNames(foo);
  printf("names: %s\n", foo);
  d->printUnits(foo);
  printf("units: %s\n", foo);

  d->printto(foo);
  printf("%s\n", foo);


  delete d;
  printf("done!\n");

  return 0;
}
*/
