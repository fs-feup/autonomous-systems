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
** TimeConv.cpp
**
** Description:
** Time conversion
** -----------------------------------------------------------------------------
*/

#include "TimeConv.h"
#include <stdio.h>
#include <inttypes.h>

#ifdef TIMECONVSTANDALONE
  #include <time.h>
#endif

#ifdef DEBUG
#include <assert.h>
#define ASSERT(x) assert(x)
#else
#define ASSERT(x)
#endif

//===========================================================================
enum Month {
  jan = 1, feb, mar, apr, may, jun, jul, aug, sep, oct, nov, dec
};

//===========================================================================
inline enum Month next_month (unsigned int *year, Month m)
{
  return (dec == m) ? (*year)++,jan : Month(m+1);
}

//===========================================================================
inline unsigned int leap (unsigned int year)
{
  return ((year%4==0 && year%100!=0) || year%400==0)?1:0;
}

//===========================================================================
// from 1970-01-01 to year-01-01
inline unsigned int nof_leaps_since_1970 (unsigned int year)
{
  unsigned int y;
  unsigned int leaps = 0;
  for (y = 1972; y < year; y+=4) {
    leaps += leap(y);
  }
  return leaps;
}

//===========================================================================
inline unsigned int days_of_year (unsigned int year)
{
  return 365 + leap(year);
}

//===========================================================================
// If someone adds an extra month in the Month-enum, it will have 31 days
inline unsigned int days_of_month (unsigned int year, enum Month month)
{
  switch (month) {
    //case jan: return 31;
    case feb: return 28 + leap(year);
    //case mar: return 31;
    case apr: return 30;
    //case may: return 31;
    case jun: return 30;
    //case jul: return 31;
    //case aug: return 31;
    case sep: return 30;
    //case oct: return 31;
    case nov: return 30;
    //case dec: return 31;
    default: return 31;
  }
}

//===========================================================================
inline unsigned int days_of_month (unsigned int year, unsigned int month)
{
  return days_of_month(year, Month(month));
}

//===========================================================================
/*
* Converts the given unix time (number of seconds since 1970-01-01 00:00)
* into date and time
* Valid years are 1970 and up
* Valid months are 1 to 12
* Valid days are 1 to days_of_month(year,month)
* Valid hours are 0 to 23
* Valid minutes are 0 to 59
* Valid seconds are 0 to 59
*/
void unix_time (uint64_t tid,
                unsigned int *year,
                unsigned int *month,
                unsigned int *day,
                unsigned int *hour,
                unsigned int *minute,
                unsigned int *second)
{
  Month mo = jan;

  unsigned int ye = 1970,da,ho,mi,se, tmp;

  se = (unsigned int)(tid % 60);
  tid /= 60;
  mi = (unsigned int)(tid % 60);
  tid /= 60;
  ho = (unsigned int)(tid % 24);
  tid /= 24;

#ifdef DEBUG
  // Print out a warning if the time seems to great...
  if (tid > 365000) {
    //PRINTF(("unix_time: Are you sure that you entered a time in seconds?"));
  }
#endif

  while ((tmp = days_of_year(ye)) <= tid) {
    tid -= tmp;
    ye++;
  }

  while ((tmp = days_of_month(ye, mo)) <= tid) {
    tid -= tmp;
    mo = next_month(&ye, mo);
  }

  // tid is now between 0 and 30
  da = (unsigned int)tid;

  da++;

  ASSERT(ye >= 1970);
  ASSERT(mo >= 1);
  ASSERT(mo <= 12);
  ASSERT(da >= 1);
  ASSERT(da <= days_of_month(ye, mo));
  // ASSERT(ho >= 0);  // comparison of unsigned expression >= 0 is always true
  ASSERT(ho <= 23);
  // ASSERT(mi >= 0);
  ASSERT(mi <= 59);
  // ASSERT(se >= 0);
  ASSERT(se <= 59);

  *year = ye;
  *month = mo;
  *day = da;
  *hour = ho;
  *minute = mi;
  *second = se;
}

//===========================================================================
/*
* Converts the given time into seconds since 1970-01-01 00:00
* Valid years are 1970 and up
* Valid months are 1 to 12
* Valid days are 1 to days_of_month(year,month)
* Valid hours are 0 to 23
* Valid minutes are 0 to 59
* Valid seconds are 0 to 59
*/
uint64_t convert_time (unsigned int year,
                              unsigned int month,
                              unsigned int day,
                              unsigned int hour,
                              unsigned int minute,
                              unsigned int second)
{

  ASSERT(year >= 1970);
  ASSERT(month >= 1);
  ASSERT(month <= 12);
  ASSERT(day >= 1);
  ASSERT(day <= days_of_month(year, month));
  // ASSERT(hour >= 0);
  ASSERT(hour <= 23);
  // ASSERT(minute >= 0);
  ASSERT(minute <= 59);
  // ASSERT(second >= 0);
  ASSERT(second <= 59);

  uint64_t ut = 0;

  ut = (uint64_t)second;
  ut += 60 * (uint64_t)minute;
  ut += 3600 * (uint64_t)hour;
  ut += 3600*24 * (uint64_t)(day-1);
  ut += (uint64_t)(year-1970) * 3600 * 24 * 365;

  // leap days
  ut += 24*3600*(uint64_t)nof_leaps_since_1970(year);

  // months
  unsigned int i;
  for (i = 1; i < month; i++) {
    ut += 24*3600*(uint64_t)days_of_month(year,i);
  }

  return ut;
}

//===========================================================================
#ifdef DEBUG
void test_time_conversions ()
{
  uint64_t tid_i_sek = 0, tmp;
  unsigned int year, month, day, hour, minute, second;
  int i = 0;
  unsigned int counter = 0;

  for (;tid_i_sek < 12208988801; tid_i_sek+= 34567) {
    counter++;
    unix_time(tid_i_sek, &year, &month, &day, &hour, &minute, &second);
    if (i++ % 1000 == 0) {
      printf("%04u-%02u-%02u %02u:%02u:%02u\n",
              year, month, day, hour, minute, second);
    }

    tmp = convert_time(year, month, day, hour, minute, second);
    if (tid_i_sek != tmp) {
      // this shouldn't happen
      ASSERT(false);
    }
  }
  printf("Converted %u different times back and forth and all seems ok\n",
          counter);
}
#ifdef TIMECONVSTANDALONE
//===========================================================================
// Calculate number of seconds since 1/1 1970 w/o messing with local time.
// y is the number of years since 1980. (valid between year 1970 and 2037)
static time_t convertTime (int y, int m, int d, int h, int min, int s)
{
  time_t tim;

  tim = y + 10;
  tim *= 365;
  switch (m) {
    case 1: break;
    case 2: tim += 31; break;
    case 3: tim += 31+28; break;
    case 4: tim += 31+28+31; break;
    case 5: tim += 31+28+31+30; break;
    case 6: tim += 31+28+31+30+31; break;
    case 7: tim += 31+28+31+30+31+30; break;
    case 8: tim += 31+28+31+30+31+30+31; break;
    case 9: tim += 31+28+31+30+31+30+31+31; break;
    case 10:tim += 31+28+31+30+31+30+31+31+30; break;
    case 11:tim += 31+28+31+30+31+30+31+31+30+31; break;
    case 12:tim += 31+28+31+30+31+30+31+31+30+31+30; break;
  }

  if (y%4==0 && m >= 3) tim++; // Has there been a leap day this year yet?
  tim += ((y+7)>>2); // Leap days since 1970, works between 1970 and 2038
  tim += d;

  tim *= 24;
  tim += h;
  tim *= 60;
  tim += min;
  tim *= 60;
  tim += s;

  return tim;
}

//===========================================================================
void test_time_conversions2 ()
{
  time_t tid_i_sek = (time_t)0;

  int i = 0;
  unsigned int counter = 0;

  for (;tid_i_sek < 2147483648; tid_i_sek+= 313){//34567) {
    struct tm *mytime;
    time_t tmp;

    counter++;
    //unix_time(tid_i_sek, &year, &month, &day, &hour, &minute, &second);
    mytime = gmtime(&tid_i_sek);
    if (/*1 || */i++ % 100000 == 0) {
      printf("%04d-%02d-%02d %02d:%02d:%02d\t%" PRIu64 "\n",
            (mytime->tm_year+1900),
            (mytime->tm_mon+1),
            (mytime->tm_mday),
            (mytime->tm_hour),
            (mytime->tm_min),
            (mytime->tm_sec),
            tid_i_sek
            );
    }

    tmp = convertTime(
      mytime->tm_year-80,
      mytime->tm_mon+1,
      mytime->tm_mday,
      mytime->tm_hour,
      mytime->tm_min,
      mytime->tm_sec);
    if (tid_i_sek != tmp) {
        printf("tid_i_sek = %" PRIu64 " = unix_time => ",tid_i_sek);
        printf("%04d-%02d-%02d %02d:%02d:%02d = convert_time => ",
            (mytime->tm_year+1900),
            (mytime->tm_mon+1),
            (mytime->tm_mday),
            (mytime->tm_hour),
            (mytime->tm_min),
            (mytime->tm_sec));
        printf("%" PRIu64 "\n", tmp);
        // this it shouldn't happen
        ASSERT(false);
      }
  }
  printf("Converted %u different times back and forth and all seems ok\n",
          counter);
}
#endif
#endif

#ifdef TIMECONVSTANDALONE
//===========================================================================
int main()
{
  test_time_conversions();
  test_time_conversions2();
  return 0;
}
#endif
