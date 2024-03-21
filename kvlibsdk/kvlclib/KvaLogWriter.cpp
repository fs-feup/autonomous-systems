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

#include <stdio.h>

#include <math.h>
#include <regex>
#include <libgen.h>
#include <dirent.h>

#include "KvaLogWriter.h"
#include "common_defs.h"
#include "string.h"
#include "TimeConv.h"
#include "os_util.h"
#include "kvdebug.h"

int PROPERTY_ON  = 1;
int PROPERTY_OFF = 0;

#define BYTES_TO_MB(x) (x/(uint64_t)1000000LL)
#define NS_TO_SECONDS(x) (x/(time_int64)ONE_BILLION)
#define SECONDS_TO_NS(x) (x*(time_int64)ONE_BILLION)

#define FILE_SPLIT_SUFFIX "%s-part%d"

KvaProperties mProperties;

/* This method should be moved to KvaLogWriter_Signal, when KvaLogWriter_MdfSignal is made to inherit from KvaLogWriter_Signal,
   like it should have */
int MuxCallbackValueChecker(KvaDbSignalHnd mux_signal, unsigned char *event_data, unsigned int len) {

  int value = 0;
  KvaDbStatus dbStat = kvaDbGetSignalValueInteger(mux_signal, &value, event_data, len);
  if (dbStat!=kvaDbOK) {return -1;}
  return value;
}

KvaLogWriter::KvaLogWriter()
{
  PRINTF(("KvaLogWriter::KvaLogWriter()\n"));
  start_of_logging = 0;
  last_clock_event = 0;
  first_timestamp = 0;
  overrun_occurred = false;
  data_truncation_occurred = false;
  dlc_mismatch_occurred = false;
  isOpened = false;
  outfile = NULL;

  fake_wallclock_time       = false;
  faked_wallclock_time      = (time_t)0;

  mFileNo = 0;
  mFirstEvent = true;
  mFirstTime = 0;
  mSplitPending = false;
  mNewFileCreated = true;
}

KvaLogWriter::~KvaLogWriter()
{
  PRINTF(("KvaLogWriter::~KvaLogWriter()"));
}

KvlcStatus KvaLogWriter::mapDbError(KvaDbStatus stat)
{
  switch (stat) {
    case kvaDbOK:
//      PRINTF(("mapDbError kvlcOK\n"));
      return kvlcOK;
    case kvaDbErr_Fail:
      PRINTF(("mapDbError KvLogCnvFail\n"));
      return kvlcFail;
    case kvaDbErr_Param:
      PRINTF(("mapDbError kvlcERR_PARAM\n"));
      return kvlcERR_PARAM;
    case kvaDbErr_Internal:
      PRINTF(("mapDbError kvlcERR_INTERNAL_ERROR 1\n"));
      return kvlcERR_INTERNAL_ERROR;
    case kvaDbErr_DbFileOpen:
      PRINTF(("mapDbError kvlcERR_FILE_ERROR\n"));
      return kvlcERR_FILE_ERROR;
    case kvaDbErr_DatabaseInternal:
      PRINTF(("mapDbError kvlcERR_INTERNAL_ERROR 2\n"));
      return kvlcERR_INTERNAL_ERROR;
    default:
      PRINTF(("mapDbError kvlcERR_INTERNAL_ERROR 3, %d\n", stat));
      return kvlcERR_INTERNAL_ERROR;
  }
}

KvlcStatus KvaLogWriter::write_event(imLogData *logEvent)
{
  KvlcStatus stat = kvlcOK;

  if (ILOG_TYPE_MESSAGE != logEvent->common.type) {
    return write_row(logEvent);
  }

  if (split_on_time(logEvent->common.time64)) {
      stat = split_file();
      if (stat != kvlcOK) return stat;
  }
  return write_row(logEvent);
}

const char* KvaLogWriter::get_next_filename()
{
  std::string drive, dir, fname, ext, path;
  char partname[2048];

  if ( !os_splitpath(mFilename, drive, dir, fname, ext) ) {
    PRINTF(("KvaLogWriter::get_next_filename(): Invalid filename:\n'%s'\n",
            mFilename.c_str()));
    return NULL;
  }
  snprintf(partname, sizeof(partname), FILE_SPLIT_SUFFIX, fname.c_str(), mFileNo++);
  os_makepath(path, drive, dir, partname, ext);
  mCurrentFilename = path;
  PRINTF(("Next_filename: '%s'", mCurrentFilename.c_str()));
  mNewFileCreated = true;
  return mCurrentFilename.c_str();
}

KvlcStatus KvaLogWriter::get_filename_status(bool *updated)
{
  if (!updated) return kvlcERR_PARAM;
  *updated = false;
  if (mNewFileCreated && isOpened) {
    mNewFileCreated = false;
    *updated = true;
  }
  return kvlcOK;
}

KvlcStatus KvaLogWriter::get_filename(char *filename, size_t len)
{
  if (!filename || !len) return kvlcERR_PARAM;
  if (strlen(get_filename()) + 1 > len) return kvlcERR_BUFFER_SIZE;
  strncpy(filename, get_filename(), len);
  return kvlcOK;
}

KvlcStatus KvaLogWriter::open_file()
{
  KvlcStatus status = kvlcOK;
  const char *filename = get_filename();

  if (mProperties.mSizeLimit != 0 || mProperties.mTimeLimit != 0) {
    filename = get_next_filename();
  }

  if (!filename || !strlen(filename)) {
    PRINTF(("Error: Empty file name. Call set_filename() first.\n"));
    return kvlcERR_FILE_ERROR;
  }

  status = file_status();
  if (status != kvlcOK) {
    return status;
  }

  if (isBinary()) {
    //outfile = fopen(filename, "wb");
	outfile = utf_fopen(filename, "wb");
  }
  else {
    //outfile = fopen(filename, "w");
	outfile = utf_fopen(filename, "w");
  }
  if (!outfile) {
    PRINTF(("Error: Could not open file: '%s'.\n", filename));
    return kvlcERR_FILE_ERROR;
  }

  setbuf(outfile, file_write_buffer);
  setvbuf(outfile, file_write_buffer, _IOFBF, sizeof(file_write_buffer));

  file_size = 0;
  file_position = 0;
  isOpened = true;
  return kvlcOK;
}

KvlcStatus KvaLogWriter::open_new_file(void)
{
  KvlcStatus status;

  write_header();
  close_file();

  mFirstEvent = true;
  mFirstTime = 0;
  file_size = 0;
  mSplitPending = false;
  status = open_file();
  if (status != kvlcOK) return status;

  status = write_header();
  return status;
}

KvlcStatus KvaLogWriter::write_file(void *buf, size_t length)
{
  KvlcStatus stat = kvlcOK;

  if (!isOpened) {
    PRINTF(("KvaLogWriter::write_file, file is not opened\n"));
    return kvlcERR_FILE_ERROR;
  }

  file_size += length;

  if (1 != fwrite(buf, length, 1, outfile)) {
    PRINTF(("KvaLogWriter::write_file, couldn't write file."));
    return kvlcERR_FILE_ERROR;
  }

  if (ferror (outfile)) {
    PRINTF(("KvaLogWriter::write_file, error writing to file\n"));
    return kvlcERR_FILE_ERROR;
  }

  if (split_on_size(file_size + length)) {
    stat = split_file();
  }

  return stat;
}

KvlcStatus KvaLogWriter::close_file()
{
  KvlcStatus kvstatus = kvlcOK;

  try {
    if (outfile) {
      fclose(outfile);
    }
  }
  catch(...) {
    PRINTF(("Failed to fclose(%p)\n", outfile));
  }
  isOpened = false;

  if (!mSplitPending) {
    KvlcStatus stat;
    PRINTF(("kvalogwriter::close_file, number of databases = %d\n", static_cast<int>(databases.size())));
    for (unsigned int i = 0; i < databases.size(); i++) {
      // Only close database handles to files that the converter has opened
      if (databases_is_file[i]) {
        stat = kvaDbClose(databases[i]);
        if (stat != kvlcOK) {
          kvstatus = stat;
        }
        PRINTF(("kvaDbClose(databases(%d)), stat = %d\n", i, stat));
      }
    }
    databases.clear();
    databases_chanmask.clear();
    databases_is_file.clear();
  }
  return kvstatus;
}

bool KvaLogWriter::get_property_start_of_measurement()
{
  return (mProperties.mUseStartOfMeasurementOffset == PROPERTY_ON);
}

bool KvaLogWriter::get_property_show_units()
{
  return (mProperties.mShowUnits == PROPERTY_ON);
}

bool KvaLogWriter::get_property_fill_blanks()
{
  return (mProperties.mFillBlanks == PROPERTY_ON);
}

time_int64 KvaLogWriter::get_property_offset()
{
  return mProperties.mLogOffsetTime;
}

time_int64 KvaLogWriter::get_property_logfile_creation_date()
{
  return mProperties.mLogCreationTime;
}

unsigned long KvaLogWriter::get_property_channel_mask()
{
  return mProperties.mChannelMask;
}

bool KvaLogWriter::get_property_hlp_j1939()
{
  return (mProperties.mJ1939Active == PROPERTY_ON);
}

bool KvaLogWriter::get_property_calendar_time_stamps()
{
  return (mProperties.mUseCalendarTimestamps == PROPERTY_ON);
}

bool KvaLogWriter::get_property_write_header()
{
  return (mProperties.mWriteHeaderActive == PROPERTY_ON);
}

char KvaLogWriter::get_property_separator_char()
{
  return mProperties.mCsvSeparatorChar;
}

char KvaLogWriter::get_property_decimal_char()
{
  return mProperties.mDecimalSeparatorChar;
}

int KvaLogWriter::get_property_time_decimals()
{
  return mProperties.mTimeDecimals;
}

int KvaLogWriter::get_property_data_decimals()
{
  return mProperties.mDataDecimals;
}

bool KvaLogWriter::get_property_id_in_hex()
{
  return (mProperties.mCsvIdFormatHex == PROPERTY_ON);
}

bool KvaLogWriter::get_property_data_in_hex()
{
  return (mProperties.mCsvDataFormatHex == PROPERTY_ON);
}

bool KvaLogWriter::get_property_name_mangling()
{
  return (mProperties.mUseMatlabNameMangling == PROPERTY_ON);
}

int KvaLogWriter::get_property_iso_8601_decimals()
{
  return mProperties.mIso8601Decimals;
}

bool KvaLogWriter::get_property_merge_lines()
{
  return (mProperties.mMergeLines == PROPERTY_ON);
}

int KvaLogWriter::get_property_resample_column()
{
  return mProperties.mResampleColumn;
}

int KvaLogWriter::get_property_version()
{
  return mProperties.mFileFormatVersion;
}

bool KvaLogWriter::get_property_show_counter()
{
  return (mProperties.mShowCounter == PROPERTY_ON);
}

bool KvaLogWriter::get_property_crop_pretrigger()
{
  return (mProperties.mCropPretrigger == PROPERTY_ON);
}

bool KvaLogWriter::get_property_enum_values()
{
  return (mProperties.mUseEnumValues == PROPERTY_ON);
}

unsigned long KvaLogWriter::get_property_size_limit()
{
  return mProperties.bytesToMB(mProperties.mSizeLimit);
}

unsigned long KvaLogWriter::get_property_time_limit()
{
  return mProperties.nsToSeconds(mProperties.mTimeLimit);
}

unsigned int KvaLogWriter::get_property_limit_databytes()
{
  return (unsigned int) mProperties.mLimitNumberOfDatabytes;
}

KvlcStatus KvaLogWriter::set_property_offset(time_int64 offset)
{
  mProperties.mLogOffsetTime = offset;
  return kvlcOK;
}

//==============================================================================
KvlcStatus KvaLogWriter::add_database_file(const char *filename, unsigned int channel_mask)
{
  KvaDbHnd dh;
  KvaDbStatus status;
  status = kvaDbOpen(&dh);
  if (status != kvaDbOK) return mapDbError(status);
  status = kvaDbCreate(dh, NULL, filename);
  if (status != kvaDbOK) return mapDbError(status);
  return add_database_handle(dh, channel_mask, true);
}

//==============================================================================
KvlcStatus KvaLogWriter::add_database_handle(KvaDbHnd dh, unsigned int channel_mask, bool is_file)
{
  databases.push_back(dh);
  databases_chanmask.push_back(channel_mask);
  databases_is_file.push_back(is_file);
  return kvlcOK;
}

//==============================================================================
//   get_J1939_id - function to extract the j1939 id.
unsigned long KvaLogWriter::get_J1939_id(unsigned int id)
{
    unsigned long pf;
    unsigned long res_id = 0;

    // Standard CAN -> return the identifier
    if ((id & 0x80000000) == 0) return id;

    pf = (id & 0x00ff0000) >> 16;

    if (pf < 0xf0) {
        // PDU1 format
        res_id = (id & 0x1ff0000) >> 8;
    }
    else {
        // PDU2 format
        res_id = (id & 0x1ffff00) >> 8;
    }
    return res_id | 0x80000000;
} // get_J1939_id


/*
  From Kvaser J1939 Overview.pdf:
  The 29-bit identifier used in J1939 is structured in the following way.
  Priority | Reserved | Data page | PDU format | PDU specific | Source Address
  3 bits   | 1 bit    | 1 bit     | 8 bits     | 8 bits       | 8 bits
*/
unsigned int KvaLogWriter::get_J1939_mask(unsigned int id)
{
  const unsigned int pf_mask = 0x1ff00ff;
  const unsigned int ps_mask = 0x000ffff;
  unsigned int pf = 0;
  unsigned int mask = 0;

  // Get Data page and PDU format bits
  pf = (id & pf_mask) >> 16;

  // Check PDU format bits (0xff) for for PDU1 (0-239) or PDU2 format (240-255)
  if ((pf & 0xff) < 240) {
    // PDU1: Addressable message; PDU specific field (ps) contains dest. address
    mask = pf_mask;
  }
  else {
    // PDU2: Broadcast message; PDU specific field (ps) contains Group Extension
    mask = pf_mask | ps_mask;
  }

  // Add extended id check
  mask |= 0x80000000;

  return mask;
}


time_t KvaLogWriter::get_wallclock_time()
{
  if (fake_wallclock_time) return faked_wallclock_time;
  else return time(NULL);
}

void KvaLogWriter::get_calendar_time(time_t input_time, struct tm *out_calendar_time)
{
  switch (mProperties.mTimezone) {
    case mProperties.KVLC_TZ_UTC:
      GMTIME(&input_time, out_calendar_time);
    break;
    default: //assume KVLC_TZ_LOCALTIME
      LOCALTIME(&input_time, out_calendar_time);
    break;
  }
}

void KvaLogWriter::next_file() {
  PRINTF(("KvaLogWriter::next_file, doesn't do anything right now"));
}


int KvaLogWriter::printCalendarDate(char *p, const struct tm &calendar_date) {
/* ALl our print functions are unsafe, callee should make sure that buffer is large enough
 * Unsafe but can be justified given that calendar times and numbers require limited space
 */
  return sprintf(p, "%4u-%02u-%02u %02u:%02u:%02u",
                 calendar_date.tm_year + 1900,
                 calendar_date.tm_mon + 1,
                 calendar_date.tm_mday,
                 calendar_date.tm_hour,
                 calendar_date.tm_min,
                 calendar_date.tm_sec);
}

int KvaLogWriter::printCalendarDate(char *p, time_int64 time64) {
  int length = 0;
  time_t sec_time;
  struct tm tmp_tm;

  sec_time = (time_t) time64 / ONE_BILLION;

  switch(mProperties.mTimezone) {
    case mProperties.KVLC_TZ_UTC:
      GMTIME(&sec_time, &tmp_tm);
    break;
    default:
      LOCALTIME(&sec_time, &tmp_tm);
    break;
  }

  length += printCalendarDate(p, tmp_tm);
  if (get_property_iso_8601_decimals()) {
    time_uint64 fraction;
    p += length;
    fraction = time64 % ONE_BILLION;
    length += sprintf(p, "%c%04u", get_property_decimal_char(), (unsigned int) fraction / 100000);
  }
  return length;
}


/*
* sprintfs a decimal number to *p, the number has to be separated into three
* parts, t1, which is the integer-part of the number, t2 which is the decimal-
* part of the number and isNegative, which is true if the number less than 0,
* true otherwise.
* You also need to supply the number of decimals (between 0 and 9, default = 4)
*
*/
int KvaLogWriter::printDecimalNumber(char *p, uint64_t t1, uint64_t t2, bool isNegative, int nofDecimals) {
  char *oldp = p;
  p += sprintf(p,
#ifdef __GNUC__
         "%s%ju",
#else
         "%s%I64u",
#endif
         isNegative?"-":"",
         t1);

  switch (nofDecimals) {
    case 9: p += sprintf(p, "%c%09u", get_property_decimal_char(), (unsigned int) t2); break;
    case 8: p += sprintf(p, "%c%08u", get_property_decimal_char(), (unsigned int) t2 / 10); break;
    case 7: p += sprintf(p, "%c%07u", get_property_decimal_char(), (unsigned int) t2 / 100); break;
    case 6: p += sprintf(p, "%c%06u", get_property_decimal_char(), (unsigned int) t2 / 1000); break;
    case 5: p += sprintf(p, "%c%05u", get_property_decimal_char(), (unsigned int) t2 / 10000); break;
    default:
    case 4: p += sprintf(p, "%c%04u", get_property_decimal_char(), (unsigned int) t2 / 100000); break;
    case 3: p += sprintf(p, "%c%03u", get_property_decimal_char(), (unsigned int) t2 / 1000000); break;
    case 2: p += sprintf(p, "%c%02u", get_property_decimal_char(), (unsigned int) t2 / 10000000); break;
    case 1: p += sprintf(p, "%c%01u", get_property_decimal_char(), (unsigned int) t2 / 100000000); break;
    case 0: break;
  }

  return (int)(p-oldp);
}

/*
* Divides number with ONE_BILLION and prints it on p usingn nofDecimal [0-9]
*/
int KvaLogWriter::printDecimalNumber(char *p, int64_t number, int nofDecimals) {
    bool isNegative = number < 0;
    if (isNegative) number = -number;
    uint64_t t1 = number / ONE_BILLION;
    uint64_t t2 = number % ONE_BILLION;
    return printDecimalNumber(p, t1, t2, isNegative, nofDecimals);
}

int KvaLogWriter::printDoubleNumber(char *p, double valdbl, int nofDecimals) {
  char *pd;
  char *oldp = p;

  switch (nofDecimals) {
    case 9: p += sprintf(p, "%.9f", valdbl); break;
    case 8: p += sprintf(p, "%.8f", valdbl); break;
    case 7: p += sprintf(p, "%.7f", valdbl); break;
    case 6: p += sprintf(p, "%.6f", valdbl); break;
    case 5: p += sprintf(p, "%.5f", valdbl); break;
    default:
    case 4: p += sprintf(p, "%.4f", valdbl); break;
    case 3: p += sprintf(p, "%.3f", valdbl); break;
    case 2: p += sprintf(p, "%.2f", valdbl); break;
    case 1: p += sprintf(p, "%.1f", valdbl); break;
    case 0: p += sprintf(p, "%.0f", valdbl); break;
  }

  if (nofDecimals) {
    for (pd = oldp; pd < p; pd++) {
      //PRINTF(("%c", *pd));
      if (*pd == '.') {
        *pd = get_property_decimal_char();
        break;
      }
    }
  }

  return (int)(p-oldp);
}

int KvaLogWriter::printEnumString(char *p, const char *str)
{
  if (!str) {
    return 0;
  }
  char *oldp = p;
  int cnt = snprintf(p, MAX_ELEM_LEN, "%s", str);
  if (cnt > 0) p += cnt;
  return (int)(p-oldp);
}

time_uint64 KvaLogWriter::setStartOfLogging(imLogData *ev)
{
  if (start_of_logging == 0 && (ILOG_TYPE_MESSAGE == ev->common.type || ILOG_TYPE_TRIGGER == ev->common.type || ILOG_TYPE_RTC == ev->common.type)) {
    start_of_logging =
      ev->common.nanos_since_1970;
    PRINTF(("Getting start_of_logging to %lu.%09lu\n",
      start_of_logging / ONE_BILLION, start_of_logging % ONE_BILLION));
  }
  return start_of_logging;
}

KvlcStatus KvaLogWriter::set_filename(const char *filename)
{
  PRINTF(("KvaLogWriter::set_filename(%s)", filename));
  const char *fname = filename;

  mFilename.assign(fname);
  mCurrentFilename.assign(fname);

  return kvlcOK;
}

const char * KvaLogWriter::get_filename()
{
  return mCurrentFilename.c_str();
}

// Should be moved to os_utils.h
KvlcStatus KvaLogWriter::file_status()
{
  if (mProperties.mOverwrite == mProperties.KVLC_FILE_DO_NOT_OVERWRITE) {
    struct stat statbuf;
    if (stat(get_filename(), &statbuf) == 0) {
      PRINTF(("KvaLogWriter::set_filename, file '%s' already exists.\n",
        get_filename()));
      return kvlcERR_FILE_EXISTS;
    }
  }
  return kvlcOK;

}

KvlcStatus KvaLogWriter::file_existence_status() {
  PRINTF(("KvaLogWriter::file_existence.\n"));
  if (mProperties.mSizeLimit == 0 && mProperties.mTimeLimit == 0) {
    return file_status();
  }
  else {
    std::string drive, dir, filename, extension;
    std::regex pattern;
    DIR* Dir;
    struct dirent *entry;

    // Split full filename path
    if (!os_splitpath(mFilename, drive, dir, filename, extension)) {
      PRINTF(("KvaLogWriter::file_existence_status: Invalid filename:'%s'\n", mFilename.c_str()));
      return kvlcERR_FILE_ERROR;
    }
    // Pattern to match with
    pattern = std::regex(filename + "(-part[0-9]+)." + extension);

    // Open directory and search for existing file conflicts
    Dir = opendir((drive + dir).c_str());
    if(Dir == NULL) {
      PRINTF(("KvaLogWriter::file_existence: Could not open directory %s.\n", (drive + dir).c_str()));
      return kvlcERR_FILE_ERROR;
    }
    while ((entry = readdir(Dir)) != NULL) {
      if (std::regex_match(entry->d_name, pattern)) {
        PRINTF(("KvaLogWriter::file_existence: file %s already exists.\n",
          (drive + dir + entry->d_name).c_str()));
        closedir(Dir);
        return kvlcERR_FILE_EXISTS;
      }
    }
    PRINTF(("KvaLogWriter::file_existence: No naming conflict found.\n"));
    closedir(Dir);
  }
  return kvlcOK;
}


KvlcStatus KvaLogWriter::split_file(void)
{
  mSplitPending = true;
  return open_new_file();
}

bool KvaLogWriter::split_on_time(time_int64 currentTime)
{
  if (mProperties.mTimeLimit == 0) return false;

  if (mFirstEvent) {
    mFirstTime = currentTime;
    mFirstEvent = false;
    return false;
  }

  if (!mSplitPending && (currentTime - mFirstTime) >= mProperties.mTimeLimit) {
    PRINTF(("KvaLogWriter::split_on_time: TRUE"));
    return true;
  }

  return false;
}

bool KvaLogWriter::split_on_size(uint64_t currentSize)
{
  if (mProperties.mSizeLimit != 0 && !mSplitPending && currentSize >= mProperties.mSizeLimit) {
    PRINTF(("KvaLogWriter::split_on_size: TRUE"));
    return true;
  }
  return false;
}

KvlcStatus KvaLogWriter::attach_file(const char* /* filename */)
{
  return kvlcERR_NOT_IMPLEMENTED;
}

KvlcStatus KvaLogWriter::set_property_value(unsigned int property, void *buf, size_t len)
{
  if (isOpened) return kvlcERR_CONVERTING;
  return mProperties.set_property_value(property, buf, len);
}

KvlcStatus KvaLogWriter::get_property_value(unsigned int property, void *buf, size_t len)
{
  return mProperties.get_property_value(property, buf, len);
}
