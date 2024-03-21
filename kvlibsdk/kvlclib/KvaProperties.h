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

#ifndef KVAPROPERTIES_H_
#define KVAPROPERTIES_H_

#include <time.h>
#include <vector>
#include "KvaConverterMisc.h"
#include "common_defs.h"

#define ON 1
#define OFF 0


class KvaProperties {
  public:

    // properties
    int mUseStartOfMeasurementOffset;
    int mUseStartOfMeasurementOffsetDefault;
    static const int DEFAULT_START_OF_MEASUREMENT_OFFSET = ON;

    int mUseFirstTriggerOffset;
    int mUseFirstTriggerOffsetDefault;
    static const int DEFAULT_FIRST_TRIGGER_OFFSET = OFF;

    int mUseOffsetOffset;
    int mUseOffsetOffsetDefault;
    static const int DEFAULT_USE_OFFSET_OFFSET = OFF;

    time_int64 mLogOffsetTime;
    time_int64 mLogOffsetTimeDefault;
    static const time_int64 DEFAULT_LOG_OFFSET_TIME = OFF;

    /**
    * Get and set the creation date/time used in file headers if not zero.
    * Seconds in standard UNIX format.
    **/
    time_int64 mLogCreationTime;
    time_int64 mLogCreationTimeDefault;
    static const time_int64 DEFAULT_LOG_CREATION_TIME = OFF;

     /**
     * 1 & 2 as default
     **/
    unsigned int mChannelMask;
    unsigned int mChannelMaskDefault;
    static const unsigned int DEFAULT_CHANNEL_ACTIVE_MASK = 0x0000001F;

    int mJ1939Active;
    int mJ1939ActiveDefault;
    static const int DEFAULT_HLP_J1939_ACTIVE = OFF;

    int mUseCalendarTimestamps;
    int mUseCalendarTimestampsDefault;
    static const int DEFAULT_CALENDAR_TIME_STAMPS_ACTIVE = OFF;

    int mWriteHeaderActive;
    int mWriteHeaderActiveDefault;
    static const int DEFAULT_WRITE_HEADER_ACTIVE = OFF;

    /**
    * Set and get separator char and decimal comma char for csv files.
    **/
    char mCsvSeparatorChar;
    char mCsvSeparatorCharDefault;
    static const char DEFAULT_CSV_SEPARATOR_CHAR = ',';

    char mDecimalSeparatorChar;
    char mDecimalSeparatorCharDefault;
    static const char DEFAULT_DECIMAL_SEPARATOR_CHAR = '.';

    /**
    * Should values propagate down in a csv-file?
    *  Example: fill_blanks = false
    *     1;;3;
    *     ;2;;4
    *     5;;;
    *  And with fill_blanks = true
    *     1;;3;           ; note that only values that has been known is shown
    *     1;2;3;4
    *     5;2;3;4
    **/
    int mFillBlanks;
    int mFillBlanksDefault;
    static const int DEFAULT_FILL_BLANKS = OFF;

    /**
    * Should the units be shown on an own row (made for csv signals, but
    * can be applied on all formats that uses a database).
    **/
    int mShowUnits;
    int mShowUnitsDefault;
    static const int DEFAULT_SHOW_UNITS = OFF;

    /**
    * Should pre-triggers be cropped
    **/
    int mCropPretrigger;
    int mCropPretriggerDefault;
    static const int DEFAULT_CROP_PRETRIGGER = OFF;

    /**
    * Should enumeration values (strings) be used
    **/
    int mUseEnumValues;
    int mUseEnumValuesDefault;
    static const int DEFAULT_ENUM_VALUES = OFF;

    /**
    * Should converted file size be limited in size, i.e. split
    * 0 == off; file size limit in bytes (user sets MB)
    **/
    uint64_t mSizeLimit;
    uint64_t mSizeLimitDefault;
    static const uint64_t DEFAULT_SIZE_LIMIT = OFF;
    unsigned long bytesToMB(uint64_t bytes);

    /**
    * Should converted file size be limited in time, i.e. split
    * 0 == off; message delta time limit in ns (user sets seconds)
    **/
    time_int64 mTimeLimit;
    time_int64 mTimeLimitDefault;
    static const time_int64 DEFAULT_TIME_LIMIT = OFF;
    unsigned long nsToSeconds(time_int64 ns);

    /**
    * Limit the number of databytes that a converter vill write. The default value
    * 64 bytes, but limiting the number to 8 bytes will reduce file size in PlainAcii
    * and CSV. Use kvlogcnv_get_status_data_truncated() to see if the converted frames
    * contained more data bytes and were truncated.
    **/
    int mLimitNumberOfDatabytes;
    int mLimitNumberOfDatabytesDefault;
    static const int MAX_LIMIT_NUMBER_OF_DATABYTES = 64;
    static const int DEFAULT_LIMIT_NUMBER_OF_DATABYTES = 64;

    int mCsvIdFormatHex;
    int mCsvIdFormatHexDefault;
    static const int DEFAULT_CSV_ID_FORMAT_HEX = OFF;

    int mCsvDataFormatHex;
    int mCsvDataFormatHexDefault;
    static const int DEFAULT_CSV_DATA_FORMAT_HEX = OFF;

    /**
    * Should the converter use name mangling to ensure that all
    * signals are converted (if such constraint exists).
    * Matlab has a maximum length of about 31 characters.
    **/

    int mUseMatlabNameMangling;
    int mUseMatlabNameManglingDefault;
    static const int DEFAULT_MATLAB_NAME_MANGLING = ON;

    /**
    * Number of decimals to print in the time stamp.
    **/
    int  mTimeDecimals;
    int  mTimeDecimalsDefault;
    static const int DEFAULT_TIME_DECIMALS = 9;

    /**
    * Number of decimals to print in the time stamp.
    **/
    int  mDataDecimals;
    int  mDataDecimalsDefault;
    static const int DEFAULT_DATA_DECIMALS = 4;

    int mUseFakeWallclockTime;
    int mUseFakeWallclockTimeDefault;
    time_t mFakedWallclockTime;

    /**
    * Merge lines, merges two lines if their signal values are equal
    **/
    int mMergeLines;
    int mMergeLinesDefault;
    static const int DEFAULT_MERGE_LINES = OFF;

    /**
    * Only print a line when the given column has been accessed.
    * -1 == off, otherwise the column number
    **/
    int mResampleColumn;
    int mResampleColumnDefault;
    static const int DEFAULT_RESAMPLE_COLUMN = -1;

    /**
    * Number of decimals to print in the calendar time stamp using ISO 8601
    * 0 == off, 1-9 allowed as well
    **/
    int mIso8601Decimals;
    int mIso8601DecimalsDefault;
    static const int DEFAULT_ISO8601_DECIMALS = OFF;

    /**
    * Get and set the version of the file format
    **/
    int mFileFormatVersion;
    int mFileFormatVersionDefault;
    static const int DEFAULT_FILE_FORMAT_VERSION = 0;

    /**
    * Counter, include a counter
    **/
    int mShowCounter;
    int mShowCounterDefault;
    static const int DEFAULT_SHOW_COUNTER = false;

    /**
    * If overwrite is
    *   KVLC_FILE_OVERWRITE it overwrites an existing file if it exists,
    *   KVLC_FILE_DO_NOT_OVERWRITE does not overwrite an existing file.
    */
    int mOverwrite;
    int mOverwriteDefault;
    static const int KVLC_FILE_DO_NOT_OVERWRITE = 0;
    static const int KVLC_FILE_OVERWRITE        = 1;

    /**
    * If timezone is
    *   KVLC_TZ_UTC , all absolute and calendar timestamps will report UTC (GPS) time
    *   KVLC_TZ_LOCALTIME , all absolute and calendar timestamps will report local time
    */
    int mTimezone;
    int mTimezoneDefault;
    static const int KVLC_TZ_LOCALTIME = 0;
    static const int KVLC_TZ_UTC       = 1;

   /**
    *
    **/
    int mShowFullyQualifiedNames;
    int mShowFullyQualifiedNamesDefault;

    int mCompressionLevel;
    int mCompressionLevelDefault;

    static const int DEFAULT_FULLY_QUALIFIED_NAMES = false;

    static const int DEFAULT_COMPRESSION_LEVEL = -1; //Z_DEFAULT_COMPRESSION;


    unsigned int mSampleAndHoldTimestep;
    unsigned int mSampleAndHoldTimestepDefault;
    static const unsigned int DEFAULT_SAMPLE_AND_HOLD_TIMESTEP = 0;

    static const int TYPE_CONVERSION_ERROR = -1;
    static const int SET_FEATURE_VALUE         = 0;
    static const int GET_FEATURE_VALUE         = 1;
    static const int GET_DEFAULT_FEATURE_VALUE = 2;
    static const int SET_DEFAULT_FEATURE_VALUE = 3;
    KvlcStatus handle_property(int op, unsigned int property, void *buf, size_t len);


    KvaProperties();

    virtual ~KvaProperties();

    KvlcStatus set_property_value (unsigned int property, void *buf, size_t len);

    KvlcStatus get_property_value (unsigned int property, void *buf, size_t len);

    KvlcStatus get_property_default_value (unsigned int property, void *buf, size_t len);

    KvlcStatus set_property_default_value (unsigned int property, void *buf, size_t len);

    KvlcStatus set_property_and_default_value (unsigned int property, void *buf, size_t len);

    // Return a feature value if the buffer length matches the type length
    template <typename ValueType> ValueType buffer_to_value(void *value, size_t len)
    {
      if (!value || sizeof(ValueType) != len) {
        throw(kvlcERR_TYPE_MISMATCH);
      }
      return *(ValueType*)value;
    }

    // Copy a feature value to buffer if the buffer length matches the type length
    template <typename ValueType> void copy_value_to_buffer(void *buf, size_t len, ValueType value)
    {
      if (!buf || sizeof(ValueType) != len) {
        throw(kvlcERR_TYPE_MISMATCH);
      }
      *(ValueType*)buf = value;
    }

    // Set, get or return the default feature value
    template <typename ValueType> void handle_value(int op, ValueType *p, ValueType *std, void *buf, size_t len)
    {
      if (!p || !buf || sizeof(ValueType) != len) {
        throw(kvlcERR_TYPE_MISMATCH);
      }
      switch(op) {
      case SET_FEATURE_VALUE:
        *p = buffer_to_value<ValueType>(buf, len);
        break;

      case GET_FEATURE_VALUE:
        copy_value_to_buffer(buf, len, *p);
        break;

      case GET_DEFAULT_FEATURE_VALUE:
        copy_value_to_buffer(buf, len, *std);
        break;

      case SET_DEFAULT_FEATURE_VALUE:
        *std = buffer_to_value<ValueType>(buf, len);
        break;

      }
    }

};

//#include "KvaWriterMaker.h"

#endif /*KVAPROPERTIES_H_*/
