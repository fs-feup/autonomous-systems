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
#include <string.h>
#include <math.h>
#include "KvaProperties.h"
#include "common_defs.h"
#include "TimeConv.h"
#include "kvdebug.h"



#define BYTES_TO_MB(x) (x/(uint64_t)1000000LL)
#define MB_TO_BYTES(x) (x*(uint64_t)1000000LL)
#define NS_TO_SECONDS(x) (x/(time_int64)ONE_BILLION)
#define SECONDS_TO_NS(x) (x*(time_int64)ONE_BILLION)


KvaProperties::KvaProperties()
{
  PRINTF(("KvaProperties::KvaProperties()\n"));

  // properties
  mUseStartOfMeasurementOffset        = DEFAULT_START_OF_MEASUREMENT_OFFSET;
  mUseStartOfMeasurementOffsetDefault = DEFAULT_START_OF_MEASUREMENT_OFFSET;

  mUseFirstTriggerOffset        = DEFAULT_FIRST_TRIGGER_OFFSET;
  mUseFirstTriggerOffsetDefault = DEFAULT_FIRST_TRIGGER_OFFSET;

  mUseOffsetOffset        = DEFAULT_USE_OFFSET_OFFSET;
  mUseOffsetOffsetDefault = DEFAULT_USE_OFFSET_OFFSET;

  mLogOffsetTime        = DEFAULT_LOG_OFFSET_TIME;
  mLogOffsetTimeDefault = DEFAULT_LOG_OFFSET_TIME;

  mLogCreationTime        = DEFAULT_LOG_CREATION_TIME;
  mLogCreationTimeDefault = DEFAULT_LOG_CREATION_TIME;

  mChannelMask        = DEFAULT_CHANNEL_ACTIVE_MASK;
  mChannelMaskDefault = DEFAULT_CHANNEL_ACTIVE_MASK;

  mJ1939Active        = DEFAULT_HLP_J1939_ACTIVE;
  mJ1939ActiveDefault = DEFAULT_HLP_J1939_ACTIVE;

  mUseCalendarTimestamps        = DEFAULT_CALENDAR_TIME_STAMPS_ACTIVE;
  mUseCalendarTimestampsDefault = DEFAULT_CALENDAR_TIME_STAMPS_ACTIVE;

  mWriteHeaderActive        = DEFAULT_WRITE_HEADER_ACTIVE;
  mWriteHeaderActiveDefault = DEFAULT_WRITE_HEADER_ACTIVE;

  mCsvSeparatorChar          = DEFAULT_CSV_SEPARATOR_CHAR;
  mCsvSeparatorCharDefault   = DEFAULT_CSV_SEPARATOR_CHAR;

  mDecimalSeparatorChar        = DEFAULT_DECIMAL_SEPARATOR_CHAR;
  mDecimalSeparatorCharDefault = DEFAULT_DECIMAL_SEPARATOR_CHAR;

  mCsvIdFormatHex            = DEFAULT_CSV_ID_FORMAT_HEX;
  mCsvIdFormatHexDefault     = DEFAULT_CSV_ID_FORMAT_HEX;

  mCsvDataFormatHex          = DEFAULT_CSV_DATA_FORMAT_HEX;
  mCsvDataFormatHexDefault   = DEFAULT_CSV_DATA_FORMAT_HEX;

  mUseMatlabNameMangling           = DEFAULT_MATLAB_NAME_MANGLING;
  mUseMatlabNameManglingDefault = DEFAULT_MATLAB_NAME_MANGLING;

  mTimeDecimals              = DEFAULT_TIME_DECIMALS;
  mTimeDecimalsDefault       = DEFAULT_TIME_DECIMALS;

  mDataDecimals              = DEFAULT_DATA_DECIMALS;
  mDataDecimalsDefault       = DEFAULT_DATA_DECIMALS;

  mFillBlanks               = DEFAULT_FILL_BLANKS;
  mFillBlanksDefault        = DEFAULT_FILL_BLANKS;

  mShowUnits                = DEFAULT_SHOW_UNITS;
  mShowUnitsDefault         = DEFAULT_SHOW_UNITS;

 // 0 == off, 1-9 allowed as well
  mIso8601Decimals           = DEFAULT_ISO8601_DECIMALS;
  mIso8601DecimalsDefault    = DEFAULT_ISO8601_DECIMALS;

  mMergeLines                = DEFAULT_MERGE_LINES;
  mMergeLinesDefault         = DEFAULT_MERGE_LINES;

  // -1 == off, otherwise the column number
  mResampleColumn            = DEFAULT_RESAMPLE_COLUMN;
  mResampleColumnDefault     = DEFAULT_RESAMPLE_COLUMN;

  mShowCounter             = DEFAULT_SHOW_COUNTER;
  mShowCounterDefault      = DEFAULT_SHOW_COUNTER;

  mCropPretrigger           = DEFAULT_CROP_PRETRIGGER;
  mCropPretriggerDefault    = DEFAULT_CROP_PRETRIGGER;

  mUseEnumValues               = DEFAULT_ENUM_VALUES;
  mUseEnumValuesDefault        = DEFAULT_ENUM_VALUES;

   // 0 == off; file size limit in bytes (user sets MB)
  mSizeLimit                = DEFAULT_SIZE_LIMIT;
  mSizeLimitDefault         = DEFAULT_SIZE_LIMIT;

  // 0 == off; message delta time limit in ns (user sets seconds)
  mTimeLimit                = DEFAULT_TIME_LIMIT;
  mTimeLimitDefault         = DEFAULT_TIME_LIMIT;

  mLimitNumberOfDatabytes        = DEFAULT_LIMIT_NUMBER_OF_DATABYTES;
  mLimitNumberOfDatabytesDefault = DEFAULT_LIMIT_NUMBER_OF_DATABYTES;

  mFileFormatVersion        = DEFAULT_FILE_FORMAT_VERSION;
  mFileFormatVersionDefault = DEFAULT_FILE_FORMAT_VERSION;

  mOverwrite        = KVLC_FILE_DO_NOT_OVERWRITE;
  mOverwriteDefault = KVLC_FILE_DO_NOT_OVERWRITE;

  mTimezone        = KVLC_TZ_LOCALTIME;
  mTimezoneDefault = KVLC_TZ_LOCALTIME;

  mShowFullyQualifiedNames = DEFAULT_FULLY_QUALIFIED_NAMES;
  mShowFullyQualifiedNamesDefault = DEFAULT_FULLY_QUALIFIED_NAMES;

  mCompressionLevel = DEFAULT_COMPRESSION_LEVEL;
  mCompressionLevelDefault = DEFAULT_COMPRESSION_LEVEL;

  mSampleAndHoldTimestep = DEFAULT_SAMPLE_AND_HOLD_TIMESTEP;
  mSampleAndHoldTimestepDefault = DEFAULT_SAMPLE_AND_HOLD_TIMESTEP;
}

KvaProperties::~KvaProperties()
{
  PRINTF(("KvaProperties::~KvaProperties()\n"));
}


unsigned long KvaProperties::bytesToMB(uint64_t bytes)
{
  return (unsigned long) (bytes/(uint64_t)1000000LL);
}

unsigned long KvaProperties::nsToSeconds(time_int64 ns)
{
  return (unsigned long) (ns/(time_int64)ONE_BILLION);
}

KvlcStatus KvaProperties::set_property_value(unsigned int property, void *buf, size_t len)
{
  return handle_property(SET_FEATURE_VALUE, property, buf, len);
}

KvlcStatus KvaProperties::get_property_value(unsigned int property, void *buf, size_t len)
{
  return handle_property(GET_FEATURE_VALUE, property, buf, len);
}

KvlcStatus KvaProperties::get_property_default_value(unsigned int property, void *buf, size_t len)
{
  return handle_property(GET_DEFAULT_FEATURE_VALUE, property, buf, len);
}

KvlcStatus KvaProperties::set_property_default_value (unsigned int property, void *buf, size_t len)
{
  return handle_property(SET_DEFAULT_FEATURE_VALUE, property, buf, len);
}

KvlcStatus KvaProperties::set_property_and_default_value (unsigned int property, void *buf, size_t len)
{
  KvlcStatus status = handle_property(SET_DEFAULT_FEATURE_VALUE, property, buf, len);
  if (status != kvlcOK) return status;
  return handle_property(SET_FEATURE_VALUE, property, buf, len);
}


KvlcStatus KvaProperties::handle_property(int op, unsigned int property, void *buf, size_t len)
{
  KvlcStatus status = kvlcOK;

  try {
    switch(property) {

    case KVLC_PROPERTY_START_OF_MEASUREMENT:
    {
      if (op == SET_FEATURE_VALUE) {
        mUseStartOfMeasurementOffset = buffer_to_value<int>(buf, len);
        if (mUseStartOfMeasurementOffset) {
          // Only one of the three offset-methods could be set
          mUseFirstTriggerOffset = OFF;
          mUseOffsetOffset = OFF;
        }
      }
      else {
        handle_value(op, &mUseStartOfMeasurementOffset, &mUseStartOfMeasurementOffsetDefault, buf, len);
      }
      break;
    }

    case KVLC_PROPERTY_FIRST_TRIGGER:
    {
      if (op == SET_FEATURE_VALUE) {
        mUseFirstTriggerOffset = buffer_to_value<int>(buf, len);
        if (mUseFirstTriggerOffset) {
          // Only one of the three offset-methods could be set
          mUseStartOfMeasurementOffset = OFF;
          mUseOffsetOffset = OFF;
        }
      }
      else {
        handle_value(op, &mUseFirstTriggerOffset,
          &mUseFirstTriggerOffsetDefault, buf, len);
      }
      break;
    }

    case KVLC_PROPERTY_USE_OFFSET:
    {
      if (op == SET_FEATURE_VALUE) {
        mUseOffsetOffset = buffer_to_value<int>(buf, len);
        if (mUseOffsetOffset) {
          // Only one of the three offset-methods could be set
          mUseFirstTriggerOffset = OFF;
          mUseStartOfMeasurementOffset = OFF;
        }
      }
      else {
        handle_value(op, &mUseOffsetOffset,
          &mUseOffsetOffsetDefault, buf, len);
      }
      break;
    }

    case KVLC_PROPERTY_OFFSET:
    {
      handle_value(op, &mLogOffsetTime,
        &mLogOffsetTimeDefault, buf, len);
      break;
    }

    case KVLC_PROPERTY_CHANNEL_MASK:
    {
      handle_value(op, &mChannelMask,
        &mChannelMaskDefault, buf, len);
      break;
    }

    case KVLC_PROPERTY_HLP_J1939:
    {
      handle_value(op, &mJ1939Active,
        &mJ1939ActiveDefault, buf, len);
      break;
    }

    case KVLC_PROPERTY_CALENDAR_TIME_STAMPS:
    {
      handle_value(op, &mUseCalendarTimestamps,
        &mUseCalendarTimestampsDefault, buf, len);
      break;
    }

    case KVLC_PROPERTY_WRITE_HEADER:
    {
      handle_value(op, &mWriteHeaderActive,
        &mWriteHeaderActiveDefault, buf, len);
      break;
    }

    case KVLC_PROPERTY_SEPARATOR_CHAR:
    {
      handle_value(op, &mCsvSeparatorChar,
        &mCsvSeparatorCharDefault, buf, len);
      break;
    }

    case KVLC_PROPERTY_DECIMAL_CHAR:
    {
      handle_value(op, &mDecimalSeparatorChar,
        &mDecimalSeparatorCharDefault, buf, len);
      break;
    }

    case KVLC_PROPERTY_ID_IN_HEX:
    {
      handle_value(op, &mCsvIdFormatHex,
        &mCsvIdFormatHexDefault, buf, len);
      break;
    }

    case KVLC_PROPERTY_DATA_IN_HEX:
    {
      handle_value(op, &mCsvDataFormatHex,
        &mCsvDataFormatHexDefault, buf, len);
      break;
    }

    case KVLC_PROPERTY_NUMBER_OF_TIME_DECIMALS :
    {
      handle_value(op, &mTimeDecimals,
        &mTimeDecimalsDefault, buf, len);
      break;
    }

    case KVLC_PROPERTY_NUMBER_OF_DATA_DECIMALS :
    {
      handle_value(op, &mDataDecimals,
        &mDataDecimalsDefault, buf, len);
      break;
    }

    case KVLC_PROPERTY_NAME_MANGLING:
    {
      handle_value(op, &mUseMatlabNameMangling,
        &mUseMatlabNameManglingDefault, buf, len);
      break;
    }

    case KVLC_PROPERTY_FILL_BLANKS:
    {
      handle_value(op, &mFillBlanks,
        &mFillBlanksDefault, buf, len);
      break;
    }

    case KVLC_PROPERTY_SHOW_UNITS:
    {
      handle_value(op, &mShowUnits,
        &mShowUnitsDefault, buf, len);
      break;
    }

    case KVLC_PROPERTY_ISO8601_DECIMALS:
    {
      handle_value(op, &mIso8601Decimals,
        &mIso8601DecimalsDefault, buf, len);
      break;
    }

    case KVLC_PROPERTY_MERGE_LINES:
    {
      handle_value(op, &mMergeLines,
        &mMergeLinesDefault, buf, len);
      break;
    }

    case KVLC_PROPERTY_RESAMPLE_COLUMN:
    {
      handle_value(op, &mResampleColumn,
        &mResampleColumnDefault, buf, len);
      break;
    }

    case KVLC_PROPERTY_VERSION:
    {
      handle_value(op, &mFileFormatVersion,
        &mFileFormatVersionDefault, buf, len);
      break;
    }

    case KVLC_PROPERTY_SHOW_COUNTER:
    {
      handle_value(op, &mShowCounter,
        &mShowCounterDefault, buf, len);
      break;
    }

    case KVLC_PROPERTY_CROP_PRETRIGGER:
    {
      handle_value(op, &mCropPretrigger,
        &mCropPretriggerDefault, buf, len);
      break;
    }

    case KVLC_PROPERTY_ENUM_VALUES:
    {
      handle_value(op, &mUseEnumValues,
        &mUseEnumValuesDefault, buf, len);
      break;
    }

    case KVLC_PROPERTY_SIZE_LIMIT:
    {
      uint64_t tmp = MB_TO_BYTES(buffer_to_value<unsigned int>(buf, len));
      handle_value(op, &mSizeLimit,
        &mSizeLimitDefault, &tmp, sizeof(tmp));
      if (op != SET_FEATURE_VALUE) {
        copy_value_to_buffer(buf, len, (unsigned int) BYTES_TO_MB(tmp));
      }
      break;
    }

    case KVLC_PROPERTY_TIME_LIMIT:
    {
      uint64_t tmp = SECONDS_TO_NS(buffer_to_value<unsigned int>(buf, len));
      handle_value(op, &mTimeLimit,
        &mTimeLimitDefault, &tmp, sizeof(tmp));
      if (op != SET_FEATURE_VALUE) {
        copy_value_to_buffer(buf, len, (unsigned int) NS_TO_SECONDS(tmp));
      }
      break;
    }

    case KVLC_PROPERTY_LIMIT_DATA_BYTES:
    {
       if (op == SET_FEATURE_VALUE) {
          int value = buffer_to_value<int>(buf, len);
          if (value > MAX_LIMIT_NUMBER_OF_DATABYTES) {
            return kvlcERR_PARAM;
          }
          mLimitNumberOfDatabytes = value;
       }
       else {
         handle_value(op, &mLimitNumberOfDatabytes,
          &mLimitNumberOfDatabytesDefault, buf, len);
       }
      break;
    }

    case KVLC_PROPERTY_CREATION_DATE:
    {
      handle_value(op, &mLogCreationTime,
        &mLogCreationTimeDefault, buf, len);
      break;
    }

    case KVLC_PROPERTY_OVERWRITE:
    {
      handle_value(op, &mOverwrite,
        &mOverwriteDefault, buf, len);
      break;
    }

    case KVLC_PROPERTY_TIMEZONE:
    {
      handle_value(op, &mTimezone,
        &mTimezoneDefault, buf, len);
      break;
    }

    case KVLC_PROPERTY_FULLY_QUALIFIED_NAMES:
    {
      handle_value(op, &mShowFullyQualifiedNames,
                   &mShowFullyQualifiedNamesDefault, buf, len);
      break;
    }

    case KVLC_PROPERTY_COMPRESSION_LEVEL:
    {
      if (op == SET_FEATURE_VALUE){
        int value = buffer_to_value<int>(buf, len);
        if (value < -1 || value > 9) {
          PRINTF(("'%d' is not a valid value (-1..9) for KVLC_PROPERTY_COMPRESSION_LEVEL\n", value));
          return kvlcERR_PARAM;
        }
        mCompressionLevel = value;

      }
      else {
        handle_value(op, &mCompressionLevel,
                     &mCompressionLevelDefault, buf, len);
      }
      break;
    }

    case KVLC_PROPERTY_SAMPLE_AND_HOLD_TIMESTEP:
    {
      handle_value(op, &mSampleAndHoldTimestep,
                   &mSampleAndHoldTimestepDefault, buf, len);
      break;
    }
    default:
      status = kvlcERR_NOT_IMPLEMENTED;
    }

  } catch (KvlcStatus err) {
    PRINTF(("Error: KvaLogWriter::handle_property(%d, %u, %p, %lu) failed with status %d.",
      op, property, buf, len, err));
    status = err;
  }
  return status;
}

