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
#include "kvaConverter.h"
#include "kvlclib.h"
#include "canlib_version.h"
#include "kvdebug.h"

#if defined(DEBUG) && defined(CRTMEMCHECK)
#  include <crtdbg.h>
#endif

#define INVALID_HANDLE_VALUE NULL
static void strncpy_s(char *str, unsigned int /*len*/, char* buf, unsigned int bytesRead)
{
  strncpy(str, buf, bytesRead);
}


static const unsigned int MAX_OPEN_HANDLES = 128;
static const KvlcHandle FREE_HANDLE = 0;
static KvlcHandle OpenConverterHandles[MAX_OPEN_HANDLES] = {FREE_HANDLE};

#define MAX_WRITER_DESCRIPTION_SIZE 2048

// ===========================================================================
static bool isInvalid (KvlcHandle h)
{
  if ( 0 == h || INVALID_HANDLE_VALUE == h ) return true;

  for (unsigned int k = 0; k < MAX_OPEN_HANDLES; k++) {
    if ( h == OpenConverterHandles[k] ) {
      return false;
    }
  }
  return true;
}

// ===========================================================================
static bool convHandleAvailable (void)
{
  for (unsigned int k = 0; k < MAX_OPEN_HANDLES; k++) {
    if ( FREE_HANDLE == OpenConverterHandles[k] ) {
      return true;
    }
  }
  return false;
}

// ===========================================================================
static void setInUse (KvlcHandle h)
{
  for (unsigned int k = 0; k < MAX_OPEN_HANDLES; k++) {
    if ( FREE_HANDLE == OpenConverterHandles[k] ) {
      OpenConverterHandles[k] = h;
      return;
    }
  }
}

// ===========================================================================
static void clearInUse (KvlcHandle h)
{
  for (unsigned int k = 0; k < MAX_OPEN_HANDLES; k++) {
    if ( h == OpenConverterHandles[k] ) {
      OpenConverterHandles[k] = FREE_HANDLE;
      return;
    }
  }
}


//===========================================================================
KvlcStatus WINAPI kvlcGetVersion (
    unsigned int *major, unsigned int *minor, unsigned int *build)
{
  if (!major || !minor || !build) {
    return kvlcERR_PARAM;
  }
  *major = CANLIB_MAJOR_VERSION;
  *minor = CANLIB_MINOR_VERSION;
  *build = 0;
  return kvlcOK;
}

//===========================================================================
KvlcStatus WINAPI kvlcCreateConverter(KvlcHandle *handle, const char *filename, int format)
{
  KvlcStatus stat;
  if (!handle) {
    PRINTF(("kvlcCreateConverter: handle = null\n"));
    return kvlcERR_PARAM;
  }

  *handle = INVALID_HANDLE_VALUE;

  if ( !convHandleAvailable() ) {
    return kvlcERR_NO_FREE_HANDLES;
  }

#if defined(DEBUG) && defined(CRTMEMCHECK)
  int tmpDbgFlag;

  _CrtSetReportMode( _CRT_ERROR, _CRTDBG_MODE_DEBUG );
  /*
    * Set the debug-heap flag to keep freed blocks in the
    * heap's linked list - This will allow us to catch any
    * inadvertent use of freed memory
    */
  tmpDbgFlag = _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG);
  tmpDbgFlag |= _CRTDBG_CHECK_ALWAYS_DF;
  //tmpDbgFlag |= _CRTDBG_CHECK_CRT_DF;


  tmpDbgFlag |= _CRTDBG_DELAY_FREE_MEM_DF;
  tmpDbgFlag |= _CRTDBG_LEAK_CHECK_DF;
  _CrtSetDbgFlag(tmpDbgFlag);
#endif

  try {
    KvaConverter *conv = new KvaConverter();
    if (conv) {
      PRINTF(("Created a new KvaConverter @ 0x%p\n", conv));

      stat = conv->set_output_format(format);
      if (stat != kvlcOK) {
        delete conv;
        return stat;
      }

      stat = conv->set_output_file(filename);
      if (stat != kvlcOK) {
        delete conv;
        return stat;
      }
      *handle = (void *)conv;
      setInUse(*handle);
    }
    else {
      return kvlcERR_NULL_POINTER;
    }
  }
  catch (...) { // use more fine grained exception catching?
    PRINTF(("kvlcCreateConverter, exception caught\n"));
    return kvlcERR_INTERNAL_ERROR;
  }

  return stat;
}


//===========================================================================
KvlcStatus WINAPI kvlcDeleteConverter(KvlcHandle handle)
{
  PRINTF(("kvlcDeleteConverter\n"));
  if (isInvalid(handle)) {
    return kvlcERR_PARAM;
  }

  KvaConverter *conv = static_cast<KvaConverter*>(handle);
  if (conv) {
    // Flush any remaing values
    PRINTF(("Will flush any pushed events first.\n"));
    conv->flush_events();
    PRINTF(("Will delete the KvaConverter @ 0x%p\n", conv));
    try {
      delete conv;
    }
    catch(...) {
      PRINTF(("kvlcDeleteConverter failed 1 (exception thrown)\n\n"));
      clearInUse(handle);
      return kvlcERR_INTERNAL_ERROR;
    }
    PRINTF(("Deleted the KvaConverter @ 0x%p\n", conv));
    clearInUse(handle);
    return kvlcOK;
  }
  else
  {
    PRINTF(("kvlcDeleteConverter failed 2 (was null)\n"));
    return kvlcERR_NULL_POINTER;
  }
}

//===========================================================================
KvlcStatus WINAPI kvlcEventCount (
    KvlcHandle handle,
    unsigned int *count)
{
  if (isInvalid(handle) || !count) {
    return kvlcERR_PARAM;
  }
  KvaConverter *conv = static_cast<KvaConverter*>(handle);

  if (conv) {
    uint64 tmp = 0;
    KvlcStatus stat = conv->row_count(&tmp);
    *count = (unsigned int) tmp;

    if (tmp != (uint64)*count) {
      return kvlcERR_RESULT_TOO_BIG;
    }
    return stat;
  }
  else {
    return kvlcERR_NULL_POINTER;
  }
}

KvlcStatus WINAPI kvlcEventCountEx (
    KvlcHandle handle,
    int64 *count)
{
  if (isInvalid(handle) || !count) {
    return kvlcERR_PARAM;
  }
  KvaConverter *conv = static_cast<KvaConverter*>(handle);

  if (conv) {
    KvlcStatus stat = conv->row_count((uint64 *)count);
    return stat;
  }
  else {
    return kvlcERR_NULL_POINTER;
  }
}

//===========================================================================
KvlcStatus WINAPI kvlcConvertEvent(KvlcHandle handle)
{
  if (isInvalid(handle)) {
    return kvlcERR_PARAM;
  }

  KvaConverter *conv = static_cast<KvaConverter*>(handle);
  if (conv) {
    return conv->convert_event();
  }
  else {
    return kvlcERR_NULL_POINTER;
  }

}


//===========================================================================
KvlcStatus WINAPI kvlcGetOutputFilename(
    KvlcHandle handle,
    char *filename,
    int len)
{
  if (isInvalid(handle)) {
    return kvlcERR_PARAM;
  }
  KvaConverter *conv = static_cast<KvaConverter*>(handle);

  if (conv) {
    return conv->get_output_filename(filename, (unsigned int)len);
  }
  else {
    return kvlcERR_NULL_POINTER;
  }
}

//===========================================================================
KvlcStatus WINAPI kvlcIsOutputFilenameNew(
    KvlcHandle handle,
    int *updated)
{
  if (isInvalid(handle) || !updated) {
    return kvlcERR_PARAM;
  }
  KvaConverter *conv = static_cast<KvaConverter*>(handle);

  if (conv) {
    bool tmp;
    KvlcStatus stat = conv->get_output_filename_status(&tmp);
    *updated = tmp?1:0;
    return stat;
  }
  else {
    return kvlcERR_NULL_POINTER;
  }
}

//===========================================================================
KvlcStatus WINAPI kvlcFeedSelectFormat (
    KvlcHandle handle,
    int format)
{
  if (isInvalid(handle)) {
    return kvlcERR_PARAM;
  }
  KvaConverter *conv = static_cast<KvaConverter*>(handle);

  if (conv) {
    return conv->select_reader_format(format);
  }
  else {
    return kvlcERR_NULL_POINTER;
  }

}

//===========================================================================
KvlcStatus WINAPI kvlcFeedNextFile (KvlcHandle handle)
{
  if (isInvalid(handle)) {
    return kvlcERR_PARAM;
  }
  KvaConverter *conv = static_cast<KvaConverter *>(handle);

  if (conv) {
    return conv->next_file();
  }
  else {
    return kvlcERR_NULL_POINTER;
  }
}

//===========================================================================
KvlcStatus WINAPI kvlcNextInputFile(
    KvlcHandle handle,
    const char *filename)
{
  if (isInvalid(handle) || !filename) {
    return kvlcERR_PARAM;
  }
  KvaConverter *conv = static_cast<KvaConverter *>(handle);

  if (conv) {
    return conv->next_input_file(filename);
  }
  else {
    return kvlcERR_NULL_POINTER;
  }
}

//===========================================================================
KvlcStatus WINAPI kvlcSetInputFile(
    KvlcHandle handle,
    const char *filename,
    int format)
{
  if (isInvalid(handle) || !filename) {
    return kvlcERR_PARAM;
  }
  KvaConverter *conv = static_cast<KvaConverter*>(handle);

  if (conv) {
    return conv->set_input_file(filename, format);
  }
  else {
    return kvlcERR_NULL_POINTER;
  }
}

//===========================================================================
KvlcStatus WINAPI kvlcFeedLogEvent (
    KvlcHandle handle,
    void *event)
{
  if (isInvalid(handle) || !event) {
    return kvlcERR_PARAM;
  }

  KvaConverter *conv = static_cast<KvaConverter*>(handle);

  if (conv) {
    return conv->feed_event(event);
  }
  return kvlcERR_NULL_POINTER;
}

//===========================================================================
KvlcStatus WINAPI kvlcIsOverrunActive (
    KvlcHandle handle,
    int *overrun)
{
  if (isInvalid(handle) || !overrun) {
    return kvlcERR_PARAM;
  }
  KvaConverter *conv = static_cast<KvaConverter*>(handle);

  if (conv) {
    bool tmp;
    KvlcStatus stat = conv->get_status_overrun(&tmp);
    *overrun = tmp?1:0;
    return stat;
  }
  return kvlcERR_NULL_POINTER;
}

//===========================================================================
KvlcStatus WINAPI kvlcResetOverrunActive(KvlcHandle handle)
{
  if (isInvalid(handle)) {
    return kvlcERR_PARAM;
  }
  KvaConverter *conv = static_cast<KvaConverter*>(handle);

  if (conv) {
    return conv->reset_status_overrun();
  }
  return kvlcERR_NULL_POINTER;
}

//===========================================================================
KvlcStatus WINAPI kvlcIsDataTruncated (
    KvlcHandle handle,
    int *truncated)
{
  if (isInvalid(handle) || !truncated) {
    return kvlcERR_PARAM;
  }
  KvaConverter *conv = static_cast<KvaConverter*>(handle);

  if (conv) {
    bool tmp;
    KvlcStatus stat = conv->get_status_data_truncated(&tmp);
    *truncated = tmp?1:0;
    return stat;
  }
  return kvlcERR_NULL_POINTER;
}

//===========================================================================
KvlcStatus WINAPI kvlcResetDataTruncated(KvlcHandle handle)
{
  if (isInvalid(handle)) {
    return kvlcERR_PARAM;
  }
  KvaConverter *conv = static_cast<KvaConverter*>(handle);

  if (conv) {
    return conv->reset_status_data_truncated();
  }
  return kvlcERR_NULL_POINTER;
}

//===========================================================================
KvlcStatus WINAPI kvlcIsDlcMismatch(
	KvlcHandle handle,
	int *mismatch)
{
	if (isInvalid(handle) || !mismatch) {
		return kvlcERR_PARAM;
	}
	KvaConverter *conv = static_cast<KvaConverter*>(handle);

	if (conv) {
		bool tmp;
		KvlcStatus stat = conv->get_status_dlc_mismatch(&tmp);
		*mismatch = tmp ? 1 : 0;
		return stat;
	}
	return kvlcERR_NULL_POINTER;
}

//===========================================================================
KvlcStatus WINAPI kvlcResetDlcMismatch(KvlcHandle handle)
{
	if (isInvalid(handle)) {
		return kvlcERR_PARAM;
	}
	KvaConverter *conv = static_cast<KvaConverter*>(handle);

	if (conv) {
		return conv->reset_status_dlc_mismatch();
	}
	return kvlcERR_NULL_POINTER;
}

//===========================================================================
KvlcStatus WINAPI kvlcGetDlcMismatchList(KvlcHandle handle, unsigned int* MsgIds, unsigned int* MsgDlc, unsigned int* MsgOccurance, unsigned int* length)
{
	if (isInvalid(handle)) {
		return kvlcERR_PARAM;
	}
	KvaConverter *conv = static_cast<KvaConverter*>(handle);

	if (conv) {
		return conv->get_list_dlc_mismatch(MsgIds, MsgDlc, MsgOccurance, length);
	}
	return kvlcERR_NULL_POINTER;
}

//==============================================================================
KvlcStatus WINAPI kvlcAddDatabaseFile(KvlcHandle handle, const char *filename,
                                      unsigned int channelMask)
{
  if (isInvalid(handle) || !filename) {
    return kvlcERR_PARAM;
  }

  KvaConverter *conv = static_cast<KvaConverter*>(handle);

  if (conv) {
    int format;
    int supported;
    KvlcStatus status = conv->get_output_format(&format);
    if (status != kvlcOK) return status;
    status = kvlcIsPropertySupported(format, KVLC_PROPERTY_SIGNAL_BASED, &supported);
    if (status != kvlcOK) return status;
    if (!supported) return kvlcERR_NOT_IMPLEMENTED;
    return conv->add_database_file(filename, channelMask);
  }
  PRINTF(("kvlcAddDatabaseFile, conv == NULL\n"));
  return kvlcERR_NULL_POINTER;
}

//==============================================================================
KvlcStatus WINAPI kvlcAddDatabase(KvlcHandle handle, KvaDbHnd dbHandle,
                                  unsigned int channelMask)
{
  if (isInvalid(handle) || !dbHandle) {
    return kvlcERR_PARAM;
  }

  KvaConverter *conv = static_cast<KvaConverter*>(handle);

  if (conv) {
    int format;
    int supported;
    KvlcStatus status = conv->get_output_format(&format);
    if (status != kvlcOK) return status;
    status = kvlcIsPropertySupported(format, KVLC_PROPERTY_SIGNAL_BASED, &supported);
    if (status != kvlcOK) return status;
    if (!supported) return kvlcERR_NOT_IMPLEMENTED;
    return conv->add_database_handle(dbHandle, channelMask);
  }
  PRINTF(("kvlcAddDatabase, conv == NULL\n"));
  return kvlcERR_NULL_POINTER;
}

//==============================================================================
KvlcStatus WINAPI kvlcGetFirstWriterFormat(int *format)
{
  if (format) {
    *format = KvaConverter::getFirstWriterFormat();
    return kvlcOK;
  }

  return kvlcERR_PARAM;
}
//===========================================================================
KvlcStatus WINAPI kvlcGetNextWriterFormat(int currentFormat, int *nextFormat)
{
  if (nextFormat) {
    *nextFormat = KvaConverter::getNextWriterFormat(currentFormat);
    return kvlcOK;
  }

  return kvlcERR_PARAM;
}
//===========================================================================
KvlcStatus WINAPI kvlcGetWriterName(int format, char *str, unsigned int len)
{
  if (str && len) {
    char buf[MAX_WRITER_DESCRIPTION_SIZE] = {0};
    unsigned int bytesRead = KvaConverter::getWriterName(format, buf) + 1;
    if (bytesRead == 1) return kvlcERR_NOT_IMPLEMENTED;
    if (bytesRead + 1 > len) return kvlcERR_BUFFER_SIZE;
    strncpy_s(str, len, buf, bytesRead);
    return kvlcOK;
  }

  return kvlcERR_PARAM;
}
//===========================================================================
KvlcStatus WINAPI kvlcGetWriterExtension(int format, char *str, unsigned int len)
{
  if (str && len) {
    char buf[MAX_WRITER_DESCRIPTION_SIZE] = {0};
    unsigned int bytesRead = KvaConverter::getWriterExtension(format, buf) + 1;
    if (bytesRead == 1) return kvlcERR_NOT_IMPLEMENTED;
    if (bytesRead + 1 > len) return kvlcERR_BUFFER_SIZE;
    strncpy_s(str, len, buf, bytesRead);
    return kvlcOK;
  }

  return kvlcERR_PARAM;
}
//===========================================================================
KvlcStatus WINAPI kvlcGetWriterDescription(int format, char *str, unsigned int len)
{
  if (str && len) {
    char buf[MAX_WRITER_DESCRIPTION_SIZE] = {0};
    unsigned int bytesRead = KvaConverter::getWriterDescription(format, buf) + 1;
    if (bytesRead == 1) return kvlcERR_NOT_IMPLEMENTED;
    if (bytesRead + 1 > len) return kvlcERR_BUFFER_SIZE;
    strncpy_s(str, len, buf, bytesRead);
    return kvlcOK;
  }

  return kvlcERR_PARAM;
}

//==============================================================================
KvlcStatus WINAPI kvlcGetFirstReaderFormat(int *format)
{
  if (format) {
    *format = KvaConverter::getFirstReaderFormat();
    return kvlcOK;
  }

  return kvlcERR_PARAM;
}
//===========================================================================
KvlcStatus WINAPI kvlcGetNextReaderFormat(int currentFormat, int *nextFormat)
{
  if (nextFormat) {
    *nextFormat = KvaConverter::getNextReaderFormat(currentFormat);
    return kvlcOK;
  }

  return kvlcERR_PARAM;
}
//===========================================================================
KvlcStatus WINAPI kvlcGetReaderName(int format, char *str, unsigned int len)
{
  if (str && len) {
    char buf[MAX_WRITER_DESCRIPTION_SIZE] = {0};
    unsigned int bytesRead = KvaConverter::getReaderName(format, buf) + 1;
    if (bytesRead == 1) return kvlcERR_NOT_IMPLEMENTED;
    if (bytesRead + 1 > len) return kvlcERR_BUFFER_SIZE;
    strncpy_s(str, len, buf, bytesRead);
    return kvlcOK;
  }

  return kvlcERR_PARAM;
}
//===========================================================================
KvlcStatus WINAPI kvlcGetReaderExtension(int format, char *str, unsigned int len)
{
  if (str && len) {
    char buf[MAX_WRITER_DESCRIPTION_SIZE] = {0};
    unsigned int bytesRead = KvaConverter::getReaderExtension(format, buf) + 1;
    if (bytesRead == 1) return kvlcERR_NOT_IMPLEMENTED;
    if (bytesRead + 1 > len) return kvlcERR_BUFFER_SIZE;
    strncpy_s(str, len, buf, bytesRead);
    return kvlcOK;
  }

  return kvlcERR_PARAM;
}
//===========================================================================
KvlcStatus WINAPI kvlcGetReaderDescription(int format, char *str, unsigned int len)
{
  if (str && len) {
    char buf[MAX_WRITER_DESCRIPTION_SIZE] = {0};
    unsigned int bytesRead = KvaConverter::getReaderDescription(format, buf) + 1;
    if (bytesRead == 1) return kvlcERR_NOT_IMPLEMENTED;
    if (bytesRead + 1 > len) return kvlcERR_BUFFER_SIZE;
    strncpy_s(str, len, buf, bytesRead);
    return kvlcOK;
  }

  return kvlcERR_PARAM;
}

//===========================================================================
KvlcStatus WINAPI kvlcGetReaderPropertyDefault(int format, unsigned int property,
    void *buf, unsigned int len)
{
  if (!buf || !len) return kvlcERR_PARAM;
  int res = KvaConverter::getReaderPropertyDefault(format, property, buf, len);
  if (res) return kvlcERR_TYPE_MISMATCH;
  return kvlcOK;
}

//===========================================================================
KvlcStatus WINAPI kvlcIsPropertySupported(
    int format, unsigned int property, int *supported)
{
  if (!supported) return kvlcERR_PARAM;
  bool tmp;
  int res = KvaConverter::isWriterPropertySupported(format, property, &tmp);
  if (res) return kvlcERR_NOT_IMPLEMENTED;
  *supported = tmp?1:0;
  return kvlcOK;
}

//===========================================================================
KvlcStatus WINAPI kvlcGetWriterPropertyDefault(int format, unsigned int property,
    void *buf, unsigned int len)
{
  if (!buf || !len) return kvlcERR_PARAM;
  int res = KvaConverter::getWriterPropertyDefault(format, property, buf, len);
  if (res) return kvlcERR_TYPE_MISMATCH;
  return kvlcOK;
}

//===========================================================================
KvlcStatus WINAPI kvlcAttachFile(
    KvlcHandle handle,
    const char *filename)
{
  if (isInvalid(handle) || !filename) {
    return kvlcERR_PARAM;
  }
  KvaConverter *conv = static_cast<KvaConverter*>(handle);

  if (conv) {
    return conv->attach_file(filename);
  }
  else {
    return kvlcERR_NULL_POINTER;
  }
}

//===========================================================================
KvlcStatus WINAPI kvlcSetProperty(
    KvlcHandle handle, unsigned int property,
    void *buf, unsigned int len)
{
  if (isInvalid(handle) || !buf) {
    return kvlcERR_PARAM;
  }

  KvaConverter *conv = static_cast<KvaConverter*>(handle);

  if (conv) {
    int format;
    int supported;
    conv->get_output_format(&format);
    KvlcStatus stat = kvlcIsPropertySupported(format, property, &supported);
    if (stat != kvlcOK) return stat;
    if (!supported) return kvlcERR_NOT_IMPLEMENTED;
    return conv->set_property_value(property, buf, len);
  }
  else {
    return kvlcERR_NULL_POINTER;
  }
}

//===========================================================================
KvlcStatus WINAPI kvlcGetProperty(
    KvlcHandle handle, unsigned int property,
    void *buf, unsigned int len)
{
  if (isInvalid(handle) || !buf) {
    return kvlcERR_PARAM;
  }

  KvaConverter *conv = static_cast<KvaConverter*>(handle);

  if (conv) {
    return conv->get_property_value(property, buf, len);
  }
  else {
    return kvlcERR_NULL_POINTER;
  }
}

// ---------------------------------------------------------------------------
// Error texts
// ---------------------------------------------------------------------------

typedef struct {
    int errcode;
    const char* msg;
} errmsg_t;

static errmsg_t errmsg_table[] = {
  {kvlcOK,                      "OK"},
  {kvlcFail,                    "Generic error"},
  {kvlcERR_PARAM,               "Error in supplied parameters"},
  {kvlcEOF,                     "End of input file reached"},
  {kvlcERR_NOT_IMPLEMENTED,     "Not implemented"},
  {kvlcERR_FILE_ERROR,          "File I/O error"},
  {kvlcERR_FILE_EXISTS,         "Output file already exists"},
  {kvlcERR_INTERNAL_ERROR,      "Unhandled internal error"},
  {kvlcERR_NULL_POINTER,        "Unexpected null pointer"},
  {kvlcERR_FILE_TOO_LARGE,      "File size too large for specified format"},
  {kvlcERR_TYPE_MISMATCH,       "Supplied parameter has incorrect type"},
  {kvlcERR_NO_FREE_HANDLES,     "Too many open KvlcHandle handles"},
  {kvlcERR_NO_INPUT_SELECTED,   "Missing call to kvlcSetInputFile or kvlcFeedSelectFormat"},
  {kvlcERR_CONVERTING,          "Call failed since conversion is running"},
  {kvlcERR_BUFFER_SIZE,         "The supplied buffer is too small to hold the result"},
  {kvlcERR_INVALID_LOG_EVENT,   "Event is not recognized by the converter"},
  {kvlcERR_NO_TIME_REFERENCE,   "Required timestamp missing"},
  {kvlcERR_TIME_DECREASING,     "Decreasing time between files"},
  {kvlcERR_MIXED_ENDIANNESS,    "Wrong data type in MDF signal format"},
  {kvlcERR_RESULT_TOO_BIG,      "Result is too big for an out-parameter"},
  {kvlcERR_UNSUPPORTED_VERSION, "Unsupported version of file format"},
};

#define ERRMSG_TABLE_LEN  sizeof(errmsg_table)/sizeof(errmsg_table[0])

static bool get_error_text(int code, const char* &msg)
{
  msg = NULL;
  for (unsigned int i=0; i<ERRMSG_TABLE_LEN; i++) {
    if (errmsg_table[i].errcode == code) {
      msg = errmsg_table[i].msg;
      return true;
    }
  }
  return false;
}

// ===========================================================================
KvlcStatus WINAPI kvlcGetErrorText (KvlcStatus error, char *buffer, unsigned int buffer_size)
{
  if (error > 0 || !buffer || buffer_size < 1) return kvlcERR_PARAM;

  const char* msg;
  if ( get_error_text(error, msg) ) {
    strncpy(buffer, msg, buffer_size);
   }
   else {
    strncpy(buffer, "Unknown error code.", buffer_size);
    buffer[buffer_size-1] = '\0';
   }
   return kvlcOK;
}

