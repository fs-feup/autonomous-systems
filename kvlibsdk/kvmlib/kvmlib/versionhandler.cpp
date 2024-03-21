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
**  Scan KME files and try to infer version.
** -----------------------------------------------------------------------------
*/

#include "versionhandler.h"

#define CompilerAssert(e)
#include "loggerDefinitions.h"
#include "kme_24_format.h"
#include "kme_60_format.h"
#include "filo_cmds.h"
#include "hydra_host_cmds.h"
#include "kvdebug.h"

FILE* utf8_fopen(const char* filename, const char* type)
{
	//return fopen(filename, type);
	PRINTF((filename));
	return fopen(filename, type);
}

kvmStatus kmeScanFileType (const char *filename, int32 *fileType)
{
  kvmStatus status = kvmFail;
  uint8_t buf[sizeof(kme24_logStruct)] = {0};
  uint8_t cmdLen  = 0;
  uint8_t cmdNo   = 0;
  uint16_t kme60MsgLen;
  FILE *fh = NULL;

  if (!fileType || !filename) {
    return kvmERR_PARAM;
  }

  //fh = fopen(filename, "rb");
  fh = utf8_fopen(filename, "rb");
  if (!fh) {
    return kvmERR_FILE_NOT_FOUND;
  }

  if (fread (buf, sizeof(buf), 1, fh) != 1) {
    fclose(fh);
    return kvmERR_FILE_ERROR;
  }

  // KME 6.0 stores length as uint16_t in buf[0-1], and version as uint8_t in buf [2].
  // If filetype was kme50, buf[2] corresponds to lioMajor which never should be equal to KME60_MSG_VERSION (100)
  memcpy(&kme60MsgLen, buf, 2);
  if (kme60MsgLen == sizeof(Kme60_Version_t) + sizeof(Kme60_MsgHead_t) && buf[2] == KME60_MSG_VERSION)
  {
      *fileType = kvmFILE_KME60;
      fclose(fh);
      return kvmOK;
  }

  // KME 5.0 and KME 4.0 use cmdLen and cmdNo
  cmdLen = buf[0];
  cmdNo = buf[1];

  // KME 5.0 ALWAYS begins with CMD_LOG_VERSION_FD
  if ((cmdLen == sizeof(hcmdLogVerFD)     && cmdNo == CMD_LOG_VERSION_FD) ||
      (cmdLen == sizeof(hcmdLogRtcTimeFD) && cmdNo == CMD_LOG_RTC_TIME_FD) ||
      (cmdLen == sizeof(hcmdLogTrigFD)    && cmdNo == CMD_LOG_TRIG_FD) ||
      cmdNo == CMD_RX_MESSAGE_FD)
  {
    *fileType = kvmFILE_KME50;
    fclose(fh);
    return kvmOK;
  }

  // KME 4.0 should begin with CMD_LOG_RTC_TIME
  if ((cmdLen == sizeof(cmdLogRtcTime) && cmdNo == CMD_LOG_RTC_TIME) ||
      (cmdLen == sizeof(cmdLogTrig)    && cmdNo == CMD_LOG_TRIG) ||
      (cmdLen == sizeof(cmdLogMessage) && cmdNo == CMD_LOG_MESSAGE))
  {
    *fileType = kvmFILE_KME40;
    fclose(fh);
    return kvmOK;
  }

  // KME 2.4 and KME 2.5 use cmdType and 16 byte size for all messages. Both
  // file types should begin with clocks and possibly a trigger.
  do {
    uint8_t cmdType = buf[0];

    // Time message
    if (cmdType == LOG_TYPE_TIME && cmdType == KME24_LOG_TYPE_TIME) {
      RtcTimeEvent *e = (RtcTimeEvent*) &buf[1];
      if (e->verMajor == LOG_FILE_MAJOR_VERSION &&
          e->verMinor == LOG_FILE_MINOR_VERSION) {
        *fileType = kvmFILE_KME25;
      } else {
        *fileType = kvmFILE_KME24;
      }
      status = kvmOK;
      break;
    }
    if (fread (buf, sizeof(buf), 1, fh)) {
      // Ignore return value
    }
  } while (!feof(fh));

  fclose(fh);
  return status;
}
