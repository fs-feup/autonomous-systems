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
**   Utility functions for memolib
** -----------------------------------------------------------------------------
*/

 #include "util.h"
 #include "logio_w32.h"
 #include "logger_config_file.h"

 //===========================================================================
//  Note: There is no way to ask firmware about the lio (KMF) and config
//  (param.lif) version that firmware needs. It not possible to get the config
//  version from a KMF file either. This forces us to infer (guess) which
//  version we need and this is done in the following  functions to at least
//  keep the ugliness contained. This should be done in firmware!

#define FW_MAJOR_WITH_FD  3
#define LOG_CONFIG_MAJOR_VERSION_PRE_FD 5
#define LOG_CONFIG_MINOR_VERSION_PRE_FD 0

//===========================================================================
void lioVerToConfigVer (uint8_t lioMajor, uint8_t /* lioMinor */,
                               uint8_t *configMajor, uint8_t *configMinor)
{
  if (lioMajor >= LIO_VERSION_FD) {
    *configMajor = LOG_CONFIG_MAJOR_VERSION;
    *configMinor = 0;
  }
  else {
    *configMajor = LOG_CONFIG_MAJOR_VERSION_PRE_FD;
    *configMinor = 0;
  }
}

//===========================================================================
void fwVerToConfigVer (uint8_t fwMajor, uint8_t /* fwMinor */,
                              uint8_t *lioMajor, uint8_t *lioMinor,
                              uint8_t *configMajor, uint8_t *configMinor)
{
  if (FW_MAJOR_WITH_FD >= fwMajor) {
    *lioMajor = LIO_VERSION_FD;
    *lioMinor = 0;
  }
  else {
    *lioMajor = LIO_VERSION;
    *lioMinor = 0;
  }
  lioVerToConfigVer(*lioMajor, *lioMinor, configMajor, configMinor);
}

//===========================================================================
uint16_t toVersion (uint8_t major, uint8_t minor)
{
  return (major << 8) + minor;
}

//===========================================================================
void splitVersion (uint16_t version, uint8_t *major, uint8_t *minor)
{
  *major = (uint8_t)((version & 0xff00) >> 8);
  *minor = (uint8_t)( version & 0x00ff);
  // Fix for old major LIO version in first byte
  if (!(*major)) {
    *major = *minor;
    *minor = 0;
  }
}

//===========================================================================
enum {
  DLC12   = 9,
  DLC16   = 10,
  DLC20   = 11,
  DLC24   = 12,
  DLC32   = 13,
  DLC48   = 14,
  DLC64   = 15
};

uint8_t dlcToBytesFD(uint8_t dlc)
{
  dlc &= 0xf;

  switch(dlc) {
  case DLC12:  return 12;
  case DLC16:  return 16;
  case DLC20:  return 20;
  case DLC24:  return 24;
  case DLC32:  return 32;
  case DLC48:  return 48;
  case DLC64:  return 64;
  default:  return dlc;
  }
}
