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
#include "kvmlib.h"
#include "TimeConv.h"
#include <cstring>
#include <cstdlib>


// =====================================================================
// Local functions
// =====================================================================
static void  printDate(uint32_t wallclock)
{
  unsigned int yy, mm, dd, ho, m, s;
  unix_time (wallclock, &yy, &mm, &dd, &ho, &m, &s);
  printf("%d-%02d-%02d %02d:%02d.%02d", yy, mm, dd, ho, m, s);
}


void printErrorText(const char * name, int error)
{
  char buf[512];

  kvmGetErrorText((kvmStatus) (error), buf, sizeof(buf));
  printf("%s failed with error code %d (%s) \n", name, error, buf);
}
// =====================================================================
// Main function
// =====================================================================
int main()
{
  kvmStatus status;
  int major, minor,build;
  kvmHandle h;
  unsigned int t;
  int channel = 0;

  kvmInitialize();

  printf("Kvaser Sample Program\n");
  status = kvmGetVersion(&major, &minor, &build);
  if (status == kvmOK) {
    printf("kvmGetVersion(%d, %d, %d)\n", major, minor, build);
  } else {
    printErrorText("kvmGetVersion", status);
    return 0;
  }


  h = kvmDeviceOpen(channel, &status, kvmDEVICE_MHYDRA_EXT);
  if (status == kvmOK) {
    printf("kvmDeviceOpen: OK\n");
  } else {
    printErrorText("kvmDeviceOpen", status);
    return 0;
  }


  status = kvmDeviceGetRTC(h, &t);
  if (status == kvmOK) {
    printf("kvmDeviceGetRTC: OK\nDate: ");
    printDate(t);
    printf("\n");
  } else {
    printErrorText("kvmDeviceGetRTC", status);
    return 0;
  }

  status = kvmDeviceSetRTC(h, t);
  if (status == kvmOK) {
    printf("kvmDeviceSetRTC: OK\n");
  } else {
    printErrorText("kvmDeviceSetRTC", status);
    return 0;
  }

  status = kvmDeviceFlashLeds(h);
  if (status == kvmOK) {
    printf("kvmDeviceFlashLeds: OK\n");
  } else {
    printErrorText("kvmDeviceFlashLeds", status);
    return 0;
  }

  status = kvmClose(h);


  printf("Done!\n");
  return 0;
}
