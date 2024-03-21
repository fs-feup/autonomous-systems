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
** Description: Sample program for kvadblib
**
** ---------------------------------------------------------------------------
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "kvaDbLib.h"


// Create a sample database.
int create_database();

int main ()
{
  KvaDbStatus status = kvaDbOK;
  KvaDbHnd db = 0;
  KvaDbMessageHnd mh = 0;
  KvaDbNodeHnd nh = 0;
  KvaDbSignalHnd sh = 0;

  // Create a sample database.
  if (create_database() ) {
    return -1;
  }

  // Create a database handle
  status = kvaDbOpen(&db);
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }
  if (status != kvaDbOK) {
    printf("Error: Could not create database handle: %d.\n", status);
    return -1;
  }

  // Open the sample database
  status = kvaDbAddFile(db, "sample.dbc");
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }
  if (status != kvaDbOK) {
    printf("Error: Could not open file 'sample.dbc': %d.\n", status);
    return -1;
  }

  // Go through all messages in the database
  status = kvaDbGetFirstMsg(db, &mh);
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }
  if (status != kvaDbOK) {
    printf("Error: Could not get first message %d.\n", status);
    return -1;
  }

  while (status == kvaDbOK) {
    char name[128] = {0};
    char qname[128] = {0};
    char comment[128] = {0};
    char node[128] = {0};
    int  dlc = 0;
    unsigned int id = 0;
    unsigned int flags = 0;

    status = kvaDbGetMsgName(mh, name, sizeof(name));
    if (status != kvaDbOK) {
      printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
      return -1;
    }
    status = kvaDbGetMsgQualifiedName (mh, qname, sizeof(qname));
    if (status != kvaDbOK) {
      printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
      return -1;
    }
    status = kvaDbGetMsgComment (mh, comment, sizeof(comment));
    if (status != kvaDbOK) {
      printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
      return -1;
    }

    status = kvaDbGetMsgSendNode(mh, &nh);
    if (status != kvaDbOK) {
      printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
      return -1;
    }
    if (status == kvaDbOK) {
      status = kvaDbGetNodeName(nh, node, sizeof(node));
      if (status != kvaDbOK) {
	printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
	return -1;
      }
    }

    status = kvaDbGetMsgId (mh, &id, &flags);
    if (status != kvaDbOK) {
      printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
      return -1;
    }
    status = kvaDbGetMsgDlc (mh, &dlc);
    if (status != kvaDbOK) {
      printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
      return -1;
    }


    printf("\n Message: name='%s'  id=0x%x, dlc=%d, flags=0x%x\n",
	   name, id, dlc, flags);
    printf("          qualified name='%s'\n", qname);
    printf("          send node='%s'\n", node);
    printf("          comment='%s'\n\n", comment);

    status = kvaDbGetFirstSignal (mh, &sh);
    if (status != kvaDbOK) {
      printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
      return -1;
    }
    while (status == kvaDbOK) {
      char unit[128] = {0};
      KvaDbSignalEncoding encoding = kvaDb_Intel;
      double minval = 0;
      double maxval = 0;
      double factor = 0;
      double offset = 0;
      int startbit = 0;
      int length = 0;

      status = kvaDbGetSignalName (sh, name, sizeof(name));
      if (status != kvaDbOK) {
	printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
	return -1;
      }
      status = kvaDbGetSignalQualifiedName(sh, qname, sizeof(qname));
      if (status != kvaDbOK) {
	printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
	return -1;
      }
      status = kvaDbGetSignalComment(sh, comment, sizeof(comment));
      if (status != kvaDbOK) {
	printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
	return -1;
      }
      status = kvaDbGetSignalUnit(sh, unit, sizeof(unit));
      if (status != kvaDbOK) {
	printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
	return -1;
      }
      status = kvaDbGetSignalEncoding(sh, &encoding);
      if (status != kvaDbOK) {
	printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
	return -1;
      }
      status = kvaDbGetSignalValueSize(sh, &startbit, &length);
      if (status != kvaDbOK) {
	printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
	return -1;
      }
      status = kvaDbGetSignalValueLimits(sh, &minval, &maxval);
      if (status != kvaDbOK) {
	printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
	return -1;
      }
      status = kvaDbGetSignalValueScaling(sh, &factor, &offset);
      if (status != kvaDbOK) {
	printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
	return -1;
      }


      printf ("    Signal: name='%s', unit='%s'\n",
	      name, unit);

      printf ("            qualified name='%s'\n",
	      qname);

      printf ("            comment='%s'\n",
	      comment);

      printf ("            startbit=%d, length=%d, encoding=%s\n",
	      startbit, length, encoding == kvaDb_Intel?"Intel":"Motorola");

      printf ("            factor=%.02f, offset=%.02f\n",
	      factor, offset);

      printf ("            minval=%.02f, maxval=%.02f\n\n",
	      minval, maxval);

      status = kvaDbGetNextSignal(mh, &sh);
      if ((status != kvaDbOK) && (status != kvaDbErr_NoSignal)) {
	printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
	return -1;
      }
    }

    status = kvaDbGetNextMsg (db, &mh);
    if ((status != kvaDbOK) && (status != kvaDbErr_NoMsg)) {
      printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
      return -1;
    }
  }


  status = kvaDbClose(db);
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }

  return 0;
}


// -------------------------------------------------------------------
// ISO C++ forbids converting a string constant to char*
char tmp[1024];
char * toChar(const char * str)
{
  snprintf(tmp, sizeof(tmp), "%s", str);
  return tmp;
}

// -------------------------------------------------------------------
int create_database ()
{
  KvaDbHnd db_handle = 0;
  KvaDbStatus status = kvaDbOK;
  KvaDbMessageHnd mh1;
  KvaDbSignalHnd sh11, sh12;
  KvaDbNodeHnd nh1, nh2;

  status = kvaDbOpen(&db_handle);
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }

  if (!db_handle) {
    printf("Error on line=%d\n",__LINE__);
    return -1;
  }

  status = kvaDbCreate(db_handle, "sample", NULL);
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }

  if (!db_handle) {
    printf("Error on line=%d\n",__LINE__);
    return -1;
  }

  status = kvaDbSetProtocol(db_handle, kvaDb_ProtocolCan);
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }

  // Add a message and two signals
  status = kvaDbAddMsg(db_handle, &mh1);
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }
  status = kvaDbSetMsgName(mh1, toChar("Measurement"));
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }
  status = kvaDbSetMsgComment(mh1, toChar("Temperature and proximity"));
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }
  status = kvaDbSetMsgId(mh1, 111, KVADB_MESSAGE_EXT);
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }
  status = kvaDbSetMsgDlc(mh1, 8);
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }


  status = kvaDbAddSignal(mh1, &sh11);
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }
  status = kvaDbSetSignalName(sh11, toChar("Position"));
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }
  status = kvaDbSetSignalValueSize(sh11, 0, 8);
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }
  status = kvaDbSetSignalComment(sh11, toChar("Proximity sensor"));
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }
  status = kvaDbSetSignalUnit(sh11, toChar("m"));
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }
  status = kvaDbSetSignalValueLimits(sh11, -10, 10);
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }
  status = kvaDbSetSignalRepresentationType(sh11, kvaDb_Unsigned);
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }
  status = kvaDbSetSignalEncoding(sh11, kvaDb_Intel);
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }
  status = kvaDbSetSignalValueScaling(sh11, 0.1, -10.0);
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }


  status = kvaDbAddSignal(mh1, &sh12);
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }
  status = kvaDbSetSignalName(sh12, toChar("Temperature"));
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }
  status = kvaDbSetSignalValueSize(sh12, 41, 32);
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }
  status = kvaDbSetSignalComment(sh12, toChar("Voltage Temperature Sensor"));
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }
  status = kvaDbSetSignalUnit(sh12, toChar("C"));
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }
  status = kvaDbSetSignalRepresentationType(sh12, kvaDb_Signed);
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }
  status = kvaDbSetSignalEncoding(sh12, kvaDb_Motorola);
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }
  status = kvaDbSetSignalValueScaling(sh12, 0.55, -32.0);
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }


  // Add nodes
  status = kvaDbAddNode(db_handle, &nh1);
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }
  status = kvaDbSetNodeName(nh1, toChar("NodeOne"));
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }
  status = kvaDbSetNodeComment(nh1, toChar("Sending node"));
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }

  status = kvaDbAddNode(db_handle, &nh2);
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }
  status = kvaDbSetNodeName(nh2, toChar("NodeTwo"));
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }
  status = kvaDbSetNodeComment(nh2, toChar("Receiving Node"));
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }

  // Set send and receive node
  status = kvaDbSetMsgSendNode(mh1, nh1);
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }
  status = kvaDbAddReceiveNodeToSignal(sh11, nh2);
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }

  status = kvaDbWriteFile(db_handle, toChar("sample.dbc"));
  if (status != kvaDbOK) {
    printf("Error on line=%d. status=%d\n",__LINE__, (int)status);
    return -1;
  }

  return 0;
}
