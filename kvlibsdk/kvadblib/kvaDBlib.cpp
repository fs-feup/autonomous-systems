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


#include "canlib_version.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "kvaDbLib.h"
#include "CANdb.h"
#include "CANdbDbc.h"
#include "kvdebug.h"
#include "compilerassert.h"

#include "handle_template.h"

static handle_class<CANdbCluster*,             KvaDbHnd>             kvadbhnd;
static handle_class<CANdbMessage*,             KvaDbMessageHnd>      kvadbmessagehnd;
static handle_class<CANdbSignal*,              KvaDbSignalHnd>       kvadbsignalhnd;
static handle_class<CANdbNode*,                KvaDbNodeHnd>         kvadbnodehnd;
static handle_class<CANdbAttribute*,           KvaDbAttributeHnd>    kvadbattributehnd;
static handle_class<CANdbAttributeDefinition*, KvaDbAttributeDefHnd> kvadbattributedefhnd;
static handle_class<CANdbEnumValue*,           KvaDbEnumValueHnd >   kvadbenumvaluehnd;

static KvaDbStatus get_enum_value(CANdbEnumValue *e, int *val, char *buf, size_t buflen);
static int roundi(double x) { return (int)(x+(x>0.0?0.5:-0.5)); }
static int roundu(double x) { return (unsigned int)(x+0.5); }
static int kvastricmp(const char *s1, const char *s2)
{
  int c1, c2;
  for(;;) {
    c1 = tolower((unsigned char) *s1++);
    c2 = tolower((unsigned char) *s2++);
    if (c1 == 0 || c1 != c2) {
      return c1 - c2;
    }
  }
}

#define ERRORLOG_MAXLEN 512
static char errorlog [ERRORLOG_MAXLEN + 1];
static bool hasRegisteredCallbacks = false;

// #ifdef DEBUG
// void dbg_printf(const char *s, ...);
// #endif

// Object instance deletion callback functions
void DbClusterDeletionCallback(CANdbCluster *pDbCluster) { kvadbhnd.remove(pDbCluster); }
void MessageDeletionCallback(CANdbMessage *pMsg) { kvadbmessagehnd.remove(pMsg); }
void SignalDeletionCallback(CANdbSignal *pSignal) { kvadbsignalhnd.remove(pSignal); }
void NodeDeletionCallback(CANdbNode *pNode) { kvadbnodehnd.remove(pNode); }
void AttributeDeletionCallback(CANdbAttribute *pAttr) { kvadbattributehnd.remove(pAttr); }
void AttributeDefinitionDeletionCallback(CANdbAttributeDefinition *pAttrDef) { kvadbattributedefhnd.remove(pAttrDef); }
void EnumValueDeletionCallback(CANdbEnumValue *pEnumValue) { kvadbenumvaluehnd.remove(pEnumValue); }

/**
 * Register deletion callbacks if not already done
 *
 * @note
 * This function must be called once before deleting any object instance.
 * Failure to do so WILL lead to a program crash due to null pointer access!
 * If unsure, add it to the beginning of every function to be on the safe side.
 */
static void RegisterDeletionCallbacks(void)
{
  if (!hasRegisteredCallbacks) {
    CANdbCluster::register_deletion_callback(DbClusterDeletionCallback);
    CANdbMessage::register_deletion_callback(MessageDeletionCallback);
    CANdbSignal::register_deletion_callback(SignalDeletionCallback);
    CANdbNode::register_deletion_callback(NodeDeletionCallback);
    CANdbAttribute::register_deletion_callback(AttributeDeletionCallback);
    CANdbAttributeDefinition::register_deletion_callback(AttributeDefinitionDeletionCallback);
    CANdbEnumValue::register_deletion_callback(EnumValueDeletionCallback);
    hasRegisteredCallbacks = true;
  }
}

//===========================================================================

KvaDbStatus WINAPI kvaDbOpen(KvaDbHnd *dh)
{
  RegisterDeletionCallbacks();
  CANdbCluster *dc;

  if (dh) {
    dc  = new CANdbCluster();
    return kvadbhnd.add(dc, dh);
  } else {
    return kvaDbErr_Param;
  }
}

//===========================================================================
KvaDbStatus WINAPI kvaDbAddFile(KvaDbHnd dh, const char *filename)
{
  RegisterDeletionCallbacks();
  if (!filename)              return kvaDbErr_Param;
  if (!kvadbhnd.is_valid(dh)) return kvaDbErr_Param;

  CANdbCluster *dc = kvadbhnd.convert(dh);
  CANdb        *db = new CANdb;

  CANdbDBC dbcio (filename);

  if (dbcio.read_file(db) != 0) {

    unsigned int buflen = ERRORLOG_MAXLEN;
    dbcio.get_errorlog(errorlog, &buflen);

    delete db;

    if (strlen(errorlog)>0)
      return kvaDbErr_DbFileParse;
    else
      return kvaDbErr_DbFileOpen;

  } else {
    errorlog[0] = '\0';
  }

  db->set_name(dbcio.get_db_name());
  db->set_filename(filename);
  dc->add_db(db);

  return kvaDbOK;
}

//===========================================================================
// Used to define a "file name" when we don't have any file that the
// data have been read from.
KvaDbStatus WINAPI kvaDbSetDummyFileName(KvaDbHnd dh, const char *filename)
{
  RegisterDeletionCallbacks();
  CANdbCluster *dc;
  CANdb        *db;

  if (!filename)              return kvaDbErr_Param;
  if (!kvadbhnd.is_valid(dh)) return kvaDbErr_Param;

  dc = kvadbhnd.convert(dh);
  db = new CANdb;
  db->set_filename(filename);
  db->set_name(filename);
  dc->add_db(db);

  return kvaDbOK;
}

//===========================================================================
// Create a database. Can both read from a file and set the database name.
KvaDbStatus WINAPI kvaDbCreate (KvaDbHnd dh, const char *localName, const char *filename)
{
  RegisterDeletionCallbacks();
  if (!kvadbhnd.is_valid(dh))  return kvaDbErr_Param;
  if (!localName && !filename) return kvaDbErr_Param;

  if (localName && !filename) {
    return kvaDbSetDummyFileName(dh, localName);
  }
  else if (!localName && filename) {
    return kvaDbAddFile(dh, filename);
  }
  else {
    CANdbCluster *dc = kvadbhnd.convert(dh);
    CANdb        *db = new CANdb;

    CANdbDBC dbcio (filename);
    if (dbcio.read_file(db) != 0) {

      unsigned int buflen = ERRORLOG_MAXLEN;
      dbcio.get_errorlog(errorlog, &buflen);

      delete db;

      if (strlen(errorlog)>0)
        return kvaDbErr_DbFileParse;
      else
        return kvaDbErr_DbFileOpen;

    } else {
      errorlog[0] = '\0';
    }
    db->set_name(localName);
    db->set_filename(filename);
    dc->add_db(db);
    return kvaDbOK;
  }
}

//===========================================================================
// Reads chosen database's name into buffer
KvaDbStatus WINAPI kvaDbGetDatabaseName(KvaDbHnd dh, char *buf, size_t buflen)
{
  RegisterDeletionCallbacks();
 CANdbCluster *dc;
 CANdb        *db;

  if (!kvadbhnd.is_valid(dh) || !buf || (buflen == 0)) return kvaDbErr_Param;

  dc = kvadbhnd.convert(dh);
  db = dc->get_first_candb();

  if (!db) return kvaDbErr_NoDatabase;

  const char *db_name = db->get_name();
  if (db_name) {
    strncpy(buf, db_name, buflen);
    buf[buflen-1] = 0;
  }
  else {
    strcpy(buf, "");
  }

  return kvaDbOK;
}

static const char *errmsg[] = {"OK",
                               "General failure",
                               "No database was found",
                               "One or more of the parameters in call is erronous",
                               "No message was found",
                               "No signal was found",
                               "An internal error occured in the library",
                               "Could not open the database file",
                               "An internal error occured in the database handler",
                               "Could not find the database node",
                               "No attribute found",
                               "Only one kvaDbLib structure is allowed",
                               "Wrong owner for attribute",
                               "An item is in use",
                               "The supplied buffer is too small to hold the result",
                               "Could not parse the database file"};
#define KVADB_NUM_ERRORS (-kvaDbErr_DbFileParse + 1)
CompilerAssert(sizeof(errmsg) == KVADB_NUM_ERRORS * sizeof(char*));

//===========================================================================
// Provides description of error code to specified buffer
KvaDbStatus WINAPI kvaDbGetErrorText(KvaDbStatus error, char *buf, size_t buflen)
{
  RegisterDeletionCallbacks();
  int err_index = -(int)error;
  if (err_index < 0 || err_index >= KVADB_NUM_ERRORS || !buf || buflen < strlen(errmsg[err_index]) + 1) {
    return kvaDbErr_Param;
  }
  strncpy(buf, errmsg[err_index], buflen);
  buf[buflen-1] = '\0';
  return kvaDbOK;
}

KvaDbStatus WINAPI kvaDbGetLastParseError(char *buf, unsigned int *buflen)
{
  RegisterDeletionCallbacks();
  unsigned int errlen = (unsigned int) strlen(errorlog);
  if (errlen > *buflen-1){
    strncpy(buf, errorlog, *buflen-1);
    *buflen = errlen + 1;
    return kvaDbErr_BufferSize;
  }

  strcpy(buf, errorlog);

  return kvaDbOK;
}



//===========================================================================
// Reads this library's version
KvaDbStatus WINAPI kvaDbGetVersion(int *major, int *minor, int *build)
{
  RegisterDeletionCallbacks();
  if (!major || !minor || !build) {
    return kvaDbErr_Param;
  }
  *major = CANLIB_MAJOR_VERSION;
  *minor = CANLIB_MINOR_VERSION;
  *build = 0;
  return kvaDbOK;
}

//===========================================================================
KvaDbStatus WINAPI kvaDbClose(KvaDbHnd dh)
{
  RegisterDeletionCallbacks();
  if (kvadbhnd.is_valid(dh)) {
    delete kvadbhnd.convert(dh);
    return kvadbhnd.remove(dh);
  } else {
    return kvaDbErr_Param;
  }
}

//===========================================================================
KvaDbStatus WINAPI kvaDbWriteFile(KvaDbHnd dh, char* filename)
{
  RegisterDeletionCallbacks();
  if (!filename)              return kvaDbErr_Param;
  if (!kvadbhnd.is_valid(dh)) return kvaDbErr_Param;

  CANdbCluster *dc = kvadbhnd.convert(dh);
  if (!dc) return kvaDbErr_NoDatabase;
  if (dc->get_candb_count() > 1) return kvaDbErr_OnlyOneAllowed;
  CANdb *candb = dc->get_first_candb();
  if (!candb) return kvaDbErr_NoDatabase;

  CANdbDBC dbcio (filename);
  if (dbcio.save_file(candb) != 0) {
    return kvaDbErr_DbFileOpen;
  }

  return kvaDbOK;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbReadFile(KvaDbHnd dh, char* filename)
{
  RegisterDeletionCallbacks();
  if (!filename)              return kvaDbErr_Param;
  if (!kvadbhnd.is_valid(dh)) return kvaDbErr_Param;

  CANdbCluster *dc = kvadbhnd.convert(dh);
  CANdb        *db = new CANdb;

  CANdbDBC dbcio (filename);
  if (dbcio.read_file(db) != 0) {

    unsigned int buflen = ERRORLOG_MAXLEN;
    dbcio.get_errorlog(errorlog, &buflen);

    delete db;

    if (strlen(errorlog)>0)
      return kvaDbErr_DbFileParse;
    else
      return kvaDbErr_DbFileOpen;

  } else {
    errorlog[0] = '\0';
  }
  db->set_name(dbcio.get_db_name());
  db->set_filename(filename);
  dc->add_db(db);

  return kvaDbOK;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbGetFlags(KvaDbHnd dh, unsigned int *flags)
{
  RegisterDeletionCallbacks();
  if (!kvadbhnd.is_valid(dh)) return kvaDbErr_Param;
  if (!flags)                 return kvaDbErr_Param;

  *flags = 0;

  CANdbCluster *dc = kvadbhnd.convert(dh);
  if (!dc) return kvaDbErr_Internal;

  if (dc->get_candb_count() > 1) return kvaDbErr_OnlyOneAllowed;

  CANdb *candb = dc->get_first_candb();
  if (!candb) return kvaDbErr_NoDatabase;

  if (candb->is_j1939()) *flags |= KVADB_DATABASE_J1939;

  return kvaDbOK;
}

#define KVADB_NUM_PROTOCOLS (kvaDb_ProtocolUnknown + 1)
static const char *KVADBPROTOCOL_STRINGS[] = {"CAN", "Van", "Lin", "MOST", "FlexRay", "BEAN",
                                                 "Ethernet", "AFDX", "J1708", "CAN FD", ""};
CompilerAssert(sizeof(KVADBPROTOCOL_STRINGS) == KVADB_NUM_PROTOCOLS * sizeof(char*));

static const char *KVADBPROTOCOL_ATTRNAME = "BusType";
static const KvaDbProtocolProperties KVADB_PROTOCOL_PROPERTIES[KVADB_NUM_PROTOCOLS] = \
               {{0xF, 32}, {0, 0}, {8, 32}, {0, 0}, {0, 0},  \
               {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0xF, 64}, {0, 0}};

//===========================================================================
KvaDbStatus WINAPI kvaDbGetProtocolProperties(KvaDbProtocolType prot, KvaDbProtocolProperties *prop)
{
  RegisterDeletionCallbacks();
  if (NULL == prop || (int)prot >= KVADB_NUM_PROTOCOLS) return kvaDbErr_Param;
  *prop = KVADB_PROTOCOL_PROPERTIES[prot];
  return kvaDbOK;
}

//===========================================================================
KvaDbStatus WINAPI kvaDbMsgDlcToBytes(KvaDbProtocolType prot, unsigned int dlc, unsigned int *numBytes)
{
  RegisterDeletionCallbacks();
  if (NULL == numBytes || (int)prot >= KVADB_NUM_PROTOCOLS)
    return kvaDbErr_Param;
  if (0 != KVADB_PROTOCOL_PROPERTIES[prot].maxMessageDlc && dlc > KVADB_PROTOCOL_PROPERTIES[prot].maxMessageDlc)
    return kvaDbErr_Param;
  switch (prot) {
    case kvaDb_ProtocolCan:
      *numBytes = (dlc > 8? 8: dlc);
      break;
      //intentional fall-through (dlc <= 8)
    case kvaDb_ProtocolCanFD:
      if (dlc > 8) {
        switch (dlc) {
          case 0x9: *numBytes = 12; break;
          case 0xa: *numBytes = 16; break;
          case 0xb: *numBytes = 20; break;
          case 0xc: *numBytes = 24; break;
          case 0xd: *numBytes = 32; break;
          case 0xe: *numBytes = 48; break;
          case 0xf: *numBytes = 64; break;
        }
      }
      else {
        *numBytes = dlc;
      }
      break;
    default:
      return kvaDbErr_Param;
  }
  return kvaDbOK;
}

//===========================================================================
KvaDbStatus WINAPI kvaDbBytesToMsgDlc(KvaDbProtocolType prot, unsigned int numBytes, unsigned int *dlc)
{
  RegisterDeletionCallbacks();
  if (NULL == dlc || (int)prot >= KVADB_NUM_PROTOCOLS)
    return kvaDbErr_Param;
  switch (prot) {
    case kvaDb_ProtocolCan:
      if (numBytes > 8)
        return kvaDbErr_Param;
      *dlc = numBytes;
      break;
    case kvaDb_ProtocolCanFD:
      if (numBytes <= 8)
        *dlc = numBytes;
      else
        switch(numBytes) {
          case 12: *dlc = 0x9; break;
          case 16: *dlc = 0xa; break;
          case 20: *dlc = 0xb; break;
          case 24: *dlc = 0xc; break;
          case 32: *dlc = 0xd; break;
          case 48: *dlc = 0xe; break;
          case 64: *dlc = 0xf; break;
          default: return kvaDbErr_Param;
        }
      break;
    default:
      return kvaDbErr_Param;
  }
  if (0 != KVADB_PROTOCOL_PROPERTIES[prot].maxMessageDlc && *dlc > KVADB_PROTOCOL_PROPERTIES[prot].maxMessageDlc)
    return kvaDbErr_Param;
  return kvaDbOK;
}

//===========================================================================
KvaDbStatus WINAPI kvaDbGetProtocol(KvaDbHnd dh, KvaDbProtocolType *prot)
{
  RegisterDeletionCallbacks();
  if (!kvadbhnd.is_valid(dh)) return kvaDbErr_Param;

  CANdbCluster *dc = kvadbhnd.convert(dh);
  if (dc->get_candb_count() > 1) return kvaDbErr_OnlyOneAllowed;
  CANdb* dbc = dc->get_first_candb();
  if (NULL == dbc) return kvaDbErr_NoDatabase;
  KvaDbProtocolType ret_val = kvaDb_ProtocolUnknown;

  CANdbAttributeList *attr_list = dbc->get_attributes();
  if (NULL != attr_list) {
    const CANdbAttribute *prot_attr = attr_list->find_by_name(KVADBPROTOCOL_ATTRNAME);
    if (NULL != prot_attr && CANDB_ATTR_TYPE_STRING == prot_attr->get_type()) {
      const char *prot_val = prot_attr->get_string_value();
      for (int i = 0; i < KVADB_NUM_PROTOCOLS; i++) {
        if (0 == kvastricmp(prot_val, KVADBPROTOCOL_STRINGS[i])) {
          ret_val = static_cast<KvaDbProtocolType>(i);
          break;
        }
    }
  }
  }

  *prot = ret_val;
  return kvaDbOK;
}

//===========================================================================
KvaDbStatus WINAPI kvaDbSetProtocol(KvaDbHnd dh, KvaDbProtocolType prot)
{
  RegisterDeletionCallbacks();
  if (!kvadbhnd.is_valid(dh)) return kvaDbErr_Param;

  CANdbCluster *dc = kvadbhnd.convert(dh);

  if (dc->get_candb_count() > 1) return kvaDbErr_OnlyOneAllowed;
  CANdb* dbc = dc->get_first_candb();

  if (NULL == dbc) return kvaDbErr_NoDatabase;
  CANdbAttributeList *attr_list = dbc->get_attributes();

  if (NULL == attr_list) return kvaDbErr_NoAttrib;
  CANdbAttribute *prot_attr = attr_list->find_by_name(KVADBPROTOCOL_ATTRNAME);

  if (NULL == prot_attr && kvaDb_ProtocolUnknown != prot)  {
    CANdbAttributeDefinition *bustype_def = dbc->find_attribute_definition_by_name(KVADBPROTOCOL_ATTRNAME);
    if (NULL == bustype_def) {
      bustype_def = new CANdbAttributeDefinition();
      bustype_def->set_owner(CANDB_ATTR_OWNER_DB);
      bustype_def->set_name(KVADBPROTOCOL_ATTRNAME);
      bustype_def->set_type(CANDB_ATTR_TYPE_STRING);
      dbc->insert_attribute_definition(bustype_def);
    }
    CANdbAttribute *new_attr = new CANdbAttribute(bustype_def);
    attr_list->insert(new_attr);
    prot_attr = new_attr;
  } else if ((NULL != prot_attr) && (CANDB_ATTR_TYPE_STRING != prot_attr->get_type())) {
    return kvaDbErr_DatabaseInternal;
    //Attribute is (un)intentionally corrupted. Or it is some future incompatible version of dbc.
  }

  if (NULL != prot_attr) {
    prot_attr->set_string_value(KVADBPROTOCOL_STRINGS[prot]); //note earlier CompilerAssert
  }

  return kvaDbOK;
}

//===========================================================================
KvaDbStatus WINAPI kvaDbGetFirstMsg(KvaDbHnd dh, KvaDbMessageHnd *mh)
{
  RegisterDeletionCallbacks();
  KvaDbStatus status;

  if (!kvadbhnd.is_valid(dh) || !mh) return kvaDbErr_Param;

  CANdbCluster *dc = kvadbhnd.convert(dh);
  if (!dc) return kvaDbErr_NoDatabase;

  CANdbMessage *m = dc->get_first_message();
  if (!m) return kvaDbErr_NoMsg;

  status = kvadbmessagehnd.find_or_add(m, mh);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbGetNextMsg(KvaDbHnd dh, KvaDbMessageHnd *mh)
{
  RegisterDeletionCallbacks();
  KvaDbStatus status;

  if (!kvadbhnd.is_valid(dh) || !mh) return kvaDbErr_Param;

  CANdbCluster *dc = kvadbhnd.convert(dh);
  if (!dc) return kvaDbErr_NoDatabase;

  CANdbMessage *m = dc->get_next_message();
  if (!m) {
    *mh = NULL;
    return kvaDbErr_NoMsg;
  }

  status = kvadbmessagehnd.find_or_add(m, mh);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbGetMsgById(KvaDbHnd dh, unsigned int id, KvaDbMessageHnd *mh)
{
  RegisterDeletionCallbacks();
  KvaDbStatus status;

  if (!kvadbhnd.is_valid(dh) || !mh) return kvaDbErr_Param;
  CANdbCluster *dc = kvadbhnd.convert(dh);

  CANdbMessage *m = dc->find_message_by_id(id);
  if (!m) return kvaDbErr_NoMsg;

  status = kvadbmessagehnd.find_or_add(m, mh);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbGetMsgByIdEx(KvaDbHnd dh, unsigned int id,  unsigned int flags, KvaDbMessageHnd *mh)
{
  RegisterDeletionCallbacks();
  KvaDbStatus status;

  if (!kvadbhnd.is_valid(dh) || !mh) return kvaDbErr_Param;
  CANdbCluster *dc = kvadbhnd.convert(dh);

  // set bits 31..29 to 0
  id &= ~0xE0000000;
  id |= flags & KVADB_MESSAGE_EXT;
  CANdbMessage *m = dc->find_message_by_id(id);
  if (!m)
    return kvaDbErr_NoMsg;

  status = kvadbmessagehnd.find_or_add(m, mh);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbGetMsgByPGN(KvaDbHnd dh, unsigned int id, KvaDbMessageHnd *mh)
{
  RegisterDeletionCallbacks();
  KvaDbStatus   status;
  CANdbCluster *dc;
  CANdbMessage *m;

  if (!kvadbhnd.is_valid(dh) || !mh) return kvaDbErr_Param;
  dc = kvadbhnd.convert(dh);

  m = dc->find_message_by_pgn(id);
  if (!m) return kvaDbErr_NoMsg;

  status = kvadbmessagehnd.find_or_add(m, mh);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbGetMsgByPGNEx(KvaDbHnd dh, unsigned int id, KvaDbMessageHnd *mh)
{
  RegisterDeletionCallbacks();
  KvaDbStatus   status;
  CANdbCluster *dc;
  CANdbMessage *m;

  if (!kvadbhnd.is_valid(dh) || !mh) return kvaDbErr_Param;
  dc = kvadbhnd.convert(dh);

  // set bits 31..29 to 0
  id &= ~0xE0000000;
  // j1939 message is always extended
  id |= KVADB_MESSAGE_EXT;
  m = dc->find_message_by_pgn(id);
  if (!m) return kvaDbErr_NoMsg;

  status = kvadbmessagehnd.find_or_add(m, mh);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbGetMsgByName(KvaDbHnd dh, const char *msg_name, KvaDbMessageHnd *mh)
{
  RegisterDeletionCallbacks();
  KvaDbStatus status;

  if (!kvadbhnd.is_valid(dh) || !mh) return kvaDbErr_Param;
  CANdbCluster *dc = kvadbhnd.convert(dh);

  CANdbMessage *m = dc->find_message_by_name(msg_name);
  if (!m) return kvaDbErr_NoMsg;

  status = kvadbmessagehnd.find_or_add(m, mh);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbGetMsgName(KvaDbMessageHnd mh, char *buf, size_t buflen)
{
  RegisterDeletionCallbacks();
  if (!kvadbmessagehnd.is_valid(mh)) return kvaDbErr_Param;
  if (!buf || (buflen == 0))         return kvaDbErr_Param;

  CANdbMessage *m = kvadbmessagehnd.convert(mh);
  if (m && m->get_name()) strncpy(buf, m->get_name(), buflen);
  else strcpy(buf, "");
  buf[buflen-1] = 0;
  return kvaDbOK;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbGetMsgQualifiedName(KvaDbMessageHnd mh, char *buf, size_t buflen)
{
  RegisterDeletionCallbacks();
  if (!kvadbmessagehnd.is_valid(mh)) return kvaDbErr_Param;
  if (!buf)                          return kvaDbErr_Param;

  CANdbMessage *m = kvadbmessagehnd.convert(mh);

  m->get_qualified_name(buf, (int)buflen);
  if (buflen > 0) {
    buf[buflen-1] = 0;
  }
  return kvaDbOK;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbGetMsgComment(KvaDbMessageHnd mh, char *buf, size_t buflen)
{
  RegisterDeletionCallbacks();
  if (!kvadbmessagehnd.is_valid(mh)) return kvaDbErr_NoMsg;
  if (!buf || (buflen == 0))         return kvaDbErr_Param;

  CANdbMessage *m = kvadbmessagehnd.convert(mh);

  if (m && m->get_comment()) strncpy(buf, m->get_comment(), buflen);
  else strcpy(buf, "");
  buf[buflen-1] = 0;
  return kvaDbOK;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbGetMsgId(KvaDbMessageHnd mh, unsigned int *id, unsigned int *flags)
{
  RegisterDeletionCallbacks();
  if (!kvadbmessagehnd.is_valid(mh)) return kvaDbErr_Param;
  if (!id || !flags)                 return kvaDbErr_Param;

  CANdbMessage *m = kvadbmessagehnd.convert(mh);
  *id = m->get_id_and_ext();
  *flags = 0;
  return kvaDbOK;
}

//===========================================================================
KvaDbStatus WINAPI kvaDbGetMsgIdEx(KvaDbMessageHnd mh, unsigned int *id)
{
  RegisterDeletionCallbacks();
  if (!kvadbmessagehnd.is_valid(mh)) return kvaDbErr_Param;
  if (!id )                          return kvaDbErr_Param;

  CANdbMessage *m = kvadbmessagehnd.convert(mh);

  *id = m->get_id();
  return kvaDbOK;
}

//===========================================================================
KvaDbStatus WINAPI kvaDbGetMsgFlags(KvaDbMessageHnd mh, unsigned int *flags)
{
  RegisterDeletionCallbacks();
  if (!kvadbmessagehnd.is_valid(mh)) return kvaDbErr_Param;
  if (!flags)                        return kvaDbErr_Param;

  CANdbMessage *m = kvadbmessagehnd.convert(mh);

  *flags = 0;
  if (m->is_extended()) *flags  = KVADB_MESSAGE_EXT;
  if (m->is_j1939())    *flags |= KVADB_MESSAGE_J1939;

  return kvaDbOK;
}

//===========================================================================
KvaDbStatus WINAPI kvaDbGetCanMsgFlags(KvaDbMessageHnd mh, unsigned int *flags)
{
  RegisterDeletionCallbacks();
  if (!kvadbmessagehnd.is_valid(mh)) return kvaDbErr_Param;
  if (!flags)                        return kvaDbErr_Param;

  CANdbMessage *m = kvadbmessagehnd.convert(mh);

  *flags = canMSG_STD;
  if (m->is_extended()) *flags = canMSG_EXT;
  if (m->is_canfd()) {
    *flags |= canFDMSG_FDF;
    if (m->is_brs())      *flags |= canFDMSG_BRS;
  }

  return kvaDbOK;
}

//===========================================================================
KvaDbStatus WINAPI kvaDbGetMsgDlc(KvaDbMessageHnd mh, int *dlc)
{
  RegisterDeletionCallbacks();
  if (!kvadbmessagehnd.is_valid(mh)) return kvaDbErr_Param;
  if (!dlc)                          return kvaDbErr_Param;

  CANdbMessage *m = kvadbmessagehnd.convert(mh);

  *dlc = m->get_dlc();
  return kvaDbOK;
}

//===========================================================================
KvaDbStatus WINAPI kvaDbGetMsgSendNode(KvaDbMessageHnd mh, KvaDbNodeHnd *nh)
{
  RegisterDeletionCallbacks();
  KvaDbStatus status;

  if (!kvadbmessagehnd.is_valid(mh)) return kvaDbErr_Param;
  if (!nh)                           return kvaDbErr_Param;

  CANdbMessage *m = kvadbmessagehnd.convert(mh);

  CANdbNode *n = m->get_send_node();
  if (!n) return kvaDbErr_NoNode;

  status = kvadbnodehnd.find_or_add(n, nh);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}

//===========================================================================
KvaDbStatus WINAPI kvaDbAddMsg(KvaDbHnd dh, KvaDbMessageHnd *mh)
{
  RegisterDeletionCallbacks();
  CANdbCluster *dc;
  KvaDbStatus   status;

  if (!kvadbhnd.is_valid(dh) || !mh ) return kvaDbErr_Param;

  dc = kvadbhnd.convert(dh);

  //if (!dc) return kvaDbErr_NoDatabase;
  //if (dc->get_candb_count() > 1) return kvaDbErr_OnlyOneAllowed;
  CANdb *candb = dc->get_first_candb();
  if (!candb) return kvaDbErr_NoDatabase;

  CANdbMessage *m = new CANdbMessage();
  if (candb->insert_message(m) < 0) {
    status = kvaDbErr_Param;
    delete m;
  } else {
    m->set_name("");
    m->set_id(0);
    status = kvadbmessagehnd.add(m, mh);
    if (status != kvaDbOK) status = kvaDbErr_Internal;
  }
  return status;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbDeleteMsg(KvaDbHnd dh, KvaDbMessageHnd mh)
{
  RegisterDeletionCallbacks();
  CANdbCluster *dc;
  KvaDbStatus   status = kvaDbOK;
  CANdbMessage *m;
  bool          success = false;

  if (!kvadbhnd.is_valid(dh) || !kvadbmessagehnd.is_valid(mh)) return kvaDbErr_Param;

  m  = kvadbmessagehnd.convert(mh);
  dc = kvadbhnd.convert(dh);

  CANdb *candb = dc->get_first_candb();
  if (!candb) return kvaDbErr_NoDatabase;

  while (candb) {
    if (candb->delete_message(m) == 0) {
      success = true;
      break;
    }
    candb = dc->get_next_candb();
  }

  status = kvadbmessagehnd.remove(mh);
  if (status != kvaDbOK) status = kvaDbErr_Internal;

  if (!success) status = kvaDbErr_DatabaseInternal;
  return status;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbSetFlags(KvaDbHnd /*dh*/, unsigned int /*flags*/)
{
  RegisterDeletionCallbacks();
  return kvaDbOK;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbSetMsgName(KvaDbMessageHnd mh, char *buf)
{
  RegisterDeletionCallbacks();
  if (!kvadbmessagehnd.is_valid(mh)) return kvaDbErr_Param;
  if (!buf)                          return kvaDbErr_Param;

  CANdbMessage *m     = kvadbmessagehnd.convert(mh);
  CANdb        *candb = m->get_candb();

  if (!candb) return kvaDbErr_NoDatabase;

  CANdbMessage *exmsg = candb->find_message_by_name(buf);
  if (!exmsg) {
    m->set_name(buf);
  }
  else if (m != exmsg) {
    return kvaDbErr_OnlyOneAllowed;
  }
  return kvaDbOK;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbSetMsgComment(KvaDbMessageHnd mh, char *buf)
{
  RegisterDeletionCallbacks();
  if (!kvadbmessagehnd.is_valid(mh)) return kvaDbErr_Param;
  if (!buf)                          return kvaDbErr_Param;

  CANdbMessage *m = kvadbmessagehnd.convert(mh);

  m->set_comment(buf);
  return kvaDbOK;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbSetMsgId(KvaDbMessageHnd mh, unsigned int id, unsigned int flags)
{
  RegisterDeletionCallbacks();
  if (!kvadbmessagehnd.is_valid(mh)) return kvaDbErr_Param;

  CANdbMessage *m = kvadbmessagehnd.convert(mh);

  m->set_id(id & ~KVADB_MESSAGE_EXT);
  m->set_extended(id & KVADB_MESSAGE_EXT? true: false);
  (void)flags; // flags are and will be ignored in kvaDbSetMsgId. This function is deprecated.
  return kvaDbOK;
}

//===========================================================================
KvaDbStatus WINAPI kvaDbSetMsgIdEx(KvaDbMessageHnd mh, unsigned int id)
{
  RegisterDeletionCallbacks();
  if (!kvadbmessagehnd.is_valid(mh)) return kvaDbErr_Param;

  CANdbMessage *m = kvadbmessagehnd.convert(mh);

  if (id & 0xE0000000 /* >29bit ID */) return kvaDbErr_Param;

  m->set_id(id);
  return kvaDbOK;
}

//===========================================================================
KvaDbStatus WINAPI kvaDbSetMsgFlags(KvaDbMessageHnd mh, unsigned int flags)
{
  RegisterDeletionCallbacks();
  if (!kvadbmessagehnd.is_valid(mh)) return kvaDbErr_Param;

  CANdbMessage *m = kvadbmessagehnd.convert(mh);

  CANdb *db = m->get_candb();
  if (!db) return kvaDbErr_NoDatabase;

  // A call to get_j1939Type() is preferable, but is currently not possible if db isn't first written to file and reopened
  CANdbAttribute *attr = NULL;
  CANdbAttributeDefinition *attr_def = db->find_attribute_definition_by_name("VFrameFormat");
  if (attr_def) {
    if (attr_def->get_owner() != CANDB_ATTR_OWNER_MESSAGE) return kvaDbErr_WrongOwner;

    CANdbAttributeList *aList = m->get_attributes();
    if (!aList) return kvaDbErr_NoAttrib;
    if (aList->find_by_definition(attr_def)) return kvaDbErr_Fail;

    attr = new CANdbAttribute(attr_def);
    aList->insert(attr);
  }

  if ( flags == KVADB_MESSAGE_STD ) {
    if (attr) {
      attr->set_enumeration_value(0);
    }
  } else if ( flags & KVADB_MESSAGE_J1939 ) {
    if (attr) {
      attr->set_enumeration_value(3);
      m->set_extended(true);
      m->set_j1939();
    } else return kvaDbErr_NoAttrib; // VFrameFormat must be defined for J1939
  }
  else if ( flags & KVADB_MESSAGE_EXT ) {
    m->set_extended(true);
    if (attr) {
      attr->set_enumeration_value(1);
    }
  } else return kvaDbErr_Param; //unknown flag

	return kvaDbOK;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbSetMsgDlc(KvaDbMessageHnd mh, int dlc)
{
  RegisterDeletionCallbacks();
  if (!kvadbmessagehnd.is_valid(mh)) return kvaDbErr_Param;

  KvaDbStatus status = kvaDbOK;
  CANdbMessage *m    = kvadbmessagehnd.convert(mh);

  m->set_dlc(dlc);
  return status;
}

//===========================================================================
KvaDbStatus WINAPI kvaDbSetMsgSendNode(KvaDbMessageHnd mh, KvaDbNodeHnd nh)
{
  RegisterDeletionCallbacks();
  if (!kvadbmessagehnd.is_valid(mh)) return kvaDbErr_Param;
  if (!kvadbnodehnd.is_valid(nh))    return kvaDbErr_Param;

  CANdbMessage *m = kvadbmessagehnd.convert(mh);
  CANdbNode    *n = kvadbnodehnd.convert(nh);
  m->set_send_node(n);
  return kvaDbOK;
}

//===========================================================================
KvaDbStatus WINAPI kvaDbGetFirstSignal(KvaDbMessageHnd mh, KvaDbSignalHnd *sh)
{
  RegisterDeletionCallbacks();
  KvaDbStatus status;

  if (!kvadbmessagehnd.is_valid(mh)) return kvaDbErr_Param;
  if (!sh)                           return kvaDbErr_Param;

  CANdbMessage *m = kvadbmessagehnd.convert(mh);
  if (!m) return kvaDbErr_NoMsg;

  CANdbSignal  *s = m->get_first_signal();
  if (!s) {
    *sh = NULL;
    return kvaDbErr_NoSignal;
  }

  status = kvadbsignalhnd.find_or_add(s, sh);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbGetNextSignal(KvaDbMessageHnd mh, KvaDbSignalHnd *sh)
{
  RegisterDeletionCallbacks();
  KvaDbStatus status;

  if (!kvadbmessagehnd.is_valid(mh)) return kvaDbErr_Param;
  if (!sh)                           return kvaDbErr_Param;

  CANdbMessage *m = kvadbmessagehnd.convert(mh);
  CANdbSignal  *s = m->get_next_signal();
  if (!s) {
    *sh = NULL;
    return kvaDbErr_NoSignal;
  }

  status = kvadbsignalhnd.find_or_add(s, sh);

  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}

//===========================================================================
// Get a signal given its name and its message.
KvaDbStatus WINAPI kvaDbGetSignalByName(KvaDbMessageHnd mh, char *buf, KvaDbSignalHnd *sh)
{
  RegisterDeletionCallbacks();
  KvaDbStatus status;

  if (!kvadbmessagehnd.is_valid(mh)) return kvaDbErr_Param;
  if (!sh || !buf)                   return kvaDbErr_Param;

  CANdbMessage *m = kvadbmessagehnd.convert(mh);
  CANdbSignal  *s = m->find_signal_by_name(buf);
  if (!s) {
    *sh = NULL;
    return kvaDbErr_NoSignal;
  }

  status = kvadbsignalhnd.find_or_add(s, sh);

  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}

//===========================================================================
// Get a message associated with the signal.
KvaDbStatus WINAPI kvaDbGetSignalMessage(KvaDbSignalHnd sh, KvaDbMessageHnd *mh)
{
  RegisterDeletionCallbacks();
  KvaDbStatus status;

  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;
  if (!mh)                          return kvaDbErr_Param;

  CANdbSignal *s = kvadbsignalhnd.convert(sh);

  status = kvadbmessagehnd.find(s->get_message(), mh);

  return status;
}

//===========================================================================
// Tell, if a signal is a multiplexor, multiplexed or not multiplexed.
KvaDbStatus WINAPI kvaDbGetSignalMode(KvaDbSignalHnd sh, int *mux)
{
  RegisterDeletionCallbacks();
  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;
  if (!mux) return kvaDbErr_Param;

  CANdbSignal *s = kvadbsignalhnd.convert(sh);
  *mux = s->is_mode_signal()?KVADB_MUX_SIGNAL:s->get_mode();
  /* takes value KVADB_MUX_SIGNAL, if mux; KVADB_INDEPENDENT, if independent; otherwise returns mux mode value*/
  return kvaDbOK;
}

//===========================================================================
// Set signal's multiplexed value, updates mod_signal indicator
KvaDbStatus WINAPI kvaDbSetSignalMode(KvaDbSignalHnd sh, int mux)
{
  RegisterDeletionCallbacks();
  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;
  if (mux < KVADB_MUX_SIGNAL)       return kvaDbErr_Param;

  CANdbSignal *s = kvadbsignalhnd.convert(sh);
  if (mux < KVADB_MUX_INDEPENDENT) {s->set_mode_signal(true);}
  else {s->set_mode_signal(false);}
  s->set_mode(mux);
  return kvaDbOK;
}

//===========================================================================
// Find a multiplexor signal for the given signal.
KvaDbStatus WINAPI kvaDbGetMsgMux(KvaDbMessageHnd mh, KvaDbSignalHnd *sh)
{
  RegisterDeletionCallbacks();
  KvaDbStatus   status;
  CANdbMessage *msg;
  CANdbSignal  *sig;

  if (!kvadbmessagehnd.is_valid(mh)) return kvaDbErr_Param;
  if (!sh)                           return kvaDbErr_Param;

  msg = kvadbmessagehnd.convert(mh);
  sig = msg->get_mode_signal();

  if (sig) {
    status = kvadbsignalhnd.find_or_add(sig, sh);
    if (status != kvaDbOK) {
      status = kvaDbErr_NoSignal;
    }
  } else {
    *sh = NULL;
    status = kvaDbOK;
  }

  return status;
}

//===========================================================================
KvaDbStatus WINAPI kvaDbGetSignalValueFloat(KvaDbSignalHnd sh, double *val,
                                            void *data, size_t len)
{
  RegisterDeletionCallbacks();
  int res = 0;

  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;
  if (!val || !data)                return kvaDbErr_Param;

  CANdbSignal *sig = kvadbsignalhnd.convert(sh);
  CANdbMessage *sig_msg = sig->get_message();

  if (!sig_msg) return kvaDbErr_Param;

  if (!sig->get_message()) return kvaDbErr_Param;

  int msg_dlc = sig_msg->get_dlc();

  if ((size_t)msg_dlc > len) return kvaDbErr_Param; //buffer is too small

  switch (sig->get_type()) {
    case CANDB_SIGNED:
    {
      int64_t value = 0;
      res = sig->get_value_int((unsigned char*)data, msg_dlc, value);
      *val = sig->raw2phys((double)value);
      break;
    }
    case CANDB_UNSIGNED:
    {
      uint64_t value = 0;
      res = sig->get_value_uint((unsigned char*)data, msg_dlc, value);
      *val = sig->raw2phys((double)value);
      break;
    }
    case CANDB_FLOAT:
    {
      double value = 0;
      res = sig->get_value_float((unsigned char*)data, msg_dlc, value);
      *val = sig->raw2phys((double)value);
      break;
    }
    case CANDB_DOUBLE:
    {
      double value = 0;
      res = sig->get_value_double((unsigned char*)data, msg_dlc, value);
      *val = sig->raw2phys((double)value);
      break;
    }
    default:
      return kvaDbErr_DatabaseInternal;
  }
  if (res != 0) return kvaDbErr_Param;
  return kvaDbOK;
}

//===========================================================================
KvaDbStatus WINAPI kvaDbGetSignalValueInteger(KvaDbSignalHnd sh, int *val,
                                              void *data, size_t len)
{
  RegisterDeletionCallbacks();
  int res = 0;

  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;
  if (!val || !data)                return kvaDbErr_Param;

  CANdbSignal *sig = kvadbsignalhnd.convert(sh);
  CANdbMessage *sig_msg = sig->get_message();

  if (!sig_msg) return kvaDbErr_Param;

  int msg_dlc = sig_msg->get_dlc();

  if ((size_t)msg_dlc > len) return kvaDbErr_Param; //buffer is too small

  switch (sig->get_type()) {
    case CANDB_SIGNED:
    {
      int64_t value = 0;
      res = sig->get_value_int((unsigned char*)data, msg_dlc, value);
      *val = roundi(sig->raw2phys((double)value));
      break;
    }
    case CANDB_UNSIGNED:
    {
      uint64_t value = 0;
      res = sig->get_value_uint((unsigned char*)data, msg_dlc, value);
      *val = roundu(sig->raw2phys((double)value));
      break;
    }
    case CANDB_FLOAT:
    {
      double value = 0;
      res = sig->get_value_float((unsigned char*)data, msg_dlc, value);
      *val = roundi(sig->raw2phys((double)value));
      break;
    }
    case CANDB_DOUBLE:
    {
      double value = 0;
      res = sig->get_value_double((unsigned char*)data, msg_dlc, value);
      *val = roundi(sig->raw2phys((double)value));
      break;
    }
    default:
      return kvaDbErr_DatabaseInternal;
  }
  if (res != 0) return kvaDbErr_Param;
  return kvaDbOK;
}

//===========================================================================
KvaDbStatus WINAPI kvaDbRetrieveSignalValuePhys(KvaDbSignalHnd sh, double *val,
                                                void *can_data, size_t len)
{
  RegisterDeletionCallbacks();
  int res = 0;

  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;
  if (!val || !can_data)            return kvaDbErr_Param;

  CANdbSignal *sig = kvadbsignalhnd.convert(sh);
  CANdbMessage *sig_msg = sig->get_message();

  if (!sig_msg) return kvaDbErr_Param;

  if (!sig->get_message()) return kvaDbErr_Param;

  // Do we have enough data in order to decode the signal?
  // len is number of bytes, signal is counting bits.
  if (sig->is_signal_too_large(len)) return kvaDbErr_Param;

  switch (sig->get_type()) {
    case CANDB_SIGNED:
    {
      int64_t value = 0;
      res = sig->get_value_int((unsigned char*)can_data, (int)len, value);
      *val = sig->raw2phys((double)value);
      break;
    }
    case CANDB_UNSIGNED:
    {
      uint64_t value = 0;
      res = sig->get_value_uint((unsigned char*)can_data, (int)len, value);
      *val = sig->raw2phys((double)value);
      break;
    }
    case CANDB_FLOAT:
    {
      double value = 0;
      res = sig->get_value_float((unsigned char*)can_data, (int)len, value);
      *val = sig->raw2phys((double)value);
      break;
    }
    case CANDB_DOUBLE:
    {
      double value = 0;
      res = sig->get_value_double((unsigned char*)can_data, (int)len, value);
      *val = sig->raw2phys((double)value);
      break;
    }
    default:
      return kvaDbErr_DatabaseInternal;
  }
  if (res != 0) return kvaDbErr_Param;
  return kvaDbOK;
}

//===========================================================================
KvaDbStatus WINAPI kvaDbRetrieveSignalValueRaw(KvaDbSignalHnd sh, int *val,
                                               void *can_data, size_t len)
{
  RegisterDeletionCallbacks();
  int res = 0;

  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;
  if (!val || !can_data)            return kvaDbErr_Param;

  CANdbSignal *sig = kvadbsignalhnd.convert(sh);
  CANdbMessage *sig_msg = sig->get_message();

  if (!sig_msg) return kvaDbErr_Param;

  // Do we have enough data in order to decode the signal?
  // len is number of bytes, signal is counting bits.
  if (sig->is_signal_too_large(len)) return kvaDbErr_Param;

  {
    uint64_t value;
    res = sig->get_value_uint((unsigned char*)can_data, (int)len, value);
    if (res != 0) return kvaDbErr_Param;
    *val = (int)value;
  }

  return kvaDbOK;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbRetrieveSignalValueRaw64(KvaDbSignalHnd sh, uint64_t *val,
                                              void *can_data, size_t len)
{
  RegisterDeletionCallbacks();
  int res = 0;

  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;
  if (!val || !can_data)            return kvaDbErr_Param;

  CANdbSignal *sig      = kvadbsignalhnd.convert(sh);
  CANdbMessage *sig_msg = sig->get_message();

  if (!sig_msg) return kvaDbErr_Param;

  // Do we have enough data in order to decode the signal?
  // len is number of bytes, signal is counting bits.
  if (sig->is_signal_too_large(len)) return kvaDbErr_Param;

  {
    uint64_t value;
    res = sig->get_value_uint((unsigned char*)can_data, (int)len, value);
    if (res != 0) return kvaDbErr_Param;
    *val = value;
  }

  return kvaDbOK;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbGetSignalValueEnum(KvaDbSignalHnd sh, char *buf,
                                            size_t buflen, void *data, size_t len)
{
  RegisterDeletionCallbacks();
  int val;

  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;
  if (!buf || !data || !buflen)     return kvaDbErr_Param;

  CANdbSignal *sig = kvadbsignalhnd.convert(sh);
  if (!sig->has_symbolic_values()) return kvaDbErr_Fail;

  KvaDbStatus status = kvaDbGetSignalValueInteger(sh, &val, data, len);
  if (status != kvaDbOK) return status;
  const char *name = sig->get_value_string(val);
  if (!name) return kvaDbErr_Fail;
  strncpy(buf, name, buflen);
  buf[buflen-1] = '\0';
  return kvaDbOK;
}

//===========================================================================
KvaDbStatus WINAPI kvaDbStoreSignalValuePhys(KvaDbSignalHnd sh, void *can_data, int dlc, double value)
{
  RegisterDeletionCallbacks();
  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;
  if (!can_data || (dlc == 0))      return kvaDbErr_Param;

  CANdbSignal *sig = kvadbsignalhnd.convert(sh);

  double rawval = sig->phys2raw(value);
  if (sig->store_value((unsigned char*)can_data, dlc, rawval)) {
    return kvaDbErr_Fail;
  }

  return kvaDbOK;
}
//===========================================================================
KvaDbStatus WINAPI kvaDbStoreSignalValueRaw(KvaDbSignalHnd sh, void *can_data, int dlc, int value)
{
  RegisterDeletionCallbacks();
  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;
  if (!can_data || (dlc == 0))      return kvaDbErr_Param;

  CANdbSignal *sig = kvadbsignalhnd.convert(sh);

  if (sig->store_value_raw((unsigned char*)can_data, dlc, value)) {
    return kvaDbErr_Fail;
  }

  return kvaDbOK;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbStoreSignalValueRaw64(KvaDbSignalHnd sh, void *can_data, int dlc, uint64_t value)
{
  RegisterDeletionCallbacks();
  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;
  if (!can_data || (dlc == 0))      return kvaDbErr_Param;

  CANdbSignal *sig = kvadbsignalhnd.convert(sh);

  if (sig->store_value_raw((unsigned char*)can_data, dlc, value)) {
    return kvaDbErr_Fail;
  }

  return kvaDbOK;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbGetSignalValueLimits(KvaDbSignalHnd sh, double *minval, double *maxval)
{
  RegisterDeletionCallbacks();
  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;
  if (!minval || !maxval)           return kvaDbErr_Param;

  CANdbSignal *sig = kvadbsignalhnd.convert(sh);
  *minval = sig->get_min_val();
  *maxval = sig->get_max_val();
  return kvaDbOK;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbGetSignalValueScaling(KvaDbSignalHnd sh, double *factor, double *offset)
{
  RegisterDeletionCallbacks();
  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;
  if (!factor || !offset)           return kvaDbErr_Param;

  CANdbSignal *sig = kvadbsignalhnd.convert(sh);
  *factor = sig->get_factor();
  *offset = sig->get_offset();
  return kvaDbOK;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbGetSignalValueSize(KvaDbSignalHnd sh, int *startbit, int *length)
{
  RegisterDeletionCallbacks();
  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;
  if (!startbit || !length)         return kvaDbErr_Param;

  CANdbSignal *sig = kvadbsignalhnd.convert(sh);
  *startbit = sig->get_start_bit();
  *length = sig->get_length();
  return kvaDbOK;
}

//===========================================================================
KvaDbStatus WINAPI kvaDbGetSignalName(KvaDbSignalHnd sh, char *buf, size_t buflen)
{
  RegisterDeletionCallbacks();
  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;
  if (!buf)                         return kvaDbErr_Param;

  CANdbSignal *sig = kvadbsignalhnd.convert(sh);
  strncpy(buf, sig->get_name(), buflen);
  if (buflen > 0) {
    buf[buflen-1] = 0;
  }
  return kvaDbOK;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbGetSignalQualifiedName(KvaDbSignalHnd sh, char *buf, size_t buflen)
{
  RegisterDeletionCallbacks();
  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;
  if (!buf)                         return kvaDbErr_Param;

  CANdbSignal *sig = kvadbsignalhnd.convert(sh);
  sig->get_qualified_name(buf, (int)buflen);
  if (buflen > 0) {
    buf[buflen-1] = 0;
  }
  return kvaDbOK;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbGetSignalComment(KvaDbSignalHnd sh, char *buf, size_t buflen)
{
  RegisterDeletionCallbacks();
  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;
  if (!buf || (buflen == 0))        return kvaDbErr_Param;

  CANdbSignal *sig = kvadbsignalhnd.convert(sh);
  if (sig && sig->get_comment()) strncpy(buf, sig->get_comment(), buflen);
  else strcpy(buf, "");
  buf[buflen-1] = 0;
  return kvaDbOK;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbGetSignalUnit(KvaDbSignalHnd sh, char *buf, size_t buflen)
{
  RegisterDeletionCallbacks();
  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;
  if (!buf)                         return kvaDbErr_Param;

  CANdbSignal *sig = kvadbsignalhnd.convert(sh);

  if (sig && sig->get_unit()) {
    strncpy(buf, sig->get_unit(), buflen);
    buf[buflen-1] = 0;
  } else {
    if (buflen > 0) buf[0] = 0;
  }
  return kvaDbOK;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbGetSignalEncoding(KvaDbSignalHnd sh, KvaDbSignalEncoding *e)
{
  RegisterDeletionCallbacks();
  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;
  if (!e)                            return kvaDbErr_Param;

  CANdbSignal *sig = kvadbsignalhnd.convert(sh);
  if (sig->is_motorola()) *e = kvaDb_Motorola;
  else *e = kvaDb_Intel;
  return kvaDbOK;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbGetSignalRepresentationType(KvaDbSignalHnd sh, KvaDbSignalType *t)
{
  RegisterDeletionCallbacks();
  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;
  if (!t)                           return kvaDbErr_Param;

  CANdbSignal *sig = kvadbsignalhnd.convert(sh);
  switch (sig->get_type()) {
    case CANDB_SIGNED:      *t = kvaDb_Signed; break;
    case CANDB_UNSIGNED:    *t = kvaDb_Unsigned; break;
    case CANDB_FLOAT:       *t = kvaDb_Float; break;
    case CANDB_DOUBLE:      *t = kvaDb_Double; break;
    default:
      return kvaDbErr_DatabaseInternal;
  }
  return kvaDbOK;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbGetSignalPresentationType(KvaDbSignalHnd sh, KvaDbSignalType *t)
{
  RegisterDeletionCallbacks();
  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;
  if (!t)                           return kvaDbErr_Param;

  CANdbSignal *sig = kvadbsignalhnd.convert(sh);
  switch (sig->get_scaled_type()) {
    case CANDB_SIGNED:      *t = kvaDb_Signed; break;
    case CANDB_UNSIGNED:    *t = kvaDb_Unsigned; break;
    case CANDB_FLOAT:       *t = kvaDb_Float; break;
    case CANDB_DOUBLE:      *t = kvaDb_Double; break;
    default:
      return kvaDbErr_DatabaseInternal;
  }
  return kvaDbOK;
}

//===========================================================================
KvaDbStatus WINAPI kvaDbAddSignal(KvaDbMessageHnd mh, KvaDbSignalHnd *sh)
{
  RegisterDeletionCallbacks();
  if (!kvadbmessagehnd.is_valid(mh)) return kvaDbErr_Param;
  if (!sh)                           return kvaDbErr_Param;

  CANdbSignal  *sig;
  CANdbMessage *m      = kvadbmessagehnd.convert(mh);
  KvaDbStatus   status = kvaDbOK;

  sig = new CANdbSignal();
  if (m->insert_signal(sig) < 0) {
    status = kvaDbErr_Param;
    delete sig;
    *sh = 0;
  } else {
    sig->set_name("");
    status = kvadbsignalhnd.add(sig, sh);
    if (status != kvaDbOK) status = kvaDbErr_Internal;
  }

  return status;
}

//===========================================================================
KvaDbStatus WINAPI kvaDbDeleteSignal(KvaDbMessageHnd mh, KvaDbSignalHnd sh)
{
  RegisterDeletionCallbacks();
  KvaDbStatus status = kvaDbOK;

  if (!kvadbsignalhnd.is_valid(sh))  return kvaDbErr_Param;
  if (!kvadbmessagehnd.is_valid(mh)) return kvaDbErr_Param;

  CANdbMessage *m = kvadbmessagehnd.convert(mh);
  CANdbSignal  *s = kvadbsignalhnd.convert(sh);

  if (m->delete_signal(s) < 0) {
    status = kvaDbErr_DatabaseInternal;
  } else {
    status = kvadbsignalhnd.remove(sh);
    if (status != kvaDbOK) status = kvaDbErr_Internal;
  }

  return status;
}

//===========================================================================
KvaDbStatus WINAPI kvaDbSetSignalValueLimits(KvaDbSignalHnd sh, double minval, double maxval)
{
  RegisterDeletionCallbacks();
  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;

  CANdbSignal *sig = kvadbsignalhnd.convert(sh);
  sig->set_min_val(minval);
  sig->set_max_val(maxval);
  return kvaDbOK;
}

//===========================================================================
KvaDbStatus WINAPI kvaDbSetSignalValueScaling(KvaDbSignalHnd sh, double factor, double offset)
{
  RegisterDeletionCallbacks();
  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;

  CANdbSignal *sig = kvadbsignalhnd.convert(sh);
  sig->set_offset(offset);
  sig->set_factor(factor);
  return kvaDbOK;
}

//===========================================================================
KvaDbStatus WINAPI kvaDbSetSignalValueSize(KvaDbSignalHnd sh, int startbit, int length)
{
  RegisterDeletionCallbacks();
  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;

  CANdbSignal *sig = kvadbsignalhnd.convert(sh);
  sig->set_start_bit(startbit);
  sig->set_length(length);
  return kvaDbOK;
}

//===========================================================================
KvaDbStatus WINAPI kvaDbSetSignalName(KvaDbSignalHnd sh, char *buf)
{
  RegisterDeletionCallbacks();
  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;

  CANdbSignal *sig = kvadbsignalhnd.convert(sh);
  sig->set_name(buf);
  return kvaDbOK;
}

//===========================================================================
KvaDbStatus WINAPI kvaDbSetSignalComment(KvaDbSignalHnd sh, char *buf)
{
  RegisterDeletionCallbacks();
  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;

  CANdbSignal *sig = kvadbsignalhnd.convert(sh);
  sig->set_comment(buf);
  return kvaDbOK;
}

//===========================================================================
KvaDbStatus WINAPI kvaDbSetSignalUnit(KvaDbSignalHnd sh, char *buf)
{
  RegisterDeletionCallbacks();
  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;

  CANdbSignal *sig = kvadbsignalhnd.convert(sh);
  sig->set_unit(buf);
  return kvaDbOK;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbSetSignalEncoding(KvaDbSignalHnd sh, KvaDbSignalEncoding e)
{
  RegisterDeletionCallbacks();
  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;

  CANdbSignal *sig = kvadbsignalhnd.convert(sh);
  if (e == kvaDb_Motorola) sig->set_motorola(true);
  else sig->set_motorola(false);
  return kvaDbOK;
}


//===========================================================================
KvaDbStatus WINAPI kvaDbSetSignalRepresentationType(KvaDbSignalHnd sh, KvaDbSignalType t)
{
  RegisterDeletionCallbacks();
  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;

  CANdbSignal *sig = kvadbsignalhnd.convert(sh);
  switch (t) {
    case kvaDb_Signed:    sig->set_type(CANDB_SIGNED); break;
    case kvaDb_Unsigned:  sig->set_type(CANDB_UNSIGNED); break;
    case kvaDb_Float:     sig->set_type(CANDB_FLOAT); break;
    case kvaDb_Double:    sig->set_type(CANDB_DOUBLE); break;
    default:
      return kvaDbErr_Param;
  }

  return kvaDbOK;
}

//===========================================================================
KvaDbStatus WINAPI kvaDbGetFirstNode(KvaDbHnd dh, KvaDbNodeHnd *nh)
{
  RegisterDeletionCallbacks();
  KvaDbStatus status;

  if (!kvadbhnd.is_valid(dh) || !nh) return kvaDbErr_Param;

  CANdbCluster *dc = kvadbhnd.convert(dh);

  CANdbNode *n = dc->get_first_node();
  if (!n) return kvaDbErr_NoNode;

  status = kvadbnodehnd.find_or_add(n, nh);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}

//===========================================================================
KvaDbStatus WINAPI kvaDbGetNextNode(KvaDbHnd dh, KvaDbNodeHnd *nh)
{
  RegisterDeletionCallbacks();
  KvaDbStatus status;

  if (!kvadbhnd.is_valid(dh) || !nh) return kvaDbErr_Param;

  CANdbCluster *dc = kvadbhnd.convert(dh);

  CANdbNode *n = dc->get_next_node();
  if (!n) {
    *nh = NULL;
    return kvaDbErr_NoNode;
  }

  status = kvadbnodehnd.find_or_add(n, nh);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetNodeAttributeIntByName(KvaDbNodeHnd nh, const char *attrName, int *val)
{
  RegisterDeletionCallbacks();
  const CANdbAttribute *attr;

  if (!kvadbnodehnd.is_valid(nh)) return kvaDbErr_Param;
  if (!val)                       return kvaDbErr_Param;

  CANdbNode *node = kvadbnodehnd.convert(nh);

  CANdbAttributeList *aList = node->get_attributes();
  if (!aList) return kvaDbErr_NoAttrib;
  attr = aList->find_by_name(attrName);
  if (!attr) return kvaDbErr_NoAttrib;
  *val = attr->get_integer_value();
  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetNodeName(KvaDbNodeHnd nh, char *buf, size_t buflen)
{
  RegisterDeletionCallbacks();
  if (!kvadbnodehnd.is_valid(nh)) return kvaDbErr_Param;
  if (!buf)                       return kvaDbErr_Param;

  CANdbNode *node = kvadbnodehnd.convert(nh);

  strncpy(buf, node->get_name(), buflen);
  if (buflen > 0) {
    buf[buflen-1] = 0;
  }

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetNodeComment(KvaDbNodeHnd nh, char *buf, size_t buflen)
{
  RegisterDeletionCallbacks();
  if (!kvadbnodehnd.is_valid(nh)) return kvaDbErr_Param;
  if (!buf || (buflen == 0))      return kvaDbErr_Param;

  CANdbNode *node = kvadbnodehnd.convert(nh);

  const char *comment = node->get_comment();
  if (comment) {
    strncpy(buf, comment, buflen);
    buf[buflen - 1] = '\0';
  }
  else strcpy(buf, "");
  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetNodeByName(KvaDbHnd dh, const char *node_name, KvaDbNodeHnd *nh)
{
  RegisterDeletionCallbacks();
  KvaDbStatus status;

  if (!kvadbhnd.is_valid(dh) || !nh) return kvaDbErr_Param;

  CANdbCluster *dc = kvadbhnd.convert(dh);

  CANdbNode *n = dc->find_node_by_name(node_name);
  if (!n) return kvaDbErr_NoNode;

  status = kvadbnodehnd.find_or_add(n, nh);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbAddNode(KvaDbHnd dh, KvaDbNodeHnd *nh)
{
  RegisterDeletionCallbacks();
  KvaDbStatus   status;
  CANdbCluster *dc;

  if (!kvadbhnd.is_valid(dh) || !nh) return kvaDbErr_Param;

  dc = kvadbhnd.convert(dh);

  CANdb *candb = dc->get_first_candb();
  if (!candb) return kvaDbErr_NoDatabase;

  CANdbNode *n = new CANdbNode;

  if (candb->insert_node(n) < 0) {
    delete n;
    return kvaDbErr_Param;
  }

  status = kvadbnodehnd.add(n, nh);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbDeleteNode(KvaDbHnd dh, KvaDbNodeHnd nh)
{
  RegisterDeletionCallbacks();
  KvaDbStatus   status;
  CANdbCluster *dc;
  CANdbNode    *n;

  if (!kvadbnodehnd.is_valid(nh)) return kvaDbErr_Param;
  if (!kvadbhnd.is_valid(dh))     return kvaDbErr_Param;

  dc = kvadbhnd.convert(dh);
  n  = kvadbnodehnd.convert(nh);

  CANdb *candb = dc->get_first_candb();
  if (!candb) return kvaDbErr_NoDatabase;

  if (candb->remove_node(n) != 0) {
    return kvaDbErr_Param;
  }

  status = kvadbnodehnd.remove(nh);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbSetNodeName(KvaDbNodeHnd nh, char *buf)
{
  RegisterDeletionCallbacks();
  CANdbNode *node;

  if (!kvadbnodehnd.is_valid(nh)) return kvaDbErr_Param;
  if (!buf)                       return kvaDbErr_Param;

  node = kvadbnodehnd.convert(nh);

  CANdb *candb = node->get_candb();
  if (!candb) return kvaDbErr_NoDatabase;

  CANdbNode *exnode = candb->find_node_by_name(buf);

  if (!exnode) {
    node->set_name(buf);
  } else if (exnode != node) {
    return kvaDbErr_OnlyOneAllowed;
  }

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbSetNodeComment(KvaDbNodeHnd nh, char *buf)
{
  RegisterDeletionCallbacks();
  CANdbNode *node;

  if (!kvadbnodehnd.is_valid(nh)) return kvaDbErr_Param;
  if (!buf)                       return kvaDbErr_Param;

  node = kvadbnodehnd.convert(nh);

  node->set_comment(buf);
  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbAddReceiveNodeToSignal(KvaDbSignalHnd sh, KvaDbNodeHnd nh)
{
  RegisterDeletionCallbacks();
  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;
  if (!kvadbnodehnd.is_valid(nh))   return kvaDbErr_Param;

  CANdbSignal *sig  = kvadbsignalhnd.convert(sh);
  CANdbNode   *node = kvadbnodehnd.convert(nh);

  sig->add_receive_node(node);
  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbRemoveReceiveNodeFromSignal(KvaDbSignalHnd sh, KvaDbNodeHnd nh)
{
  RegisterDeletionCallbacks();
  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;
  if (!kvadbnodehnd.is_valid(nh))   return kvaDbErr_Param;

  CANdbSignal *sig  = kvadbsignalhnd.convert(sh);
  CANdbNode   *node = kvadbnodehnd.convert(nh);

  sig->remove_receive_node(node);
  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbSignalContainsReceiveNode(KvaDbSignalHnd sh, KvaDbNodeHnd nh)
{
  RegisterDeletionCallbacks();
  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;
  if (!kvadbnodehnd.is_valid(nh))   return kvaDbErr_Param;

  CANdbSignal *sig  = kvadbsignalhnd.convert(sh);
  CANdbNode   *node = kvadbnodehnd.convert(nh);

  if (!sig->contains_node(node)) return kvaDbErr_NoNode;

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbAddAttributeDefinition(KvaDbHnd dh, KvaDbAttributeDefHnd *adh)
{
  RegisterDeletionCallbacks();
  KvaDbStatus status;

  if (!kvadbhnd.is_valid(dh) || !adh) return kvaDbErr_Param;

  CANdbCluster * dc = kvadbhnd.convert(dh);
  if (!dc)                       return kvaDbErr_NoDatabase;
  if (dc->get_candb_count() > 1) return kvaDbErr_OnlyOneAllowed;

  CANdb *db = dc->get_first_candb();
  if (!db) return kvaDbErr_NoDatabase;

  CANdbAttributeDefinition *attr_def = new CANdbAttributeDefinition();

  attr_def->set_name("");
  attr_def->set_type(CANDB_ATTR_TYPE_INVALID);
  attr_def->set_owner(CANDB_ATTR_OWNER_INVALID);
  attr_def->set_candb(db);

  db->insert_attribute_definition(attr_def);

  status = kvadbattributedefhnd.add(attr_def, adh);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbDeleteAttributeDefinition(KvaDbHnd dh, KvaDbAttributeDefHnd adh)
{
  RegisterDeletionCallbacks();
  KvaDbStatus               status;
  CANdbAttributeDefinition *attr_def;
  CANdbCluster             *dc;
  CANdb                    *db;
  int                       delete_result;
  CANdbAttribute           *attr;
  CANdbAttributeList       *aList;

  if (!kvadbhnd.is_valid(dh) || !kvadbattributedefhnd.is_valid(adh)) return kvaDbErr_Param;

  dc = kvadbhnd.convert(dh);
  if (!dc)                       return kvaDbErr_NoDatabase;
  if (dc->get_candb_count() > 1) return kvaDbErr_OnlyOneAllowed;

  db = dc->get_first_candb();
  if (!db) return kvaDbErr_NoDatabase;

  attr_def = kvadbattributedefhnd.convert(adh);

  //don't delete if we still have attributes
  aList = db->get_attributes();
  if (aList) {
    attr = aList->find_by_definition(attr_def);
    if (attr) return kvaDbErr_InUse;
  }

  delete_result = db->delete_attribute_definition(attr_def);

  if (delete_result) {
    return kvaDbErr_Internal;
  } else {
    status = kvadbattributedefhnd.remove(adh);
    if (status != kvaDbOK) return kvaDbErr_Internal;
  }

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbSetAttributeDefinitionName(KvaDbAttributeDefHnd adh, const char *adName)
{
  RegisterDeletionCallbacks();
  if (!kvadbattributedefhnd.is_valid(adh)) return kvaDbErr_Param;
  if (!adName)                             return kvaDbErr_Param;

  CANdbAttributeDefinition *attr_def = kvadbattributedefhnd.convert(adh);

  attr_def->set_name(adName);

  CANdb* candb = attr_def->get_candb();
  if (!candb) return kvaDbErr_NoDatabase;

  CANdbAttributeDefinition *exattr_def = candb->find_attribute_definition_by_name(adName);
  if (!exattr_def) {
    attr_def->set_name(adName);
  }
  else if (attr_def != exattr_def) {
    return kvaDbErr_OnlyOneAllowed;
  }

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbSetAttributeDefinitionType(KvaDbAttributeDefHnd adh, KvaDbAttributeType adType)
{
  RegisterDeletionCallbacks();
  if (!kvadbattributedefhnd.is_valid(adh)) return kvaDbErr_Param;

  CANdbAttributeDefinition *attr_def = kvadbattributedefhnd.convert(adh);

  attr_def->set_type((CANdbAttributeType) adType);

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbSetAttributeDefinitionOwner(KvaDbAttributeDefHnd adh, KvaDbAttributeOwner adOwner)
{
  RegisterDeletionCallbacks();
  if (!kvadbattributedefhnd.is_valid(adh)) return kvaDbErr_Param;

  CANdbAttributeDefinition *attr_def = kvadbattributedefhnd.convert(adh);

  attr_def->set_owner((CANdbAttributeOwner) adOwner);

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbAddAttributeDefinitionEnum(KvaDbAttributeDefHnd adh, const char* eName, int eValue)
{
  RegisterDeletionCallbacks();
  if (!kvadbattributedefhnd.is_valid(adh)) return kvaDbErr_Param;
  if (!eName)                              return kvaDbErr_Param;

  CANdbAttributeDefinition *attr_def = kvadbattributedefhnd.convert(adh);

  if (attr_def->add_enumeration(eName, eValue) != 0) {
    return kvaDbErr_Fail;
  }

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbDeleteAttributeDefinitionEnum(KvaDbAttributeDefHnd adh, const char* eName, int eValue)
{
  RegisterDeletionCallbacks();
  if (!kvadbattributedefhnd.is_valid(adh)) return kvaDbErr_Param;
  if (!eName)                              return kvaDbErr_Param;

  CANdbAttributeDefinition *attr_def = kvadbattributedefhnd.convert(adh);

  if (attr_def->delete_enumeration(eName, eValue) != 0) {
    return kvaDbErr_Fail;
  }

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetAttributeDefinitionEnumFirst(KvaDbAttributeDefHnd adh, int *eValue, char* eName, size_t *buflen)
{
  RegisterDeletionCallbacks();
        CANdbAttributeDefinition *attr_def;
  const char*                     name;

  if (!kvadbattributedefhnd.is_valid(adh)) return kvaDbErr_Param;
  if (!eValue || !eName || !buflen)        return kvaDbErr_Param;

  attr_def = kvadbattributedefhnd.convert(adh);
  *eValue  = attr_def->get_first_enumeration();

  if (*eValue < 0) {
    return kvaDbErr_NoAttrib;
  }

  name = attr_def->get_enumeration_name_by_value(*eValue);

  if (!name) {
    return kvaDbErr_NoAttrib;
  }

  if (strlen(name) + 1 > *buflen) {
    *buflen = strlen(name) + 1;
    return kvaDbErr_Param;
  }
  else {
    strcpy(eName, name);
  }

  return kvaDbOK;
}


//===========================================================================

KvaDbStatus WINAPI kvaDbGetAttributeDefinitionEnumNext(KvaDbAttributeDefHnd adh, int *eValue, char* eName, size_t *buflen)
{
  RegisterDeletionCallbacks();
        CANdbAttributeDefinition *attr_def;
  const char*                     name;

  if (!kvadbattributedefhnd.is_valid(adh)) return kvaDbErr_Param;
  if (!eValue || !eName || !buflen)        return kvaDbErr_Param;

  attr_def = kvadbattributedefhnd.convert(adh);
  *eValue  = attr_def->get_next_enumeration();

  if (*eValue < 0) {
    return kvaDbErr_NoAttrib;
  }

  name = attr_def->get_enumeration_name_by_value(*eValue);

  if (!name) {
    return kvaDbErr_NoAttrib;
  }

  if (strlen(name) + 1 > *buflen) {
    *buflen = strlen(name) + 1;
    return kvaDbErr_Param;
  }
  else {
    strcpy(eName, name);
  }

  return kvaDbOK;
}


//===========================================================================

KvaDbStatus WINAPI kvaDbGetAttributeDefinitionEnumValueByName(KvaDbAttributeDefHnd adh, const char* eName, int* eValue)
{
  RegisterDeletionCallbacks();
  int                       val;
  CANdbAttributeDefinition *attr_def;

  if (!kvadbattributedefhnd.is_valid(adh)) return kvaDbErr_Param;
  if (!eValue || !eName)                   return kvaDbErr_Param;

  attr_def = kvadbattributedefhnd.convert(adh);
  val      = attr_def->get_enumeration_value_by_name(eName);

  if (val < 0) {
    return kvaDbErr_Fail;
  }

  *eValue = val;
  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetAttributeDefinitionEnumNameByValue(KvaDbAttributeDefHnd adh, int eValue, char* buf, size_t *buflen)
{
  RegisterDeletionCallbacks();
  if (!kvadbattributedefhnd.is_valid(adh)) return kvaDbErr_Param;
  if (!buf || !buflen)                     return kvaDbErr_Param;

  CANdbAttributeDefinition *attr_def = kvadbattributedefhnd.convert(adh);

  const char* eName = attr_def->get_enumeration_name_by_value(eValue);
  if (!eName) {
    return kvaDbErr_Fail;
  }

  if (strlen(eName) + 1 > *buflen) {
    *buflen = strlen(eName) + 1;
    return kvaDbErr_Param;
  }
  else {
    strcpy(buf, eName);
  }

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbSetAttributeDefinitionEnumDefault(KvaDbAttributeDefHnd adh, int def)
{
  RegisterDeletionCallbacks();
  if (!kvadbattributedefhnd.is_valid(adh)) return kvaDbErr_Param;

  CANdbAttributeDefinition *attr_def = kvadbattributedefhnd.convert(adh);

  attr_def->set_enumeration_default(def);

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbSetAttributeDefinitionInt(KvaDbAttributeDefHnd adh, int def, int min, int max)
{
  RegisterDeletionCallbacks();
  if (!kvadbattributedefhnd.is_valid(adh)) return kvaDbErr_Param;

  CANdbAttributeDefinition *attr_def = kvadbattributedefhnd.convert(adh);
  if (!attr_def) return kvaDbErr_NoAttrib;

  attr_def->set_integer_default(def);
  attr_def->set_integer_min(min);
  attr_def->set_integer_max(max);

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbSetAttributeDefinitionFloat(KvaDbAttributeDefHnd adh, float def, float min, float max)
{
  RegisterDeletionCallbacks();
  if (!kvadbattributedefhnd.is_valid(adh)) return kvaDbErr_Param;

  CANdbAttributeDefinition *attr_def = kvadbattributedefhnd.convert(adh);
  if (!attr_def) return kvaDbErr_NoAttrib;

  attr_def->set_float_default((double)def);
  attr_def->set_float_min((double)min);
  attr_def->set_float_max((double)max);

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbSetAttributeDefinitionString(KvaDbAttributeDefHnd adh, const char *buf)
{
  RegisterDeletionCallbacks();
  if (!kvadbattributedefhnd.is_valid(adh)) return kvaDbErr_Param;
  if (!buf)                                return kvaDbErr_Param;

  CANdbAttributeDefinition *attr_def = kvadbattributedefhnd.convert(adh);
  if (!attr_def) return kvaDbErr_NoAttrib;

  attr_def->set_string_default(buf);

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetFirstAttributeDefinition(KvaDbHnd dh, KvaDbAttributeDefHnd *adh)
{
  RegisterDeletionCallbacks();
  KvaDbStatus status;

  if (!kvadbhnd.is_valid(dh) || !adh) return kvaDbErr_Param;

  CANdbCluster * dc = kvadbhnd.convert(dh);
  if (!dc) return kvaDbErr_NoDatabase;
  if (dc->get_candb_count() > 1) return kvaDbErr_OnlyOneAllowed;
  CANdb *db = dc->get_first_candb();
  if (!db) return kvaDbErr_NoDatabase;

  CANdbAttributeDefinition *attr_def = db->get_first_attribute_definition();
  if (!attr_def) return kvaDbErr_NoAttrib;

  status = kvadbattributedefhnd.find_or_add(attr_def, adh);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetAttributeDefinitionByName(KvaDbHnd dh, const char *attrName, KvaDbAttributeDefHnd *adh)
{
  RegisterDeletionCallbacks();
  KvaDbStatus               status;
  CANdbAttributeDefinition *attr_def;

  if (!kvadbhnd.is_valid(dh) || !adh || !attrName) return kvaDbErr_Param;

  CANdbCluster *dc = kvadbhnd.convert(dh);
  if (!dc)                       return kvaDbErr_NoDatabase;
  if (dc->get_candb_count() > 1) return kvaDbErr_OnlyOneAllowed;

  CANdb *db = dc->get_first_candb();
  if (!db) return kvaDbErr_NoDatabase;

  attr_def = db->find_attribute_definition_by_name(attrName);
  if (!attr_def) return kvaDbErr_NoAttrib;

  status = kvadbattributedefhnd.find_or_add(attr_def, adh);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetNextAttributeDefinition(KvaDbAttributeDefHnd adh, KvaDbAttributeDefHnd *nadh)
{
  RegisterDeletionCallbacks();
  KvaDbStatus status;

  if (!kvadbattributedefhnd.is_valid(adh)) return kvaDbErr_Param;
  if (!nadh)                               return kvaDbErr_Param;

  CANdbAttributeDefinition *attr_def = kvadbattributedefhnd.convert(adh);
  CANdbAttributeDefinition *n_attr_def = attr_def->get_next();
  if (!n_attr_def) return kvaDbErr_NoAttrib;

  status = kvadbattributedefhnd.find_or_add(n_attr_def, nadh);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetAttributeDefinitionType(KvaDbAttributeDefHnd adh, KvaDbAttributeType *at)
{
  RegisterDeletionCallbacks();
  if (!kvadbattributedefhnd.is_valid(adh)) return kvaDbErr_Param;
  if (!at)                                 return kvaDbErr_Param;

  CANdbAttributeDefinition *attr_def = kvadbattributedefhnd.convert(adh);
  if (!attr_def) return kvaDbErr_NoAttrib;
  *at = (KvaDbAttributeType)attr_def->get_type();
  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetAttributeDefinitionOwner(KvaDbAttributeDefHnd adh, KvaDbAttributeOwner *ao)
{
  RegisterDeletionCallbacks();
  if (!kvadbattributedefhnd.is_valid(adh)) return kvaDbErr_Param;
  if (!ao)                                 return kvaDbErr_Param;

  CANdbAttributeDefinition *attr_def = kvadbattributedefhnd.convert(adh);
  if (!attr_def) return kvaDbErr_NoAttrib;
  *ao = (KvaDbAttributeOwner)attr_def->get_owner();
  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetAttributeDefinitionName(KvaDbAttributeDefHnd adh, char *buf, size_t buflen)
{
  RegisterDeletionCallbacks();
  if (!kvadbattributedefhnd.is_valid(adh)) return kvaDbErr_Param;
  if (!buf || (buflen == 0))               return kvaDbErr_Param;

  CANdbAttributeDefinition *attr_def = kvadbattributedefhnd.convert(adh);
  if (!attr_def) return kvaDbErr_NoAttrib;

  if (attr_def && attr_def->get_name()) strncpy(buf, attr_def->get_name(), buflen);
  else strcpy(buf, "");
  buf[buflen-1] = 0;
  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetAttributeDefinitionInt(KvaDbAttributeDefHnd adh, int *def, int *min, int *max)
{
  RegisterDeletionCallbacks();
  if (!kvadbattributedefhnd.is_valid(adh)) return kvaDbErr_Param;
  if (!def || !min || !max)                return kvaDbErr_Param;

  CANdbAttributeDefinition *attr_def = kvadbattributedefhnd.convert(adh);
  if (!attr_def) return kvaDbErr_NoAttrib;
  *def = attr_def->get_integer_default();
  *min = attr_def->get_integer_min();
  *max = attr_def->get_integer_max();
  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetAttributeDefinitionFloat(KvaDbAttributeDefHnd adh, float *def, float *min, float *max)
{
  RegisterDeletionCallbacks();
  if (!kvadbattributedefhnd.is_valid(adh)) return kvaDbErr_Param;
  if (!def || !min || !max)                return kvaDbErr_Param;

  CANdbAttributeDefinition *attr_def = kvadbattributedefhnd.convert(adh);
  if (!attr_def) return kvaDbErr_NoAttrib;
  *def = (float)attr_def->get_float_default();
  *min = (float)attr_def->get_float_min();
  *max = (float)attr_def->get_float_max();
  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetAttributeDefinitionString(KvaDbAttributeDefHnd adh, char *buf, size_t buflen)
{
  RegisterDeletionCallbacks();
  if (!kvadbattributedefhnd.is_valid(adh)) return kvaDbErr_Param;
  if (!buf || (buflen == 0))               return kvaDbErr_Param;

  CANdbAttributeDefinition *attr_def = kvadbattributedefhnd.convert(adh);
  if (!attr_def) return kvaDbErr_NoAttrib;

  if (attr_def && attr_def->get_string_default()) strncpy(buf, attr_def->get_string_default(), buflen);
  else strcpy(buf, "");
  buf[buflen-1] = 0;
  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetAttributeDefinitionEnumeration(KvaDbAttributeDefHnd adh, int *def)
{
  RegisterDeletionCallbacks();
  if (!kvadbattributedefhnd.is_valid(adh)) return kvaDbErr_Param;
  if (!def)                                return kvaDbErr_Param;

  CANdbAttributeDefinition *attr_def = kvadbattributedefhnd.convert(adh);
  if (!attr_def) return kvaDbErr_NoAttrib;
  *def = (int)attr_def->get_enumeration_default();
  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbAddMsgAttribute(KvaDbMessageHnd mh, KvaDbAttributeDefHnd adh, KvaDbAttributeHnd *ah)
{
  RegisterDeletionCallbacks();
  KvaDbStatus status;

  if (!kvadbmessagehnd.is_valid(mh))       return kvaDbErr_Param;
  if (!kvadbattributedefhnd.is_valid(adh)) return kvaDbErr_Param;
  if (!ah)                                 return kvaDbErr_Param;

  CANdbMessage *msg = kvadbmessagehnd.convert(mh);
  if (!msg) return kvaDbErr_NoMsg;

  CANdbAttributeDefinition *attr_def = kvadbattributedefhnd.convert(adh);
  if (!attr_def) return kvaDbErr_NoAttrib;

  CANdbAttributeOwner ao = attr_def->get_owner();
  if (ao != CANDB_ATTR_OWNER_MESSAGE) return kvaDbErr_WrongOwner;

  CANdbAttributeList *aList = msg->get_attributes();
  if (!aList) return kvaDbErr_NoAttrib;

  CANdbAttribute *a_tmp = aList->find_by_definition(attr_def);
  if (a_tmp) return kvaDbErr_Fail;

  CANdbAttribute *attr = new CANdbAttribute(attr_def);
  aList->insert(attr);

  status = kvadbattributehnd.add(attr, ah);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbAddAttribute(KvaDbHnd dh, KvaDbAttributeDefHnd adh, KvaDbAttributeHnd *ah)
{
  RegisterDeletionCallbacks();
  KvaDbStatus   status;
  CANdbCluster *my_dc;
  CANdb        *my_db;

  if (!kvadbhnd.is_valid(dh))              return kvaDbErr_Param;
  if (!kvadbattributedefhnd.is_valid(adh)) return kvaDbErr_Param;
  if (!ah)                                 return kvaDbErr_Param;

  my_dc = kvadbhnd.convert(dh);

  my_db = my_dc->get_first_candb();
  if (!my_db) return kvaDbErr_NoDatabase;

  CANdbAttributeDefinition *attr_def = kvadbattributedefhnd.convert(adh);
  if (!attr_def) return kvaDbErr_NoAttrib;

  CANdbAttributeOwner ao = attr_def->get_owner();
  if (ao != CANDB_ATTR_OWNER_DB) return kvaDbErr_WrongOwner;

  CANdbAttributeList *aList = my_db->get_attributes();
  if (!aList) return kvaDbErr_NoAttrib;

  CANdbAttribute *a_tmp = aList->find_by_definition(attr_def);
  if (a_tmp) return kvaDbErr_Fail;

  CANdbAttribute *attr = new CANdbAttribute(attr_def);
  aList->insert(attr);

  status = kvadbattributehnd.add(attr, ah);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbDeleteMsgAttribute(KvaDbMessageHnd mh, KvaDbAttributeHnd ah)
{
  RegisterDeletionCallbacks();
  KvaDbStatus status;

  if (!kvadbmessagehnd.is_valid(mh))   return kvaDbErr_Param;
  if (!kvadbattributehnd.is_valid(ah)) return kvaDbErr_Param;

  CANdbMessage *msg = kvadbmessagehnd.convert(mh);
  if (!msg) return kvaDbErr_NoMsg;

  CANdbAttribute *attr = kvadbattributehnd.convert(ah);
  if (!attr) return kvaDbErr_NoAttrib;

  CANdbAttributeList *aList = msg->get_attributes();
  if (!aList) return kvaDbErr_NoAttrib;

  aList->remove(attr);

  status = kvadbattributehnd.remove(ah);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}


//===========================================================================

KvaDbStatus WINAPI kvaDbDeleteAttribute(KvaDbHnd dh, KvaDbAttributeHnd ah)
{
  RegisterDeletionCallbacks();
  KvaDbStatus   status;
  CANdbCluster *my_dc;
  CANdb        *my_db;

  if (!kvadbhnd.is_valid(dh))          return kvaDbErr_Param;
  if (!kvadbattributehnd.is_valid(ah)) return kvaDbErr_Param;

  my_dc = kvadbhnd.convert(dh);

  my_db = my_dc->get_first_candb();
  if (!my_db) return kvaDbErr_NoDatabase;

  CANdbAttribute *attr = kvadbattributehnd.convert(ah);
  if (!attr) return kvaDbErr_NoAttrib;

  CANdbAttributeList *aList = my_db->get_attributes();
  if (!aList) return kvaDbErr_NoAttrib;

  aList->remove(attr);

  status = kvadbattributehnd.remove(ah);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbAddSignalAttribute(KvaDbSignalHnd sh, KvaDbAttributeDefHnd adh, KvaDbAttributeHnd *ah)
{
  RegisterDeletionCallbacks();
  KvaDbStatus status;
  if (!kvadbsignalhnd.is_valid(sh))        return kvaDbErr_Param;
  if (!kvadbattributedefhnd.is_valid(adh)) return kvaDbErr_Param;
  if (!ah)                                 return kvaDbErr_Param;

  CANdbSignal *sig = kvadbsignalhnd.convert(sh);
  CANdbAttributeDefinition *attr_def = kvadbattributedefhnd.convert(adh);
  if (!attr_def) return kvaDbErr_NoAttrib;
  CANdbAttributeOwner ao = attr_def->get_owner();
  if (ao != CANDB_ATTR_OWNER_SIGNAL) return kvaDbErr_WrongOwner;
  CANdbAttributeList *aList = sig->get_attributes();
  if (!aList) return kvaDbErr_NoAttrib;
  CANdbAttribute *a_tmp = aList->find_by_definition(attr_def);
  if (a_tmp) return kvaDbErr_Fail;
  CANdbAttribute *attr = new CANdbAttribute(attr_def);
  aList->insert(attr);

  status = kvadbattributehnd.add(attr, ah);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetFirstEnumValue(KvaDbSignalHnd sh, KvaDbEnumValueHnd *eh)
{
  RegisterDeletionCallbacks();
  KvaDbStatus status;

  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;
  if (!eh)                          return kvaDbErr_Param;

  CANdbSignal    *sig = kvadbsignalhnd.convert(sh);
  CANdbEnumValue *e   = sig->get_first_value();
  if (!e) {
    *eh = NULL;
    return kvaDbErr_NoAttrib;
  }

  status = kvadbenumvaluehnd.find_or_add(e, eh);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetNextEnumValue(KvaDbSignalHnd sh, KvaDbEnumValueHnd *eh)
{
  RegisterDeletionCallbacks();
  KvaDbStatus status;

  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;
  if (!eh)                          return kvaDbErr_Param;

  CANdbSignal    *sig = kvadbsignalhnd.convert(sh);
  CANdbEnumValue *e   = sig->get_next_value();
  if (!e) {
    *eh = NULL;
    return kvaDbErr_NoAttrib;
  }

  status = kvadbenumvaluehnd.find_or_add(e, eh);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetFirstEnumValuePair(KvaDbSignalHnd sh, KvaDbEnumValueHnd *eh, int *val, char *buf, size_t buflen)
{
  RegisterDeletionCallbacks();
  KvaDbStatus     status;
  CANdbSignal    *sig;
  CANdbEnumValue *e;

  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;
  if (!eh)                          return kvaDbErr_Param;

  *eh = NULL;

  sig = kvadbsignalhnd.convert(sh);
  e   = sig->get_first_value();

  if (!e) {
    return kvaDbErr_NoAttrib;
  }

  status = kvadbenumvaluehnd.find_or_add(e, eh);

  if (status == kvaDbOK) {
    status = get_enum_value(e, val, buf, buflen);
  } else {
    return kvaDbErr_Internal;
  }

  return status;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetNextEnumValuePair(KvaDbSignalHnd sh, KvaDbEnumValueHnd *eh, int *val, char *buf, size_t buflen)
{
  RegisterDeletionCallbacks();
  KvaDbStatus     status;
  CANdbSignal    *sig;
  CANdbEnumValue *e;

  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;
  if (!eh)                          return kvaDbErr_Param;

  *eh = NULL;

  sig = kvadbsignalhnd.convert(sh);
  e   = sig->get_next_value();
  if (!e) {
    return kvaDbErr_NoAttrib;
  }

  status = kvadbenumvaluehnd.find_or_add(e, eh);

  if (status == kvaDbOK) {
    status = get_enum_value(e, val, buf, buflen);
  } else {
    return kvaDbErr_Internal;
  }

  return status;
}

//===========================================================================


KvaDbStatus WINAPI kvaDbGetEnumValue(KvaDbEnumValueHnd eh, int *val, char *buf, size_t buflen)
{
  RegisterDeletionCallbacks();
  if (!kvadbenumvaluehnd.is_valid(eh)) return kvaDbErr_Param;

  return get_enum_value(kvadbenumvaluehnd.convert(eh), val, buf, buflen);
}

//===========================================================================

KvaDbStatus WINAPI kvaDbAddEnumValue(KvaDbSignalHnd sh, int val, const char* name)
{
  RegisterDeletionCallbacks();
  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;
  if (!name)                        return kvaDbErr_Param;

  CANdbSignal *sig = kvadbsignalhnd.convert(sh);
  if (sig->add_value(val, name)) return kvaDbErr_Fail;
  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbDeleteEnumValue(KvaDbSignalHnd sh, KvaDbEnumValueHnd eh)
{
  RegisterDeletionCallbacks();
  KvaDbStatus status;

  if (!kvadbsignalhnd.is_valid(sh))    return kvaDbErr_Param;
  if (!kvadbenumvaluehnd.is_valid(eh)) return kvaDbErr_Param;

  CANdbSignal    *sig = kvadbsignalhnd.convert(sh);
  CANdbEnumValue *e   = kvadbenumvaluehnd.convert(eh);

  if (!e)                   return kvaDbErr_NoAttrib;
  if (sig->remove_value(e)) return kvaDbErr_Fail;

  status = kvadbenumvaluehnd.remove(eh);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbDeleteSignalAttribute(KvaDbSignalHnd sh, KvaDbAttributeHnd ah)
{
  RegisterDeletionCallbacks();
  KvaDbStatus status;

  if (!kvadbsignalhnd.is_valid(sh))    return kvaDbErr_Param;
  if (!kvadbattributehnd.is_valid(ah)) return kvaDbErr_Param;

  CANdbSignal    *sig  = kvadbsignalhnd.convert(sh);
  CANdbAttribute *attr = kvadbattributehnd.convert(ah);

  if (!attr) return kvaDbErr_NoAttrib;

  CANdbAttributeList *aList = sig->get_attributes();

  if (!aList) return kvaDbErr_NoAttrib;
  aList->remove(attr);

  status = kvadbattributehnd.remove(ah);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbAddNodeAttribute(KvaDbNodeHnd nh, KvaDbAttributeDefHnd adh, KvaDbAttributeHnd *ah)
{
  RegisterDeletionCallbacks();
  KvaDbStatus status;

  if (!kvadbnodehnd.is_valid(nh))          return kvaDbErr_Param;
  if (!kvadbattributedefhnd.is_valid(adh)) return kvaDbErr_Param;
  if (!ah)                                 return kvaDbErr_Param;

  CANdbNode *n = kvadbnodehnd.convert(nh);
  CANdbAttributeDefinition *attr_def = kvadbattributedefhnd.convert(adh);
  if (!attr_def) return kvaDbErr_NoAttrib;
  CANdbAttributeOwner ao = attr_def->get_owner();
  if (ao != CANDB_ATTR_OWNER_NODE) return kvaDbErr_WrongOwner;
  CANdbAttributeList *aList = n->get_attributes();
  if (!aList) return kvaDbErr_NoAttrib;
  CANdbAttribute *a_tmp = aList->find_by_definition(attr_def);
  if (a_tmp) return kvaDbErr_Fail;
  CANdbAttribute *attr = new CANdbAttribute(attr_def);
  aList->insert(attr);

  status = kvadbattributehnd.add(attr, ah);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbDeleteNodeAttribute(KvaDbNodeHnd nh, KvaDbAttributeHnd ah)
{
  RegisterDeletionCallbacks();
  KvaDbStatus status;

  if (!kvadbnodehnd.is_valid(nh))      return kvaDbErr_Param;
  if (!kvadbattributehnd.is_valid(ah)) return kvaDbErr_Param;

  CANdbNode *n = kvadbnodehnd.convert(nh);
  CANdbAttribute *attr = kvadbattributehnd.convert(ah);
  if (!attr) return kvaDbErr_NoAttrib;
  CANdbAttributeList *aList = n->get_attributes();
  if (!aList) return kvaDbErr_NoAttrib;
  aList->remove(attr);

  status = kvadbattributehnd.remove(ah);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetFirstMsgAttribute(KvaDbMessageHnd mh, KvaDbAttributeHnd *ah)
{
  RegisterDeletionCallbacks();
  KvaDbStatus status;

  if (!kvadbmessagehnd.is_valid(mh)) return kvaDbErr_Param;
  if (!ah)                           return kvaDbErr_Param;

  CANdbMessage *msg = kvadbmessagehnd.convert(mh);
  if (!msg) return kvaDbErr_NoMsg;

  CANdbAttributeList *aList = msg->get_attributes();
  if (!aList) return kvaDbErr_NoAttrib;
  CANdbAttribute *attr = aList->get_first_attribute();
  if (!attr) return kvaDbErr_NoAttrib;

  status = kvadbattributehnd.find_or_add(attr, ah);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetMsgAttributeByName(KvaDbMessageHnd mh, const char *attrName, KvaDbAttributeHnd *ah)
{
  RegisterDeletionCallbacks();
  KvaDbStatus     status;
  CANdbAttribute *attr;

  if (!kvadbmessagehnd.is_valid(mh)) return kvaDbErr_Param;
  if (!attrName || !ah)              return kvaDbErr_Param;

  CANdbMessage *msg = kvadbmessagehnd.convert(mh);
  if (!msg) return kvaDbErr_NoMsg;

  CANdbAttributeList *aList = msg->get_attributes();
  if (!aList) return kvaDbErr_NoAttrib;
  attr = aList->find_by_name(attrName);
  if (!attr) return kvaDbErr_NoAttrib;

  status = kvadbattributehnd.find_or_add(attr, ah);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetAttributeByName(KvaDbHnd dh, const char *attrName, KvaDbAttributeHnd *ah)
{
  RegisterDeletionCallbacks();
  KvaDbStatus     status;
  CANdbAttribute *attr;
  CANdbCluster   *my_dc;
  CANdb          *my_db;

  if (!kvadbhnd.is_valid(dh)) return kvaDbErr_Param;
  if (!attrName || !ah)       return kvaDbErr_Param;

  my_dc = kvadbhnd.convert(dh);

  my_db = my_dc->get_first_candb();
  if (!my_db) return kvaDbErr_NoDatabase;

  CANdbAttributeList *aList = my_db->get_attributes();
  if (!aList) return kvaDbErr_NoAttrib;
  attr = aList->find_by_name(attrName);
  if (!attr) return kvaDbErr_NoAttrib;

  status = kvadbattributehnd.find_or_add(attr, ah);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetFirstSignalAttribute(KvaDbSignalHnd sh, KvaDbAttributeHnd *ah)
{
  RegisterDeletionCallbacks();
  KvaDbStatus status;

  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;
  if (!ah)                          return kvaDbErr_Param;

  CANdbSignal *sig = kvadbsignalhnd.convert(sh);

  CANdbAttributeList *aList = sig->get_attributes();
  if (!aList) return kvaDbErr_NoAttrib;
  CANdbAttribute *attr = aList->get_first_attribute();
  if (!attr) return kvaDbErr_NoAttrib;

  status = kvadbattributehnd.find_or_add(attr, ah);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetSignalAttributeByName(KvaDbSignalHnd sh, const char *attrName, KvaDbAttributeHnd *ah)
{
  RegisterDeletionCallbacks();
  KvaDbStatus     status;
  CANdbAttribute *attr;

  if (!kvadbsignalhnd.is_valid(sh)) return kvaDbErr_Param;
  if (!attrName || !ah)             return kvaDbErr_Param;

  CANdbSignal *sig = kvadbsignalhnd.convert(sh);

  CANdbAttributeList *aList = sig->get_attributes();
  if (!aList) return kvaDbErr_NoAttrib;
  attr = aList->find_by_name(attrName);
  if (!attr) return kvaDbErr_NoAttrib;

  status = kvadbattributehnd.find_or_add(attr, ah);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetFirstNodeAttribute(KvaDbNodeHnd nh, KvaDbAttributeHnd *ah)
{
  RegisterDeletionCallbacks();
  KvaDbStatus status;

  if (!kvadbnodehnd.is_valid(nh)) return kvaDbErr_Param;
  if (!ah)                        return kvaDbErr_Param;

  CANdbNode *n = kvadbnodehnd.convert(nh);

  CANdbAttributeList *aList = n->get_attributes();
  if (!aList) return kvaDbErr_NoAttrib;

  CANdbAttribute *attr = aList->get_first_attribute();
  if (!attr) return kvaDbErr_NoAttrib;

  status = kvadbattributehnd.find_or_add(attr, ah);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetNodeAttributeByName(KvaDbNodeHnd nh, const char *attrName, KvaDbAttributeHnd *ah)
{
  RegisterDeletionCallbacks();
  KvaDbStatus     status;
  CANdbAttribute *attr;

  if (!kvadbnodehnd.is_valid(nh)) return kvaDbErr_Param;
  if (!attrName || !ah)           return kvaDbErr_Param;

  CANdbNode *n = kvadbnodehnd.convert(nh);

  CANdbAttributeList *aList = n->get_attributes();
  if (!aList) return kvaDbErr_NoAttrib;

  attr = aList->find_by_name(attrName);
  if (!attr) return kvaDbErr_NoAttrib;

  status = kvadbattributehnd.find_or_add(attr, ah);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetFirstAttribute(KvaDbHnd dh, KvaDbAttributeHnd *nah)
{
  RegisterDeletionCallbacks();
  KvaDbStatus         status;
  CANdbAttributeList *attrL;
  CANdbAttribute     *attr;
  CANdbCluster       *my_dc;
  CANdb              *my_db;

  if (!kvadbhnd.is_valid(dh)) return kvaDbErr_Param;
  if (!nah)                    return kvaDbErr_Param;

  my_dc = kvadbhnd.convert(dh);

  my_db = my_dc->get_first_candb();
  if (!my_db) return kvaDbErr_NoDatabase;

  attrL = my_db->get_attributes();
  if (!attrL) return kvaDbErr_NoAttrib;

  attr  = attrL->get_first_attribute();
  if (!attr) return kvaDbErr_NoAttrib;

  status = kvadbattributehnd.find_or_add(attr, nah);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetNextAttribute(KvaDbAttributeHnd ah, KvaDbAttributeHnd *nah)
{
  RegisterDeletionCallbacks();
  KvaDbStatus status;

  if (!kvadbattributehnd.is_valid(ah)) return kvaDbErr_Param;
  if (!nah)                            return kvaDbErr_Param;

  CANdbAttribute *attr   = kvadbattributehnd.convert(ah);
  CANdbAttribute *n_attr = attr->get_next();
  if (!n_attr) return kvaDbErr_NoAttrib;

  status = kvadbattributehnd.find_or_add(n_attr, nah);
  if (status != kvaDbOK) return kvaDbErr_Internal;

  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetAttributeType(KvaDbAttributeHnd ah, KvaDbAttributeType *at)
{
  RegisterDeletionCallbacks();
  if (!kvadbattributehnd.is_valid(ah)) return kvaDbErr_Param;
  if (!at)                             return kvaDbErr_Param;

  CANdbAttribute *attr = kvadbattributehnd.convert(ah);
  *at = (KvaDbAttributeType)attr->get_type();
  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetAttributeName(KvaDbAttributeHnd ah, char *buf, size_t buflen)
{
  RegisterDeletionCallbacks();
  if (!kvadbattributehnd.is_valid(ah)) return kvaDbErr_Param;
  if (!buf || (buflen == 0))           return kvaDbErr_Param;

  CANdbAttribute *attr = kvadbattributehnd.convert(ah);
  if (!attr) return kvaDbErr_NoAttrib;

  CANdbAttributeDefinition *attr_def = attr->get_definition();
  if (!attr_def) return kvaDbErr_NoAttrib;

  if (attr_def && attr_def->get_name()) strncpy(buf, attr_def->get_name(), buflen);
  else strcpy(buf, "");
  buf[buflen-1] = 0;
  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbSetAttributeValueInt(KvaDbAttributeHnd ah, int val)
{
  RegisterDeletionCallbacks();
  if (!kvadbattributehnd.is_valid(ah)) return kvaDbErr_Param;

  CANdbAttribute *attr = kvadbattributehnd.convert(ah);
  if (!attr) return kvaDbErr_NoAttrib;
  attr->set_integer_value(val);
  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetAttributeValueInt(KvaDbAttributeHnd ah, int *val)
{
  RegisterDeletionCallbacks();
  if (!kvadbattributehnd.is_valid(ah)) return kvaDbErr_Param;
  if (!val)                            return kvaDbErr_Param;

  CANdbAttribute *attr = kvadbattributehnd.convert(ah);
  if (!attr) return kvaDbErr_NoAttrib;
  *val = attr->get_integer_value();
  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbSetAttributeValueFloat(KvaDbAttributeHnd ah, float val)
{
  RegisterDeletionCallbacks();
  if (!kvadbattributehnd.is_valid(ah)) return kvaDbErr_Param;

  CANdbAttribute *attr = kvadbattributehnd.convert(ah);
  if (!attr) return kvaDbErr_NoAttrib;
  attr->set_float_value(val);
  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetAttributeValueFloat(KvaDbAttributeHnd ah, float *val)
{
  RegisterDeletionCallbacks();
  if (!kvadbattributehnd.is_valid(ah)) return kvaDbErr_Param;
  if (!val)                            return kvaDbErr_Param;

  CANdbAttribute *attr = kvadbattributehnd.convert(ah);
  if (!attr) return kvaDbErr_NoAttrib;

  *val = (float)attr->get_float_value();
  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbSetAttributeValueString(KvaDbAttributeHnd ah, const char *buf, int /* buflen */)
{
  RegisterDeletionCallbacks();
  if (!kvadbattributehnd.is_valid(ah)) return kvaDbErr_Param;
  if (!buf)                            return kvaDbErr_Param;

  CANdbAttribute *attr = kvadbattributehnd.convert(ah);
  if (!attr) return kvaDbErr_NoAttrib;

  attr->set_string_value(buf);
  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetAttributeValueString(KvaDbAttributeHnd ah, char *buf, size_t buflen)
{
  RegisterDeletionCallbacks();
  if (!kvadbattributehnd.is_valid(ah))  return kvaDbErr_Param;
  if (!buf || (buflen == 0))            return kvaDbErr_Param;

  CANdbAttribute *attr = kvadbattributehnd.convert(ah);
  if (!attr) return kvaDbErr_NoAttrib;

  if (attr && attr->get_string_value()) strncpy(buf, attr->get_string_value(), buflen);
  else strcpy(buf, "");
  buf[buflen-1] = 0;
  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbSetAttributeValueEnumeration(KvaDbAttributeHnd ah, int val)
{
  RegisterDeletionCallbacks();
  if (!kvadbattributehnd.is_valid(ah)) return kvaDbErr_Param;

  CANdbAttribute *attr = kvadbattributehnd.convert(ah);
  if (!attr) return kvaDbErr_NoAttrib;
  attr->set_enumeration_value(val);
  return kvaDbOK;
}

//===========================================================================

KvaDbStatus WINAPI kvaDbGetAttributeValueEnumeration(KvaDbAttributeHnd ah, int *val)
{
  RegisterDeletionCallbacks();
  if (!kvadbattributehnd.is_valid(ah)) return kvaDbErr_Param;
  if (!val)                            return kvaDbErr_Param;

  CANdbAttribute *attr = kvadbattributehnd.convert(ah);
  if (!attr) return kvaDbErr_NoAttrib;

  *val = (int)attr->get_enumeration_value();
  return kvaDbOK;
}

static KvaDbStatus get_enum_value(CANdbEnumValue *e, int *val, char *buf, size_t buflen)
{
  RegisterDeletionCallbacks();
  if (!val || !buf || !buflen) return kvaDbErr_Param;

  *val = e->get_value();
  strncpy(buf, e->get_name(), buflen);
  buf[buflen-1] = 0;
  return kvaDbOK;
}


//===========================================================================
