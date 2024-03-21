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
** Description: Class to handle transmit lists in XML settings for
**              Kvaser Memorator Pro(2nd generation)
** -----------------------------------------------------------------------------
*/

#include <trigger.h>
#include <string>
#include <cstring>
#include <sstream>
#include <algorithm>

#include "action.h"
#include "busparams.h"
#include "statement.h"
#include "errorhandler.h"
#include "util.h"
#include "convert.h"
#include "xmlwriter.h"
#include "kv_parser.h"

// ----------------------------------------------------------------------------------

Trigger* Trigger::createTriggerFactory(std::string name, uint8_t type)
{
  Trigger *t = NULL;

  switch (type)
    {
    case TRIGVAR_TYPE_MSG_ID:
      t = new MsgIdTrigger(name);
      break;

    case TRIGVAR_TYPE_MSG_DLC:
      t = new DlcTrigger(name);
      break;

    case TRIGVAR_TYPE_MSG_FLAG:
      t = new ErrorFrameTrigger(name);
      break;

    case TRIGVAR_TYPE_SIGVAL:
      t = new SigValTrigger(name);
      break;

    case TRIGVAR_TYPE_EXTERNAL:
      t = new ExtTrigger(name);
      break;

    case TRIGVAR_TYPE_TIMER:
      t = new TimerTrigger(name);
      break;

    case TRIGVAR_TYPE_DISK_FULL:
    case TRIGVAR_TYPE_STARTUP:
      t = new Trigger(name);
      break;

    default:
      t = new Trigger(name);
      break;
    }

  t->mType = type;
  return t;
}

// ----------------------------------------------------------------------------------

Trigger::Trigger(std::string name) : Xmlref(name)
{
    mTimeout = 0;
}

void Trigger::print()
{
  PRINTF(("%d %s [", mIdx, mName.c_str()));
  PRINTF(("  Type           %d", mType));
  PRINTF(("  Name           %s", mName.c_str()));
  PRINTF(("  Timeout        %u", mTimeout));
}

int Trigger::createBinary(unsigned char *buffer, unsigned char version)
{
  if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR_OLD) {
    TriggerVar *tv = (TriggerVar*) buffer;

    tv->type    = mType;
    tv->timeOut = mTimeout;
  }
  else if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR) {
    TriggerVarFd *tv = (TriggerVarFd*) buffer;

    tv->type    = mType;
    tv->timeOut = mTimeout;
  }

  return 0;
}

int Trigger::parseBinary(const char *buffer, const unsigned char version)
{
  if (!buffer) throw_nullpointer(__FUNCTION__);

  if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR_OLD) {
    TriggerVar *tp = (TriggerVar *) buffer;
    mType    = tp->type;
    mTimeout = tp->timeOut;
  }
  else if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR) {
    TriggerVarFd *tpfd = (TriggerVarFd *) buffer;
    mType    = tpfd->type;
    mTimeout = tpfd->timeOut;
  }
  else {
    PRINTF(("%s: UNSUPPORTED VERSION %d", __FUNCTION__, version));
    return 0;
  }

  PRINTF(("Read Trigger %s\n", mName.c_str()));

  return 1;
}

int Trigger::createXml(xmlTextWriterPtr writer) const
{
  XMLWriter xml(writer);

  xml.beginElement(uint8_to_name(mType, TrigvarTypeStrings, TrigvarTypeValues, TrigvarTypeLength));
  xml.writeAttr(XML_TRIGGERS_ATTR_NAME, mName);
  xml.writeAttr(XML_TRIGGERS_ATTR_TMO, "%d", mTimeout);
  xml.endElement();

  return 0;
}

// ----------------------------------------------------------------------------------

MsgIdTrigger::MsgIdTrigger(std::string name) : Trigger(name)
{
  mCh        = 0;
  mMsgId     = 0;
  mMsgIdMin  = 0;
  mIsExtId   = 0;
  mHelpMask  = 0;
}

void MsgIdTrigger::print()
{
  Trigger::print();
  PRINTF(("  Channel        %u", mCh));
  PRINTF(("  Message id     %u", mMsgId));
  PRINTF(("  Message id min %u", mMsgIdMin));
  PRINTF(("  Extended id    %s", mIsExtId ? "Yes" : "No"));
  PRINTF(("  HLP mask       0x%x", mHelpMask));
  PRINTF(("]"));
}

int MsgIdTrigger::createBinary(unsigned char *buffer, unsigned char version)
{
  if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR_OLD) {
    TriggerVar *tv = (TriggerVar*) buffer;

    tv->type    = mType;
    tv->timeOut = mTimeout;
    tv->channel = mCh;

    tv->t.msgId.id      = mMsgId;
    tv->t.msgId.id_min  = mMsgIdMin;
    tv->t.msgId.id_mask = 0xffffffff;
    tv->t.msgId.HLP     = mHelpMask;

    if (mIsExtId) {
      tv->t.msgId.id     |= CAN_IDENT_IS_EXTENDED;
      tv->t.msgId.id_min |= CAN_IDENT_IS_EXTENDED;
    }
  }
  else if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR) {
    TriggerVarFd *tv = (TriggerVarFd*) buffer;

    tv->type    = mType;
    tv->timeOut = mTimeout;
    tv->channel = mCh;

    tv->t.msgId.id      = mMsgId;
    tv->t.msgId.id_min  = mMsgIdMin;
    tv->t.msgId.id_mask = 0xffffffff;
    tv->t.msgId.HLP     = mHelpMask;

    if (mIsExtId) {
      tv->t.msgId.id     |= CAN_IDENT_IS_EXTENDED;
      tv->t.msgId.id_min |= CAN_IDENT_IS_EXTENDED;
    }
  }

  return 0;
}

int MsgIdTrigger::parseBinary(const char *buffer, const unsigned char version)
{
  if (!buffer) throw_nullpointer(__FUNCTION__);

  if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR_OLD) {
    TriggerVar *tp = (TriggerVar *) buffer;
    mType     = tp->type;
    mTimeout  = tp->timeOut;
    mCh       = tp->channel;
    mIsExtId  = (tp->t.msgId.id & CAN_IDENT_IS_EXTENDED) ? true : false;
    mMsgId    = tp->t.msgId.id & ~CAN_IDENT_IS_EXTENDED;
    mMsgIdMin = tp->t.msgId.id_min & ~CAN_IDENT_IS_EXTENDED;
    mHelpMask = tp->t.msgId.HLP;
  }
  else if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR) {
    TriggerVarFd *tpfd = (TriggerVarFd *) buffer;
    mType     = tpfd->type;
    mTimeout  = tpfd->timeOut;
    mCh       = tpfd->channel;
    mIsExtId  = (tpfd->t.msgId.id & CAN_IDENT_IS_EXTENDED) ? true : false;
    mMsgId    = tpfd->t.msgId.id & ~CAN_IDENT_IS_EXTENDED;
    mMsgIdMin = tpfd->t.msgId.id_min & ~CAN_IDENT_IS_EXTENDED;
    mHelpMask = tpfd->t.msgId.HLP;
  }
  else {
    PRINTF(("%s: UNSUPPORTED VERSION %d", __FUNCTION__, version));
    return 0;
  }

  PRINTF(("Read MsgIdTrigger %s\n", mName.c_str()));

  return 1;
}

int MsgIdTrigger::createXml(xmlTextWriterPtr writer) const
{
  XMLWriter xml(writer);

  xml.beginElement(uint8_to_name(mType, TrigvarTypeStrings, TrigvarTypeValues, TrigvarTypeLength));
  xml.writeAttr(XML_TRIGGERS_ATTR_NAME, mName);
  xml.writeAttr(XML_TRIGGERS_ATTR_TMO,    "%d", mTimeout);
  xml.writeAttr(XML_TRIGGERS_ATTR_CHAN,   "%u", mCh);
  xml.writeAttr(XML_TRIGGERS_ATTR_ID,     "%u", mMsgId);
  xml.writeAttr(XML_TRIGGERS_ATTR_ID_MIN, "%u", mMsgIdMin);
  xml.writeAttrYesNo(XML_TRIGGERS_ATTR_ID_EXT, mIsExtId);
  xml.writeAttr(XML_TRIGGERS_ATTR_ID_PROT, uint8_to_protocol(mHelpMask));
  if (mHelpMask & HLP_SPEC_MASK) {
    xml.writeAttr(XML_TRIGGERS_ATTR_ID_MSK, uint8_to_protocol_msk(mHelpMask));
  }
  xml.endElement();

  return 0;
}

// ----------------------------------------------------------------------------------

DlcTrigger::DlcTrigger(std::string name) : Trigger(name)
{
  mCh      = 0;
  mDlc     = 0;
  mDlcMin  = 0;
}

void DlcTrigger::print()
{
  Trigger::print();
  PRINTF(("  Channel        %u", mCh));
  PRINTF(("  Dlc            %u", mDlc));
  PRINTF(("  Dlc min        %u", mDlcMin));
  PRINTF(("]"));
}

int DlcTrigger::createBinary(unsigned char *buffer, unsigned char version)
{
  if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR_OLD) {
    TriggerVar *tv = (TriggerVar*) buffer;

    tv->type    = mType;
    tv->timeOut = mTimeout;
    tv->channel = mCh;

    tv->t.msgDlc.dlc     = mDlc;
    tv->t.msgDlc.dlc_min = mDlcMin;
  }
  else if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR) {
    TriggerVarFd *tv = (TriggerVarFd*) buffer;
    tv->type    = mType;
    tv->timeOut = mTimeout;
    tv->channel = mCh;

    tv->t.msgDlc.dlc     = mDlc;
    tv->t.msgDlc.dlc_min = mDlcMin;
  }

  return 0;
}

int DlcTrigger::parseBinary(const char *buffer, const unsigned char version)
{
  if (!buffer) throw_nullpointer(__FUNCTION__);

  if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR_OLD) {
    TriggerVar *tp = (TriggerVar *) buffer;
    mType    = tp->type;
    mTimeout = tp->timeOut;
    mCh      = tp->channel;
    mDlc     = tp->t.msgDlc.dlc;
    mDlcMin  = tp->t.msgDlc.dlc_min;
  }
  else if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR) {
    TriggerVarFd *tpfd = (TriggerVarFd *) buffer;
    mType    = tpfd->type;
    mTimeout = tpfd->timeOut;
    mCh      = tpfd->channel;
    mDlc     = tpfd->t.msgDlc.dlc;
    mDlcMin  = tpfd->t.msgDlc.dlc_min;
  }
  else {
    PRINTF(("%s: UNSUPPORTED VERSION %d", __FUNCTION__, version));
    return 0;
  }

  PRINTF(("Read DlcTrigger %s\n", mName.c_str()));

  return 1;
}

int DlcTrigger::createXml(xmlTextWriterPtr writer) const
{
  XMLWriter xml(writer);

  xml.beginElement(uint8_to_name(mType, TrigvarTypeStrings, TrigvarTypeValues, TrigvarTypeLength));
  xml.writeAttr(XML_TRIGGERS_ATTR_NAME, mName);
  xml.writeAttr(XML_TRIGGERS_ATTR_TMO,     "%d", mTimeout);
  xml.writeAttr(XML_TRIGGERS_ATTR_CHAN,    "%u", mCh);
  xml.writeAttr(XML_TRIGGERS_ATTR_DLC,     "%u", mDlc);
  xml.writeAttr(XML_TRIGGERS_ATTR_DLC_MIN, "%u", mDlcMin);
  xml.endElement();

  return 0;
}

// ----------------------------------------------------------------------------------

ErrorFrameTrigger::ErrorFrameTrigger(std::string name) : Trigger(name)
{
  mCh = 0;
}

void ErrorFrameTrigger::print()
{
  Trigger::print();
  PRINTF(("  Channel        %u", mCh));
  PRINTF(("]"));
}

int ErrorFrameTrigger::createBinary(unsigned char *buffer, unsigned char version)
{
  if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR_OLD) {
    TriggerVar *tv = (TriggerVar*) buffer;

    tv->type    = mType;
    tv->timeOut = mTimeout;
    tv->channel = mCh;

    tv->t.msgFlag.flag = TRIGVAR_MSG_FLAG_ERROR_FRAME;
  }
  else if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR) {
    TriggerVarFd *tv = (TriggerVarFd*) buffer;

    tv->type    = mType;
    tv->timeOut = mTimeout;
    tv->channel = mCh;

    tv->t.msgFlag.flag = TRIGVAR_MSG_FLAG_ERROR_FRAME;
  }
  return 0;
}

int ErrorFrameTrigger::parseBinary(const char *buffer, const unsigned char version)
{
  if (!buffer) throw_nullpointer(__FUNCTION__);

  if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR_OLD) {
    TriggerVar *tp = (TriggerVar *) buffer;
    mType    = tp->type;
    mTimeout = tp->timeOut;
    mCh      = tp->channel;
  }
  else if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR) {
    TriggerVarFd *tpfd = (TriggerVarFd *) buffer;
    mType    = tpfd->type;
    mTimeout = tpfd->timeOut;
    mCh      = tpfd->channel;
  }
  else {
    PRINTF(("%s: UNSUPPORTED VERSION %d", __FUNCTION__, version));
    return 0;
  }

  PRINTF(("Read ErrorFrameTrigger %s\n", mName.c_str()));

  return 1;
}

int ErrorFrameTrigger::createXml(xmlTextWriterPtr writer) const
{
  XMLWriter xml(writer);

  xml.beginElement(uint8_to_name(mType, TrigvarTypeStrings, TrigvarTypeValues, TrigvarTypeLength));
  xml.writeAttr(XML_TRIGGERS_ATTR_NAME, mName);
  xml.writeAttr(XML_TRIGGERS_ATTR_TMO,     "%d", mTimeout);
  xml.writeAttr(XML_TRIGGERS_ATTR_CHAN,    "%u", mCh);
  xml.endElement();

  return 0;
}

// ----------------------------------------------------------------------------------

SigValTrigger::SigValTrigger(std::string name) : Trigger(name)
{
  mCh        = 0;
  mMsgId     = 0;
  mIsExtId   = false;
  mIsCanFd   = false;
  mDataType  = 0;
  mDlc       = 0;
  mIgnoreDlc = false;
  mStartBit  = 0;
  mLength    = 0;
  mHelpMask  = 0;
  mCondition = ON_DATA_EQUAL_TO;
  mData      = 0;
  mDataMin   = 0;
}

void SigValTrigger::print()
{
  Trigger::print();
  PRINTF(("  Channel        %u", mCh));
  PRINTF(("  Message id     %u", mMsgId));
  PRINTF(("  Extended id    %s", mIsExtId ? "Yes" : "No"));
  PRINTF(("  CAN            %s", mIsCanFd ? "FD" : "Classic"));
  PRINTF(("  Datatype       %u", mDataType));
  PRINTF(("  Dlc            %u", mDlc));
  PRINTF(("  Ignore dlc     %s", mIgnoreDlc ? "Yes" : "No"));
  PRINTF(("  Start bit      %u", mStartBit));
  PRINTF(("  Length         %u", mLength));
  PRINTF(("  HLP mask       0x%x", mHelpMask));
  PRINTF(("  Condition      %u", mCondition));
  PRINTF(("  Data           0x%x", mData));
  PRINTF(("  Data min       0x%x", mDataMin));
  PRINTF(("]"));
}

int SigValTrigger::createBinary(unsigned char *buffer, unsigned char version)
{
  if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR_OLD) {
    TriggerVar *tv = (TriggerVar*) buffer;

    tv->type    = mType;
    tv->timeOut = mTimeout;
    tv->channel = mCh;

    tv->t.sigVal.id        = mMsgId | (mIsExtId ? CAN_IDENT_IS_EXTENDED : 0);
    tv->t.sigVal.startbit  = (uint8_t)mStartBit;
    tv->t.sigVal.length    = (uint8_t)mLength;
    tv->t.sigVal.dataType  = mDataType;
    tv->t.sigVal.data      = mData;
    tv->t.sigVal.data_min  = mDataMin;
    tv->t.sigVal.condition = mCondition;
    tv->t.sigVal.HLP       = mHelpMask;
    if (mIgnoreDlc) {
      tv->t.sigVal.flags = TRIGVAR_FLAG_IGNORE_DLC;
    }
    else {
      tv->t.sigVal.dlc = mDlc;
    }
  }
  else if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR) {
    TriggerVarFd *tv = (TriggerVarFd*) buffer;

    tv->type    = mType;
    tv->timeOut = mTimeout;
    tv->channel = mCh;

    tv->t.sigVal.id        = mMsgId | (mIsExtId ? CAN_IDENT_IS_EXTENDED : 0);
    tv->t.sigVal.startbit  = mStartBit;
    tv->t.sigVal.length    = mLength;
    tv->t.sigVal.dataType  = mDataType;
    tv->t.sigVal.data      = mData;
    tv->t.sigVal.data_min  = mDataMin;
    tv->t.sigVal.condition = mCondition;
    tv->t.sigVal.HLP       = mHelpMask;
    if (mIgnoreDlc) {
      tv->t.sigVal.flags = TRIGVAR_FLAG_IGNORE_DLC;
    }
    else {
      tv->t.sigVal.dlc = mDlc;
    }
  }

  return 0;
}

int SigValTrigger::parseBinary(const char *buffer, const unsigned char version)
{
  if (!buffer) throw_nullpointer(__FUNCTION__);

  if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR_OLD) {
    TriggerVar *tp = (TriggerVar *) buffer;
    mType      = tp->type;
    mTimeout   = tp->timeOut;
    mCh        = tp->channel;
    mIsExtId   = (tp->t.sigVal.id & CAN_IDENT_IS_EXTENDED) ? true : false;
    mMsgId     = tp->t.sigVal.id & ~CAN_IDENT_IS_EXTENDED;
    mStartBit  = tp->t.sigVal.startbit;
    mLength    = tp->t.sigVal.length;
    mDataType  = tp->t.sigVal.dataType;
    mData      = tp->t.sigVal.data;
    mDataMin   = tp->t.sigVal.data_min;
    mCondition = tp->t.sigVal.condition;
    mHelpMask  = tp->t.sigVal.HLP;
    mIgnoreDlc = (tp->t.sigVal.flags & TRIGVAR_FLAG_IGNORE_DLC) ? true : false;
    mDlc       = tp->t.sigVal.dlc;
  }
  else if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR) {
    TriggerVarFd *tpfd = (TriggerVarFd *) buffer;
    mType      = tpfd->type;
    mTimeout   = tpfd->timeOut;
    mCh        = tpfd->channel;
    mIsExtId   = (tpfd->t.sigVal.id & CAN_IDENT_IS_EXTENDED) ? true : false;
    mMsgId     = tpfd->t.sigVal.id & ~CAN_IDENT_IS_EXTENDED;
    mStartBit  = tpfd->t.sigVal.startbit;
    mLength    = tpfd->t.sigVal.length;
    mDataType  = tpfd->t.sigVal.dataType;
    mData      = tpfd->t.sigVal.data;
    mDataMin   = tpfd->t.sigVal.data_min;
    mCondition = tpfd->t.sigVal.condition;
    mHelpMask  = tpfd->t.sigVal.HLP;
    mIgnoreDlc = (tpfd->t.sigVal.flags & TRIGVAR_FLAG_IGNORE_DLC) ? true : false;
    mDlc       = tpfd->t.sigVal.dlc;
  }
  else {
    PRINTF(("%s: UNSUPPORTED VERSION %d", __FUNCTION__, version));
    return 0;
  }

  PRINTF(("Read SigValTrigger %s\n", mName.c_str()));

  return 1;
}

int SigValTrigger::createXml(xmlTextWriterPtr writer) const
{
  XMLWriter xml(writer);

  xml.beginElement(uint8_to_name(mType, TrigvarTypeStrings, TrigvarTypeValues, TrigvarTypeLength));
  xml.writeAttr(XML_TRIGGERS_ATTR_NAME, mName);
  xml.writeAttr(XML_TRIGGERS_ATTR_TMO,    "%d", mTimeout);
  xml.writeAttr(XML_TRIGGERS_ATTR_CHAN,   "%u", mCh);
  xml.writeAttr(XML_TRIGGERS_ATTR_ID,     "%u", mMsgId);
  xml.writeAttrYesNo(XML_TRIGGERS_ATTR_ID_EXT, mIsExtId);
  if (!mIgnoreDlc) {
    xml.writeAttr(XML_TRIGGERS_ATTR_DLC,    "%u", mDlc);
  }
  xml.writeAttr(XML_TRIGGERS_ATTR_STARTBIT,  "%u", mStartBit);
  xml.writeAttr(XML_TRIGGERS_ATTR_BITLENGTH, "%u", mLength);
  xml.writeAttr(XML_TRIGGERS_ATTR_DATATYPE, datatype_to_string(mDataType));
  xml.writeAttr(XML_TRIGGERS_ATTR_ENDIAN, endian_to_string(mDataType));
  xml.writeAttr(XML_TRIGGERS_ATTR_ID_PROT, uint8_to_protocol(mHelpMask));
  if (mHelpMask & HLP_SPEC_MASK) {
    xml.writeAttr(XML_TRIGGERS_ATTR_ID_MSK, uint8_to_protocol_msk(mHelpMask));
  }
  xml.writeAttr(XML_TRIGGERS_ATTR_DATA,     "0x%lx", mData);
  xml.writeAttr(XML_TRIGGERS_ATTR_DATA_MIN, "0x%lx", mDataMin);
  xml.writeAttr(XML_TRIGGERS_ATTR_DATA_COND, "%s",
                uint8_to_name(mCondition, OnDataTypeStrings, OnDataTypeValues, OnDataTypeLength));

  xml.endElement();

  return 0;
}


// ----------------------------------------------------------------------------------

ExtTrigger::ExtTrigger(std::string name) : Trigger(name)
{
  mCh    = 0;
  mLevel = 0;
}

void ExtTrigger::print()
{
  Trigger::print();
  PRINTF(("  Channel        %u", mCh));
  PRINTF(("  Level          %s", mLevel == 1 ? "LO_HI" : "HI_LO"));
  PRINTF(("]"));
}

int ExtTrigger::createBinary(unsigned char *buffer, unsigned char version)
{
  if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR_OLD) {
    TriggerVar *tv = (TriggerVar*) buffer;

    tv->type    = mType;
    tv->timeOut = mTimeout;
    tv->channel = mCh;

    tv->t.extSign.level    = mLevel;
    //Set to 3 in firmware if value is 0
    tv->t.extSign.debounce = 0; // Not available in XML 2.0
  }
  else if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR) {
    TriggerVarFd *tv = (TriggerVarFd*) buffer;

    tv->type    = mType;
    tv->timeOut = mTimeout;
    tv->channel = mCh;

    tv->t.extSign.level    = mLevel;
    //Set to 3 in firmware if value is 0
    tv->t.extSign.debounce = 0; // Not available in XML 2.0
  }

  return 0;
}

int ExtTrigger::parseBinary(const char *buffer, const unsigned char version)
{
  if (!buffer) throw_nullpointer(__FUNCTION__);

  if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR_OLD) {
    TriggerVar *tp = (TriggerVar *) buffer;
    mType    = tp->type;
    mTimeout = tp->timeOut;
    mCh      = tp->channel;
    mLevel   = tp->t.extSign.level;
  }
  else if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR) {
    TriggerVarFd *tpfd = (TriggerVarFd *) buffer;
    mType    = tpfd->type;
    mTimeout = tpfd->timeOut;
    mCh      = tpfd->channel;
    mLevel   = tpfd->t.extSign.level;
  }
  else {
    PRINTF(("%s: UNSUPPORTED VERSION %d", __FUNCTION__, version));
    return 0;
  }

  PRINTF(("Read ExtTrigger %s\n", mName.c_str()));

  return 1;
}

int ExtTrigger::createXml(xmlTextWriterPtr writer) const
{
  XMLWriter xml(writer);

  xml.beginElement(uint8_to_name(mType, TrigvarTypeStrings, TrigvarTypeValues, TrigvarTypeLength));
  xml.writeAttr(XML_TRIGGERS_ATTR_NAME, mName);
  xml.writeAttr(XML_TRIGGERS_ATTR_TMO,    "%d", mTimeout);
  xml.writeAttr(XML_TRIGGERS_ATTR_CHAN,   "%u", mCh);
  xml.writeAttr(XML_TRIGGERS_ATTR_EXTLEVEL, "%s", uint8_to_name(mLevel, TrigLevelStrings, TrigLevelValues, TrigLevelLength));
  xml.endElement();

  return 0;
}

// ----------------------------------------------------------------------------------

TimerTrigger::TimerTrigger(std::string name) : Trigger(name)
{
  mOffset   = 0;
  mIsRepeat = false;
}

void TimerTrigger::print()
{
  Trigger::print();
  PRINTF(("  Offset         %u", mOffset));
  PRINTF(("  Repeat         %s", mIsRepeat ? "Yes" : "No"));
  PRINTF(("]"));
}

int TimerTrigger::createBinary(unsigned char *buffer, unsigned char version)
{
  if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR_OLD) {
    TriggerVar *tv = (TriggerVar*) buffer;

    tv->type    = mType;
    tv->timeOut = mTimeout;

    tv->t.timer.last      = 0; // Not available in XML 2.0
    tv->t.timer.offset    = mOffset;
    tv->t.timer.activated = 1; // Not available in XML 2.0
    tv->t.timer.repeat    = mIsRepeat;
  }
  else if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR) {
    TriggerVarFd *tv = (TriggerVarFd*) buffer;

    tv->type    = mType;
    tv->timeOut = mTimeout;

    tv->t.timer.last      = 0; // Not available in XML 2.0
    tv->t.timer.offset    = mOffset;
    tv->t.timer.activated = 1; // Not available in XML 2.0
    tv->t.timer.repeat    = mIsRepeat;
  }

  return 0;
}

int TimerTrigger::parseBinary(const char *buffer, const unsigned char version)
{
  if (!buffer) throw_nullpointer(__FUNCTION__);

  if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR_OLD) {
    TriggerVar *tp = (TriggerVar *) buffer;
    mType     = tp->type;
    mTimeout  = tp->timeOut;
    mOffset   = tp->t.timer.offset;
    mIsRepeat = tp->t.timer.repeat != 0;
  }
  else if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR) {
    TriggerVarFd *tpfd = (TriggerVarFd *) buffer;
    mType     = tpfd->type;
    mTimeout  = tpfd->timeOut;
    mOffset   = tpfd->t.timer.offset;
    mIsRepeat = tpfd->t.timer.repeat != 0;
  }
  else {
    PRINTF(("%s: UNSUPPORTED VERSION %d", __FUNCTION__, version));
    return 0;
  }

  PRINTF(("Read TimerTrigger %s\n", mName.c_str()));

  return 1;
}

int TimerTrigger::createXml(xmlTextWriterPtr writer) const
{
  XMLWriter xml(writer);

  xml.beginElement(uint8_to_name(mType, TrigvarTypeStrings, TrigvarTypeValues, TrigvarTypeLength));
  xml.writeAttr(XML_TRIGGERS_ATTR_NAME, mName);
  xml.writeAttr(XML_TRIGGERS_ATTR_TMO,    "%d", mTimeout);
  xml.writeAttr(XML_TRIGGERS_ATTR_OFFSET, "%u", mOffset);
  xml.writeAttrYesNo(XML_TRIGGERS_ATTR_REPEAT, mIsRepeat);
  xml.endElement();

  return 0;
}

// ----------------------------------------------------------------------------------

Triggers::Triggers()
{
  mLogAll             = true;
  mFifoMode           = false;
  mTrigScripted       = false;
  mPreTriggerSectors  = 0;
}

Triggers::~Triggers()
{
  mTriggers.clear();
  mStatements.clear();
}

int Triggers::createBinary(unsigned char* buffer, unsigned char version)
{
  unsigned char *pb;
  BlockHead     *ph;

  pb = buffer;
  ph = (BlockHead*) pb;

  if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR_OLD){
    ConfigFileTrigger *trigger;
    uint8_t statNo = 0;
    uint8_t trigNo = 0;

    ph->id  = BLOCK_ID_TRIGGER;
    ph->len = sizeof(BlockHead) + sizeof(ConfigFileTrigger);

    trigger = (ConfigFileTrigger*) (pb + sizeof(BlockHead));
    memset(trigger, 0, sizeof(ConfigFileTrigger));

    trigger->logMode |= (mLogAll ? TRIG_LOG_ALL : 0);
    trigger->logMode |= (mFifoMode ? TRIG_LOG_FIFO : 0);
    trigger->logMode |= (mTrigScripted ? TRIG_SCRIPTED : 0);

    for (std::vector<Xmlref*>::iterator stat = mStatements.mList.begin(); stat != mStatements.mList.end(); ++stat) {
      (*stat)->createBinary((uint8_t*)&trigger->listOfTrigStatements[statNo++], version);
    }

    for (std::vector<Xmlref*>::iterator trig = mTriggers.mList.begin(); trig != mTriggers.mList.end(); ++trig) {
      (*trig)->createBinary((uint8_t*)&trigger->listOfTrigVars[trigNo++], version);
    }

    trigger->noOfStatements    = statNo;
    trigger->noOfTriggers      = trigNo;
    trigger->preTriggerSectors = mPreTriggerSectors;
  }
  else if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR){
    ConfigFileTriggerFd *trigger;
    uint8_t statNo = 0;
    uint8_t trigNo = 0;

    ph->id  = BLOCK_ID_TRIGGER_FD;
    ph->len = sizeof(BlockHead) + sizeof(ConfigFileTriggerFd);

    trigger = (ConfigFileTriggerFd*) (pb + sizeof(BlockHead));
    memset(trigger, 0, sizeof(ConfigFileTriggerFd));

    trigger->logMode |= (mLogAll ? TRIG_LOG_ALL : 0);
    trigger->logMode |= (mFifoMode ? TRIG_LOG_FIFO : 0);
    trigger->logMode |= (mTrigScripted ? TRIG_SCRIPTED : 0);

    for (std::vector<Xmlref*>::iterator stat = mStatements.mList.begin(); stat != mStatements.mList.end(); ++stat) {
      (*stat)->createBinary((uint8_t*)&trigger->listOfTrigStatements[statNo++], version);
    }

    for (std::vector<Xmlref*>::iterator trig = mTriggers.mList.begin(); trig != mTriggers.mList.end(); ++trig) {
      (*trig)->createBinary((uint8_t*)&trigger->listOfTrigVars[trigNo++], version);
    }

    trigger->noOfStatements    = statNo;
    trigger->noOfTriggers      = trigNo;
    trigger->preTriggerSectors = mPreTriggerSectors;
  }

  pb += ph->len;
  print_block(ph);

  return (int32_t)(pb - buffer);
}

int Triggers::parseBinary(const char *buffer, const unsigned char version)
{
  if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR_OLD){
    ConfigFileTrigger *tf = (ConfigFileTrigger *)buffer;

   mLogAll       = (tf->logMode & TRIG_LOG_ALL)  == TRIG_LOG_ALL;
   mFifoMode     = (tf->logMode & TRIG_LOG_FIFO) == TRIG_LOG_FIFO;
   mTrigScripted = (tf->logMode & TRIG_SCRIPTED) == TRIG_SCRIPTED;
   mPreTriggerSectors = tf->preTriggerSectors;

    for (int trigNo = 0; trigNo < tf->noOfTriggers; ++trigNo) {
      std::string name = intToString(trigNo);
      TriggerVarFd *trigvar = (TriggerVarFd*)&tf->listOfTrigVars[trigNo];
      Trigger *trigger = Trigger::createTriggerFactory(name, trigvar->type);
      if (trigger->parseBinary((const char*)trigvar, version) != 0) {
        mTriggers.add(trigger);
      }
    }

    for (int stateNo = 0; stateNo < tf->noOfStatements; ++stateNo) {
      std::string name = "Statement" + intToString(stateNo);
      TriggerStatement *statements = (TriggerStatement*)&tf->listOfTrigStatements[stateNo];
      Statement *statement = new Statement(name);
      if (statement->parseBinary((const char*)statements, version) != 0) {
        mStatements.add(statement);
      }
    }
  }
  else if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR){
    ConfigFileTriggerFd *tffd = (ConfigFileTriggerFd *)buffer;

    mLogAll       = (tffd->logMode & TRIG_LOG_ALL)  == TRIG_LOG_ALL;
    mFifoMode     = (tffd->logMode & TRIG_LOG_FIFO) == TRIG_LOG_FIFO;
    mTrigScripted = (tffd->logMode & TRIG_SCRIPTED) == TRIG_SCRIPTED;
    mPreTriggerSectors = tffd->preTriggerSectors;

    for (int trigNo = 0; trigNo < tffd->noOfTriggers; ++trigNo) {
      std::string name = intToString(trigNo);
      TriggerVarFd *trigvar = (TriggerVarFd*)&tffd->listOfTrigVars[trigNo];
      Trigger *trigger = Trigger::createTriggerFactory(name, trigvar->type);
      if (trigger->parseBinary((const char*)trigvar, version) != 0) {
        mTriggers.add(trigger);
      }
    }

    for (int stateNo = 0; stateNo < tffd->noOfStatements; ++stateNo) {
      std::string name = "Statement" + intToString(stateNo);
      TriggerStatement *statements = (TriggerStatement*)&tffd->listOfTrigStatements[stateNo];
      Statement *statement = new Statement(name);
      if (statement->parseBinary((const char*)statements, version) != 0) {
        mStatements.add(statement);
      }
    }
  }
  else {
    PRINTF(("%s: UNSUPPORTED VERSION %d", __FUNCTION__, version));
    return 0;
  }

  PRINTF(("Read TriggerStatement block"));

  return 1;
}

int Triggers::createXml(xmlTextWriterPtr writer) const
{
  XMLWriter xml(writer);

  xml.beginElement(XML_TRIGGER_BLOCK);

  xml.beginElement(XML_TRIGGERS);
  mTriggers.createXml(writer);
  xml.endElement();

  xml.beginElement(XML_STATEMENT_BLOCK);
  mStatements.createXml(writer);
  xml.endElement();

  xml.endElement(); // Trigger block

  return 0;
}

void Triggers::parseXml(xmlNode *root_node, BusrefList &busparams, TransrefList &transLists)
{
  xmlNode *cur_node = NULL;
  xmlNode *trigger_block_node = NULL;
  xmlNode *child_node = NULL;
  int statementNo = 0;

  if (!root_node) throw_nullpointer(__FUNCTION__);

  // Part of logger mode is now found in SETTINGS
  cur_node = findFirstElementNode(root_node, XML_SETTINGS);
  if (!cur_node) throw_element_not_found(root_node, XML_SETTINGS);

  for (child_node = cur_node->children; child_node != NULL; child_node = child_node->next) {
    // Parse the mode part of the settings block
    if ( isModeElement(child_node) ) {
      mLogAll   = yes_to_uint8(child_node, XML_MODE_ATTR_LOGALL) != 0;
      mFifoMode = yes_to_uint8(child_node,XML_MODE_ATTR_FIFO) != 0;
    }
  }

  // Part of logger mode is now found in SCRIPTS
  cur_node = findFirstElementNode(root_node, XML_SCRIPT_BLOCK);
  if (cur_node) {
    for (child_node = cur_node->children; child_node != NULL; child_node = child_node->next)
    {
      // Search for active scripts
      if ( isScriptElement(child_node) ) {
        print_prop(child_node);
        mTrigScripted = true;
        break;
      }
    }
  }

  // Parse the TRIGGERBLOCK
  trigger_block_node = findFirstElementNode(root_node, XML_TRIGGER_BLOCK);
  if (!trigger_block_node) throw_element_not_found(root_node, XML_TRIGGER_BLOCK);
  PRINTF(("'TRIGGERBLOCK'"));
  parseTriggerBlock(trigger_block_node);

  // Parse the STATEMENTS block
  cur_node = findFirstElementNode(trigger_block_node, XML_STATEMENT_BLOCK);
  if (cur_node) {
    PRINTF((" 'STATEMENTS'"));
    for (child_node = cur_node->children; child_node != NULL; child_node = child_node->next) {
      // Parse the statements part of the trigger block
      if ( isStatementElement(child_node) ) {
        parseStatement(child_node, transLists);
        statementNo++;
        if (statementNo >= MAX_NR_OF_STATEMENTS) {
          throw_value_range(child_node, "number of STATEMENTs", statementNo, 0, MAX_NR_OF_STATEMENTS-1);
        }
      }
    }
  }

  // Calculate pre trigger sectors.
  // Note: bus parameter block must have been read before this.
  mPreTriggerSectors = getPreTriggerSectors(busparams);
}

int Triggers::getTrigger(xmlNode *child_node, Trigger **pTrigger)
{
  int triggerNo = 0;
  std::string name;
  uint8_t type;

  if ( !getAttributeString(child_node, XML_TRIGGERS_ATTR_NAME, name) ) {
    throw_attribute_not_found(child_node, XML_TRIGGERS_ATTR_NAME);
  }

  if ( !mTriggers.isUnique(name) ){
    throw_string_unique (child_node, XML_TRIGGERS_ATTR_NAME, name);
  }

  type = node_name_to_uint8(child_node, TrigvarTypeStrings, TrigvarTypeValues, TrigvarTypeLength);
  *pTrigger = Trigger::createTriggerFactory(name, type);
  triggerNo = mTriggers.add(*pTrigger);

  PRINTF(("Add trigger: 0x%x %s\n", triggerNo, name.c_str()));

  if (triggerNo >= MAX_NR_OF_TRIGVARS) {
    throw_value_range(child_node, "number of TRIGGERs", triggerNo, 0, MAX_NR_OF_TRIGVARS-1);
  }
  return triggerNo;
}

void Triggers::parseTriggerBlock(xmlNode *trigger_block_node)
{
  xmlNode *cur_node   = NULL;
  xmlNode *child_node = NULL;
  Trigger *pTrigger   = NULL;

  cur_node = findFirstElementNode(trigger_block_node, XML_TRIGGERS);
  if (cur_node) {
    for (child_node = cur_node->children; child_node != NULL; child_node = child_node->next) {
    {
      // Parse the trigger part of the trigger block
      if (isTriggerElement(child_node)) {
        if (getTrigger(child_node, &pTrigger)) {
          // Ignore return value from  getTrigger()
        }

        switch (pTrigger->mType)
        {
          case TRIGVAR_TYPE_DISABLED:
            // Empty
            break;

          case TRIGVAR_TYPE_MSG_ID:
            {
              MsgIdTrigger *msgTrigger = (MsgIdTrigger*)pTrigger;
              msgTrigger->mMsgId     = num_to_uint32(child_node, XML_TRIGGERS_ATTR_ID);
              msgTrigger->mMsgIdMin  = num_to_uint32(child_node, XML_TRIGGERS_ATTR_ID_MIN);
              msgTrigger->mIsExtId   = yes_to_uint8_or_default(child_node, XML_TRIGGERS_ATTR_ID_EXT, 0) != 0;
              msgTrigger->mHelpMask  = protocol_to_uint8_or_default(child_node);
              msgTrigger->mHelpMask  |= protocol_msk_to_uint8_or_default(child_node);
              msgTrigger->mCh        = num_to_uint8(child_node, XML_TRIGGERS_ATTR_CHAN);
            }
            break;

          case TRIGVAR_TYPE_MSG_DLC:
            {
              DlcTrigger *dlcTrigger = (DlcTrigger*)pTrigger;
              dlcTrigger->mDlc    = num_to_uint8(child_node, XML_TRIGGERS_ATTR_DLC);
              dlcTrigger->mDlcMin = num_to_uint8(child_node, XML_TRIGGERS_ATTR_DLC_MIN);
              dlcTrigger->mCh     = num_to_uint8(child_node, XML_TRIGGERS_ATTR_CHAN);
            }
            break;

          case TRIGVAR_TYPE_MSG_FLAG:
            {
              ErrorFrameTrigger *errTrigger = (ErrorFrameTrigger*)pTrigger;
              errTrigger->mCh = num_to_uint8(child_node, XML_TRIGGERS_ATTR_CHAN);
            }
            break;

          case TRIGVAR_TYPE_SIGVAL:
            {
              SigValTrigger* sigTrigger = (SigValTrigger*)pTrigger;
              sigTrigger->mMsgId     = num_to_uint32(child_node, XML_TRIGGERS_ATTR_ID);
              sigTrigger->mIsExtId   = yes_to_uint8_or_default(child_node, XML_TRIGGERS_ATTR_ID_EXT, 0) != 0;
              sigTrigger->mDataType  = datatype_to_uint8(child_node);
              sigTrigger->mStartBit  = num_to_uint16(child_node, XML_TRIGGERS_ATTR_STARTBIT);
              sigTrigger->mLength    = num_to_uint16(child_node, XML_TRIGGERS_ATTR_BITLENGTH);
              sigTrigger->mData      = num_to_uint32(child_node, XML_TRIGGERS_ATTR_DATA);
              sigTrigger->mDataMin   = num_to_uint32(child_node, XML_TRIGGERS_ATTR_DATA_MIN);
              sigTrigger->mCondition = string_to_uint8(child_node, XML_TRIGGERS_ATTR_DATA_COND,
                                                       OnDataTypeStrings, OnDataTypeValues, OnDataTypeLength);
              sigTrigger->mHelpMask  = protocol_to_uint8_or_default(child_node);
              sigTrigger->mHelpMask |= protocol_msk_to_uint8_or_default(child_node);
              sigTrigger->mCh        = num_to_uint8(child_node, XML_TRIGGERS_ATTR_CHAN);
              if ( hasAttribute(child_node, XML_TRIGGERS_ATTR_DLC) ) {
                sigTrigger->mDlc       = num_to_uint8(child_node, XML_TRIGGERS_ATTR_DLC);
                sigTrigger->mIgnoreDlc = false;
              } else {
                sigTrigger->mIgnoreDlc = true;
              }
            }
            break;

          case TRIGVAR_TYPE_EXTERNAL:
            {
              ExtTrigger* extTrigger = (ExtTrigger*)pTrigger;
              extTrigger->mCh    = num_to_uint8(child_node, XML_TRIGGERS_ATTR_CHAN);
              extTrigger->mLevel = string_to_uint8(child_node, XML_TRIGGERS_ATTR_EXTLEVEL, TrigLevelStrings, TrigLevelValues, TrigLevelLength);
            }
            break;

          case TRIGVAR_TYPE_TIMER:
            {
              TimerTrigger* timerTrigger = (TimerTrigger*)pTrigger;
              timerTrigger->mOffset    = num_to_uint32(child_node, XML_TRIGGERS_ATTR_OFFSET);
              timerTrigger->mIsRepeat  = yes_to_uint8(child_node, XML_TRIGGERS_ATTR_REPEAT) != 0;
            }
          break;

          case TRIGVAR_TYPE_DISK_FULL:
            // Empty
          break;

          case TRIGVAR_TYPE_STARTUP:
            // Empty
          break;

          default:
            throw_general_failure("Unknown TRIGGER_XXX in XML-file.");
          break;
        }
        // Common for all trigvars
        pTrigger->mTimeout = num_to_uint32_or_default(child_node, XML_TRIGGERS_ATTR_TMO, 0);

      }
      else {
          if (child_node->type == XML_ELEMENT_NODE) {
            warning_element_unknown(child_node);
          }
        }
      }
    }
  }
}

void Triggers::parseStatement(xmlNode * a_node, TransrefList &transLists) {
  xmlNode *child_node = NULL;
  xmlNode *action_node = NULL;
  int actionNo = 0;
  int expressionNo = 0;
  Statement *statement;

  statement = new Statement(std::string("Statement"));

  // Get the properties
  statement->mPreTrigger  = num_to_uint32(a_node, XML_STATEMENT_ATTR_PRE);
  statement->mPostTrigger = num_to_uint32(a_node, XML_STATEMENT_ATTR_POST);

  // The children: actions and a trigger expression
  for (child_node = a_node->children; child_node != NULL; child_node = child_node->next)
  {
    if ( isExpressionElement(child_node) ) {
      std::string expr;
      print_element_content(child_node);
      expressionNo++;
      getElementString(child_node, expr);
      PRINTF(("Trigger expression: '%s'\n", expr.c_str()));
      statement->mInFixExpr = expr;
      getTriggersFromInfix(expr, statement->mTriggNames);
      xmlstringToPostfix(child_node, expr.c_str(), statement->mPostFixExpr);
    }
    else if ( isActionBlock(child_node) ) {
      for (action_node = child_node->children; action_node != NULL; action_node = action_node->next) {
        if (isActionElement (action_node) ) {
          uint8_t actionFcn = node_name_to_uint8 (action_node, ActionTypeStrings,
                                                  ActionTypeValues, ActionTypeLength);
          Action *action = NULL;

          switch (actionFcn) {
          case ACTION_START_LOG:
          case ACTION_STOP_LOG:
          case ACTION_STOP_LOG_COMPLETELY:
            action = new Action(std::string("LogAction"));
            action->mType = actionFcn;
            break;

          case ACTION_EXTERNAL_PULSE:
            {
              PulseAction *pulseAction = new PulseAction(std::string("PulseAction"));
              action = pulseAction;
              pulseAction->mType = actionFcn;
              pulseAction->mDuration = num_to_uint32(action_node, XML_ACTIONS_ATTR_DURATION);
            }
            break;

          case ACTION_ACTIVATE_AUTO_TRANSMIT_CHAIN:
          case ACTION_DEACTIVATE_AUTO_TRANSMIT_CHAIN:
          {
            std::string name;
            TransmitAction *transAction = new TransmitAction(std::string("TlistAction"));
            action = transAction;
            transAction->mType = actionFcn;

            if ( !getAttributeString(action_node, XML_ACTIONS_ATTR_TRANSMITLIST, name) ) {
              throw_attribute_not_found(action_node, XML_ACTIONS_ATTR_TRANSMITLIST);
            }
            if ( transLists.isUnique(name) ) {
              PRINTF(("ERROR: The transmit list '%s' in ACTIONS is not defined.\n", name.c_str()));
              set_error_and_throw ( KvaXmlStatusERR_EXPRESSION, "The transmit list '%s' in ACTIONS is not defined.", name.c_str());
            }
            transAction->mTlistName  = name;
            transAction->mTlistIndex = transLists.getIdx(name);
          }
          break;

          default:
            set_error_and_throw(KvaXmlStatusERR_VALUE_RANGE, "ACTION %d is not implemented.", actionFcn);
            break;
          }
          statement->mActions.add(action);
        }
        if (actionNo > MAX_ACTIONS_PER_STATEMENT) {
          throw_wrong_element_count (a_node, XML_ACTIONS, actionNo, 1, MAX_ACTIONS_PER_STATEMENT);
        }
      }
    }
    else {
      warning_element_unknown(child_node);
    }
  }

  if (expressionNo != 1 )
  {
    // Validation: Each statement must contain exactly one trigger expression
    throw_wrong_element_count (a_node, XML_EXPRESSION, expressionNo, 1, 1);
  }
  if (statement->mActions.mLastIdx < 1)
  {
    // Validation: Each statement must contain at least one action
    throw_wrong_element_count (a_node, "ACTION", actionNo, 1, MAX_ACTIONS_PER_STATEMENT);
  }

  mStatements.add(statement);
}

uint32_t Triggers::getPreTriggerSectors(BusrefList &busparams) const
{
  uint32_t preTriggerSectors = 0;
  uint64_t maxPreTrigger = 0;
  uint64_t totBitrate = busparams.getTotalBitrate();

  for(std::vector<Xmlref*>::const_iterator it = mStatements.mList.begin(); it != mStatements.mList.end(); ++it) {
    if (((Statement*)(*it))->mPreTrigger > maxPreTrigger) {
      maxPreTrigger = ((Statement*)(*it))->mPreTrigger;
    }
  }

  if (maxPreTrigger) {
    preTriggerSectors = (uint32_t)((totBitrate *  maxPreTrigger) / 2000000ULL) + 1;
  }

  PRINTF(("PreTriggerSectors %u, Max Pretrigger %u, Total bitrate %u",
          preTriggerSectors, (unsigned int)maxPreTrigger, (unsigned int)totBitrate));

  return preTriggerSectors;
}

void Triggers::getTriggersFromInfix(std::string& infixExpr, std::vector<std::string>& list)
{
  std::vector<std::string> items;
  Xmlref* pTrigger = NULL;

  tokenizeStr(infixExpr, items);

  for (std::vector<std::string>::iterator it = items.begin(); it != items.end(); ++it) {
    if (isOp(*it)) continue;
    pTrigger = mTriggers.get(*it);
    if (pTrigger) {
      list.push_back(*it);
    }
  }
}

// Return a postfix expression with names or numbers
void Triggers::parseExpression(std::string& trigger_expr, bool /* convert */)
{
  std::vector<std::string> items;
  std::string str;

  if (trigger_expr.size() == 0 && mTriggers.mList.size() == 0) {
    // Empty expression and no triggers.
    return;
  }

  stringToList(trigger_expr, items, ',');

  // Each item that is not an operator (OR and AND) must be a trigger
  for(std::vector<std::string>::iterator it = items.begin(); it != items.end(); ++it) {
    if (isOp(*it)) {
      str.append(*it);
    }
    else {
      if (mTriggers.isUnique(*it)) {
        PRINTF(("ERROR: The trigger '%s' in '%s' is not defined.\n", (*it).c_str(), trigger_expr.c_str()));
        set_error_and_throw (KvaXmlStatusERR_EXPRESSION, "The trigger '%s' in '%s' is not defined.\n", (*it).c_str(),
          trigger_expr.c_str());
      }
       str.append(intToString(mTriggers.getIdx(*it)));
    }
    str.append(", ");
  }

  // Remove the last ', ' from expression
  if (str.size() > 2) {
    str.erase(str.end() - 2, str.end());
  }

  if ( !str.size()  ) {
    PRINTF(("ERROR: Trigger expression '%s' is not a proper trigger expression.",
            trigger_expr.c_str()));
    set_error_and_throw (KvaXmlStatusERR_EXPRESSION,
                         "ERROR: Trigger expression '%s' is not a proper trigger expression.",
                         trigger_expr.c_str());
  }

  trigger_expr = str;
}

void Triggers::stringToList(const std::string& trigger_expr,
                            std::vector<std::string>& list,
                            const char splitter) const
{
  std::stringstream ss( trigger_expr );
  // Break the string ("T_1", "T_2", "OR") into items and remove ws, () and ":s
  while( ss.good() ) {
    std::string substr;
    getline( ss, substr, splitter);
    clearws(substr);
    substr.erase (std::remove(substr.begin(), substr.end(), '"'), substr.end());
    substr.erase (std::remove(substr.begin(), substr.end(), '('), substr.end());
    substr.erase (std::remove(substr.begin(), substr.end(), ')'), substr.end());
    if (!substr.empty()) {
      list.push_back( substr );
    }
  }
}

// ---------------------------------------------------------------------------
// Takes a string containing a trigger expression, converts it to a postfix
// string and passes that string on for conversion to a binary representation
// of the postfix expression -- a perfect execution of the magma flow design
// pattern...
void Triggers::xmlstringToPostfix(xmlNode *a_node, const char *expr,
                                  uint8_t *postfixBin)
{
  std::string infixExprString(expr);
  std::string postfixExprString("");
  char *infixExpr = NULL;
  size_t len;
  KvParseHandle *handle;
  Token *exprToken;

  PRINTF(("infixExprString to parse: %s\n", infixExprString.c_str()));

  // Perform subsitutions /"//, /AND/&/ and /OR/|/ in infixExprString to conform with
  // the expectations of kvaToolsParseExpr:
  replaceAll(infixExprString, "\"", "");
  replaceAllWithNeighbourConditions(infixExprString, "AND", "&", "() ");
  replaceAllWithNeighbourConditions(infixExprString, "OR", "|", "() ");
  replaceAllWithNeighbourConditions(infixExprString, "and", "&", "() ");
  replaceAllWithNeighbourConditions(infixExprString, "or", "|", "() ");

  // Copy contents of infixExprString to char* buffer that is not const, so
  // that kvaToolsParseExpr can poke it:
  len = infixExprString.size();
  if (!len) throw_invalid_expression(a_node);
  infixExpr = (char*) malloc(len + 1);
  memcpy(infixExpr, infixExprString.c_str(), len + 1);
  if (!infixExpr) throw_invalid_expression(a_node);

  PRINTF(("infixExpr: %s\n", infixExpr));

  // Start parser and call it to get a linked list of tokens representing the
  // postfix expression:
  handle = kvaToolsParseCreate();
  kvaToolsParseExpr(handle, infixExpr, &exprToken);

  // Walk postfix expression to create a string representation again:
  walkExpression(exprToken, postfixExprString, 0);

  // Replace trigger names with indices:
  parseExpression(postfixExprString);

  PRINTF(("postfixExprString, converted: %s\n", postfixExprString.c_str()));

  // Finally, get a binary representation from the postfix string:
  postfixToBin(a_node, postfixExprString.c_str(), postfixBin);

  kvaToolsParseDestroy(handle);
  free(infixExpr);
}

// ---------------------------------------------------------------------------
// Takes a comma string containing a comma separated postfix expression and
// converts it to a binary representation for the benefit of PARAM.LIF
void Triggers::postfixToBin(xmlNode *a_node, const char *postfixExpr, uint8_t *postfixBin)
{
  char *tok, *string= NULL;
  unsigned int len=0, i=0, j=0;
  int counter = 0;

  if (!a_node || !postfixExpr || !postfixBin) throw_nullpointer(__FUNCTION__);

  len = (unsigned int) strlen(postfixExpr);

  if (!len) throw_invalid_expression(a_node);

  // Copy and remove white space
  string = (char*) malloc(len+1);
  memset(string, 0, len+1);
  for (j=0; j < len; j++) {
    if (!isspace(postfixExpr[j])) {
      string[i] = postfixExpr[j];
      i++;
    }
  }

  PRINTF(("Postfix: '%s' ",string));
  // Parse
  tok = strtok(string, ",");
  j = 0;
  while (tok != NULL) {
    if ((strcmp(tok, "AND") == 0) || (strcmp(tok, "and") == 0)){
      postfixBin[j] = EXPR_OP_AND;
    }
    else if ((strcmp(tok, "OR") == 0) || (strcmp(tok, "or") == 0)){
      postfixBin[j] = EXPR_OP_OR;
    }
    else if (IS_VARIABLE(atoi(tok))) {
      postfixBin[j] = atoi(tok);
    } else {
      postfixBin[j] = EXPR_OP_NONE;
    }
    j++;
    tok = strtok(NULL,",");
    if (j > MAX_EXPR_LEN) {
      free(string);
      throw_value_range (a_node, "postfix expression length", j, 1, MAX_EXPR_LEN);
    }
  }
  postfixBin[j] = EXPR_OP_NONE;
  free(string);
  PRINTF(("contains %d items ", j+1));
  /* Validate and convert postfix
  # Initialize the counter to zero.
  # When you see a literal, increment the counter.
  # When you see a binary operator, decrement the counter twice, then increment it.
  # When you see a binary operator, decrement the counter, then increment it.
  # At the end of the string, if the counter is one, and if it never went below zero, the string is valid. */
  for (j=0; postfixBin[j] != EXPR_OP_NONE; j++) {
      if (IS_OPERATOR(postfixBin[j]))
        counter--;
      else
        counter++;
  }
  if (counter != 1) throw_invalid_expression(a_node);
  PRINTF(("and is valid.\n"));
}

// ---------------------------------------------------------------------------
// Walk through the token list representation of the postfix expression and
// generate a string representation.
void Triggers::walkExpression(Token *exprToken, std::string &postfixExpr, int iter)
{
  if (!exprToken) {
    return;
  }

  if (exprToken->type == T_OP_AND) {
    walkExpression(exprToken->left, postfixExpr, iter + 1);
    walkExpression(exprToken->right, postfixExpr, iter + 1);
    postfixExpr += ",AND";
  } else if (exprToken->type == T_OP_OR) {
    walkExpression(exprToken->left, postfixExpr, iter + 1);
    walkExpression(exprToken->right, postfixExpr, iter + 1);
    postfixExpr += ",OR";
  } else if (exprToken->type == T_ID) {
    if (postfixExpr.size()) {
      postfixExpr += ",";
    }
    postfixExpr += exprToken->name;
  }
}

void Triggers::getTriggerStatus(std::string& trigger_expr, std::vector<std::string>& undefined)
{
  std::vector<std::string> items;
  stringToList(trigger_expr, items, ' ');

  // Each item that is not an operator (OR and AND) must be a trigger
  for(std::vector<std::string>::iterator it = items.begin(); it != items.end(); ++it) {
    if (isOp(*it)) continue;
    if (mTriggers.isUnique(*it)) {
      undefined.push_back((*it));
    }
  }
}

void Triggers::getTriggers(std::string& trigger_expr, std::vector<std::string>& list)
{
  std::vector<std::string> items;
  Xmlref* pTrigger = NULL;

  stringToList(trigger_expr, items, ',');

  for(std::vector<std::string>::iterator it = items.begin(); it != items.end(); ++it) {
    if (isOp(*it)) continue;
    pTrigger = mTriggers.get(*it);
    if (pTrigger) {
      list.push_back(*it);
    }
  }
}

void Triggers::getTriggers(std::vector<std::string>& list)
{
  for(std::vector<Xmlref*>::iterator it = mTriggers.mList.begin(); it != mTriggers.mList.end(); ++it) {
    list.push_back((*it)->mName);
  }
}
