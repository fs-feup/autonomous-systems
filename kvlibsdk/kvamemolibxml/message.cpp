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
** Description: Class to handle CAN messages in XML settings for
**              Kvaser Memorator Pro(2nd generation)
** -----------------------------------------------------------------------------
*/

#include <message.h>
#include <stdio.h>

#include "util.h"
#include "hydra_host_cmds.h"
#include "xmlwriter.h"
#include "errorhandler.h"
#include "convert.h"

Message::Message(std::string name)
{
    mName = name;
    mIdx = INVALID_IDX;
    mId = 0;
    mDlc = 0;
    mDataBytes = 0;
    mIsExtended = false;
    mIsErrorFrame = false;
    mIsRemoteFrame = false;
    mIsCanFd = false;
    mIsCanFdBrs = false;
}

Message::~Message()
{
  mData.clear();
}

bool Message::isEqual(const Message* other) const
{
  return  (mId            == other->mId &&
           mDlc           == other->mDlc &&
           mDataBytes     == other->mDataBytes &&
           mIsExtended    == other->mIsExtended &&
           mIsErrorFrame  == other->mIsErrorFrame &&
           mIsRemoteFrame == other->mIsRemoteFrame &&
           mIsCanFd       == other->mIsCanFd &&
           mIsCanFdBrs    == other->mIsCanFdBrs &&
           mData          == other->mData);
}

void Message::print()
{
  PRINTF(("%d %s Id: %u (%s) %d [", mIdx, mName.c_str(), mId, mIsExtended?"EXT":"STD", mDlc));
  for (unsigned int k = 0; k < mData.size(); k++) {
    PRINTF(("0x%02x ", mData[k]));
  }
  PRINTF(("]\n"));
}

int Message::createBinary(unsigned char *buffer, unsigned char version)
{
  if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR_OLD) {
    autoTransmitMessage *msg = (autoTransmitMessage *)buffer;

    msg->id  = mId | (mIsExtended ? CAN_IDENT_IS_EXTENDED : 0);
    msg->dlc = mDlc;

    if (mIsRemoteFrame) {
      msg->flags |= MSGFLAG_REMOTE_FRAME;
    }

    // if (mIsExtended) {
    //   msg->flags |= MSGFLAG_EXT;
    // }

    if (mIsErrorFrame) {
      msg->flags  |= MSGFLAG_ERROR_FRAME;
    }
    else {
      size_t len = mData.size();
      if (len > 8) len = 8;
      for (size_t i=0; i < len; i++) {
        msg->data[i] = mData[i];
      }
    }
  }

  else if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR) {
    autoTransmitMessageFd *msg = (autoTransmitMessageFd *)buffer;

    msg->id         = mId | (mIsExtended ? CAN_IDENT_IS_EXTENDED : 0);
    msg->dlc        = mDlc;
    msg->databytes  = mDataBytes;

    if (mIsRemoteFrame) {
      msg->flags |= MSGFLAG_REMOTE_FRAME;
    }

    if (mIsExtended) {
      msg->flags |= MSGFLAG_EXT;
    }

    if (mIsCanFd) {
      msg->flags |= MSGFLAG_FDF;
      if (mIsCanFdBrs) {
        msg->flags |= MSGFLAG_BRS;
      }
    }

    if (mIsErrorFrame) {
      msg->flags |= MSGFLAG_ERROR_FRAME;
    }
    else {
      size_t len = mData.size();
      if (len > 64) len = 64;
      for (size_t i=0; i < len; i++) {
        msg->data[i] = mData[i];
      }
    }
  }
  else {
    PRINTF(("%s: UNSUPPORTED VERSION %d", __FUNCTION__, version));
  }
  return 0;
}

int Message::parseBinary(const char *buffer, const unsigned char version)
{
  if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR_OLD) {
    autoTransmitMessage *msg = (autoTransmitMessage *)buffer;

    mId            = msg->id & ~CAN_IDENT_IS_EXTENDED;
    mDlc           = msg->dlc;
    mIsExtended    = (msg->id & CAN_IDENT_IS_EXTENDED) == CAN_IDENT_IS_EXTENDED;
    mIsErrorFrame  = (msg->flags & MSGFLAG_ERROR_FRAME) == MSGFLAG_ERROR_FRAME;
    mIsRemoteFrame = (msg->flags & MSGFLAG_REMOTE_FRAME) == MSGFLAG_REMOTE_FRAME;

    if (!mIsErrorFrame) {
      int len = mDlc;
      if (len > 8) len = 8;
      mData.clear();
      for (int i = 0; i < len; i++) {
        mData.push_back(msg->data[i]);
      }
    }
  }

  else if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR) {
    autoTransmitMessageFd *msg = (autoTransmitMessageFd *)buffer;

    mId            = msg->id & ~CAN_IDENT_IS_EXTENDED;
    mDlc           = msg->dlc;
    mDataBytes     = msg->databytes;
    mIsExtended    = (msg->id & CAN_IDENT_IS_EXTENDED) == CAN_IDENT_IS_EXTENDED;
    mIsErrorFrame  = (msg->flags & MSGFLAG_ERROR_FRAME) == MSGFLAG_ERROR_FRAME;
    mIsRemoteFrame = (msg->flags & MSGFLAG_REMOTE_FRAME) == MSGFLAG_REMOTE_FRAME;
    mIsCanFd       = (msg->flags & MSGFLAG_FDF) == MSGFLAG_FDF;
    mIsCanFdBrs    = (msg->flags & MSGFLAG_BRS) == MSGFLAG_BRS;

    if (!mIsErrorFrame) {
      int len = mDataBytes;
      if (len > 64) len = 64;
      mData.clear();
      for (int i = 0; i < len; i++) {
        mData.push_back(msg->data[i]);
      }
    }
  }

  else {
    PRINTF(("%s: UNSUPPORTED VERSION %d", __FUNCTION__, version));
    return 0;
  }

  PRINTF(("Read Message %s with id 0x%x", mName.c_str(), mId));

  return 1;
}

int Message::createXml(xmlTextWriterPtr writer) const
{
  XMLWriter xml(writer);

  xml.beginElement(XML_MSG);
  xml.writeAttr(XML_MSG_ATTR_NAME, mName);
  xml.writeAttr(XML_MSG_ATTR_ID, "0x%x", mId);
  xml.writeAttrYesNo(XML_MSG_ATTR_ID_EXT, mIsExtended);
  xml.writeAttrYesNo(XML_MSG_ATTR_FLAG_FD, mIsCanFd);
  xml.writeAttrYesNo(XML_MSG_ATTR_FLAG_BRS, mIsCanFdBrs);
  xml.writeAttrYesNo(XML_MSG_ATTR_FLAG_EF, mIsErrorFrame);
  xml.writeAttrYesNo(XML_MSG_ATTR_FLAG_RF, mIsRemoteFrame);
  xml.writeAttr(XML_MSG_ATTR_DLC, "%u", mDlc);

  uint8_t i = 0;
  for(std::vector<uint8_t>::const_iterator it = mData.begin(); it != mData.end(); ++it) {
    char tmp_b[8];
    sprintf(tmp_b, "b%d", i++);
    xml.writeAttr(tmp_b, "0x%x", *it);
  }

  xml.endElement();

  return 0;
}

bool MessagerefList::getMessage(std::string name, Message *&msg)
{
  for(std::vector<Xmlref*>::iterator it = mList.begin(); it != mList.end(); ++it) {
    if ((*it)->mName == name) {
      msg = (Message*)(*it);
      return true;
    }
  }
  return false;
}


Message* MessagerefList::find(Message *m)
{
  for(std::vector<Xmlref*>::iterator it = mList.begin(); it != mList.end(); ++it) {
    if (m->isEqual((Message*)(*it))) return (Message*)(*it);
  }
  return NULL;
}

int MessagerefList::createXml(xmlTextWriterPtr writer) const
{
  XMLWriter xml(writer);

  xml.beginElement(XML_MSG_BLOCK);

  for(std::vector<Xmlref*>::const_iterator it = mList.begin(); it != mList.end(); ++it) {
    if ((*it)) {
      (*it)->createXml(writer);
    }
  }

  xml.endElement(); // Messages

  return 0;
}

void MessagerefList::parseXml(xmlNode *root_element)
{
  xmlNode *msg_block_node = NULL;
  xmlNode *msg_node = NULL;

  if (!root_element) throw_nullpointer(__FUNCTION__);

  msg_block_node = findFirstElementNode(root_element, XML_MSG_BLOCK);
  if (msg_block_node) {
    for (msg_node = msg_block_node->children; msg_node != NULL; msg_node = msg_node->next) {
      if (isMessageElement(msg_node) ) {
        std::string name;
        Message * pmsg;
        if ( !getAttributeString(msg_node, XML_MSG_ATTR_NAME, name) ) {
          throw_attribute_not_found(msg_node, XML_MSG_ATTR_NAME);
        }
        pmsg = new Message(name);
        pmsg->mId            = num_to_uint32(msg_node, XML_MSG_ATTR_ID);
        pmsg->mIsExtended    = yes_to_uint8_or_default(msg_node, XML_MSG_ATTR_ID_EXT,   0)?true:false;
        pmsg->mDlc           = num_to_uint8(msg_node, XML_MSG_ATTR_DLC);
        pmsg->mIsErrorFrame  = yes_to_uint8_or_default(msg_node, XML_MSG_ATTR_FLAG_EF,  0)?true:false;
        pmsg->mIsRemoteFrame = yes_to_uint8_or_default(msg_node, XML_MSG_ATTR_FLAG_RF,  0)?true:false;
        pmsg->mIsCanFd       = yes_to_uint8_or_default(msg_node, XML_MSG_ATTR_FLAG_FD,  0)?true:false;
        pmsg->mIsCanFdBrs    = yes_to_uint8_or_default(msg_node, XML_MSG_ATTR_FLAG_BRS, 0)?true:false;
        pmsg->mDataBytes     = dlcToBytes(pmsg->mDlc, pmsg->mIsCanFd);

        if (pmsg->mIsErrorFrame && ((pmsg->mIsCanFd) || (pmsg->mIsCanFdBrs))) {
          PRINTF(("ERROR: Message '%s' is a CANFD message and thus cannot be a remote frame.", name.c_str()));
          set_error_and_throw ( KvaXmlStatusERR_ATTR_VALUE, "Message '%s' is a CANFD message and thus cannot be a remote frame.", name.c_str());
        }

        for (int k=0;k<64;k++) {
          char byteNo[4]; //b0,b1,b2,
          sprintf(byteNo, "b%d", k);
          if (! hasAttribute(msg_node, byteNo)) break;
          pmsg->mData.push_back(num_to_uint8(msg_node,byteNo));
        }
        add(pmsg);
      }
    }
  }
}
