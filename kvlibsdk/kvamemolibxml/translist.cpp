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

#include <translist.h>
#include <cstdio>
#include <cstring>
#include <string>

#include "hydra_host_cmds.h"
#include "util.h"
#include "convert.h"
#include "xmlwriter.h"
#include "errorhandler.h"

// ===========================================================================
// TransMsg
// ===========================================================================
int TransMsg::parseBinary(const char *buffer, const unsigned char version)
{
  if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR_OLD) {
    autoTransmitMessage *aMsg = (autoTransmitMessage *)buffer;
    mCh = aMsg->channel;
  }
  else if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR) {
    autoTransmitMessageFd *aMsg = (autoTransmitMessageFd *)buffer;
    mCh = aMsg->channel;
  }
  return 1;
}

int TransMsg::createXml(xmlTextWriterPtr writer) const
{
  XMLWriter xml(writer);

  xml.beginElement(XML_TRANSMIT_MSG);
  xml.writeAttr(XML_TRANSMIT_MSG_NAME, mName);
  xml.writeAttr(XML_TRANSMIT_CHAN, "%u", mCh);
  xml.endElement();

  return 0;
}

// ===========================================================================
// Translist
// ===========================================================================
Translist::Translist(std::string name)
{
    mName       = name;
    mIdx        = INVALID_IDX;
    mMsgDelay   = 0;
    mCycleDelay = 0;
    mCyclic     = false;
    mAutostart  = false;
}

Translist::~Translist()
{
  for(std::vector<TransMsg*>::iterator it = mMessages.begin(); it != mMessages.end(); ++it) {
    if ((*it)) delete (*it);
  }
  mMessages.clear();
}

void Translist::print()
{
  PRINTF(("%d %s [", mIdx, mName.c_str()));
  PRINTF(("  Msg delay   %u", mMsgDelay));
  PRINTF(("  Cycle delay %u", mCycleDelay));
  PRINTF(("  Cyclic      %s", mCyclic ? "Yes" : "No"));
  PRINTF(("  Autostart   %s", mAutostart ? "Yes" : "No"));
  PRINTF(("  Messages ["));
  for(std::vector<TransMsg*>::iterator it = mMessages.begin(); it != mMessages.end(); ++it) {
    PRINTF(("    Name    %s", (*it)->mName.c_str()));
    PRINTF(("    Channel %u", (*it)->mCh));
  }
  PRINTF(("  ]"));
  PRINTF(("]"));
}

int Translist::createBinary(unsigned char *buffer, unsigned char msgNo)
{
  autoTransmitChainConfig *chain = NULL;

  chain = (autoTransmitChainConfig*) (buffer);
  memset(chain, 0, sizeof (autoTransmitChainConfig));

  chain->first      = msgNo;
  chain->last       = msgNo + (unsigned char)mMessages.size() - 1;
  chain->msgDelay   = mMsgDelay;
  chain->cycleDelay = mCycleDelay;
  chain->flags      = AUTO_TRANSMIT_FLAG_CHAIN_IN_USE;
  chain->flags     |= mCyclic ? AUTO_TRANSMIT_FLAG_CYCLIC : 0;
  chain->flags     |= mAutostart ? AUTO_TRANSMIT_FLAG_AUTOSTART : 0;

  return chain->last;
}

int Translist::parseBinary(const char *buffer, const unsigned char /* version */)
{
  autoTransmitChainConfig *chain = (autoTransmitChainConfig *)buffer;

  mMsgDelay   = chain->msgDelay;
  mCycleDelay = chain->cycleDelay;
  mCyclic     = (chain->flags & AUTO_TRANSMIT_FLAG_CYCLIC) ==  AUTO_TRANSMIT_FLAG_CYCLIC;
  mAutostart  = (chain->flags & AUTO_TRANSMIT_FLAG_AUTOSTART) == AUTO_TRANSMIT_FLAG_AUTOSTART;

  return 1;
}

int Translist::createXml(xmlTextWriterPtr writer) const
{
  XMLWriter xml(writer);

  xml.beginElement(XML_TRANSMIT_LIST);
  xml.writeAttr(XML_TRANSMIT_LIST_ATTR_NAME, mName);
  xml.writeAttr(XML_TRANSMIT_LIST_ATTR_MSG_DELAY, "%u", mMsgDelay);
  xml.writeAttr(XML_TRANSMIT_LIST_ATTR_CYCLIC_DELAY, "%u", mCycleDelay);
  xml.writeAttrYesNo(XML_TRANSMIT_LIST_ATTR_CYCLIC, mCyclic);
  xml.writeAttrYesNo(XML_TRANSMIT_LIST_ATTR_AUTO_START, mAutostart);

  for (std::vector<TransMsg*>::const_iterator it = mMessages.begin(); it != mMessages.end(); ++it) {
    (*it)->createXml(writer);
  }

  xml.endElement();

  return 0;
}


// ===========================================================================
// TransrefList
// ===========================================================================
int TransrefList::createBinary(unsigned char *buffer, unsigned char version)
{
  unsigned char *pb;
  BlockHead     *ph;
  unsigned int   chainNo = 0;
  unsigned char  msgNo = 0;

  if (mList.size() == 0) {
    return 0;
  }

  pb = buffer;
  ph = (BlockHead*) pb;

  if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR_OLD) {
    ConfigFileAutoTransmit *trans = NULL;

    ph->id  = BLOCK_ID_AUTO_TRANSMIT;
    ph->len = sizeof(BlockHead) + sizeof(ConfigFileAutoTransmit);

    trans = (ConfigFileAutoTransmit*) (pb + sizeof(BlockHead));
    memset(trans, 0, sizeof (ConfigFileAutoTransmit));

    trans->used = 1;
    for (std::vector<Xmlref*>::iterator it = mList.begin(); it != mList.end(); ++it) {

      (void)(*it)->createBinary((unsigned char *)&trans->chains[chainNo], msgNo);

      for (std::vector<TransMsg*>::iterator msgIt = ((Translist*)(*it))->mMessages.begin(); msgIt != ((Translist*)(*it))->mMessages.end(); ++msgIt) {
        autoTransmitMessage *atMsg = &trans->msgs[msgNo];
        Message             *msg   = (*msgIt)->mMsg;

        msg->createBinary((unsigned char *)atMsg, version);
        atMsg->channel = (*msgIt)->mCh;

        msgNo++;
      }
      chainNo++;
    }

    pb += ph->len;
    print_block(ph);
  }
  else if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR) {
    ConfigFileAutoTransmitFd *trans = NULL;

    ph->id  = BLOCK_ID_AUTO_TRANSMIT_FD;
    ph->len = sizeof(BlockHead) + sizeof(ConfigFileAutoTransmitFd);

    trans = (ConfigFileAutoTransmitFd*) (pb + sizeof(BlockHead));
    memset(trans, 0, sizeof (ConfigFileAutoTransmitFd));

    trans->used = 1;
    for (std::vector<Xmlref*>::iterator it = mList.begin(); it != mList.end(); ++it) {

      (void)(*it)->createBinary((unsigned char *)&trans->chains[chainNo], msgNo);

      for (std::vector<TransMsg*>::iterator msgIt = ((Translist*)(*it))->mMessages.begin(); msgIt != ((Translist*)(*it))->mMessages.end(); ++msgIt) {
        autoTransmitMessageFd *atMsg = &trans->msgs[msgNo];
        Message               *msg   = (*msgIt)->mMsg;

        msg->createBinary((unsigned char *)atMsg, version);
        atMsg->channel = (*msgIt)->mCh;

        msgNo++;
      }
      chainNo++;
    }

    pb += ph->len;
    print_block(ph);
  }

  return (int32_t)(pb - buffer);
}

int TransrefList::parseBinary(const char *buffer, const unsigned char version, MessagerefList &messages)
{
  unsigned int   chainNo;

  if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR_OLD) {
    ConfigFileAutoTransmit *trans = (ConfigFileAutoTransmit *)buffer;

    if (!trans->used) return 0;

    for (chainNo = 0; chainNo < MAX_NR_OF_AUTO_TRANSMIT_CHAINS; ++chainNo) {
      autoTransmitChainConfig *chain = &trans->chains[chainNo];

      if (!(chain->flags & AUTO_TRANSMIT_FLAG_CHAIN_IN_USE)) continue;

      Translist *tlist = new Translist("TransmitList" + intToString(chainNo));
      tlist->parseBinary((const char*)chain, version);

      for (int msgIdx = chain->first; msgIdx <= chain->last; ++msgIdx) {
        Message *msg = new Message("Message" + intToString(msgIdx));
        if (msg->parseBinary((const char*)&trans->msgs[msgIdx], version)) {

          // Check if "equal" message already exists
          Message *existingMsg = messages.find(msg);
          if (!existingMsg) {
            messages.add(msg);
          }
          else {
            delete msg;
            msg = existingMsg;
          }

          TransMsg *tMsg = new TransMsg();
          tMsg->parseBinary((const char*)&trans->msgs[msgIdx], version);
          tMsg->mName = msg->mName;
          tMsg->mMsg  = msg;
          tlist->mMessages.push_back(tMsg);
        }
        else {
          delete msg;
        }
      }
      add(tlist);
    }
  }

  else if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR) {
    ConfigFileAutoTransmitFd *trans = (ConfigFileAutoTransmitFd *)buffer;

    if (!trans->used) return 0;

    for (chainNo = 0; chainNo < MAX_NR_OF_AUTO_TRANSMIT_CHAINS; ++chainNo) {
      autoTransmitChainConfig *chain = &trans->chains[chainNo];

      if (!(chain->flags & AUTO_TRANSMIT_FLAG_CHAIN_IN_USE)) continue;

      Translist *tlist = new Translist("TransmitList" + intToString(chainNo));
      tlist->parseBinary((const char*)chain, version);

      for (int msgIdx = chain->first; msgIdx <= chain->last; ++msgIdx) {
        Message *msg = new Message("Message" + intToString(msgIdx));
        if (msg->parseBinary((const char*)&trans->msgs[msgIdx], version)) {

          // Check if "equal" message already exists
          Message *existingMsg = messages.find(msg);
          if (!existingMsg) {
            messages.add(msg);
          }
          else {
            delete msg;
            msg = existingMsg;
          }

          TransMsg *tMsg = new TransMsg();
          tMsg->parseBinary((const char*)&trans->msgs[msgIdx], version);
          tMsg->mName = msg->mName;
          tMsg->mMsg  = msg;
          tlist->mMessages.push_back(tMsg);
        }
        else {
          delete msg;
        }
      }
      add(tlist);
    }
  }

  else {
    PRINTF(("%s: UNSUPPORTED VERSION %d", __FUNCTION__, version));
    return 0;
  }

  PRINTF(("Read Transmitlist"));

  return 1;
}

int TransrefList::createXml(xmlTextWriterPtr writer) const
{
  XMLWriter xml(writer);

  xml.beginElement(XML_TRANSMIT_LIST_BLOCK);

  for(std::vector<Xmlref*>::const_iterator it = mList.begin(); it != mList.end(); ++it) {
    if ((*it)) {
      (*it)->createXml(writer);
    }
  }

  xml.endElement();

  return 0;
}

void TransrefList::parseXml(xmlNode *root_node, MessagerefList &messages)
{
  xmlNode  *tlist_node = NULL;
  xmlNode  *child_node = NULL;
  xmlNode  *msg_node = NULL;
  unsigned int chainNo = 0;
  unsigned int msgNo = 0;
  Translist* ptrans = NULL;

  if (!root_node) throw_nullpointer(__FUNCTION__);

  tlist_node = findFirstElementNode(root_node, XML_TRANSMIT_LIST_BLOCK);
  if (tlist_node == NULL) {
    return;
  }

  // Get the transmit list first
  for(child_node = tlist_node->children; child_node != NULL; child_node = child_node->next) {
    if ( isTransmitListElement(child_node) ) {
      std::string name;
      if ( !getAttributeString(child_node, XML_TRANSMIT_LIST_ATTR_NAME, name) ) {
        throw_attribute_not_found(child_node, XML_TRANSMIT_LIST_ATTR_NAME);
      }
      if ( !isUnique(name) ){
          throw_string_unique (child_node, XML_TRIGGERS_ATTR_NAME, name);
      }
      ptrans = new Translist(name);
      chainNo = add(ptrans);

      if (chainNo > MAX_NR_OF_AUTO_TRANSMIT_CHAINS - 1) throw_value_range(child_node, "Number of <TRANSMIT_LIST>", chainNo, 0, MAX_NR_OF_AUTO_TRANSMIT_CHAINS - 1);

      ptrans->mMsgDelay   = num_to_uint32(child_node, XML_TRANSMIT_LIST_ATTR_MSG_DELAY);
      ptrans->mCycleDelay = num_to_uint32_or_default(child_node, XML_TRANSMIT_LIST_ATTR_CYCLIC_DELAY, 0);
      ptrans->mCyclic     = yes_to_uint8_or_default(child_node, XML_TRANSMIT_LIST_ATTR_CYCLIC, 0) != 0;
      ptrans->mAutostart  = yes_to_uint8_or_default(child_node, XML_TRANSMIT_LIST_ATTR_AUTO_START, 0) != 0;

      for (msg_node = child_node->children; msg_node != NULL; msg_node = msg_node->next) {
        if (isTransmitMessageElement(msg_node) ) {
          std::string name;
          Message *msg;

          if ( !getAttributeString(msg_node, XML_TRANSMIT_MSG_NAME, name) ) {
            throw_attribute_not_found(msg_node, XML_TRANSMIT_MSG_NAME);
          }
          if (msgNo > MAX_NR_OF_AUTO_TRANSMIT_MSGS - 1) {
            throw_value_range(msg_node, "transmitMessage count", msgNo, 0, MAX_NR_OF_AUTO_TRANSMIT_MSGS - 1);
          }

          if (messages.getMessage(name, msg)) {
            TransMsg *tm = new TransMsg();
            ptrans->mMessages.push_back(tm);
            tm->mName = name;
            tm->mCh   = num_to_uint8(msg_node,XML_TRANSMIT_CHAN);
            tm->mMsg  = msg;
          }
          else {
            throw_general_failure(("Message '" + name + "' not found").c_str());
          }
        }
      }
    }
  }
}
