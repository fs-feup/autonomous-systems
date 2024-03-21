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
** Description: Class to handle data structure representing the XML for
**              Kvaser Memorator Pro(2nd generation)
** -----------------------------------------------------------------------------
*/

#include <string>
#include <cstring>
#include <vector>
#include <sstream>
#include <algorithm>

#include <libxml/xmlwriter.h>

#include "xmlstruct.h"
#include "kvdebug.h"
#include "errorhandler.h"
#include "filter.h"
#include "util.h"
#include "convert.h"
#include "crc32.h"
#include "statement.h"
#include "busparams.h"
#include "xmlwriter.h"

using namespace std;

// ===========================================================================
// XmlStruct
// ===========================================================================
XmlStruct::XmlStruct() {
  mAfterburner = 0;
  mXmlMajor    = KVASER_XML_VERSION_ELEMENT_MAJOR;
  mXmlMinor    = KVASER_XML_VERSION_ELEMENT_MINOR;
}

// ---------------------------------------------------------------------------
XmlStruct::~XmlStruct() {
  mBusParams.clear();
  mMessages.clear();
  mTransmitlists.clear();
  mFilters.clear();
  mEanList.clear();
};

// ---------------------------------------------------------------------------
void XmlStruct::print()
{
  PRINTF(("Printing data from XML:"));

  PRINTF(("XML version: %u.%u", mXmlMajor, mXmlMinor));
  PRINTF(("Binary version: %u.%u", mBinMajor, mBinMinor));

  PRINTF(("Target EANs ["));
  for(std::vector<std::string>::iterator it = mEanList.begin(); it != mEanList.end(); ++it) {
    PRINTF(("  %s", (*it).c_str()));
  }
  PRINTF(("]"));

  PRINTF(("Logger mode: Log all=%s, fifo mode=%s, scripts=%s, afterburner=%u",
          mTriggerStatements.mLogAll ? "yes" : "no",
          mTriggerStatements.mFifoMode ? "yes" : "no",
          mTriggerStatements.mTrigScripted ? "yes" : "no",
          mAfterburner));

  PRINTF(("Bus parameters:"));
  mBusParams.print();

  if (mMessages.mList.size() != 0) {
    PRINTF(("Messages:"));
    mMessages.print();
  }

  if (mTransmitlists.mList.size() != 0) {
    PRINTF(("TransmitLists:"));
    mTransmitlists.print();
  }

  if (mTriggerStatements.mTriggers.mList.size() != 0) {
    PRINTF(("Triggers:"));
    mTriggerStatements.mTriggers.print();
  }

  if (mTriggerStatements.mStatements.mList.size() != 0) {
    PRINTF(("Statements:"));
    mTriggerStatements.mStatements.print();
  }

  if (mFilters.mList.size() != 0) {
    PRINTF(("Filters:"));
    mFilters.print();
  }

  if (mScripts.mList.size() != 0) {
    PRINTF(("Scripts:"));
    mScripts.print();
  }
}

// ---------------------------------------------------------------------------
int XmlStruct::parseXml(xmlNode *root_element)
{
  parseXmlVersion(root_element, &mXmlMajor, &mXmlMinor);
  if (!isSupportedXml(mXmlMajor, mXmlMinor)) {
    set_error_and_throw(KvaXmlStatusFail, "Error: XML contains an unsupported '%s' "
                        "element %d.%d. Supported versions: %s.\n",
                        XML_KVASER_VERSION, mXmlMajor, mXmlMinor, KVASER_XML_VERSIONS_STRING);
  }

  parseXmlBinaryVersion(root_element, &mBinMajor, &mBinMinor);
  if (!isSupportedBinary(mBinMajor, mBinMinor)) {
    set_error_and_throw(KvaXmlStatusFail, "Error: XML contains an unsupported '%s' "
                        "element %d.%d. Supported versions: %s.\n",
                        XML_KVASER_BIN_VERSION, mBinMajor, mBinMinor, KVASER_BINARY_VERSIONS_STRING);
  }

  parseXmlTargetEan(root_element, mEanList);
  parseXmlAfterburner(root_element);
  mBusParams.parseXml(root_element);
  mMessages.parseXml(root_element);
  mTransmitlists.parseXml(root_element, mMessages);
  mTriggerStatements.parseXml(root_element, mBusParams, mTransmitlists);
  mFilters.parseXml(root_element);
  mScripts.parseXml(root_element);

  return 0;
}

// ---------------------------------------------------------------------------
int XmlStruct::createBinary(unsigned char *buffer)
{
  unsigned char  *pb;
  BlockHead      *ph;
  uint32_t       calcCrc = 0;

  pb = buffer;

  // Version
  {
    ConfigFileVersion *ver = NULL;

    ph = (BlockHead*) pb;
    ph->id  = BLOCK_ID_VERSION;
    ph->len = sizeof(BlockHead) + sizeof(ConfigFileVersion);

    ver  = (ConfigFileVersion*) (pb + sizeof(BlockHead));
    memset(ver, 0, sizeof(ConfigFileVersion));
    ver->version = (mBinMajor << 8) + mBinMinor;
    ver->magic = CONFIG_MAGIC;

    pb += ph->len;
    print_block(ph);
  }

  // Bus parameters
  pb += mBusParams.createBinary(pb, mBinMajor);

  // Transmit lsits
  pb += mTransmitlists.createBinary(pb, mBinMajor);

  // Triggers and Statements
  pb += mTriggerStatements.createBinary(pb, mBinMajor);

  // Filters
  pb += mFilters.createBinary(pb, mBinMajor);

  // IDENT (empty)
  {
    ConfigFileIdentInfo *id = NULL;

    ph = (BlockHead*) pb;
    ph->id  = BLOCK_ID_IDENT;
    ph->len = sizeof(BlockHead) + sizeof(ConfigFileIdentInfo);
    id  = (ConfigFileIdentInfo*) (pb + sizeof(BlockHead));
    memset(id, 0, sizeof(ConfigFileIdentInfo));
    pb += ph->len;
    print_block(ph);
  }

  // Scripts
  pb += mScripts.createBinary(pb, mBinMajor);

  // Afterburner
  {
    ConfigFileAfterburner* afb;

    ph = (BlockHead*) pb;
    ph->id  = BLOCK_ID_AFTERBURNER;
    ph->len = sizeof(BlockHead) + sizeof(ConfigFileAfterburner);

    afb  = (ConfigFileAfterburner*) (pb + sizeof(BlockHead));
    memset(afb, 0, sizeof(ConfigFileAfterburner));
    afb->canPowerTimeout = mAfterburner;

    pb += ph->len;
    print_block(ph);
  }

  // EOF and CRC
  ph = (BlockHead*) pb;
  ph->id = BLOCK_ID_END;
  pb += sizeof(ph->id);

  crc32_init(&calcCrc);
  crc32_generate(&calcCrc, buffer, (uint32_t)(pb-buffer));
  crc32_finalize(&calcCrc);
  *((uint32_t*)pb) = calcCrc;
  pb += sizeof(calcCrc);
  print_block(ph);

  return (int32_t)(pb - buffer);
}

// ---------------------------------------------------------------------------
int XmlStruct::parseBinary(const char *buf, const unsigned int len, const char *scriptpath)
{
  uint32_t   foundCrc;
  uint32_t   expectedCrc = 0;
  uint32_t   bufptr = 0;
  BlockHead *bp;
  uint32_t   blockId;
  uint32_t   blockLen;
  uint32_t   filterNo = 0;
  uint32_t   scriptNo = 0;

  crc32_init(&foundCrc);

  try {

    while (bufptr < len) {

      if ((len - bufptr) < sizeof(BlockHead))  {
        break;
      }

      bp = (BlockHead*)&buf[bufptr];
      blockId = bp->id;
      blockLen = bp->len;

      if (blockId == BLOCK_ID_END) {
        // crc of BLOCK_ID_END
        crc32_generate(&foundCrc, &blockId, sizeof(blockId));
        crc32_finalize(&foundCrc);
        expectedCrc = blockLen;

        if (expectedCrc != foundCrc) {
          // WRONG CRC
          throw_xml_writer_failure ("Wrong CRC in buffer file");
        }
        break;
      }

      // crc of header
      crc32_generate(&foundCrc, (void *)&buf[bufptr], sizeof(BlockHead));

      if ((len - bufptr) < (blockLen - sizeof(BlockHead)))  {
        char tmp_err[256];
        sprintf(tmp_err, "Inconsistent BlockHead length, blockId %d inlen %d inbuf_ptr %d, blocklen %d blockhead size(%u)",
                blockId, len, bufptr, blockLen, (unsigned int)sizeof(BlockHead));
        throw_xml_writer_failure (tmp_err);
      }

      bufptr += sizeof(BlockHead);

      // crc of rest of block
      crc32_generate(&foundCrc, (void *)&buf[bufptr], blockLen - sizeof(BlockHead));

      switch (blockId) {

      case BLOCK_ID_AFTERBURNER: {
        ConfigFileAfterburner *ab = (ConfigFileAfterburner *)&buf[bufptr];
        mAfterburner = ab->canPowerTimeout;
        break;
      }

      case BLOCK_ID_VERSION: {
        ConfigFileVersion *ver = (ConfigFileVersion *)&buf[bufptr];
        mBinMajor = ver->version >> 8;
        mBinMinor = ver->version & 0xFF;
        if (!isSupportedBinary(mBinMajor, mBinMinor)) {
          set_error_and_throw(KvaXmlStatusFail, "Error: XML contains an unsupported '%s' "
                              "element %d.%d. Supported versions: %s.\n",
                              XML_KVASER_BIN_VERSION, mBinMajor, mBinMinor, KVASER_BINARY_VERSIONS_STRING);
        }
        break;
      }

      case BLOCK_ID_BUSPARAMS:
      case BLOCK_ID_BUSPARAMS_FD: {
        mBusParams.parseBinary(&buf[bufptr], mBinMajor);
        break;
      }

      case BLOCK_ID_TRIGGER:
      case BLOCK_ID_TRIGGER_FD: {
        mTriggerStatements.parseBinary(&buf[bufptr], mBinMajor);
        break;
      }

      case BLOCK_ID_AUTO_TRANSMIT:
      case BLOCK_ID_AUTO_TRANSMIT_FD: {
        mTransmitlists.parseBinary(&buf[bufptr], mBinMajor, mMessages);
        break;
      }

      case BLOCK_ID_FILTER:
      case BLOCK_ID_FILTER_FD: {
        mFilters.parseBinary(&buf[bufptr], mBinMajor, filterNo);
        break;
      }

      case BLOCK_ID_SCRIPT: {
      case BLOCK_ID_SCRIPT_SD:
        mScripts.parseBinary(&buf[bufptr], blockId == BLOCK_ID_SCRIPT_SD, scriptpath, scriptNo);
        break;
      }

      default:
        PRINTF(("UNHANDLED BLOCK ID %d", blockId));
        break;
      }

      bufptr += (blockLen - sizeof(BlockHead));

    } // while

  }

  catch (KvaXmlStatus err)
  {
    set_error_status(err);
    return get_error_status();
  }

  return 0;
}

// ---------------------------------------------------------------------------
int XmlStruct::createXml(xmlTextWriterPtr writer) const
{
  XMLWriter xml(writer);

  xml.beginElement(XML_KVASER_ROOT_NODE);

  // Version block
  xml.beginElement(XML_KVASER_VERSION);
  xml.writeElementString("%d.%d", mXmlMajor, mXmlMinor);
  xml.endElement();

  // Binary version
  xml.beginElement(XML_KVASER_BIN_VERSION);
  xml.writeElementString("%d.%d", mBinMajor, mBinMinor);
  xml.endElement();

  // Settings block
  xml.beginElement(XML_SETTINGS);
  xml.beginElement(XML_MODE);
  xml.writeAttrYesNo(XML_MODE_ATTR_LOGALL, mTriggerStatements.mLogAll);
  xml.writeAttrYesNo(XML_MODE_ATTR_FIFO, mTriggerStatements.mFifoMode);
  xml.endElement(); // Mode
  xml.beginElement(XML_AFTERBURNER);
  xml.writeAttr(XML_SETTINGS_ATTR_TIMEOUT, "%d", mAfterburner);
  xml.endElement(); // Power
  xml.endElement(); // Settings

  mBusParams.createXml(writer);
  mTriggerStatements.createXml(writer);
  mMessages.createXml(writer);
  mTransmitlists.createXml(writer);
  mFilters.createXml(writer);
  mScripts.createXml(writer);

  xml.endElement(); // Kvaser

  return 0;
}

// ---------------------------------------------------------------------------
void XmlStruct::parseXmlAfterburner(xmlNode *root_node) {

  xmlNode  *child_node = NULL;
  xmlNode  *cur_node = NULL;

  if (!root_node) throw_nullpointer(__FUNCTION__);

  cur_node = findFirstElementNode(root_node, XML_SETTINGS);
  if (!cur_node) {
    return;
  }

  for (child_node = cur_node->children; child_node != NULL; child_node = child_node->next)
  {
    if ( isAfterburnerElement(child_node) )
    {
      print_prop(child_node);
      mAfterburner = num_to_uint32(child_node, XML_SETTINGS_ATTR_TIMEOUT);
      if (mAfterburner > KVASER_MAX_AFTERBURNER_TIME)
      {
        throw_value_range(child_node,
                          XML_SETTINGS_ATTR_TIMEOUT,
                          mAfterburner,
                          0,
                          KVASER_MAX_AFTERBURNER_TIME);
      }
    }
  }
}

// ---------------------------------------------------------------------------
bool XmlStruct::parseXmlVersion (xmlNode *root_node, unsigned int *major, unsigned int *minor)
{
  return getVersion(root_node, XML_KVASER_VERSION, major, minor);
}

// ---------------------------------------------------------------------------
bool XmlStruct::parseXmlBinaryVersion (xmlNode *root_node, unsigned int *major, unsigned int *minor)
{
  return getVersion(root_node, XML_KVASER_BIN_VERSION, major, minor);
}

// ---------------------------------------------------------------------------
void XmlStruct::parseXmlTargetEan(xmlNode *root_node, std::vector<std::string>& ean_list)
{
  xmlNode *settings_node = NULL;
  xmlNode *child_node = NULL;
  std::string ean;

  if (!root_node) throw_nullpointer(__FUNCTION__);

  settings_node = findFirstElementNode(root_node, XML_SETTINGS);

  // SETTINGS is mandatory
  if (!settings_node) throw_element_not_found(root_node, XML_SETTINGS);

  for (child_node = settings_node->children; child_node != NULL; child_node = child_node->next)
  {
    if ( isEanElement(child_node) ) {
      size_t len = getElementString(child_node, ean);
      if (len) ean_list.push_back(ean);
      ean.clear();
    }
  }
}
