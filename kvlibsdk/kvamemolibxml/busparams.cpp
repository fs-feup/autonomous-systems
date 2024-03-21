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
** Description: Class to handle bus parameters for a CAN channel in XML
**              settings for Kvaser Memorator Pro(2nd generation)
** -----------------------------------------------------------------------------
*/

#define NOMINMAX
#include <algorithm>
#include <cstdio>
#include <string>
#include <cstring>

#include <busparams.h>
#include <libxml/xmlwriter.h>

#include "errorhandler.h"
#include "xmlwriter.h"
#include "convert.h"
#include "util.h"
#include "kvdebug.h"

static uint8_t checkChannels (xmlNode *bus_node, bool *channelInUse)
{
  uint8_t ch = 0;
  ch = num_to_uint8(bus_node, XML_BUSPARAM_ATTR_CHAN);
  if (ch > MAX_NR_OF_CAN_CHANNELS - 1) {
    throw_value_range(bus_node, XML_BUSPARAM_ATTR_CHAN, ch, 0, MAX_NR_OF_CAN_CHANNELS-1);
  }
  if (channelInUse[ch]) {
    throw_value_unique(bus_node, XML_BUSPARAM_ATTR_CHAN, ch);
  }
  channelInUse[ch] = true;
  return ch;
}

// ---------------------------------------------------------------------------

Busparams::Busparams()
{
  mCh          = 0;
  mTseg1       = 0;
  mTseg2       = 0;
  mSjw         = 0;
  mBitrate     = 0;
  mPresc       = 0;
  mTseg1Brs    = 0;
  mTseg2Brs    = 0;
  mSjwBrs      = 0;
  mBitrateBrs  = 0;
  mIsCanFd     = false;
  mIsSilent    = false;
  mIsIso       = false;
}

void Busparams::print()
{
  PRINTF(("  Channel %u (CAN %s) [", mCh, mIsCanFd ? "FD" : "Classic"));
  PRINTF(("    Silent  %s", mIsSilent ? "Yes" : "No"));
  PRINTF(("    Bitrate %u", mBitrate));
  PRINTF(("    Tseg1   %u", mTseg1));
  PRINTF(("    Tseg2   %u", mTseg2));
  PRINTF(("    Sjw     %u", mSjw));
  PRINTF(("    Presc   %u", mPresc));
  if (mIsCanFd) {
    PRINTF(("    Bitrate brs %u", mBitrateBrs));
    PRINTF(("    Tseg1 brs   %u", mTseg1Brs));
    PRINTF(("    Tseg2 brs   %u", mTseg2Brs));
    PRINTF(("    Sjw brs     %u", mSjwBrs));
    PRINTF(("    Iso         %s", mIsIso ? "Yes" : "No"));
  }
  PRINTF(("  ]"));
}

int Busparams::createBinary(unsigned char *buffer, unsigned char version)
{
  unsigned char *pb;
  BlockHead     *ph;

  pb = buffer;
  ph = (BlockHead*) pb;

  if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR_OLD) {
    ConfigFileBusParams *bus = NULL;

    ph->id  = BLOCK_ID_BUSPARAMS;
    ph->len = sizeof(BlockHead) + sizeof(ConfigFileBusParams);

    bus = (ConfigFileBusParams*) (pb + sizeof(BlockHead));
    memset(bus, 0, sizeof (ConfigFileBusParams));

    bus->used     = 1;
    bus->channel  = mCh;
    bus->psc      = mPresc;
    bus->prSeg    = 1;
    bus->phSeg1   = mTseg1 - bus->prSeg;
    bus->phSeg2   = (uint8_t)mTseg2;
    bus->sjw      = mSjw;
    bus->samples  = 1;
    bus->silent   = mIsSilent;

    pb += ph->len;
    print_block(ph);
  }
  else if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR) {
    ConfigFileBusParamsFd *bus = NULL;

    ph->id  = BLOCK_ID_BUSPARAMS_FD;
    ph->len = sizeof(BlockHead) + sizeof(ConfigFileBusParamsFd);

    bus = (ConfigFileBusParamsFd*) (pb + sizeof(BlockHead));
    memset(bus, 0, sizeof (ConfigFileBusParamsFd));

    bus->used        = 1;
    bus->channel     = mCh;
    bus->bitrate     = mBitrate;
    bus->bitrate_brs = mBitrateBrs;
    bus->tseg1       = mTseg1;
    bus->tseg2       = mTseg2;
    bus->tseg1_brs   = mTseg1Brs;
    bus->tseg2_brs   = mTseg2Brs;
    bus->sjw         = mSjw;
    bus->sjw_brs     = mSjwBrs;
    bus->silent      = mIsSilent;

    if (mIsCanFd) {
      bus->can_mode = mIsIso ? OPEN_AS_CANFD_ISO : OPEN_AS_CANFD_NONISO;
    }
    else {
      bus->can_mode = OPEN_AS_CAN;
    }

    pb += ph->len;
    print_block(ph);
  }

  return (int32_t)(pb - buffer);
}

int Busparams::parseBinary(const char *buffer, const unsigned char version)
{

  if (!buffer) throw_nullpointer(__FUNCTION__);

  if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR_OLD) {
    ConfigFileBusParams *bp = (ConfigFileBusParams *) buffer;
    uint8_t tseg1_8bit;
    uint8_t tseg2_8bit;

    if (!bp->used) return 0;

    busParamsToBitrate(bp, &mBitrate, &tseg1_8bit, &tseg2_8bit);

    mCh       = bp->channel;
    mIsSilent = bp->silent != 0;
    mSjw      = bp->sjw;
    mTseg1    = tseg1_8bit;
    mTseg2    = tseg2_8bit;

    PRINTF(("Read Busparams classic for channel %d\n", mCh));
  }
  else if (version == KVASER_BINARY_VERSION_ELEMENT_MAJOR) {
    ConfigFileBusParamsFd *bpfd = (ConfigFileBusParamsFd *) buffer;

    if (!bpfd->used) return 0;

    mCh       = bpfd->channel;
    mIsSilent = bpfd->silent != 0;
    mBitrate  = bpfd->bitrate;
    mTseg1    = bpfd->tseg1;
    mTseg2    = bpfd->tseg2;
    mSjw      = bpfd->sjw;

    if (bpfd->can_mode != OPEN_AS_CAN) {
      mIsCanFd     = true;
      mBitrateBrs  = bpfd->bitrate_brs;
      mTseg1Brs    = bpfd->tseg1_brs;
      mTseg2Brs    = bpfd->tseg2_brs;
      mSjwBrs      = bpfd->sjw_brs;
      mIsIso       = bpfd->can_mode == OPEN_AS_CANFD_ISO;
    }

    PRINTF(("Read Busparams FD for channel %d\n", mCh));
  }
  else {
    PRINTF(("%s: UNSUPPORTED VERSION %d", __FUNCTION__, version));
    return 0;
  }

  return 1;
}

int Busparams::createXml(xmlTextWriterPtr writer) const
{
  XMLWriter xml(writer);

  xml.beginElement(XML_BUSPARAM);

  xml.writeAttr(XML_BUSPARAM_ATTR_CHAN,    "%d", mCh);
  xml.writeAttr(XML_BUSPARAM_ATTR_BITRATE, "%d", mBitrate);
  xml.writeAttr(XML_BUSPARAM_ATTR_TSEG1,   "%d", mTseg1);
  xml.writeAttr(XML_BUSPARAM_ATTR_TSEG2,   "%d", mTseg2);
  xml.writeAttr(XML_BUSPARAM_ATTR_SJW,     "%d", mSjw);
  xml.writeAttrYesNo(XML_BUSPARAM_ATTR_SILENT, mIsSilent);

  if (mIsCanFd) {
    xml.writeAttr(XML_BUSPARAM_ATTR_BITRATE_BRS, "%d", mBitrateBrs);
    xml.writeAttr(XML_BUSPARAM_ATTR_TSEG1_BRS,   "%d", mTseg1Brs);
    xml.writeAttr(XML_BUSPARAM_ATTR_TSEG2_BRS,   "%d", mTseg2Brs);
    xml.writeAttr(XML_BUSPARAM_ATTR_SJW_BRS,     "%d", mSjwBrs);
    xml.writeAttrYesNo(XML_BUSPARAM_ATTR_ISO_BRS, mIsIso);
  }

  xml.endElement();

  return 0;
}

uint64_t BusrefList::getTotalBitrate() const
{
  uint64_t totBitrate = 0;

  for(std::vector<Xmlref*>::const_iterator it = mList.begin(); it != mList.end(); ++it) {
    totBitrate += std::max(((Busparams*)(*it))->mBitrate, ((Busparams*)(*it))->mBitrateBrs);
  }

  return totBitrate;
}


int BusrefList::parseBinary(const char *buffer, const unsigned char version)
{
  Busparams* params = new Busparams();

  if (params->parseBinary(buffer, version) != 0) {
    add(params);
  }
  else {
    delete params;
  }

  return 1;
}


int BusrefList::createXml(xmlTextWriterPtr writer) const
{
  XMLWriter xml(writer);

  xml.beginElement(XML_BUSPARAM_BLOCK);

  for(std::vector<Xmlref*>::const_iterator it = mList.begin(); it != mList.end(); ++it) {
    if ((*it)) {
      (*it)->createXml(writer);
    }
  }

  xml.endElement();

  return 0;
}

void BusrefList::parseXml(xmlNode *root_node)
{
  xmlNode *cur_node = NULL;
  xmlNode *child_node = NULL;
  uint8_t ch = 0;
  bool channelInUse[MAX_NR_OF_CAN_CHANNELS] = {false};

  if (!root_node) throw_nullpointer(__FUNCTION__);

  cur_node = findFirstElementNode(root_node, XML_BUSPARAM_BLOCK);
  if (!cur_node) throw_element_not_found(root_node, XML_BUSPARAM_BLOCK);

  for (child_node = cur_node->children; child_node != NULL; child_node = child_node->next)
  {
    if ( isBusParameterElement(child_node) ) {
      Busparams *pBpars;

      ch = checkChannels(child_node, channelInUse);
      pBpars = new Busparams();
      pBpars->mCh = ch;
      pBpars->mIsSilent = yes_to_uint8_or_default(child_node, XML_BUSPARAM_ATTR_SILENT, 1) != 0;

      if ( !isCanFdBusParameters(child_node) ) {
        uint32_t psc;

        getBusPar(child_node, &pBpars->mBitrate, (uint8_t*)&pBpars->mTseg1,
                  (uint8_t*)&pBpars->mTseg2, &pBpars->mSjw);

        psc = CAN_SYS_FREQ / (PSC_FACTOR * pBpars->mBitrate * (1 + pBpars->mTseg1 + pBpars->mTseg2));
        if (psc < PSC_MIN || psc > PSC_MAX) {
          set_error_and_throw ( KvaXmlStatusERR_INTERNAL,
                                "The calculated CAN pre-scaler value is invalid: %lu. "
                                "Please check bitrate settings in '%s'.", psc, XML_BUSPARAM);
        }
        pBpars->mPresc = (uint8_t)psc;
      }
      else {
        getBusParFd(child_node, &pBpars->mBitrate, &pBpars->mTseg1, &pBpars->mTseg2, &pBpars->mSjw,
                    &pBpars->mBitrateBrs, &pBpars->mTseg1Brs, &pBpars->mTseg2Brs, &pBpars->mSjwBrs);
        pBpars->mIsCanFd = true;
        pBpars->mIsIso   = yes_to_uint8_or_default(child_node, XML_BUSPARAM_ATTR_ISO_BRS, 0) != 0;
      }

      add(pBpars);
    }
  }
}
