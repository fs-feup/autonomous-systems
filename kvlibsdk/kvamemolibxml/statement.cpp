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
** Description: Class to handle statements in XML settings for
**              Kvaser Memorator Pro(2nd generation)
** -----------------------------------------------------------------------------
*/

#include <cstdio>
#include <cstring>
#include <string>

#include "statement.h"
#include "action.h"
#include "errorhandler.h"
#include "convert.h"
#include "util.h"
#include "xmlwriter.h"

Statement::Statement(std::string name) : Xmlref(name)
{
  mPreTrigger   = 0;
  mPostTrigger  = 0;
  mInFixExpr    = "";
  memset(mPostFixExpr, 0, MAX_EXPR_LEN);
}

Statement::~Statement()
{
  mActions.clear();
}

void Statement::print()
{
  PRINTF(("%d %s [", mIdx, mName.c_str()));
  PRINTF(("  Pretrigger   = %u", mPreTrigger));
  PRINTF(("  Posttrigger  = %u", mPostTrigger));
  PRINTF(("  InfFixExpr   = %s", mInFixExpr.c_str()));
  PRINTF(("  Actions  [\n"));
  for (std::vector<Xmlref*>::iterator it = mActions.mList.begin(); it != mActions.mList.end(); ++it) {
    (*it)->print();
  }
  PRINTF(("  ]\n"));
  PRINTF(("]\n"));
}

int Statement::createBinary(unsigned char *buffer, unsigned char /* version */)
{
  uint8_t actionNo = 0;
  TriggerStatement *ts = (TriggerStatement*)buffer;

  ts->preTrigger  = mPreTrigger;
  ts->postTrigger = mPostTrigger;
  memcpy(ts->postFixExpr, mPostFixExpr, MAX_EXPR_LEN);

  for (std::vector<Xmlref*>::iterator it = mActions.mList.begin(); it != mActions.mList.end(); ++it) {
    ts->actionList[actionNo] = (*it)->mType;
    ts->call[actionNo].actionParam = ((Action*)(*it))->actionParam();
    actionNo++;
  }

  ts->noOfActions = actionNo;

  return 0;
}

int Statement::parseBinary(const char *buffer, const unsigned char /* version */)
{
  if (!buffer) throw_nullpointer(__FUNCTION__);

  TriggerStatement *ts = (TriggerStatement*)buffer;

  mPreTrigger  = ts->preTrigger;
  mPostTrigger = ts->postTrigger;
#ifdef __GNUC__
// Suppress faulty warnings caused by GCC 13 bug
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
# if __GNUC__ >= 13
#pragma GCC diagnostic ignored "-Wstringop-overread"
# endif
#endif
  memcpy(mPostFixExpr, ts->postFixExpr, MAX_EXPR_LEN);
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
  mInFixExpr   = getInfixExpr(ts->postFixExpr);

  for (uint8_t actionNo = 0; actionNo < ts->noOfActions; ++actionNo) {

    Action  *action;
    uint8_t  actionType  = ts->actionList[actionNo];
    uint32_t actionParam = ts->call[actionNo].actionParam;

    action = NULL;

    switch (actionType) {

    case ACTION_START_LOG:
    case ACTION_STOP_LOG:
    case ACTION_STOP_LOG_COMPLETELY: {
      action = new Action(std::string("LogAction"));
      action->mType = actionType;
      break;
    }

    case ACTION_EXTERNAL_PULSE: {
      PulseAction *pulseAction = new PulseAction(std::string("PulseAction"));
      pulseAction->mType = actionType;
      pulseAction->mDuration = actionParam;
      action = pulseAction;
      break;
    }

    case ACTION_ACTIVATE_AUTO_TRANSMIT_CHAIN:
    case ACTION_DEACTIVATE_AUTO_TRANSMIT_CHAIN: {
      std::string name;
      TransmitAction *transAction = new TransmitAction(std::string("TlistAction"));
      transAction->mType = actionType;
      transAction->mTlistIndex = actionParam;
      transAction->mTlistName = "TransmitList" + intToString(actionParam);
      action = transAction;
      break;
    }

    default:
      PRINTF(("UNKNOWN ACTION TYPE %d", ts->actionList[actionNo]));
    }

    if (action) {
      mActions.add(action);
    }
  }

  PRINTF(("Read Statement %s\n", mName.c_str()));

  return 1;
}

int Statement::createXml(xmlTextWriterPtr writer) const
{
  XMLWriter xml(writer);

  xml.beginElement(XML_STATEMENT);
  xml.writeAttr(XML_STATEMENT_ATTR_PRE,  "%u", mPreTrigger);
  xml.writeAttr(XML_STATEMENT_ATTR_POST, "%u", mPostTrigger);

  xml.beginElement(XML_EXPRESSION);
  xml.writeElementString("%s", mInFixExpr.c_str());
  xml.endElement();

  xml.beginElement(XML_ACTIONS);
  mActions.createXml(writer);
  xml.endElement();

  xml.endElement(); // Statement

  return 0;
}
