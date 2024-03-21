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
** Description: Class to handle triggers in XML settings for
**              Kvaser Memorator Pro(2nd generation)
** -----------------------------------------------------------------------------
*/

#ifndef TRIGGER_H_
#define TRIGGER_H_

#include <string>
#include <vector>

#include "kvaMemoLibXMLConst.h"
#include "xmlref.h"
#include "busparams.h"
#include "translist.h"

class Trigger : public Xmlref {
  public:
    uint32_t mTimeout;

    Trigger(std::string);
    ~Trigger(){};

    void print();
    int createBinary(unsigned char *buffer, unsigned char version);
    int parseBinary(const char *buffer, const unsigned char version);
    int createXml(xmlTextWriterPtr writer) const;

    static Trigger* createTriggerFactory(std::string name, uint8_t type);
};

class MsgIdTrigger : public Trigger {
  public:
    uint8_t  mCh;
    uint32_t mMsgId;
    uint32_t mMsgIdMin;
    bool     mIsExtId;
    uint8_t  mHelpMask;

    void print();
    int createBinary(unsigned char *buffer, unsigned char version);
    int parseBinary(const char *buffer, const unsigned char version);
    int createXml(xmlTextWriterPtr writer) const;

    MsgIdTrigger(std::string name);
};

class DlcTrigger : public Trigger {
  public:
    uint8_t  mCh;
    uint8_t  mDlc;
    uint8_t  mDlcMin;

    void print();
    int createBinary(unsigned char *buffer, unsigned char version);
    int parseBinary(const char *buffer, const unsigned char version);
    int createXml(xmlTextWriterPtr writer) const;

    DlcTrigger(std::string name);
};

class ErrorFrameTrigger : public Trigger {
  public:
    uint8_t mCh;

    void print();
    int createBinary(unsigned char *buffer, unsigned char version);
    int parseBinary(const char *buffer, const unsigned char version);
    int createXml(xmlTextWriterPtr writer) const;

    ErrorFrameTrigger(std::string name);
};

class SigValTrigger : public Trigger {
  public:
    uint8_t  mCh;
    uint32_t mMsgId;
    bool     mIsExtId;
    bool     mIsCanFd;
    uint8_t  mDataType;
    uint8_t  mByteOrder;
    uint8_t  mDlc;
    bool     mIgnoreDlc;
    uint16_t mStartBit;
    uint16_t mLength;
    uint8_t  mHelpMask;
    uint8_t  mCondition;
    uint32_t mData;
    uint32_t mDataMin;

    void print();
    int createBinary(unsigned char *buffer, unsigned char version);
    int parseBinary(const char *buffer, const unsigned char version);
    int createXml(xmlTextWriterPtr writer) const;

    SigValTrigger(std::string name);
};

class ExtTrigger : public Trigger {
  public:
    uint8_t mCh;
    uint8_t mLevel;

    void print();
    int createBinary(unsigned char *buffer, unsigned char version);
    int parseBinary(const char *buffer, const unsigned char version);
    int createXml(xmlTextWriterPtr writer) const;

    ExtTrigger(std::string name);
};

class TimerTrigger : public Trigger {
  public:
    uint32_t mOffset;
    bool     mIsRepeat;

    void print();
    int createBinary(unsigned char *buffer, unsigned char version);
    int parseBinary(const char *buffer, const unsigned char version);
    int createXml(xmlTextWriterPtr writer) const;

    TimerTrigger(std::string name);
};

// ----------------------------------------------------------------------------------
class Triggers {

 private:
  uint32_t getPreTriggerSectors(BusrefList &busparams) const;
  void stringToList(const std::string& trigger_expr, std::vector<std::string>& list, const char splitter) const;
  void getTriggersFromInfix(std::string& infixExpr, std::vector<std::string>& list);
  void parseExpression(std::string& triggerExpr, bool convert = true);
  void xmlstringToPostfix(xmlNode *a_node, const char * expr, uint8_t *postFixExpr);
  void postfixToBin(xmlNode *a_node, const char *postfixExpr, uint8_t *postfixBin);
  void walkExpression(Token *exprToken, std::string &postfixExpr, int iter);


 public:
  XmlrefList    mTriggers;
  XmlrefList    mStatements;

  bool          mLogAll;
  bool          mFifoMode;
  bool          mTrigScripted;
  uint32_t      mPreTriggerSectors;

  int getTrigger(xmlNode *child_node, Trigger **pTrigger);
  void parseTriggerBlock(xmlNode *trigger_block_node);
  void parseStatement(xmlNode * a_node, TransrefList &transLists);
  void getTriggerStatus(std::string& trigger_expr, std::vector<std::string>& undefined);
  void getTriggers(std::string& triggerExpr, std::vector<std::string>& list);
  void getTriggers(std::vector<std::string>& list);

  int createBinary(unsigned char* buffer, unsigned char version);
  int parseBinary(const char *buffer, const unsigned char version);
  int createXml(xmlTextWriterPtr writer) const;
  void parseXml(xmlNode *root_node, BusrefList &busparams, TransrefList &transLists);

  Triggers();
  ~Triggers();
};

#endif
