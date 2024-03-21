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
** Description: Convert XML strings to values
** -----------------------------------------------------------------------------
*/

#include <string>
#include <vector>
#include <libxml/tree.h>
#include "kvaMemoLibXMLConst.h"


uint8_t num_to_uint8 (xmlNode *a_node, const char *attr_name);

uint8_t num_to_uint8_or_default (xmlNode *a_node, const char *attr_name, uint8_t val);

uint16_t num_to_uint16 (xmlNode *a_node, const char *attr_name);

uint32_t num_to_uint32 (xmlNode *a_node, const char *attr_name);

uint32_t num_to_uint32_or_default (xmlNode *a_node, const char *attr_name, uint32_t val);

uint8_t yes_to_uint8 (xmlNode *a_node, const char *attr_name);

uint8_t yes_to_uint8_or_default (xmlNode *a_node, const char *attr_name, uint8_t val);

uint8_t protocol_to_uint8_or_default (xmlNode *a_node);

uint8_t protocol_msk_to_uint8_or_default (xmlNode *a_node);

std::string uint8_to_protocol_msk(uint8_t msk);

std::string uint8_to_protocol(uint8_t msk);

bool getAttributeString (xmlNode *a_node, const char *attr_name, char * buf, unsigned int len);

bool getAttributeString (xmlNode *a_node, const char *attr_name, std::string& str);

uint8_t node_name_to_uint8 (xmlNode *a_node, char const * const string_list[],
                                    const int* value_list, unsigned int len);

const char *uint8_to_name (uint8_t val, char const * const string_list[],
                           const int* value_list, unsigned int len);

uint8_t string_to_uint8 (xmlNode *a_node, const char *attr_name,
                         char const * const string_list[],
                         const int* value_list, unsigned int len);

bool postfixUsesTrigger (uint8_t *postFixExpr, const uint8_t triggerIdx);

bool postfixUsesAnyTrigger (uint8_t *postFixExpr, const uint8_t *triggerIdxList, int cnt);

int postfixToTriggerList (uint8_t *postFixExpr, uint8_t *buf, int len);

std::string getInfixExpr(uint8_t *postfixExpr);

uint16_t filterflags_to_uint16(xmlNode *a_node);

uint8_t datatype_to_uint8(xmlNode *a_node);

uint8_t pass_to_uint8 (xmlNode *a_node, const char *attr_name);

uint8_t filtertype_bin_to_xml(uint8_t bin_type);

size_t getElementString (xmlNode *a_node, char *buf, size_t len);

size_t getElementString(xmlNode *a_node, std::string & str);

long int element_string_to_long (xmlNode *element_node);


std::string datatype_to_string(uint8_t type);

std::string endian_to_string(uint8_t type);

void getBusPar(xmlNode *bus_node, uint32_t *bitrate, uint8_t *tseg1,
               uint8_t *tseg2, uint8_t *sjw);

void getBusParFd(xmlNode *bus_node, uint32_t *bitrate, uint16_t *tseg1,
                 uint16_t *tseg2, uint8_t *sjw,
                 uint32_t *bitrate_brs, uint16_t *tseg1_brs,
                 uint16_t *tseg2_brs, uint8_t *sjw_brs);

uint32_t bitrateToBusParams (xmlNode *bus_node, ConfigFileBusParams* bus);

uint32_t bitrateToBusParamsFd (xmlNode *bus_node, std::vector<std::string> *eanList, ConfigFileBusParamsFd* busfd);

void busParamsToBitrate (ConfigFileBusParams* bus, uint32_t *bitrate, uint8_t *tseg1, uint8_t *tseg2);
