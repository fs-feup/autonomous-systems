/*
**             Copyright 2018 by Kvaser AB, Molndal, Sweden
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
  Description: This template is designed to "hide" handles in kvaDbLib.cpp
*/

#include <set>

template <typename item, typename handle>

  class handle_class {

  std::set<item> myset;

  typedef typename std::set<item>::iterator my_iterator;

 public:

  //add an item to the set
  KvaDbStatus add (item data, handle *hnd) {
    if (data == 0) {
      return kvaDbErr_Param;
    }

    *hnd = (handle)data;
    myset.insert (data);
    return kvaDbOK;
  }

  //convert hnd to an item
  item convert (handle hnd) {
    return (item)hnd;
  }

  //remove hnd from the set
  KvaDbStatus remove (handle hnd) {
    my_iterator it;

    it = myset.find((item)hnd);
    if (it != myset.end ()) {
      myset.erase (it);
    }
    return kvaDbOK;
  }

  //if data existst in the set, return the matching handle
  KvaDbStatus find (item data, handle* hnd) {
    my_iterator it;

    if (data == 0) {
      return kvaDbErr_Param;
    }

    it = myset.find(data);

    if (it == myset.end ()) {
      return kvaDbErr_NoNode;
    } else {
      *hnd = (handle)data;
      return kvaDbOK;
    }
  }

  //returns 1 if hnd is valid
  int is_valid (handle hnd) {
    my_iterator it;

    if (hnd == 0) {
      return 0;
    }

    it = myset.find((item)hnd);

    if (it == myset.end ()) {
      return 0;
    } else {
      return 1;
    }
  }

  //if data exists in the set, return the matching handle,
  //else add data to the set and return a new handle
  KvaDbStatus find_or_add (item data, handle *hnd) {
    KvaDbStatus status = find (data, hnd);

    if (status == kvaDbErr_NoNode) {
      status = add (data, hnd);
    }

    return status;
  }
};
