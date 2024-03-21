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

#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <malloc.h>
#include <string>
#include "bque_var_size.h"
#include "kvdebug.h"
#include "os_util.h"
#include "kvassert.h"

bque_var_size::bque_var_size (unsigned int n_element, unsigned int max_element_size, char *name)
{
  storage_size = n_element * (max_element_size + 4);
  storage      = (char*)malloc(storage_size);

  if (storage == NULL) {
    PRINTF(("Couldn't allocate bque_var_size"));
    ASSERT(false);
  }

  read_only        = false;
  tmp_file         = NULL;
  filename         = NULL;
  push_index       = 0;
  pop_index        = 0;
  n_bytes_read     = 0;
  n_pushed_element = 0;
  strncpy(file_prefix, name, sizeof(file_prefix) - 1);
  file_prefix[sizeof(file_prefix)-1] = '\0';
}

bque_var_size::~bque_var_size ()
{
  close ();
  free (storage);
}

bool bque_var_size::open (char *prefix)
{
  tmp_file = os_open_tmp_file(prefix, &filename);

  if (tmp_file == NULL) {
    PRINTF(("Failed to open file!"));
    ASSERT(false);
    return false;
  }

  return true;
}

void bque_var_size::close () {
  if (tmp_file && (fclose(tmp_file) == EOF)) {
    PRINTF(("bque_var_size failed to close temp file"));
    ASSERT(false);
  }

  if (filename) {
    if (remove(filename)) {
      PRINTF(("Failed to remove temp file '%s'",filename));
      ASSERT(false);
    }
    PRINTF(("Free %s", filename));
    free(filename);
  }

  tmp_file = NULL;
  filename = NULL;
}

bool bque_var_size::write_block ()
{
  size_t r;
  fpos_t position;

  if (tmp_file == NULL) {
    if (!open (file_prefix)) {
      PRINTF(("Failed to open file"));
      ASSERT(false);
      return false;
    }
  }

  // Ensure we can write (the file is opened with a+)
  if (os_fgetpos(tmp_file, &position) || os_fsetpos(tmp_file, &position)) {
    PRINTF(("Failed to get/set position in temp file, errno=%d\n", errno));
    ASSERT(false);
    return false;
  }

  r = fwrite(storage, push_index, 1, tmp_file);

  if (r == 1) {
    block_size.push_back(push_index);
    push_index = 0;
    return true;
  } else {
    PRINTF(("Failed writing to temp file, r=%lu, errno=%d\n", r, errno));
    ASSERT(false);
    return false;
  }
}

bool bque_var_size::read_block ()
{
  if (tmp_file == NULL) {
    PRINTF(("Failed, no temporary file"));
    ASSERT(false);
    return false;
  }

  if (os_fseek(tmp_file, n_bytes_read, SEEK_SET) != 0) {
    PRINTF(("Failed to set position in temporary file"));
    ASSERT(false);
    return false;
  }

  if (fread(storage, block_size[0], 1, tmp_file) != 1) {
    if (feof(tmp_file)) {
      PRINTF(("Reached EOF"));
    }
    else {
      PRINTF(("Failed to read temporary file"));
      ASSERT(false);
      return false;
    }
  }

  n_bytes_read += block_size[0];

  return true;
}

bool bque_var_size::push (void *data, unsigned int n_bytes)
{
  if (read_only) {
    //we have started to pop
    return false;
  }

  if ((n_bytes + 4) > storage_size) {
    PRINTF(("(n_bytes + 4) > storage_size"));
    return false;
  }

  if ((push_index + n_bytes + 4) > storage_size) {
    // save storage to file
    PRINTF(("Queue is full, dumping to disk"));
    if (!write_block()) {
      PRINTF(("Failed to dump block to disk"));
      ASSERT(false);
      return false;
    }
  }

  //save number of bytes
  memcpy(&storage[push_index], &n_bytes, 4);
  push_index += 4;

  //save data
  memcpy(&storage[push_index], data, n_bytes);
  push_index += n_bytes;

  n_pushed_element++;

  return true;
}

bool bque_var_size::pop(void *data)
{
  if (!n_pushed_element) {
    //nothing more to pop
    return false;
  }

  if (!read_only) {
    // when going from write-mode to read-mode,
    // write the last block and load the first
    if (!write_block()) {
      PRINTF(("Failed to write last block to disk"));
      ASSERT(false);
      return false;
    }

    if (!read_block()) {
      PRINTF(("Failed to read the first block"));
      ASSERT(false);
      return false;
    }
    read_only = true;
  }

  {
    unsigned int n_bytes;

    //get number of bytes
    memcpy(&n_bytes, &storage [pop_index], 4);
    pop_index += 4;

    //read data
    memcpy (data, &storage[pop_index], n_bytes);
    pop_index += n_bytes;
    n_pushed_element--;
  }

  if (pop_index == block_size[0]) {
    //we have read all data in storage
    pop_index = 0;
    block_size.erase(block_size.begin());

    if (block_size.size() > 0) {
      //we have more data to read
      if (!read_block()) {
        PRINTF(("Failed to read block"));
        ASSERT(false);
        return false;
      }
    }
  }

  if (block_size.size() == 0) {
    //no more data to pop
    pop_index    = 0;
    push_index   = 0;
    n_bytes_read = 0;
    read_only    = false;

    if (n_pushed_element) {
      PRINTF(("mismatch between n_element and block_size"));
      ASSERT(false);
    }

    close ();
  }

  return true;
}

unsigned int bque_var_size::get_n_element ()
{
  return n_pushed_element;
}
