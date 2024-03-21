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
#include "BufQueue.h"
#include "kvdebug.h"
#include "kvassert.h"
#include "os_util.h"


//===========================================================================
bool BQueue::open_temp_file (char *prefix)
{
  tmp_file = os_open_tmp_file(prefix, &filename);

  if (tmp_file == NULL) {
    PRINTF(("Failed to open file!"));
    ASSERT(false);
    return false;
  }

  return true;
}

//===========================================================================
bool BQueue::write_block ()
{
  size_t r;
  fpos_t position;

  //PRINTF(("write_block (head = %d, current_size = %d, nof_blocks = %d)",
  //        head, current_size, nof_blocks));

  *(unsigned int *)&storage[size_of_element*size_of_queue] = head;
  *(unsigned int *)&storage[size_of_element*size_of_queue+sizeof(int)] =
                      current_size;
  if (tmp_file == NULL) {
    if (!open_temp_file(file_prefix)) {
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

  r = fwrite(storage,
             size_of_element * size_of_queue + 2*sizeof(int),
             1, tmp_file);

  if (r == 1) {
    nof_blocks++;
    current_block = 0;
    head = 0;
    current_size = 0;
    memset(storage, 0x00, size_of_queue * size_of_element + 2*sizeof(int));
    return true;
  } else {
    PRINTF(("Failed writing to temp file, r=%lu, errno=%d\n", r, errno));
    ASSERT(false);
    return false;
  }
}

//===========================================================================
bool BQueue::read_block ()
{
  unsigned int block_size;

  block_size = size_of_queue * size_of_element + 2 * sizeof(int);

  PRINTF(("read_block (head = %d, current_size = %d, nof_blocks = %d)",
    head, current_size, nof_blocks));

  if (tmp_file == NULL) {
    PRINTF(("Failed, no temporary file"));
    ASSERT(false);
    return false;
  }
  if (nof_blocks == current_block) {
    PRINTF(("Block already loaded"));
  }

  if (os_fseek(tmp_file, current_block * block_size, SEEK_SET) != 0) {
    PRINTF(("Failed to set position in temporary file"));
    ASSERT(false);
    return false;
  }

  if (fread(storage, block_size, 1, tmp_file) != 1) {
    if (feof(tmp_file)) {
      PRINTF(("Reached EOF"));
    }
    else {
      PRINTF(("Failed to read temporary file"));
      ASSERT(false);
      return false;
    }
  }
  head = *(unsigned int *)&storage[size_of_element*size_of_queue];
  current_size = *(unsigned int *)&storage[
    size_of_element*size_of_queue+sizeof(int)
  ];

  current_block++;
  PRINTF(("cb = %d, cs = %d", current_block, current_size));
  return true;
}

//===========================================================================
BQueue::BQueue (char *tmp_prefix,
                unsigned int queue_size,
                unsigned int element_size)
{
  //PRINTF(("allocates new memory for the BQueue (%s)", tmp_prefix));
  size_of_queue = queue_size;
  size_of_element = element_size;
  storage = (char*)malloc(size_of_queue * size_of_element + 2*sizeof(int));
  if (storage == NULL) {
    PRINTF(("Couldn't allocate BQueue"));
    // signal this?
    ASSERT(false);
  }
  memset(storage, 0x00, size_of_queue * size_of_element + 2*sizeof(int));
  head = 0;
  current_size = 0;
  read_only = false;
  nof_blocks = 0;
  current_block = 0;
  total_size = 0;
  tmp_file = NULL;
  filename = NULL;
  strncpy(file_prefix, tmp_prefix, sizeof(file_prefix));
  file_prefix[sizeof(file_prefix)-1] = '\0';
}

//===========================================================================
BQueue::~BQueue ()
{
  // ASSERT(head >= 0);  // comparison of unsigned expression >= 0 is always true
  //PRINTF(("~BQueue: head = %d, size_of_queue = %d, current_size = %d\n", head, size_of_queue, current_size));
  ASSERT(head < size_of_queue);
  // ASSERT(current_size >= 0);
  ASSERT(current_size <= size_of_queue);

  if (tmp_file && fclose(tmp_file) == EOF) {
    PRINTF(("BQueue failed to close temp file"));
    ASSERT(false);
    // retry or fail?
  }
  if (filename) {
    if (remove(filename)) {
      PRINTF(("Failed to remove temp file '%s'",filename));
      ASSERT(false);
      // retry, fail or notify user?
    }
    PRINTF(("Free %s", filename));
    free(filename);
  }
  free(storage);
}

//===========================================================================
bool BQueue::push (void *mem)
{
  // ASSERT(head >= 0);
  ASSERT(head < size_of_queue);
  // ASSERT(current_size >= 0);
  ASSERT(current_size <= size_of_queue);

  if (read_only) {
    PRINTF(("Cannot push, queue is in read only mode"));
    return false;
  }
  //PRINTF(("Push (head = %d, current_size = %d)", head, current_size));
  if (current_size == size_of_queue) {
    PRINTF(("Queue is full, dumping to disk"));
    if (!write_block()) {
      PRINTF(("Failed to dump block to disk"));
      ASSERT(false);
      return false;
    }
  }
  memcpy(&storage[((current_size + head) % size_of_queue)* size_of_element],
    mem, size_of_element);
  total_size++;
  current_size++;

  return true;
}

//===========================================================================
bool BQueue::pop(void *mem)
{
  // ASSERT(head >= 0);
  ASSERT(head < size_of_queue);
  // ASSERT(current_size >= 0);
  ASSERT(current_size <= size_of_queue);

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

    //PRINTF(("nb = %d, cb = %d",nof_blocks,current_block));
  }

  if (size() > 0) {
    memcpy(mem, &storage [head * size_of_element], size_of_element);
    current_size--;
    head++;
    head = head % size_of_queue;
    total_size--;
    if (current_size == 0) {
      if (!read_block()) {
        PRINTF(("Failed to read block"));
        ASSERT(false);
        return false;
      }
    }
    if (total_size == 0) {
      head = 0;
      current_size = 0;
      read_only = false;
      nof_blocks = 0;
      current_block = 0;
      total_size = 0;
      if (tmp_file && fclose(tmp_file) == EOF) {
        PRINTF(("BQueue failed to close temp file"));
        ASSERT(false);
        // retry or fail?
      }
      tmp_file = NULL;
      if (filename) {
        if (remove(filename)) {
          PRINTF(("Failed to remove temp file '%s'",filename));
          ASSERT(false);
          // retry, fail or notify user?
        }
        PRINTF(("Delete %s", filename));
        delete filename;
        filename = NULL;
      }
    }
  }
  else {
    PRINTF(("NO MORE TO POP"));
    ASSERT(false);
    return false;
  }
  return true;
}

//===========================================================================
unsigned int BQueue::size ()
{
  // ASSERT(head >= 0);
  ASSERT(head < size_of_queue);
  // ASSERT(current_size >= 0);
  ASSERT(current_size <= size_of_queue);

  return total_size;
}

#if defined(BUF_EXE)
// Small test program for BQueue
// * Will push 1000 of __int64 into a buffer with buffer of size 13
// * When everything is popped, redo once more
// * Use process explorer or similar to close temp-file
int main() {
  BQueue *b = new BQueue("tjoff", 13, sizeof(__int64));

  __int64 i,j;
  __int64 limit = 1000;

  printf("test\n");

  for ( i = 0; i < limit; i++) {
    if (!b->push(&i)) {
      printf("push failed\n");
      goto STOP;
    }
  }

  for ( i = 0; i < limit; i++) {
    j = 0;
    if (!b->pop(&j)) {
      printf("pop failed\n");
      goto STOP;
    }
    if (i != j) {
      printf("1: i = %I64d, j = %I64d\n", i, j);
      ASSERT(false);
    }
  }

  for ( i = 5; i < limit + 5; i++) {
    if (!b->push(&i)) {
      printf("push failed\n");
      goto STOP;
    }
  }

  for ( i = 5; i < limit + 5; i++) {
    j = 0;
    if (!b->pop(&j)) {
      printf("pop failed\n");
      goto STOP;
    }
    if (i != j) {
      printf("2: i = %I64d, j = %I64d\n", i, j);
      ASSERT(false);
    }
  }
STOP:
  delete b;
}

#endif
