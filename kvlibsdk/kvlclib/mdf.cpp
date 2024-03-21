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
** Description:
**   Read and write Vector binary log files (mdf format)
**   The MDF spec is here: p:\standards\fileformats\mdf_bosch
** ---------------------------------------------------------------------------
*/

// #define TEST_MDF 1
#include "os_util.h"
#include "mdf.h"
//#include "kvaconvertermisc.h"

typedef struct {
  const char *name;
  int len;
  int flag1, flag2;
} MdfColumn;

const int mdfColumnCount = 15;
MdfColumn mdfColumns[mdfColumnCount] = {
  {"type",     8, 0, 0},
  {"can",     16, 0, 1},
  {"id",      32, 0, 1},
  {"dir",      8, 0, 0},
  {"rtr",      8, 0, 0},
  {"dlc",      8, 0, 0},
  {"msg[0]",   8, 0, 0},
  {"msg[1]",   8, 0, 0},
  {"msg[2]",   8, 0, 0},
  {"msg[3]",   8, 0, 0},
  {"msg[4]",   8, 0, 0},
  {"msg[5]",   8, 0, 0},
  {"msg[6]",   8, 0, 0},
  {"msg[7]",   8, 0, 0},
  {"msgTime", 32, 1, 0}};

#ifdef MDF_DIAGNOSTICS
#define PRINTF(x) printf x
#else
#define PRINTF(x)
#endif


MDFfile::MDFfile(void)
{
  f = NULL;
  writing = false;
  tmpStr = NULL;
  tmpBlock = NULL;
  dgBlock = NULL;
  cgBlock = NULL;
  ccBlock = NULL;
  cnBlockCount = 0;
  corrupt = false;
  mBytesWritten = 0;
}

MDFfile::~MDFfile()
{
  close();
}


// Read an MDF file from an opened file handle.
int MDFfile::handle(FILE *fh)
{
  f = fh;
  corrupt = false;

  // Read the header, verifying it really is a mdf file.
  size_t n = fread(&header, 1, sizeof(header), f);
  if (n != sizeof(header)) {
    close();
    PRINTF(("Bad header size %d (0x%x)\n", n, n));
    corrupt = true;
    return 1;
  }
  if (strcmp(header.signature, MDF_HEADER)) {
    close();
    PRINTF(("Bad header signature (%s)\n",
            header.signature));
    PRINTF(("Expected signature '%s' and magic 0x%x\n",
            MDF_HEADER, MDF_MAGIC));

    corrupt = true;
    return 1;
  }

  if (header.magic != MDF_MAGIC || header.magic != MDF_NEWMAGIC) {
    PRINTF(("INFO: Unexpected magic number 0x%04x\n", header.magic));
  }

  n = fread(&hdBlock, 1, sizeof(hdBlock), f);
  if (n != sizeof(hdBlock)) {
    close();
    PRINTF(("Failed reading hdBlock\n"));
    corrupt = true;
    return 1;
  }
  if (hdBlock.magic != MDF_HD_BLOCK_MAGIC || hdBlock.length != sizeof(hdBlock)) {
    close();
    PRINTF(("Bad hdBlock magic (%d, 0x%x) or length (%d, 0x%x)\n",
            hdBlock.magic, hdBlock.magic,
            hdBlock.length, hdBlock.length));
    corrupt = true;
    return 1;
  }

  mdf_block *block = readBlock(hdBlock.dgPos, true);
  for (;;) {
    DWORD next = block->next0;
    if (!next)
      next = block->next1;

    switch(block->magic) {
      case MDF_DG_BLOCK_MAGIC:
        dgBlock = (mdf_dg_block*)block;
        break;
      case MDF_CG_BLOCK_MAGIC:
        cgBlock = (mdf_cg_block*)block;
        break;
      case MDF_CC_BLOCK_MAGIC:
        ccBlock = (mdf_cc_block*)block;
        break;
      case MDF_CN_BLOCK_MAGIC:
        if (cnBlockCount < sizeof(cnBlocks)/sizeof(*cnBlocks))
          cnBlocks[cnBlockCount++] = (mdf_cn_block*)block;
        else
          free(block);
        break;
      default:
        PRINTF(("Unknown block magic 0x%x\n", block->magic));
    }
    if (!next)
      break;
    block = readBlock(next, true);
  }

  return 0;
}

// Opens a existing mdf file
int MDFfile::open(const char *fileName)
{
  if (f)
    close();
  f = fopen(fileName, "rb");
  //f = utf_fopen(fileName, "rb");
  if (!f) {
    close();
    PRINTF(("Could not open file '%s'\n", fileName));
    return 1;
  }

  return handle (f);
}

// Create a mdf file, add header and blocks
int MDFfile::create(const char *fileName, time_t wallclock)
{
  size_t n, i;
  char tmp[128] = {0};

  if (f)
    close();

  mBytesWritten = 0;
  corrupt = false;
  memset(&header, 0, sizeof(header));
  memcpy(header.signature, MDF_HEADER, sizeof(header.signature));
  strncpy(header.creator, "CANwin I", sizeof(header.creator));
  header.magic = MDF_NEWMAGIC;

  memset(&hdBlock, 0, sizeof(hdBlock));
  hdBlock.magic = MDF_HD_BLOCK_MAGIC;
  hdBlock.length = sizeof(hdBlock);
  hdBlock.nnn = 1;

  struct tm *tblock = localtime(&wallclock);
  sprintf(tmp, "%02d:%02d:%04d", tblock->tm_mday, 1+tblock->tm_mon, 1900+tblock->tm_year);
  memcpy(hdBlock.date, tmp, sizeof(hdBlock.date));
  sprintf(tmp, "%02d:%02d:%02d", tblock->tm_hour, 1+tblock->tm_min, tblock->tm_sec);
  memcpy(hdBlock.time, tmp, sizeof(hdBlock.time));

  dgBlock = (mdf_dg_block*)calloc(1, sizeof(*dgBlock));
  dgBlock->magic = MDF_DG_BLOCK_MAGIC; dgBlock->length = sizeof(*dgBlock);
  dgBlock->z3 = 0xe4;
  dgBlock->z4 = 0x01;
  dgBlock->z5 = 0x01;

  cgBlock = (mdf_cg_block*)calloc(1, sizeof(*cgBlock));
  cgBlock->magic = MDF_CG_BLOCK_MAGIC; cgBlock->length = sizeof(*cgBlock);
  cgBlock->z3 = 0x01;
  cgBlock->z4 = 0x0f;
  cgBlock->z5 = 0x16;

  ccBlock = (mdf_cc_block*)calloc(1, sizeof(*ccBlock));
  ccBlock->magic = MDF_CC_BLOCK_MAGIC; ccBlock->length = sizeof(*ccBlock);
  ccBlock->z[32] = 2;

  // The CN blocks are not necessary for the CANalyzer. But we include
  // them anyway.
  cnBlockCount = 0;
  //CompilerAssert(mdfColumnCount <= sizeof(cnBlocks)/sizeof(*cnBlocks));
  //mdfColumnCount <= sizeof(cnBlocks)/sizeof(*cnBlocks);
  for (i = 0; i < mdfColumnCount; i++) {
    cnBlocks[i] = new mdf_cn_block;
    cnBlockCount++;
    memset(cnBlocks[i], 0, sizeof(mdf_cn_block));
    cnBlocks[i]->magic = MDF_CN_BLOCK_MAGIC;
    cnBlocks[i]->length = sizeof(mdf_cn_block);
    strcpy(cnBlocks[i]->colName, mdfColumns[i].name);
    cnBlocks[i]->len = mdfColumns[i].len;
    if (i == 0)
      cnBlocks[i]->pos = 0;
     else
      cnBlocks[i]->pos = cnBlocks[i-1]->pos+cnBlocks[i-1]->len;
    cnBlocks[i]->flag1 = mdfColumns[i].flag1;
    cnBlocks[i]->flag2 = mdfColumns[i].flag2;
  }

  ccBlock->dt = 0.00001;

  f = fopen(fileName, "wb");
  //f = utf_fopen(fileName, "wb");
  if (!f) {
    close();
    return 1;
  }

  n = fwrite(&header, 1, sizeof(header), f);
  n += fwrite(&hdBlock, 1, sizeof(hdBlock), f); // We update the pointers when closing
  if (n != sizeof(header)+sizeof(hdBlock)) {
    close();
    return 1;
  }
  writing = true;
  mBytesWritten +=n;
  return 0;
}


// Write an event record to the file.
// Returns 0 if OK.
int MDFfile::writeEvent(mdf_event_block *evtBlock)
{
  if (!f || !writing)
    return 1;
  evtBlock->z1 = 1;
  size_t bytesWritten = fwrite(evtBlock, 1, sizeof(*evtBlock), f);
  //Check that the whole block was actually written. (extel 2001-12-05)
  if (bytesWritten < sizeof(*evtBlock))
    return 1; //The whole block was not written.
  cgBlock->recCount++;
  mBytesWritten +=bytesWritten;
  return 0;
}

void MDFfile::sync(void)
{
  if (f) fflush(f);
}


// Closes the file and frees all memory.
// Return 0 if OK.
// If writing=true, the last part is written and the header is updated.
int MDFfile::close(void)
{
  int i;
  DWORD pos;

  if (f && writing) {
    // The file pointer now points after the last event record.
    pos = 0;
    if (ccBlock) {
      ccBlock->next = pos;
      pos = (DWORD)os_ftell(f);
      fwrite(ccBlock, 1, sizeof(*ccBlock), f);
    }
    for (i = cnBlockCount-1; i >= 0; i--) {
      if (i == (int)cnBlockCount-1)
        cnBlocks[i]->next1 = pos;
      else
        cnBlocks[i]->next0 = pos;
      pos = (DWORD)os_ftell(f);
      fwrite(cnBlocks[i], 1, sizeof(*cnBlocks[i]), f);
    }
    if (cgBlock) {
      cgBlock->next = pos;
      pos = (DWORD)os_ftell(f);
      fwrite(cgBlock, 1, sizeof(*cgBlock), f);
    }
    if (dgBlock) {
      dgBlock->next = pos;
      pos = (DWORD)os_ftell(f);
      fwrite(dgBlock, 1, sizeof(*dgBlock), f);
      hdBlock.dgPos = pos;
    }
    // The TX-block at the end.
    hdBlock.txPos = (DWORD)os_ftell(f);
    mdf_tx_block txBlock;
    const char *txStr = "CANwin InternalVersion\x0d\x1a";
    txBlock.magic = MDF_TX_BLOCK_MAGIC;
    txBlock.length = 4+(WORD)strlen(txStr)+1;
    fwrite(&txBlock, 1, sizeof(txBlock)-1, f); // Do not include txBlock.data
    fwrite(txStr, 1, strlen(txStr)+1, f); // Include the '\0' at the end.

    // Update the header.
    os_fseek(f, sizeof(header), SEEK_SET);
    fwrite(&hdBlock, 1, sizeof(hdBlock), f);

    fclose(f);
    f = NULL;
    writing = false;
  }

  if (tmpStr) {
    free(tmpStr);
    tmpStr = NULL;
  }
  if (tmpBlock) {
    free(tmpBlock);
    tmpBlock = NULL;
  }

  if (dgBlock) {
    free(dgBlock);
    dgBlock = NULL;
  }
  if (cgBlock) {
    free(cgBlock);
    cgBlock = NULL;
  }
  if (ccBlock) {
    free(ccBlock);
    ccBlock = NULL;
  }
  for (i = 0; i < (int)cnBlockCount; i++)
    free(cnBlocks[i]);

  cnBlockCount = 0;
  mBytesWritten = 0;
  return 0;
}

// Returns a pointer to a '\0'-terminated temporary string with a copy of s
// (which might be unterminated).
// Returns NULL if no file has been opened.
char *MDFfile::mkTmpStr(const char *s, int len)
{
  if (!f)
    return NULL;
  if (tmpStr)
    free(tmpStr);
  tmpStr = (char*)malloc(len+1);
  if (!tmpStr)
    return NULL;
  memcpy(tmpStr, s, len);
  tmpStr[len] = '\0';
  return tmpStr;
}

// Read a block from the file, returns a pointer to it.
// If alloc=true, memory is allocated that the caller must free,
// otherwise a temporary block that is reused the next call will be used.
// Returns NULL in case of error.
mdf_block *MDFfile::readBlock(uint32_t pos, bool alloc)
{
  mdf_block block, *bptr;
  if (tmpBlock)
    free(tmpBlock);
  if (os_fseek(f, pos, SEEK_SET))
    return NULL;
  size_t n = fread(&block, 1, 4, f); // Read magic and length
  if (n != 4)
    return NULL;
  tmpBlock = (mdf_block*)malloc(block.length);
  tmpBlock->magic = block.magic;
  tmpBlock->length = block.length;
  n = fread(&tmpBlock->data, 1, block.length-4, f);
  if (n != (size_t)block.length-4)
    return NULL;
  bptr = tmpBlock;
  if (alloc)
    tmpBlock = NULL; // So it isn't free'd the next call.
  return bptr;
}

// Read an event block from the file, returns a pointer to it.
// A temporary block that is reused the next call will be used.
// Returns NULL in case of error.
mdf_event_block *MDFfile::readEvent(uint64_t i)
{
  if (os_fseek(f, sizeof(mdf_header_t)+sizeof(mdf_hd_block_t) + (uint32_t)i * sizeof(mdf_event_block), SEEK_SET))
    return NULL;
  size_t n = fread(&evtBlock, 1, sizeof(evtBlock), f);
  if (n != sizeof(evtBlock))
    return NULL;
  return &evtBlock;
}

// Returns the text pointed to by the (first) HD-block.
char *MDFfile::getText(void)
{
  mdf_tx_block *tb = (mdf_tx_block*)readBlock(hdBlock.txPos);
  return mkTmpStr(tb->text, tb->length-4);
}

//int MDFfile::save(char *fileName) {
//  return 1; // Not implemeted
//}


void print_mdf(MDFfile *mdf)
{
  unsigned int i, j;

  printf("MDF file\n");
  printf("========\n");
  printf("Date: %s", mdf->getDate());
  printf(" %s\n", mdf->getTime());
  printf("Header signature: '%s' and magic: 0x%04x\n", mdf->header.signature, (unsigned int)mdf->header.magic);
  printf("Creator: '%s'\n", mdf->getCreator());
  printf("Comment: '%s'\n", mdf->getComment());
  printf("DG offset: 0x%x\n", mdf->hdBlock.dgPos);
  printf("TX offset: 0x%x\n", mdf->hdBlock.txPos);
  printf("offset 3: 0x%x\n", mdf->hdBlock.offset_3);
  printf("Text: '%s'\n", mdf->getText()); // Text ends with \x0d\x1a\x00 for some reason.

  printf("CN blocks:\n");
  for (i = 0; i < mdf->cnBlockCount; i++) {
    printf(" {name %-7s  ", mdf->mkTmpStr(mdf->cnBlocks[i]->colName,
                                          sizeof(mdf->cnBlocks[i]->colName)));
    printf("%4x, %4x, %4x, %4x",
           mdf->cnBlocks[i]->pos,
           mdf->cnBlocks[i]->len,
           mdf->cnBlocks[i]->flag1,
           mdf->cnBlocks[i]->flag2);
    printf("}\n");
  }

  printf("\n");
  printf("dt: %f\n", mdf->ccBlock->dt);
  printf("Event count: %" PRIu64 "\n", mdf->eventCount());

  {
    uint64_t i;
    for (i = 0; i < mdf->eventCount(); i++) {
      mdf_event_block *evtBlock;
      
      evtBlock = mdf->readEvent(i);
      printf("%3" PRIu64 " %12.4f: ", i, evtBlock->time*0.00001);
      
      if (evtBlock->type == MDF_EVTTYPE_MESSAGE) {
	mdf_evtMsg_block *msgBlock = (mdf_evtMsg_block*)evtBlock;
	printf("MSG %d ", msgBlock->channel);
	printf("%8x%c ", msgBlock->canId & 0x7fffffff, msgBlock->canId & MDF_ID_EXT ? 'x' : ' ');
	switch(msgBlock->dir) {
	  case MDF_EVTDIR_RX:   printf("Rx  "); break;
	  case MDF_EVTDIR_TX:   printf("Tx  "); break;
	  case MDF_EVTDIR_TXRQ: printf("TxRq"); break;
	  default:   printf("[%02x]", msgBlock->dir); break;
	}
	printf(msgBlock->rtr ? " r" : " d");
	printf(" %d", msgBlock->dlc);
	for (j = 0; (int)j < msgBlock->dlc; j++)
	  printf(" %02x", msgBlock->data[j]);
      }
      
      else if (evtBlock->type == MDF_EVTTYPE_ERRORFRAME) {
	mdf_evtMsg_block *msgBlock = (mdf_evtMsg_block*)evtBlock;
	printf("ERR %d ", msgBlock->channel);
	printf("%8x%c ", msgBlock->canId & 0x7fffffff, msgBlock->canId & MDF_ID_EXT ? 'x' : ' ');
	switch(msgBlock->dir) {
	  case MDF_EVTDIR_RX:   printf("Rx  "); break;
	  case MDF_EVTDIR_TX:   printf("Tx  "); break;
	  case MDF_EVTDIR_TXRQ: printf("TxRq"); break;
	  default:   printf("[%02x]", msgBlock->dir); break;
	}
	printf(msgBlock->rtr ? " r" : " d");
	printf(" %d", msgBlock->dlc);
	for (j = 0; (int)j < msgBlock->dlc; j++)
	  printf(" %02x", msgBlock->data[j]);
      }
      
      else if (evtBlock->type == MDF_EVTTYPE_BUSSTAT) {
	mdf_evtBusstat_block *bsBlock = (mdf_evtBusstat_block*)evtBlock;
	printf("BUS %d {", bsBlock->channel);
	for (j = 0; j < 15; j++) {
	  if (j)
	    printf(" ");
	  printf("%02x", bsBlock->b[j]);
	}
	printf("}");
      } else
	printf(" Unknown event type %d!", evtBlock->type);
      printf("\n");
    }
  }
}

// Copy the contents of mdfR to a new log file outFile.
// Only the event records are actually copied; all other
// data are assigned default values.
void copy_mdf(MDFfile *mdfR, char *outFile) {
  uint64_t i;

  MDFfile *mdfW = new MDFfile;
  mdfW->create(outFile);

  for (i = 0; i < mdfR->eventCount(); i++) {
    mdf_event_block *evtBlock = mdfR->readEvent(i);
    mdfW->writeEvent(evtBlock);
  }

  mdfW->close();
  delete mdfW;
}

#ifdef TEST_MDF
int main(int argc, char* argv[])
{
  char *inFile, *outFile;

  if (argc < 2 || argc > 3) {
    printf ("usage: mdfDump [mdf-infile [mdf-outFile]]\n");
    return -1;
  }
  inFile = argv[1];
  outFile = (argc > 2) ? argv[2] : NULL;

  MDFfile *mdf = new MDFfile;
  if (strcmp(inFile, "-") != 0)
    mdf->open(inFile);
  print_mdf(mdf);
  if (outFile) {
    printf("\n\nSaving to %s\n", outFile);
    copy_mdf(mdf, outFile);
  }
  delete mdf;
  return 0;
}
#endif // TEST_MDF
