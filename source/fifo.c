/**************************************************************************/
/*!
    @file     fifo.c
    @author   Thach Ha (tinyusb.org)

    @section DESCRIPTION

    Light-weight FIFO buffer with basic mutex support

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2012, K. Townsend (microBuilder.eu)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/
#include "fifo.h"

static inline bool is_fifo_initalized(fifo_t* f) ATTR_ALWAYS_INLINE;

/**************************************************************************/
/*!
    @brief Read one byte out of the RX buffer.

    This function will return the byte located at the array index of the
    read pointer, and then increment the read pointer index.  If the read
    pointer exceeds the maximum buffer size, it will roll over to zero.

    @param[in]  f
                Pointer to the FIFO buffer to manipulate
    @param[in]  p_buffer
                Pointer to the place holder for data read from the buffer

    @returns TRUE if the queue is not empty
*/
/**************************************************************************/
bool fifo_read(fifo_t* f, void * p_buffer)
{
  if( !is_fifo_initalized(f) || fifo_isEmpty(f) )
  {
    return false;
  }

  memcpy(p_buffer,
         f->buffer + (f->rd_idx * f->item_size),
         f->item_size);
  f->rd_idx = (f->rd_idx + 1) % f->depth;
  f->count--;

  return true;
}

/**************************************************************************/
/*!
    @brief Reads one item without removing it from the FIFO

    @param[in]  f
                Pointer to the FIFO buffer to manipulate
    @param[in]  position
                Position to read from in the FIFO buffer
    @param[in]  p_buffer
                Pointer to the place holder for data read from the buffer

    @returns TRUE if the queue is not empty
*/
/**************************************************************************/
bool fifo_peek(fifo_t* f, uint16_t position, void * p_buffer)
{
  if( !is_fifo_initalized(f) || fifo_isEmpty(f) || (position >= f->count) )
  {
    return false;
  }

  uint16_t index = (f->rd_idx + position) % f->depth; // rd_idx is position=0
  memcpy(p_buffer,
         f->buffer + (index * f->item_size),
         f->item_size);

  return true;
}

/**************************************************************************/
/*!
    @brief Write one byte into the RX buffer.

    This function will write one byte into the array index specified by
    the write pointer and increment the write index. If the write index
    exceeds the max buffer size, then it will roll over to zero.

    @param[in]  f
                Pointer to the FIFO buffer to manipulate
    @param[in]  p_data
                The byte to add to the FIFO

    @returns TRUE if the data was written to the FIFO (overwrittable
             FIFO will always return TRUE)
*/
/**************************************************************************/
bool fifo_write(fifo_t* f, void const * p_data)
{
  if ( !is_fifo_initalized(f) || (fifo_isFull(f) && !f->overwritable) )
  {
    return false;
  }

  memcpy( f->buffer + (f->wr_idx * f->item_size),
          p_data,
          f->item_size);

  f->wr_idx = (f->wr_idx + 1) % f->depth;

  if (fifo_isFull(f))
  {
    f->rd_idx = f->wr_idx; // keep the full state (rd == wr && len = size)
  }
  else
  {
    f->count++;
  }

  return true;
}

/**************************************************************************/
/*!
    @brief Clear the fifo read and write pointers and set length to zero

    @param[in]  f
                Pointer to the FIFO buffer to manipulate
*/
/**************************************************************************/
void fifo_clear(fifo_t *f)
{
  f->rd_idx = f->wr_idx = f->count = 0;
}

//--------------------------------------------------------------------+
// HELPER FUNCTIONS
//--------------------------------------------------------------------+

static inline bool is_fifo_initalized(fifo_t* f)
{
  if( f->buffer == NULL || f->depth == 0 || f->item_size == 0)
  {
    return false;
  }
  else
  {
    return true;
  }
}
