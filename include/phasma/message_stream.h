/* -*- c++ -*- */
/* 
 * Copyright 2017 <+YOU OR YOUR COMPANY+>.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */


#ifndef INCLUDED_PHASMA_MESSAGE_STREAM_H
#define INCLUDED_PHASMA_MESSAGE_STREAM_H

#include <phasma/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
  namespace phasma {

    /*!
     * \brief <+description of block+>
     * \ingroup phasma
     *
     */
    class PHASMA_API message_stream : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<message_stream> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of phasma::message_stream.
       *
       * To avoid accidental use of raw pointers, phasma::message_stream's
       * constructor is in a private implementation
       * class. phasma::message_stream::make is the public interface for
       * creating new instances.
       */
      static sptr make(size_t ifft_size, size_t conseq_channel_num);
    };

  } // namespace phasma
} // namespace gr

#endif /* INCLUDED_PHASMA_MESSAGE_STREAM_H */

