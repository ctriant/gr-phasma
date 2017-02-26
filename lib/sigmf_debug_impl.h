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

#ifndef INCLUDED_PHASMA_SIGMF_DEBUG_IMPL_H
#define INCLUDED_PHASMA_SIGMF_DEBUG_IMPL_H

#include <phasma/sigmf_debug.h>

namespace gr {
  namespace phasma {

    class sigmf_debug_impl : public sigmf_debug
    {
     private:
      size_t d_ifft_size;
      size_t d_conseq_channel_num;

      bool d_in_frame;
      size_t d_remaining;
      size_t d_curr_sig;
      gr_complex* d_msg_buf;

     public:
      sigmf_debug_impl(size_t ifft_size, size_t conseq_channel_num);
      ~sigmf_debug_impl();


      int work(int noutput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
    };

  } // namespace phasma
} // namespace gr

#endif /* INCLUDED_PHASMA_SIGMF_DEBUG_IMPL_H */

