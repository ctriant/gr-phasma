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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "sigmf_debug_impl.h"

namespace gr {
  namespace phasma {

    sigmf_debug::sptr
    sigmf_debug::make(size_t ifft_size, size_t conseq_channel_num)
    {
      return gnuradio::get_initial_sptr
        (new sigmf_debug_impl(ifft_size, conseq_channel_num));
    }

    /*
     * The private constructor
     */
    sigmf_debug_impl::sigmf_debug_impl(size_t ifft_size, size_t conseq_channel_num)
      : gr::sync_block("sigmf_debug",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
	      d_ifft_size(ifft_size),
	      d_conseq_channel_num(conseq_channel_num),
	      d_in_frame(false),
	      d_remaining(0),
	      d_curr_sig(0)
    {
      set_output_multiple (d_ifft_size);
      message_port_register_in(pmt::mp("sigmf"));

      d_msg_buf = new gr_complex[d_ifft_size * d_conseq_channel_num];
    }

    /*
     * Our virtual destructor.
     */
    sigmf_debug_impl::~sigmf_debug_impl()
    {
      delete [] d_msg_buf;
    }

    int
    sigmf_debug_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      size_t available_items;
      size_t max_available;
      pmt::pmt_t msg;
      pmt::pmt_t tuple;
      gr_complex* data;

      gr_complex *out = (gr_complex *) output_items[0];


      if (d_remaining > 0) {
	max_available = std::min((size_t)noutput_items, d_remaining);
	memcpy(out, d_msg_buf, max_available*sizeof(gr_complex));
	d_remaining = d_remaining - max_available;
//	std::cout << "IN FRAME" << std::endl;
//	std::cout << "Noutput: " << noutput_items << std::endl;
//	std::cout << "Available: " << max_available << std::endl;
//	std::cout << "Remaining: " << d_remaining << std::endl;
	return max_available;
      }

      /* No way to know a priori the vector length, so just catch exception */
      while (true) {
	try {
//	  std::cout << "Trying to get messages" << std::endl;
	  msg = delete_head_blocking(pmt::mp ("sigmf"));
	  tuple = pmt::vector_ref(msg,0);
//	  std::cout << pmt::tuple_ref (tuple, 0) << std::endl;
	  d_remaining = pmt::blob_length (pmt::tuple_ref (tuple, 1))
	      / sizeof(gr_complex);
//	  std::cout << "Blob: " << d_remaining << std::endl;
//	  std::cout << "Noutput: " << noutput_items << std::endl;
	  max_available = std::min((size_t)noutput_items, d_remaining);
	  data = (gr_complex *)pmt::blob_data(pmt::tuple_ref(tuple, 1));
	  memcpy(d_msg_buf, data, pmt::blob_length(pmt::tuple_ref(tuple, 1)));
	  memcpy(out, d_msg_buf, max_available*sizeof(gr_complex));
	  d_remaining = d_remaining - max_available;
//	  std::cout << "Available: " << max_available << std::endl;
//	  std::cout << "Remaining: " << d_remaining << std::endl;
	  memset(d_msg_buf, 0, d_ifft_size * d_conseq_channel_num * sizeof(gr_complex));
	  return max_available;
	}
	catch (pmt::out_of_range&) {
	  /* You are out of range so break */
	  d_curr_sig = 0;
	  break;
	}
	d_curr_sig++;
      }

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace phasma */
} /* namespace gr */

