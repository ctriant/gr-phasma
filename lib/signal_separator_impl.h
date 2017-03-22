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

#ifndef INCLUDED_PHASMA_SIGNAL_SEPARATOR_IMPL_H
#define INCLUDED_PHASMA_SIGNAL_SEPARATOR_IMPL_H

#include <phasma/signal_separator.h>
#include <gnuradio/filter/api.h>
#include <gnuradio/filter/fft_filter.h>
#include <gnuradio/filter/fft_filter_ccc.h>
#include <gnuradio/blocks/rotator.h>
#include <gnuradio/blocks/rotator_cc.h>
#include <string>
#include <fstream>

namespace gr
{
  namespace phasma
  {

    class signal_separator_impl : public signal_separator
    {
    private:

      const float d_samp_rate;
      const float d_channel_bw;

      const size_t d_ifft_size;

      std::vector<gr_complex> d_taps;

      const float d_silence_guardband;
      size_t d_silence_guardband_samples;
      
      float d_center_freq;

      size_t d_channel_num;
      size_t d_fft_size;

      const float d_signal_duration;
      float d_min_sig_samps;
      size_t d_snapshots_required;

      const float d_threshold_db;
      const float d_threshold_margin_db;
      const float d_thresh_marg_lin;
      const float d_threshold_linear;
      const size_t d_sig_num;
      const size_t d_conseq_channel_num;

      size_t d_abs_signal_start;
      size_t d_abs_signal_end;

      typedef struct
      {
	gr_complex* iq_samples;
	size_t start_slot_idx;
	size_t end_slot_idx;
      } phasma_signal_t;

      std::vector<phasma_signal_t> d_signals;

      gr_complex* d_tmp;

      std::vector<float*> d_blackmann_harris_win;

      std::vector<gr::filter::kernel::fft_filter_ccc*> d_filter;
      gr::blocks::rotator d_rotator;

      std::vector<pmt::pmt_t> d_msg_queue;

      /* Vector that holds linear noise floor */
      float* d_noise_floor;
      
      void
      record_signal (const gr_complex* fft_in,
		     std::vector<phasma_signal_t>* signals,
		     size_t sig_slot_checkpoint, size_t curr_slot,
		     std::string time);

    public:
      signal_separator_impl (float samp_rate, float channel_bw,
			     size_t ifft_size,
			     const std::vector<gr_complex> &taps,
			     const float silence_guardband,
			     float center_freq,
			     float signal_duration,
			     float min_sig_bw,
			     float threshold_db,
			     float threshold_margin_db, 
			     size_t sig_num);

      ~signal_separator_impl ();

      void
      msg_handler_noise_floor (pmt::pmt_t msg);

      int
      work (int noutput_items, gr_vector_const_void_star &input_items,
	    gr_vector_void_star &output_items);

    };

  } // namespace phasma
} // namespace gr

#endif /* INCLUDED_PHASMA_SIGNAL_SEPARATOR_IMPL_H */

