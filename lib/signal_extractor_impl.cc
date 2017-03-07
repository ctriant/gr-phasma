/* -*- c++ -*- */
/* 
 * Copyright 2017 Kostis Triantafyllakis.
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
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <phasma/utils/sigMF.h>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "signal_extractor_impl.h"

namespace gr
{
  namespace phasma
  {

    using namespace gr::filter::kernel;
    using namespace boost::posix_time;
    using namespace pmt;

    signal_extractor::sptr
    signal_extractor::make (float samp_rate, float channel_bw, size_t ifft_size,
			    const std::vector<gr_complex> &taps,
			    const float silence_guardband,
			    float signal_duration, float min_sig_bw,
			    float threshold_db, float threshold_margin_db,
			    size_t sig_num)
    {
      return gnuradio::get_initial_sptr (
	  new signal_extractor_impl (samp_rate, channel_bw, ifft_size, taps,
				     silence_guardband, signal_duration,
				     min_sig_bw, threshold_db,
				     threshold_margin_db, sig_num));
    }

    /*
     * The private constructor
     */
    signal_extractor_impl::signal_extractor_impl (
	float samp_rate, float channel_bw, size_t ifft_size,
	const std::vector<gr_complex> &taps, const float silence_guardband,
	float signal_duration, float min_sig_bw, float threshold_db,
	float threshold_margin_db, size_t sig_num) :
	    gr::sync_block (
		"signal_extractor",
		gr::io_signature::make2 (2, 2, sizeof(float),
					 sizeof(gr_complex)),
		gr::io_signature::make (0, 0, 0)),
	    d_samp_rate (samp_rate),
	    d_channel_bw (channel_bw),
	    d_ifft_size (ifft_size),
	    d_silence_guardband (silence_guardband),
	    d_silence_guardband_samples (
		ceil ((d_ifft_size * d_silence_guardband) / d_channel_bw)),
	    d_channel_num (d_samp_rate / d_channel_bw),
	    d_fft_size ((d_samp_rate * d_ifft_size) / d_channel_bw),
	    d_signal_duration (signal_duration),
	    d_min_sig_samps (ceil ((d_ifft_size * min_sig_bw) / d_channel_bw)),
	    d_snapshots_required (1),
	    d_threshold_db (threshold_db),
	    d_threshold_margin_db (threshold_margin_db),
	    d_thresh_marg_lin (pow (10, d_threshold_margin_db / 10)),
	    d_threshold_linear (pow (10, d_threshold_db / 10)),
	    d_sig_num (sig_num),
	    d_conseq_channel_num (250),
	    d_abs_signal_start (0),
	    d_abs_signal_end (0)
    {
      /* Process in a per-FFT basis */
      set_output_multiple (d_fft_size);

      message_port_register_in (mp ("threshold_rcv"));
      set_msg_handler (
	  mp ("threshold_rcv"),
	  boost::bind (&signal_extractor_impl::msg_handler_noise_floor, this,
		       _1));

      message_port_register_out (mp ("out"));

      d_taps = taps;

      d_tmp = new gr_complex[d_conseq_channel_num * d_ifft_size];

      /* Calculation of Blackmann-Harris window */
      for (size_t j = 0; j < d_conseq_channel_num; j++) {
	d_blackmann_harris_win.push_back (
	    (float*) volk_malloc ((j + 1) * d_fft_size * sizeof(float), 32));
	for (size_t i = 0; i < d_ifft_size; i++) {
	  d_blackmann_harris_win[j][i] = 0.35875
	      - 0.48829 * cos ((2 * M_PI * i) / (d_fft_size - 1))
	      + 0.14128 * cos ((4 * M_PI * i) / (d_fft_size - 1))
	      - 0.01168 * cos ((6 * M_PI * i) / (d_fft_size - 1));
	}
      }

      /* Allocate memory for possible useful IFFT plans*/
      for (size_t i = 0; i < d_conseq_channel_num; i++) {
	d_filter.push_back (
	    new fft_filter_ccc (d_channel_num / (i + 1), d_taps, 1));
      }

      d_rotator.set_phase_incr (gr_complex (0, 0));

      d_noise_floor = new float[d_fft_size];
      for (size_t i = 0; i < d_fft_size; i++) {
	d_noise_floor[i] = d_threshold_linear;
      }
    }

    /*
     * Our virtual destructor.
     */
    signal_extractor_impl::~signal_extractor_impl ()
    {
      for (size_t i = 0; i < d_conseq_channel_num; i++) {
	delete d_filter[i];
      }

      delete [] d_tmp;

      delete[] d_noise_floor;
    }

    int
    signal_extractor_impl::work (int noutput_items,
				 gr_vector_const_void_star &input_items,
				 gr_vector_void_star &output_items)
    {
      ptime curr_microsecs;
      long millisecs;
      time_duration current_time_milliseconds;
      const float *spectrum_mag = (const float *) input_items[0];
      const gr_complex *in = (const gr_complex *) input_items[1];

      size_t available_snapshots = noutput_items / d_fft_size;

      size_t sig_count = 0;
      size_t silence_count = 0;
      size_t sig_slot_checkpoint;
      size_t sig_slot_curr;
      size_t start_idx;
      size_t end_idx;

      pmt_t msg;

      bool sig_alarm = false;
      // TODO: Fix the case of multiple available snapshots
      for (size_t s = 0; s < available_snapshots; s++) {
	/**
	 * TODO: Search spectrum for transmissions.
	 * For each identified transmission apply an IFFT to get the time
	 * domain samples and repeat until required snapshots are taken.
	 * In favor of simplicity, only a fixed number of simultaneous
	 * transmissions is handled.
	 *
	 * If this is the first snapshot of the timeslot try to identify
	 * transmissions and lock on channels to observe. Transmissions that
	 * span multiple channels should be treated as one.
	 */

	if (s == 0) {
	  for (size_t i = 0; i < d_fft_size; i++) {

	    sig_slot_curr = i / d_ifft_size;

	    if (spectrum_mag[i] > (d_noise_floor[i] * d_thresh_marg_lin)) {
	      silence_count = 0;
	      /**
	       * If there is silence and a peak is detected in the spectrum
	       * raise a signal alarm and mark the current slot.
	       */
	      if (!sig_alarm) {
		sig_alarm = true;
		d_abs_signal_start = i;
		sig_slot_checkpoint = sig_slot_curr;
	      }
	    }

	    else if ((spectrum_mag[i] < d_noise_floor[i] * d_thresh_marg_lin)
		&& sig_alarm) {
	      silence_count++;
	      if (((silence_count == d_silence_guardband_samples)
		  || (i == d_fft_size - 1)) && (sig_count < d_sig_num)) {
		curr_microsecs = microsec_clock::local_time ();
		millisecs = curr_microsecs.time_of_day ().total_milliseconds ();
		current_time_milliseconds = milliseconds (millisecs);
		ptime curr_milliseconds (curr_microsecs.date (),
					 current_time_milliseconds);
		d_abs_signal_end = i - silence_count + 1;
		sig_slot_curr = d_abs_signal_end / d_ifft_size;
		if (d_abs_signal_end - d_abs_signal_start > d_min_sig_samps) {
		  record_signal (in, &d_signals, sig_slot_checkpoint,
		  			       sig_slot_curr,
		  			       to_iso_extended_string (curr_milliseconds));
		  sig_count++;
		}
		sig_alarm = false;
		silence_count = 0;
	      }
	    }
	  }
	}
	/**
	 * Else iterate through the identified channels and store the time
	 * domain samples for each transmission.
	 */
	else {
	  // TODO: What about timestamp when more than one snapshot available?
	}

	if (sig_count > 0) {
	  msg = make_vector (sig_count, PMT_NIL);
	  for (size_t c = 0; c < sig_count; c++) {
	    pmt::vector_set (msg, c, d_msg_queue[c]);
	  }
	  message_port_pub (mp ("out"), msg);
	  sig_count = 0;
	}
	d_signals.clear ();
	d_msg_queue.clear ();
      }
      return noutput_items;
    }

    void
    signal_extractor_impl::record_signal (const gr_complex* in,
					  std::vector<phasma_signal_t>* signals,
					  size_t sig_slot_checkpoint,
					  size_t curr_slot,
					  std::string timestamp)
    {
      // Insert record in the signal vector.
      phasma_signal_t sig_rec;
      size_t ifft_idx;
      size_t span;
      size_t iq_size_bytes;
      float phase_inc;
      std::vector<gr_complex> taps_new(d_taps.size());

      ifft_idx = curr_slot - sig_slot_checkpoint;
      span = curr_slot - sig_slot_checkpoint + 1;
      iq_size_bytes = span * d_ifft_size * sizeof(gr_complex);

      sig_rec.start_slot_idx = sig_slot_checkpoint;
      sig_rec.end_slot_idx = curr_slot;
      sig_rec.iq_samples = new gr_complex[iq_size_bytes];

      /* TODO: Mind the case of multiple available snapshots */
      const gr_complex* sig_ptr = &in[sig_slot_checkpoint * d_ifft_size];

//      d_center_freq = ((((d_abs_signal_end * d_channel_bw) / d_ifft_size)
//	  + ((d_abs_signal_start * d_channel_bw) / d_ifft_size)) / 2)
//	  - (d_samp_rate / 2);

      d_center_freq = -1 * (((curr_slot * d_channel_bw
	  + sig_slot_checkpoint * d_channel_bw) / 2) - (d_samp_rate / 2));


      phase_inc = (2.0 * M_PI * d_center_freq);
      for (size_t t = 0; t < d_taps.size (); t++) {
	taps_new[t] = d_taps[t] * std::exp (gr_complex (0, t * phase_inc));
      }
      d_rotator.set_phase_incr (exp(gr_complex(0, -1 * (d_channel_num / span) * phase_inc)));

      d_filter[ifft_idx]->set_taps (taps_new);
      d_rotator.rotateN (d_tmp, in, d_fft_size);
      d_filter[ifft_idx]->filter (d_ifft_size*span, d_tmp, sig_rec.iq_samples);


      d_signals.push_back (sig_rec);

      sigMF meta = sigMF ("cf32", "./", "0.0.1", 0, d_samp_rate, timestamp,
			  d_abs_signal_start,
			  d_abs_signal_end - d_abs_signal_start,
			  sig_slot_checkpoint * d_channel_bw,
			  curr_slot * d_channel_bw, "");
      pmt_t tup;
      tup = make_tuple (string_to_symbol (meta.toJSON ()),
      			make_blob (sig_rec.iq_samples, iq_size_bytes));
      d_msg_queue.push_back (tup);

      delete [] sig_rec.iq_samples;

    }

    void
    signal_extractor_impl::msg_handler_noise_floor (pmt_t msg)
    {
      PHASMA_LOG_INFO("Signal extractor received estimated noise-floor");
      memcpy (d_noise_floor, blob_data (msg), blob_length (msg));
    }

  } /* namespace phasma */
} /* namespace gr */

