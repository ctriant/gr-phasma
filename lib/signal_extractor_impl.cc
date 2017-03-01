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

    signal_extractor::sptr
    signal_extractor::make (float samp_rate, float total_bw,
			    float channel_bw, size_t ifft_size,
			    size_t silence_guardband, float signal_duration,
			    float threshold_db, float threshold_margin_db,
			    size_t sig_num)
    {
      return gnuradio::get_initial_sptr (
	  new signal_extractor_impl (samp_rate, total_bw, channel_bw, ifft_size,
				     silence_guardband, signal_duration,
				     threshold_db, threshold_margin_db, sig_num));
    }

    /*
     * The private constructor
     */
    signal_extractor_impl::signal_extractor_impl (float samp_rate,
						  float total_bw,
						  float channel_bw,
						  size_t ifft_size,
						  size_t silence_guardband,
						  float signal_duration,
						  float threshold_db,
						  float threshold_margin_db,
						  size_t sig_num) :
	    gr::sync_block ("signal_extractor",
			    gr::io_signature::make2(2, 2, sizeof(float), sizeof(gr_complex)),
			    gr::io_signature::make(0, 0, 0)),
	    d_samp_rate (samp_rate),
	    d_total_bw (total_bw),
	    d_channel_bw (channel_bw),
	    d_ifft_size (ifft_size),
	    d_silence_guardband (silence_guardband),
	    d_channel_num (d_total_bw / d_channel_bw),
	    d_fft_size ((d_samp_rate * d_ifft_size) / d_channel_bw),
	    d_signal_duration (signal_duration),
	    d_snapshots_required(1),
	    d_threshold_db (threshold_db),
	    d_threshold_margin_db(threshold_margin_db),
	    d_threshold_margin_linear(std::pow (10, d_threshold_margin_db / 10)),
	    d_threshold_linear (std::pow (10, d_threshold_db / 10)),
	    d_sig_num(sig_num),
	    d_conseq_channel_num(250),
	    d_abs_signal_start(0),
	    d_abs_signal_end(0)
    {
      /* Process in a per-FFT basis */
      set_output_multiple (d_fft_size);

      message_port_register_in (pmt::mp ("threshold_rcv"));
      set_msg_handler (
	  pmt::mp ("threshold_rcv"),
	  boost::bind (&signal_extractor_impl::msg_handler_noise_floor, this, _1));

      message_port_register_out (pmt::mp ("out"));

      d_ishift = (gr_complex*) volk_malloc (
	  d_conseq_channel_num * d_ifft_size * sizeof(gr_complex), 32);

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
	d_ifft_plans.push_back (
	    new fft::fft_complex ((i + 1) * d_ifft_size, false, 1));
      }

      d_noise_floor = new float[d_fft_size];
      memset(d_noise_floor, d_threshold_linear, d_fft_size * sizeof(float));

    }

    /*
     * Our virtual destructor.
     */
    signal_extractor_impl::~signal_extractor_impl ()
    {
      for (size_t i = 0; i < d_conseq_channel_num; i++) {
	delete d_ifft_plans[i];
	volk_free(d_blackmann_harris_win[i]);
      }
      volk_free (d_ishift);

      delete [] d_noise_floor;
    }

    int
    signal_extractor_impl::work (int noutput_items,
				 gr_vector_const_void_star &input_items,
				 gr_vector_void_star &output_items)
    {

      /***C++11 Style:***/
      boost::posix_time::ptime current_date_microseconds;
      long milliseconds;
      boost::posix_time::time_duration current_time_milliseconds;
      const float *spectrum_mag = (const float *) input_items[0];
      const gr_complex *fft_in = (const gr_complex *) input_items[1];

      size_t available_snapshots = noutput_items / d_fft_size;

      size_t sig_count = 0;
      size_t silence_count = 0;
      size_t sig_slot_checkpoint;
      size_t sig_slot_curr;
      size_t start_idx;
      size_t end_idx;

      pmt::pmt_t msg;

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
	    if (spectrum_mag[i] > (d_noise_floor[i]*d_threshold_margin_linear)) {
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
	    else if ((spectrum_mag[i] < (d_noise_floor[i]*d_threshold_margin_linear)) && sig_alarm) {
	      silence_count++;
	      if (((silence_count == d_silence_guardband) || (i == d_fft_size - 1))
		  && (sig_count < d_sig_num)) {
		// Full -legal span- signal observed
		current_date_microseconds = boost::posix_time::microsec_clock::local_time();
		milliseconds = current_date_microseconds.time_of_day().total_milliseconds();
		current_time_milliseconds = boost::posix_time::milliseconds(milliseconds);
		boost::posix_time::ptime current_date_milliseconds(current_date_microseconds.date(),
								      current_time_milliseconds);
		d_abs_signal_end = i-silence_count+1;
		sig_slot_curr = d_abs_signal_end / d_ifft_size;
		record_signal (fft_in, &d_signals, sig_slot_checkpoint, sig_slot_curr,
			       boost::posix_time::to_iso_extended_string(current_date_milliseconds));
		sig_alarm = false;
		sig_count++;
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
	  msg = pmt::make_vector(sig_count, pmt::PMT_NIL);
	  for (size_t c=0; c < sig_count; c++) {
	    pmt::vector_set(msg, c, d_msg_queue[c]);
	  }
	  message_port_pub (pmt::mp ("out"), msg);
	  sig_count = 0;
	}
	d_signals.clear ();
	d_msg_queue.clear ();
      }
      return noutput_items;
    }

    void
    signal_extractor_impl::record_signal (const gr_complex* fft_in,
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

      /* TODO: Mind the case of multiple available snapshots */
      const gr_complex* sig_ptr = &fft_in[sig_slot_checkpoint * d_ifft_size];

      ifft_idx = curr_slot - sig_slot_checkpoint;
      span = curr_slot - sig_slot_checkpoint + 1;

      memcpy (&d_ifft_plans[ifft_idx]->get_inbuf ()[0], sig_ptr,
	      d_ifft_size * span * sizeof(gr_complex));

      d_ifft_plans[ifft_idx]->execute ();
      iq_size_bytes = span * d_ifft_size * d_snapshots_required * sizeof(gr_complex);

      sig_rec.start_slot_idx = sig_slot_checkpoint;
      sig_rec.end_slot_idx = curr_slot;
      sig_rec.iq_samples = (gr_complex*) malloc (iq_size_bytes);
      memcpy (sig_rec.iq_samples, d_ifft_plans[ifft_idx]->get_outbuf (),
	      iq_size_bytes);

      d_signals.push_back (sig_rec);
      sigMF meta = sigMF("cf32",
			 "./",
			 "0.0.1",
			 0,
			 d_samp_rate,
			 timestamp,
			 d_abs_signal_start,
			 d_abs_signal_end - d_abs_signal_start,
			 sig_slot_checkpoint * d_channel_bw,
			 curr_slot * d_channel_bw,
			 "");
      pmt::pmt_t tup = pmt::make_tuple(pmt::string_to_symbol(meta.toJSON()),
				       pmt::make_blob(sig_rec.iq_samples, iq_size_bytes));
      d_msg_queue.push_back(tup);

      free(sig_rec.iq_samples);
    }

    void
    signal_extractor_impl::msg_handler_noise_floor (pmt::pmt_t msg)
    {
      PHASMA_DEBUG("Signal extractor received estiamted noise-floor");
      memcpy(d_noise_floor, pmt::blob_data(msg), pmt::blob_length(msg));
    }

  } /* namespace phasma */
} /* namespace gr */

