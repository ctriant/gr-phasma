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
#include <chrono>
#include <ctime>
#include "signal_extractor_impl.h"

namespace gr
{
  namespace phasma
  {

    signal_extractor::sptr
    signal_extractor::make (float samp_rate, float total_bw,
			    float channel_bw, size_t ifft_size,
			    size_t silence_guardband, float signal_duration,
			    float threshold_db, size_t sig_num)
    {
      return gnuradio::get_initial_sptr (
	  new signal_extractor_impl (samp_rate, total_bw, channel_bw, ifft_size,
				     silence_guardband, signal_duration,
				     threshold_db, sig_num));
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
	    d_snapshots_required(5),
	    d_threshold_db (threshold_db),
	    d_threshold_linear (std::pow (10, d_threshold_db / 10)),
	    d_sig_num(sig_num),
	    d_conseq_channel_num(150)
    {
      message_port_register_in (pmt::mp ("threshold"));
      message_port_register_out (pmt::mp ("out"));

      /* Process in a per-FFT basis */
      set_output_multiple (d_fft_size);

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
	std::cout << i << std::endl;
	d_ifft_plans.push_back (
	    new fft::fft_complex ((i + 1) * d_ifft_size, false, 1));
      }

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
    }

    int
    signal_extractor_impl::work (int noutput_items,
				 gr_vector_const_void_star &input_items,
				 gr_vector_void_star &output_items)
    {

      /***C++11 Style:***/
      std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

      const float *spectrum_mag = (const float *) input_items[0];
      const gr_complex *fft_in = (const gr_complex *) input_items[1];

      size_t available_snapshots = noutput_items / d_fft_size;

      size_t sig_count = 0;
      size_t silence_count = d_silence_guardband;
      size_t sig_slot_checkpoint;
      size_t sig_slot_curr;
      size_t start_idx;
      size_t end_idx;

      pmt::pmt_t msg;

      bool sig_alarm = false;

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
	    if (spectrum_mag[i] > d_threshold_linear) {
	      silence_count = d_silence_guardband;
	      /**
	       * If there is silence and a peak is detected in the spectrum
	       * raise a signal alarm and mark the current slot.
	       */
	      if (!sig_alarm) {
		sig_alarm = true;
		sig_slot_checkpoint = sig_slot_curr;
	      }
	      /**
	       * Signal probably span in more channels, but have to split in
	       * favor of simplicity.
	       * Channels with multiple transmissions or mixings may occur,
	       * but that's life.
	       */
	      else if (sig_slot_curr - sig_slot_checkpoint
		  == d_conseq_channel_num - 1) {
		std::time_t timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
		record_signal (fft_in, &d_signals, sig_slot_checkpoint, sig_slot_curr,
			       std::ctime(&timestamp));
		sig_slot_checkpoint = sig_slot_curr;
		sig_alarm = false;
		silence_count = d_silence_guardband;
		sig_count++;
	      }
	    }
	    else if ((spectrum_mag[i] < d_threshold_linear) && sig_alarm) {
	      silence_count--;
	      if ((!silence_count || i == d_fft_size - 1)
		  && sig_count < d_sig_num) {
		// Full -legal span- signal observed
		sig_alarm = false;
		silence_count = d_silence_guardband;
		std::time_t timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
		record_signal (fft_in, &d_signals, sig_slot_checkpoint, sig_slot_curr,
			       std::ctime(&timestamp));
		sig_count++;
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
//	  for (size_t i = 0; i < d_signals.size (); i++) {
//	    	record_signal (fft_in, &d_signals, d_signals[i].start_slot_idx,
//			   d_signals[i].end_slot_idx, std::ctime(&t));
//	  	sig_count++;
//	  }
	}
	msg = pmt::make_vector(sig_count, pmt::PMT_NIL);
	for (size_t c=0; c < sig_count; c++) {
	  pmt::vector_set(msg, c, d_msg_queue[c]);
	}
	message_port_pub (pmt::mp ("out"), msg);
	d_signals.clear ();
	d_signals.clear ();
      }
//      std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() <<std::endl;
      return noutput_items;
    }

    void
    signal_extractor_impl::record_signal (const gr_complex* fft_in,
					  std::vector<phasma_signal_t>* signals,
					  size_t sig_slot_checkpoint,
					  size_t curr_slot, std::string timestamp)
    {
      // Insert record in the signal vector.
      phasma_signal_t sig_rec;
      sig_rec.start_slot_idx = sig_slot_checkpoint;
      sig_rec.end_slot_idx = curr_slot;
      size_t span = sig_rec.end_slot_idx - sig_rec.start_slot_idx;
      size_t iq_size_bytes;

      const gr_complex* sig_ptr = &fft_in[sig_rec.start_slot_idx*d_ifft_size];
      /**
       * Windowing and IFFT
       */
      volk_32fc_32f_multiply_32fc(&d_ifft_plans[span]->get_inbuf()[0], sig_ptr,
      			d_blackmann_harris_win[span], d_ifft_size*span);
      d_ifft_plans[span]->execute();
      memcpy(&d_ishift[0],&d_ifft_plans[span]->get_outbuf()[(d_ifft_size*span) / 2],
			      sizeof(gr_complex) * (d_ifft_size*span) / 2);
      memcpy(&d_ishift[(d_ifft_size*span) / 2], &d_ifft_plans[span]->get_outbuf()[0],
      			      sizeof(gr_complex) * (d_ifft_size*span) / 2);

      if (curr_slot - sig_slot_checkpoint == 0) {
	iq_size_bytes = d_ifft_size * d_snapshots_required * sizeof(gr_complex);
	sig_rec.iq_samples = (gr_complex*) malloc(iq_size_bytes);
      }
      else {
	iq_size_bytes = ((sig_rec.end_slot_idx - sig_rec.start_slot_idx) + 1) * d_ifft_size
	      * d_snapshots_required * sizeof(gr_complex);
	sig_rec.iq_samples = (gr_complex*) malloc(iq_size_bytes);
      }
      memcpy(sig_rec.iq_samples, d_ishift, sizeof(gr_complex) * (d_ifft_size*span));

      d_signals.push_back (sig_rec);
      PHASMA_DEBUG("--- SIGNAL RECORDED ---");
      PHASMA_DEBUG("START: %d", d_signals.back().start_slot_idx);
      PHASMA_DEBUG("END: %d", d_signals.back().end_slot_idx);

      sigMF meta = sigMF("cf32",
			 "./",
			 "0.0.1",
			 0,
			 d_samp_rate,
			 timestamp,
			 sig_slot_checkpoint*d_ifft_size,
			 (curr_slot-sig_slot_checkpoint)*d_ifft_size,
			 sig_slot_checkpoint * d_channel_bw,
			 curr_slot * d_channel_bw,
			 "");

      pmt::pmt_t tup = pmt::make_tuple(pmt::string_to_symbol(meta.toJSON()),
				       pmt::make_blob(sig_rec.iq_samples, iq_size_bytes));
      d_msg_queue.push_back(tup);

      free(sig_rec.iq_samples);
    }

  } /* namespace phasma */
} /* namespace gr */

