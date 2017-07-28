/* -*- c++ -*- */
/* 
 * Copyright 2016 Kostis Triantafyllakis.
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

#define ARMA_DONT_USE_WRAPPER
#include <armadillo>
#include <volk/volk.h> 
#include <boost/algorithm/minmax.hpp>
#include <math.h>
#include <phasma/api.h> 
#include <phasma/log.h>
#include <gnuradio/io_signature.h>

#include "eigenvalue_signal_detector_impl.h"

namespace gr
{
  namespace phasma
  {

    eigenvalue_signal_detector::sptr
    eigenvalue_signal_detector::make (size_t eigen_samples,
				      size_t smoothing_factor, double pfa,
				      size_t fft_size, float sampling_rate,
				      float actual_bw,
				      size_t noise_floor_est_cnt)
    {
      return gnuradio::get_initial_sptr (
	  new eigenvalue_signal_detector_impl (eigen_samples, smoothing_factor,
					       pfa, fft_size, sampling_rate,
					       actual_bw, noise_floor_est_cnt));
    }

    /*
     * The private constructor
     */
    eigenvalue_signal_detector_impl::eigenvalue_signal_detector_impl (
	size_t eigen_samples, size_t smoothing_factor, double pfa,
	size_t fft_size, float sampling_rate, float actual_bw,
	size_t noise_floor_est_cnt) :
	    gr::sync_block ("eigenvalue_signal_detector",
			    gr::io_signature::make (1, 1, sizeof(gr_complex)),
			    gr::io_signature::make (0, 0, 0)),
	    d_eigen_samples (eigen_samples),
	    d_smoothing_factor (smoothing_factor),
	    d_pfa (pfa),
	    d_fft_size (fft_size),
	    d_sampling_rate (sampling_rate),
	    d_bw (actual_bw),
	    d_num_samps_to_discard (
		ceil (
		    (d_fft_size - ((d_bw * (float) d_fft_size) / d_sampling_rate))
			/ 2)),
	    d_num_samps_per_side ((d_fft_size / 2) - d_num_samps_to_discard),
	    d_subcarriers_num (2 * d_num_samps_per_side),
	    d_norm_factor (1 / (float) d_fft_size, 0),
	    d_noise_floor_est_cnt (noise_floor_est_cnt),
	    d_noise_floor_est (true),
	    d_status(true)

    {
      /* Process in a per-FFT basis */
      set_output_multiple (d_fft_size);

      /* Outgoing message port */
      message_port_register_out (pmt::mp ("threshold"));

      /* Compute the decision threshold for the eigenvalue detector */
      d_eigen_threshold = eigen_threshold_estimation ();
      PHASMA_DEBUG("Eigenvalue threshold %f \n", d_eigen_threshold);

      d_fft = new fft::fft_complex (d_fft_size, true, 1);
      d_shift = (float*) volk_malloc ((d_subcarriers_num) * sizeof(float), 32);

      d_blackmann_harris_win = (float*) volk_malloc (d_fft_size * sizeof(float),
						     32);

      /* Calculation of Blackmann-Harris window */
      for (size_t i = 0; i < d_fft_size; i++) {
	d_blackmann_harris_win[i] = 0.35875
	    - 0.48829 * cos ((2 * M_PI * i) / (d_fft_size - 1))
	    + 0.14128 * cos ((4 * M_PI * i) / (d_fft_size - 1))
	    - 0.01168 * cos ((6 * M_PI * i) / (d_fft_size - 1));
      }

      d_psd = (float*) volk_malloc (d_fft_size * sizeof(float), 32);
      d_noise_floor = (float*) volk_malloc (d_fft_size * sizeof(float), 32);
      for (size_t i = 0; i < d_fft_size; i++) {
	d_noise_floor[i] = -200;
      }

    }

    eigenvalue_signal_detector_impl::~eigenvalue_signal_detector_impl ()
    {
      delete d_fft;
      volk_free (d_blackmann_harris_win);
      volk_free (d_psd);
      volk_free (d_noise_floor);
      volk_free (d_shift);
    }

    float
    eigenvalue_signal_detector_impl::eigen_threshold_estimation ()
    {
      float threshold;
      float p_1;
      float p_2;

      /* Maximum-minimum eigenvalue (MME) algorithm threshold */
      p_1 = pow (sqrt (d_eigen_samples) + sqrt (d_smoothing_factor), 2)
	  / pow (sqrt (d_eigen_samples) - sqrt (d_smoothing_factor), 2);

      p_2 = 1
	  + (pow (sqrt (d_eigen_samples) + sqrt (d_smoothing_factor),
		  (-1) * 2 / 3.0)
	      / pow (d_eigen_samples * d_smoothing_factor, 1 / 6.0)) * 0.59;

      threshold = p_1 / p_2;

      return threshold;
    }

    int
    eigenvalue_signal_detector_impl::work (
	int noutput_items, gr_vector_const_void_star &input_items,
	gr_vector_void_star &output_items)
    {

      const gr_complex *in = (const gr_complex*) input_items[0];

      float ratio;
      size_t noise_free = 0;

      switch (d_status)
	{
	case 1:
	  /*
	   * TODO: Actually the number of possible iterations is larger.
	   * Number of input samples is a multiple of fft_size.
	   * For now a kind of decimation is performed and only the first chunk of
	   * fft_size samples is handled.
	   */

	  size_t iter_num = d_fft_size / d_eigen_samples;
	  const gr_complex *win_ptr;
	  arma::cx_fmat X = arma::cx_fmat (d_eigen_samples, d_smoothing_factor);
	  arma::cx_fmat R;
	  arma::cx_fcolvec eigval;
	  std::vector<float> feigval (d_smoothing_factor);
	  // Handle all possible chunks of eigen_samples
	  for (size_t it = 0; it < iter_num; it++) {
	    // Last iteration should use previous samples
	    if (it == iter_num - 1) {
	      win_ptr =
		  &in[(it - 1) + d_eigen_samples - d_smoothing_factor - 1];
	    }
	    else {
	      win_ptr = &in[it];
	    }
	    memcpy (&X[0], win_ptr, d_eigen_samples * sizeof(gr_complex));
	    for (size_t i = 1; i < d_smoothing_factor; i++) {
	      memcpy (&X[(d_eigen_samples * i)], &win_ptr[i],
		      d_eigen_samples * sizeof(gr_complex));
	    }

	    R = arma::cov (X);
	    arma::eig_gen (eigval, R);
	    for (size_t t = 0; t < eigval.n_elem; t++) {
	      feigval[t] = std::abs (eigval (t));
	    }
	    std::sort (feigval.begin (), feigval.end (),
		       std::greater<float> ());
	    /* Maximum-minimum eigenvalue detection algorithm */
	    ratio = feigval[0] / feigval[feigval.size () - 1];

	    if (ratio <= d_eigen_threshold) {
	      noise_free++;
	      PHASMA_DEBUG("***** Signal Absent *****");
	      PHASMA_DEBUG("Eigenvalues num: %d", eigval.n_elem);
	      PHASMA_DEBUG("Min Eigenvalue: %f", feigval[feigval.size () - 1]);
	      PHASMA_DEBUG("Max Eigenvalue: %f", feigval[0]);
	      PHASMA_DEBUG("Ratio: %f", ratio);
	      PHASMA_DEBUG("Threshold: %f \n", d_eigen_threshold);
	    }
	    else {

	    }
	  }

	  if (d_noise_floor_est && noise_free == iter_num
	      && d_noise_floor_est_cnt > 0) {
	    PHASMA_DEBUG(
		"Ok sailor! Bring me a good bottle of noise-floor rum.");
	    noise_floor_estimation (in, noutput_items);
	    /* Perform shifting and cropping on squared magnitude max noise-floor*/
	    memcpy (&d_shift[0],
		    &d_noise_floor[(d_fft_size / 2) + d_num_samps_to_discard],
		    sizeof(float) * (d_num_samps_per_side));
	    memcpy (&d_shift[d_num_samps_per_side], &d_noise_floor[0],
		    sizeof(float) * (d_num_samps_per_side));
	    if (!d_noise_floor_est_cnt) {
	      d_noise_floor_est_cnt = 500;
//	      d_status = false;
	      message_port_pub (
		  pmt::mp ("threshold"),
		  pmt::make_blob (d_shift, d_subcarriers_num * sizeof(float)));
	    }
	  }
	  break;
	}

      return noutput_items;

    }

    void
    eigenvalue_signal_detector_impl::noise_floor_estimation (
	const gr_complex* in, int available_items)
    {

      d_noise_floor_est_cnt--;

      /* FIX: Decide whether to apply window, check if need unaligned version */

      memcpy(&d_fft->get_inbuf ()[0], in, d_fft_size*sizeof(gr_complex));
      d_fft->execute ();

      /* De-normalize with the size of wide FFT */
      volk_32fc_s32fc_multiply_32fc (d_fft->get_outbuf (), d_fft->get_outbuf (),
				     d_norm_factor, d_fft_size);

      /* Calculate abs^2  (no dB scale) */
      volk_32fc_magnitude_squared_32f (d_psd, d_fft->get_outbuf (), d_fft_size);

      /* Get maximum for each sub-carrier*/
      for (size_t b = 0; b < d_fft_size; b++) {
	d_noise_floor[b] = boost::minmax (d_noise_floor[b], d_psd[b]).get<1> ();
      }

    }

  } /* namespace phasma */
} /* namespace gr */

