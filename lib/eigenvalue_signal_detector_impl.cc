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

#include <gnuradio/io_signature.h>
#include "eigenvalue_signal_detector_impl.h"

namespace gr
{
  namespace phasma
  {

    eigenvalue_signal_detector::sptr
    eigenvalue_signal_detector::make (size_t samples_num,
				      size_t oversampling_factor,
				      size_t smoothing_factor, double pfa,
				      uint8_t algo)
    {
      return gnuradio::get_initial_sptr(
	  new eigenvalue_signal_detector_impl(samples_num, oversampling_factor,
					      smoothing_factor, pfa, algo));
    }

    /*
     * The private constructor
     */
    eigenvalue_signal_detector_impl::eigenvalue_signal_detector_impl (
	size_t samples_num, size_t oversampling_factor, size_t smoothing_factor,
	double pfa, uint8_t algo) :
	    gr::sync_decimator(
		"eigenvalue_signal_detector",
		gr::io_signature::make(1, 1, sizeof(gr_complex)),
		gr::io_signature::make(1, 1, sizeof(int)),
		(samples_num * oversampling_factor) + smoothing_factor),
	    d_samples_num(samples_num),
	    d_oversampling_factor(oversampling_factor),
	    d_duration(samples_num * oversampling_factor),
	    d_smoothing_factor(smoothing_factor),
	    d_pfa(pfa),
	    d_algo(algo),
	    d_sample_vec(d_duration + d_smoothing_factor)
    {
      d_threshold = threshold_estimation(algo);
    }

    eigenvalue_signal_detector_impl::~eigenvalue_signal_detector_impl ()
    {
    }

    float
    eigenvalue_signal_detector_impl::threshold_estimation (uint8_t algo)
    {
      float threshold;
      float p_1;
      float p_2;

      /**
       * TODO: Replace constant multiplier with an argument for the inverse
       * second-order Tracy-Widom function
       */
      if (d_algo == 1) {
	/* Maximum-minimum eigenvalue (MME) algorithm threshold */
	p_1 = pow(
	    sqrt(d_duration) + sqrt(d_oversampling_factor * d_smoothing_factor),
	    2)
	    / pow(
		sqrt(d_duration)
		    - sqrt(d_oversampling_factor * d_smoothing_factor),
		2);

	p_2 = 1
	    + (pow(
		sqrt(d_duration)
		    + sqrt(d_oversampling_factor * d_smoothing_factor),
		(-1) * 2 / 3.0)
		/ pow(d_duration * d_oversampling_factor * d_smoothing_factor,
		      1 / 6.0)) * 0.59;

	threshold = p_1 / p_2;
      }
      else if (d_algo == 0) {
	/* Energy with minimum eigenvalue (EME) algorithm threshold */
	p_1 = 0.59 * sqrt(2 * d_duration)
	    + (d_duration * sqrt(d_oversampling_factor));

	p_2 = sqrt(d_oversampling_factor)
	    * pow(
		sqrt(d_duration)
		    - sqrt(d_oversampling_factor * d_smoothing_factor),
		2);

	threshold = p_1 / p_2;
      }
      return threshold;
    }

    int
    eigenvalue_signal_detector_impl::work (
	int noutput_items, gr_vector_const_void_star &input_items,
	gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex*) input_items[0];
      size_t *out = (size_t*) output_items[0];

      float ratio;

      memcpy(
	  &d_sample_vec[0],
	  &in[0],
	  (d_duration * d_oversampling_factor + d_smoothing_factor)
	      * sizeof(gr_complex));

      arma::cx_fmat X = arma::cx_fmat(d_duration * d_oversampling_factor,
				      d_smoothing_factor);
      for (int i = 0; i < d_smoothing_factor; i++) {
	memcpy(&X[d_duration * d_oversampling_factor * i], &in[i],
	       d_duration * d_oversampling_factor * sizeof(gr_complex));
      }

      arma::cx_fmat R = arma::cov(X);
      arma::cx_fcolvec eigval;
      arma::eig_gen(eigval, R);
      std::vector<float> feigval(eigval.n_elem);
      for (size_t t = 0; t < eigval.n_elem; t++) {
	feigval[t] = std::abs(eigval(t));
      }
      std::sort(feigval.begin(), feigval.end(), std::greater<float>());

      switch (d_algo)
	{
	case 1:
	  /* Maximum-minimum eigenvalue detection algorithm */
	  ratio = feigval[0] / feigval[feigval.size() - 1];
	  break;
	case 0:
	  /* TODO: Average energy-minimum eigenvalue detection algorithm */
	  break;
	default:
	  PHASMA_ERROR("Invalid detection algorithm");
	  exit(-1);
	}

      if (ratio > d_threshold) {
	PHASMA_LOG_INFO("***** Signal Detected *****");
	out[0] = 1;
      }
      else {
	PHASMA_LOG_INFO("===== Signal Absent =====");
	out[0] = 0;
      }
      PHASMA_LOG_INFO("Algorithm: %u", d_algo);
      PHASMA_LOG_INFO("Min Eigenvalue: %f", feigval[feigval.size() - 1]);
      PHASMA_LOG_INFO("Max Eigenvalue: %f", feigval[0]);
      PHASMA_LOG_INFO("Ratio: %f", ratio);
      PHASMA_LOG_INFO("Threshold: %f \n", d_threshold);

      return 1;
    }

  } /* namespace phasma */
} /* namespace gr */

