/* -*- c++ -*- */
/* 
 * Copyright 2017 Kostis Triantafyllakis - ctriant.
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

#include <iostream>
#include <gnuradio/io_signature.h>
#include <phasma/featuresets/jaga.h>
#include <volk/volk.h>
#include <gnuradio/math.h>
#include <complex>
#include <algorithm>
#include <numeric>

namespace gr
{
  namespace phasma
  {
    namespace featureset
    {

      jaga::jaga (size_t samples_num) :
	      d_samples_num (samples_num),
	      d_features_num (JAGA_FEATURES_NUM)
      {
	d_outbuf = new float[JAGA_FEATURES_NUM];

	d_fftw_in_buf = (float *) fftwf_malloc (d_samples_num * sizeof(float));

	d_fftw_out_buf = (fftwf_complex *) fftwf_malloc (
	    d_samples_num * sizeof(fftwf_complex));

	// create fft plan to be used for channel power measurements
	d_fft = fftwf_plan_dft_r2c_1d (d_samples_num, d_fftw_in_buf,
				       d_fftw_out_buf, FFTW_MEASURE);

	unsigned int alignment = volk_get_alignment ();
	d_mean = (float*) volk_malloc (sizeof(float), alignment);
	d_stddev = (float*) volk_malloc (sizeof(float), alignment);
	d_abs = (float*) volk_malloc (d_samples_num * sizeof(float), alignment);
	d_psd = (float*) volk_malloc (d_samples_num * sizeof(float), alignment);

	d_max = (uint16_t*) volk_malloc (sizeof(uint16_t), alignment);
      }

      jaga::~jaga ()
      {
	delete[] d_outbuf;
	fftwf_destroy_plan (d_fft);
	volk_free (d_mean);
	volk_free (d_stddev);
	volk_free (d_abs);
	volk_free (d_psd);
	volk_free (d_max);
	fftwf_free (d_fftw_in_buf);
	fftwf_free (d_fftw_out_buf);
      }

      void
      jaga::generate (const gr_complex* in)
      {
	memset (d_abs, 0, d_samples_num * sizeof(float));
	memset (d_fftw_in_buf, 0, d_samples_num * sizeof(float));
	memset (d_fftw_out_buf, 0, d_samples_num * sizeof(gr_complex));
	memset (d_psd, 0, d_samples_num * sizeof(float));
	memset (d_outbuf, 0, JAGA_FEATURES_NUM * sizeof(float));

	float inst_amp_var = compute_instant_amp_variance (in);
	float max_psd_inst_amp = compute_max_psd_instant_amp (in);
	d_outbuf[0] = inst_amp_var;
	d_outbuf[1] = max_psd_inst_amp;

	/*
	 * Standard deviation of angle, imaginary and real part
	 */
	for (size_t s = 0; s < d_samples_num; s++) {
	  d_tmp_angle.push_back (gr::fast_atan2f (in[s]));
	  d_tmp_i.push_back (in[s].imag ());
	  d_tmp_q.push_back (in[s].real ());
	}
	d_outbuf[2] = compute_standard_deviation (&d_tmp_angle);
	d_outbuf[3] = compute_standard_deviation (&d_tmp_i);
	d_outbuf[4] = compute_standard_deviation (&d_tmp_q);

	/*
	 * Standard deviation of angle, imaginary and real part differences
	 */
	for (size_t s = 0; s < d_samples_num; s = s + 2) {
	  d_tmp_angle_diff.push_back (
	      gr::fast_atan2f (in[s + 1]) - gr::fast_atan2f (in[s]));
	  d_tmp_i_diff.push_back (in[s + 1].imag () - in[s].imag ());
	  d_tmp_q_diff.push_back (in[s + 1].real () - in[s].real ());
	}

	d_outbuf[5] = compute_standard_deviation (&d_tmp_angle_diff);

	d_tmp_angle.clear ();
	d_tmp_i.clear ();
	d_tmp_q.clear ();
	d_tmp_angle_diff.clear ();
	d_tmp_i_diff.clear ();
	d_tmp_q_diff.clear ();
      }

      double
      jaga::compute_standard_deviation (std::vector<float>* in)
      {

	double sum;
	double mean;
	double sq_sum;
	double stdev;
	std::vector<double> diff (in->size ());
	sum = std::accumulate (in->begin (), in->end (), 0.0);
	mean = sum / in->size ();
	std::transform (in->begin (), in->end (), diff.begin (),
			[mean](double x) {return x - mean;});
	sq_sum = std::inner_product (diff.begin (), diff.end (), diff.begin (),
				     0.0);
	stdev = std::sqrt (sq_sum / in->size ());
	diff.clear ();

	return stdev;
      }

      float
      jaga::compute_max_psd_instant_amp (const gr_complex* in)
      {
	size_t sum = 0;
	float inst_amp = 0;

	for (size_t i = 0; i < d_samples_num; i++) {
	  d_fftw_in_buf[i] = (d_abs[i] / d_mean[0]) - 1;
	}
	fftwf_execute (d_fft);

	for (size_t i = 0; i < d_samples_num; i++) {
	  d_psd[i] = std::pow (
	      std::abs (reinterpret_cast<float*> (d_fftw_out_buf)[i]), 2);
	}

	volk_32f_index_max_16u (d_max, d_psd, d_samples_num);

	return d_psd[*d_max] / d_samples_num;
      }

      float
      jaga::compute_instant_amp_variance (const gr_complex* in)
      {
	for (size_t i = 0; i < d_samples_num; i++) {
	  d_abs[i] = std::abs (in[i]);
	}
	volk_32f_stddev_and_mean_32f_x2 (d_stddev, d_mean, d_abs,
					 d_samples_num);
	return std::pow (d_stddev[0], 2);
      }

      size_t
      jaga::get_features_num () const
      {
	return d_features_num;
      }

      float*
      jaga::get_outbuf () const
      {
	return d_outbuf;
      }

      void
      jaga::set_samples_num (size_t samples_num)
      {
        d_samples_num = samples_num;
      }

    } /* namespace featurest */
  } /* namespace phasma */
} /* namespace gr */
