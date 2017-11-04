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
#include <math.h>
#include <cmath>

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

	// create fft plans to be used for channel power measurements
	for (size_t i = 0; i < 6; i++) {
	  samples_num = d_spf[i];
	  d_fft_plans.push_back (new fft::fft_complex (samples_num, true, 1));
	}

	d_fft = new fft::fft_complex (samples_num, true, 1);

	unsigned int alignment = volk_get_alignment ();
	d_mean = (float*) volk_malloc (sizeof(float), alignment);
	d_stddev = (float*) volk_malloc (sizeof(float), alignment);
	d_abs = (float*) volk_malloc (d_samples_num * sizeof(float), alignment);
	d_psd = (float*) volk_malloc (d_samples_num * sizeof(float), alignment);
	d_power = (gr_complex*) volk_malloc (d_samples_num * sizeof(gr_complex), alignment);
	d_max = (uint16_t*) volk_malloc (sizeof(uint16_t), alignment);
	d_samples = (gr_complex*) volk_malloc (
	    d_samples_num * sizeof(gr_complex), alignment);
      }

      jaga::~jaga ()
      {
	delete[] d_outbuf;
	for (size_t i = 0; i < 6; i++) {
	  delete d_fft_plans[i];
	}
	volk_free (d_mean);
	volk_free (d_stddev);
	volk_free (d_abs);
	volk_free (d_psd);
	volk_free (d_max);
	volk_free (d_samples);
      }

      void
      jaga::generate (const gr_complex* in)
      {
	memset (d_abs, 0, d_samples_num * sizeof(float));
	memset (d_psd, 0, d_samples_num * sizeof(float));
	memset (d_outbuf, 0, JAGA_FEATURES_NUM * sizeof(float));
	memset (d_samples, 0, d_samples_num * sizeof(float));

	d_mean[0] = 0;
	d_stddev[0] = 0;

	size_t feature_cnt = 0;
	size_t samples_num;

	for (size_t i = 0; i < 6; i++) {
	  samples_num = d_spf[i];
	  memcpy (d_samples, in, samples_num * sizeof(gr_complex));
	  d_outbuf[feature_cnt++] = compute_instant_amp_variance (d_samples,
								  samples_num);
	  d_outbuf[feature_cnt++] = compute_max_psd_instant_amp (d_samples,
								 samples_num);
	}
	memcpy(d_power, in, samples_num * sizeof(gr_complex));
	d_outbuf[feature_cnt++] = compute_fft_power_variance (in, 1);
	d_outbuf[feature_cnt++] = compute_fft_power_variance (in, 1);
	d_outbuf[feature_cnt++] = compute_fft_power_variance (in, 1);
      }

      float
      jaga::compute_max_psd_instant_amp (gr_complex* in, size_t samples_num)
      {
	size_t idx = std::distance (d_spf,
				    std::find (d_spf, d_spf + 6, samples_num));
	volk_32fc_magnitude_32f (d_abs, in, samples_num);
	volk_32f_stddev_and_mean_32f_x2 (d_stddev, d_mean, d_abs, samples_num);
	for (size_t i = 0; i < samples_num; i++) {
	  d_fft_plans[idx]->get_inbuf ()[i] = (d_abs[i] / d_mean[0]) - 1;
	}

	d_fft_plans[idx]->execute ();
	
	volk_32fc_magnitude_squared_32f (
	    d_psd, (const gr_complex*) d_fft_plans[idx]->get_outbuf (),
	    samples_num);

	for (size_t i = 0; i < samples_num; i++) {
	  if (!std::isfinite (d_psd[i])) {
	    d_psd[i] = 0;
	  }
	}

	volk_32f_index_max_16u (d_max, d_psd, samples_num);
	return d_psd[d_max[0]] / samples_num;
      }

      float
      jaga::compute_instant_amp_variance (gr_complex* in, size_t samples_num)
      {
	volk_32fc_magnitude_32f (d_abs, in, samples_num);
	volk_32f_index_max_16u (d_max, d_abs, samples_num);
	volk_32f_s32f_multiply_32f (d_abs, (const float*) d_abs,
				    1 / d_abs[*d_max], samples_num);
	volk_32f_stddev_and_mean_32f_x2 (d_stddev, d_mean, d_abs, samples_num);
	return std::pow (d_stddev[0], 2);
      }

      float
      jaga::compute_fft_power_variance (const gr_complex* in, size_t iter)
      {
	// TODO: Change hardcoded value
	size_t samples_num = 1024;
	size_t idx = std::distance (d_spf,
				    std::find (d_spf, d_spf + 6, samples_num));
	for (size_t i=0; i<iter; i++) {
	  volk_32fc_x2_multiply_32fc (d_power, d_power, d_power, samples_num);
	}
	memcpy(d_fft->get_inbuf (), d_power, samples_num * sizeof(gr_complex));
	d_fft->execute ();
	memcpy (d_samples, &(d_fft->get_outbuf ())[samples_num / 2],
		(samples_num / 2) * sizeof(gr_complex));
	memcpy (&d_samples[samples_num / 2], d_fft->get_outbuf (),
		(samples_num / 2) * sizeof(gr_complex));
	volk_32fc_magnitude_32f (d_abs, d_samples, samples_num);
	volk_32f_stddev_and_mean_32f_x2 (d_stddev, d_mean, d_abs, samples_num);
	volk_32f_index_max_16u (d_max, d_abs, samples_num);
	return std::log10(std::pow (d_stddev[0], 2) / (d_abs[d_max[0]]+1e-20) + 1e-20);
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
