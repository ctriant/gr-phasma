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
#include <phasma/featuresets/dummy_featureset.h>
#include <gnuradio/math.h>
#include <algorithm>
#include <numeric>

namespace gr
{
  namespace phasma
  {
    namespace featureset
    {

      dummy_featureset::dummy_featureset (size_t samples_num) :
	      d_samples_num (samples_num),
	      d_features_num (DUMMY_FEATURES_NUM)
      {
	d_outbuf = new float[DUMMY_FEATURES_NUM];
      }

      dummy_featureset::~dummy_featureset ()
      {
	delete[] d_outbuf;
      }

      void
      dummy_featureset::generate (const gr_complex* in)
      {
	/*
	 * Standard deviation of angle, imaginary and real part
	 */
	for (size_t s = 0; s < d_samples_num; s++) {
	  d_tmp_angle.push_back (gr::fast_atan2f (in[s]));
	  d_tmp_i.push_back (in[s].imag ());
	  d_tmp_q.push_back (in[s].real ());
	}
	d_outbuf[0] = compute_standard_deviation (&d_tmp_angle);
	d_outbuf[1] = compute_standard_deviation (&d_tmp_i);
	d_outbuf[2] = compute_standard_deviation (&d_tmp_q);

	/*
	 * Standard deviation of angle, imaginary and real part differences
	 */
	for (size_t s = 0; s < d_samples_num; s = s + 2) {
	  d_tmp_angle_diff.push_back (
	      gr::fast_atan2f (in[s + 1]) - gr::fast_atan2f (in[s]));
	  d_tmp_i_diff.push_back (in[s + 1].imag () - in[s].imag ());
	  d_tmp_q_diff.push_back (in[s + 1].real () - in[s].real ());
	}

	d_outbuf[3] = compute_standard_deviation (&d_tmp_angle_diff);

	d_tmp_angle.clear();
	d_tmp_i.clear();
	d_tmp_q.clear();
	d_tmp_angle_diff.clear();
	d_tmp_i_diff.clear();
	d_tmp_q_diff.clear();
      }

      double
      dummy_featureset::compute_standard_deviation (std::vector<float>* in)
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

      size_t
      dummy_featureset::get_features_num () const
      {
	return d_features_num;
      }

      float*
      dummy_featureset::get_outbuf () const
      {
	return d_outbuf;
      }

    } /* namespace featurest */
  } /* namespace phasma */
} /* namespace gr */
