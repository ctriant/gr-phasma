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

#ifndef INCLUDED_PHASMA_JAGA_H
#define INCLUDED_PHASMA_JAGA_H

#include <phasma/api.h>
#include <fftw3.h>

#define JAGA_FEATURES_NUM 6

namespace gr
{
  namespace phasma
  {
    namespace featureset
    {

      /*!
       * \brief <+description+>
       *
       */
      class PHASMA_API jaga
      {
      public:
	jaga (size_t samples_num);
	~jaga ();

	void
	generate (const gr_complex* in);

	size_t
	get_features_num () const;

	float*
	get_outbuf () const;

      private:
	float*
	compute_instant_amp (const gr_complex* in);
	float
	compute_max_psd_instant_amp (const gr_complex* in);
	float
	compute_instant_amp_variance (const gr_complex* in);

	double
	compute_standard_deviation (std::vector<float>* in);

	std::vector<float> d_tmp_angle;
	std::vector<float> d_tmp_angle_diff;
	std::vector<float> d_tmp_i;
	std::vector<float> d_tmp_q;
	std::vector<float> d_tmp_i_diff;
	std::vector<float> d_tmp_q_diff;

	float* d_outbuf;
	float* d_mean;
	float* d_stddev;
	float* d_abs;
	float* d_psd;

	uint16_t* d_max;

	float* d_fftw_in_buf;
	fftwf_complex* d_fftw_out_buf;

	// fft plan for channel measurements
	fftwf_plan d_fft;

	size_t d_samples_num;
	size_t d_features_num;

      };
    } // namespace featureset
  } // namespace phasma
} // namespace gr

#endif /* INCLUDED_PHASMA_JAGA_H */

