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
#include <gnuradio/fft/fft.h>

#define JAGA_FEATURES_NUM 14

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

	void
	set_samples_num (size_t samples_num);

      private:
	float
	compute_max_psd_instant_amp (gr_complex* in, size_t samples_num);
	
	float
	compute_instant_amp_variance (gr_complex* in, size_t samples_num);
	
	float
	compute_fft_power_variance (const gr_complex* in, float power);
	
	size_t d_spf[6] =
		  { 64, 128, 256, 512, 768, 1024 };
	
	std::vector<fft::fft_complex*> d_fft_plans;
	fft::fft_complex* d_fft;
	
	float* d_outbuf;
	float* d_mean;
	float* d_stddev;
	float* d_abs;
	float* d_psd;
	
	gr_complex* d_samples;
	uint16_t* d_max;

	size_t d_samples_num;
	size_t d_features_num;

      };
    } // namespace featureset
  } // namespace phasma
} // namespace gr

#endif /* INCLUDED_PHASMA_JAGA_H */

