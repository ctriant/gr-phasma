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

#ifndef INCLUDED_PHASMA_DUMMY_FEATURESET_H
#define INCLUDED_PHASMA_DUMMY_FEATURESET_H

#include <phasma/api.h>

#define DUMMY_FEATURES_NUM 4

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
      class PHASMA_API dummy_featureset
      {
      public:
	dummy_featureset (size_t samples_num);
	~dummy_featureset ();

	void
	generate (const gr_complex* in);

	size_t
	get_features_num () const;

	float*
	get_outbuf () const;

      private:
	double
	compute_standard_deviation(std::vector<float>* in);

	float* d_outbuf;

	size_t d_samples_num;
	size_t d_features_num;

	std::vector<float> d_tmp_angle;
	std::vector<float> d_tmp_angle_diff;
	std::vector<float> d_tmp_i;
	std::vector<float> d_tmp_q;
	std::vector<float> d_tmp_i_diff;
	std::vector<float> d_tmp_q_diff;

      };
    } // namespace featureset
  } // namespace phasma
} // namespace gr

#endif /* INCLUDED_PHASMA_DUMMY_FEATURESET_H */

