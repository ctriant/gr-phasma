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

#include <gnuradio/io_signature.h>
#include <phasma/featuresets/raw_iq_featureset.h>

namespace gr
{
  namespace phasma
  {
    namespace featureset
    {

      raw_iq_featureset::raw_iq_featureset (size_t samples_num) :
	      d_samples_num (samples_num),
	      d_features_num (d_samples_num)
      {
	d_outbuf = new float[d_features_num];
      }

      raw_iq_featureset::~raw_iq_featureset ()
      {
	delete[] d_outbuf;
      }

      void
      raw_iq_featureset::generate (const float* in)
      {
	memcpy (d_outbuf, in, d_features_num * sizeof(float));
      }

      size_t
      raw_iq_featureset::get_features_num () const
      {
	return d_features_num;
      }

      float*
      raw_iq_featureset::get_outbuf () const
      {
	return d_outbuf;
      }
    }
  } /* namespace phasma */
} /* namespace gr */

