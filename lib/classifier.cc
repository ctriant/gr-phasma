/* -*- c++ -*- */
/* 
 * Copyright 2017 <+YOU OR YOUR COMPANY+>.
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
#include <phasma/classifier.h>

namespace gr
{
  namespace phasma
  {

    classifier::classifier (size_t hist_size) :
	    d_pred_history (hist_size)
    {
    }

    classifier::~classifier ()
    {
    }

    void
    classifier::record_prediction (int predicted, int actual)
    {
      d_pred_history.push_back (std::make_pair (predicted, actual));
    }

    float
    classifier::calc_pred_accuracy ()
    {
      size_t counter = 0;
      for (size_t i=0; i < d_pred_history.size(); i++) {
	if (d_pred_history[i].first == d_pred_history[i].second) {
	  counter++;
	}
      }
      return (float)counter/d_pred_history.size();
    }

    void
    classifier::clear_pred_history() {
      d_pred_history.clear();
    }

  } /* namespace phasma */
} /* namespace gr */

