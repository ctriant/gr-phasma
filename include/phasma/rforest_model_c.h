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

#ifndef INCLUDED_PHASMA_RFOREST_MODEL_C_H
#define INCLUDED_PHASMA_RFOREST_MODEL_C_H

#include <phasma/api.h>
#include <gnuradio/sync_block.h>

namespace gr
{
  namespace phasma
  {

    /*!
     * \brief <+description of block+>
     * \ingroup phasma
     *
     */
    class PHASMA_API rforest_model_c : virtual public gr::sync_block
    {
    public:
      typedef boost::shared_ptr<rforest_model_c> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of phasma::rforest_model_c.
       *
       * To avoid accidental use of raw pointers, phasma::rforest_model_c's
       * constructor is in a private implementation
       * class. phasma::rforest_model_c::make is the public interface for
       * creating new instances.
       */
      static sptr
      make (const size_t npredictors, const size_t nobservations,
	    const size_t ninport, const std::vector<uint16_t> &labels,
	    const size_t max_depth, const int min_sample_count,
	    const size_t regression_accu, const uint8_t use_surrogates,
	    const size_t max_categories, const uint8_t calc_var_importance,
	    const size_t active_var_count, const size_t max_iter,
	    const std::string filename);
    };

  } // namespace phasma
} // namespace gr

#endif /* INCLUDED_PHASMA_RFOREST_MODEL_C_H */

