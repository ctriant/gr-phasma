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

#ifndef INCLUDED_PHASMA_ANN_MODEL_C_H
#define INCLUDED_PHASMA_ANN_MODEL_C_H

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
  class PHASMA_API ann_model_c : virtual public gr::sync_block
  {
  public:
    typedef boost::shared_ptr<ann_model_c> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of phasma::ann_model_c.
     *
     * To avoid accidental use of raw pointers, phasma::ann_model_c's
     * constructor is in a private implementation
     * class. phasma::ann_model_c::make is the public interface for
     * creating new instances.
     */
    static sptr
    make (const size_t npredictors, const size_t nobservations,
        size_t ninport, const std::vector<uint16_t> &labels,
        size_t activation_func, double activation_a,
        double activation_b,
        const std::vector<uint16_t> &layer_sizes,
        size_t train_method, const std::string filename);
  };

}
 // namespace phasma
}// namespace gr

#endif /* INCLUDED_PHASMA_ANN_MODEL_C_H */

