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

#ifndef INCLUDED_PHASMA_OPENCV_PREDICT_H
#define INCLUDED_PHASMA_OPENCV_PREDICT_H

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
    class PHASMA_API opencv_predict : virtual public gr::sync_block
    {
    public:
      typedef boost::shared_ptr<opencv_predict> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of phasma::opencv_predict.
       *
       * To avoid accidental use of raw pointers, phasma::opencv_predict's
       * constructor is in a private implementation
       * class. phasma::opencv_predict::make is the public interface for
       * creating new instances.
       */
      static sptr
      make (const size_t input_multiplier, const size_t classifier_type,
	    size_t npredictors, const size_t d_ninport,
	    const std::vector<uint16_t> &labels, const std::string filename);
    };

  } // namespace phasma
} // namespace gr

#endif /* INCLUDED_PHASMA_OPENCV_PREDICT_H */

