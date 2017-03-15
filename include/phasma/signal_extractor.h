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

#ifndef INCLUDED_PHASMA_SIGNAL_EXTRACTOR_H
#define INCLUDED_PHASMA_SIGNAL_EXTRACTOR_H

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
    class PHASMA_API signal_extractor : virtual public gr::sync_block
    {
    public:
      typedef boost::shared_ptr<signal_extractor> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of phasma::signal_extractor.
       *
       * To avoid accidental use of raw pointers, phasma::signal_extractor's
       * constructor is in a private implementation
       * class. phasma::signal_extractor::make is the public interface for
       * creating new instances.
       */
      static sptr
      make (float samp_rate, float channel_bw, size_t ifft_size,
	    const std::vector<gr_complex> &taps, const float silence_guardband,
	    float center_freq, float signal_duration, float min_sig_bw, 
	    float threshold_db, float threshold_margin_db, size_t sig_num);
    };

  } // namespace phasma
} // namespace gr

#endif /* INCLUDED_PHASMA_SIGNAL_EXTRACTOR_H */

