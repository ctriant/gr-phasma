/* -*- c++ -*- */
/* 
 * Copyright 2016 <+YOU OR YOUR COMPANY+>.
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

#ifndef INCLUDED_PHASMA_EIGENVALUE_SIGNAL_DETECTOR_H
#define INCLUDED_PHASMA_EIGENVALUE_SIGNAL_DETECTOR_H

#include <phasma/api.h>
#include <gnuradio/sync_decimator.h>

namespace gr {
namespace phasma {

/*!
 * \brief <+description of block+>
 * \ingroup phasma
 *
 */
class PHASMA_API eigenvalue_signal_detector: virtual public gr::sync_block {
public:
	typedef boost::shared_ptr<eigenvalue_signal_detector> sptr;

	/*!
	 * \brief Return a shared_ptr to a new instance of phasma::eigenvalue_signal_detector.
	 *
	 * To avoid accidental use of raw pointers, phasma::eigenvalue_signal_detector's
	 * constructor is in a private implementation
	 * class. phasma::eigenvalue_signal_detector::make is the public interface for
	 * creating new instances.
	 */
	static sptr
	make(size_t eigen_samples, size_t smoothing_factor, double pfa,
			size_t fft_size, float sampling_rate, float actual_bw,
			size_t noise_floor_est_cnt);
};

} // namespace phasma
} // namespace gr

#endif /* INCLUDED_PHASMA_EIGENVALUE_SIGNAL_DETECTOR_H */

