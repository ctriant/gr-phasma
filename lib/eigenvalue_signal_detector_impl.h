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

#ifndef INCLUDED_PHASMA_EIGENVALUE_SIGNAL_DETECTOR_IMPL_H
#define INCLUDED_PHASMA_EIGENVALUE_SIGNAL_DETECTOR_IMPL_H

#include <phasma/eigenvalue_signal_detector.h>
#include <gnuradio/fft/fft.h>

namespace gr {
namespace phasma {

class eigenvalue_signal_detector_impl: public eigenvalue_signal_detector {
private:
	size_t d_eigen_samples;
	size_t d_smoothing_factor;
	
	/* The FFT size */       
	const size_t d_fft_size;

	double d_pfa;
	
	/* Decision threshold of eigenvalue-based method */       
	float d_eigen_threshold;
	
	/* Operational sampling rate */
	const double d_sampling_rate;
	
    /* The available bandwidth in MHz */
	float d_bw;
	
    /* Number of samples to discard per side in the observed spectrum band*/
	size_t d_num_samps_to_discard; 
	
	/* Number of samples per side in the observed spectrum band */
	size_t d_num_samps_per_side;
	
    /* Number of sub-carriers */
	size_t d_subcarriers_num;
	
    /* Blackmann-Harris window */
	float* d_blackmann_harris_win; 
	
	/* Vector to store the estimated PSD */
	float* d_psd; 
	
	/* Auxiliary FFT vector */
	fft::fft_complex* d_fft; 
	
	 /* Auxiliary vector to store FFT shift */
	float* d_shift;
	
    /* Vector that holds the noise-floor estimation */
	float* d_noise_floor;
	
	/* Normalization factor used in the calculation of the PSD */
	gr_complex d_norm_factor;
	
	/* The number of requested noise-floor estimations */
	size_t d_noise_floor_est_cnt;
	
	bool d_noise_floor_est;
	
	static bool sort_using_greater_than(float u, float v) {
		return u > v;
	}

	float
	eigen_threshold_estimation();
	
	void 
	noise_floor_estimation(const gr_complex* in, int available_items);

public:
	eigenvalue_signal_detector_impl(size_t eigen_samples,
									size_t smoothing_factor, 
									double pfa,
									size_t fft_size,
									float sampling_rate,
									float actual_bw,
									size_t noise_floor_est_cnt);
	
	~eigenvalue_signal_detector_impl();

	int
	work(int noutput_items, gr_vector_const_void_star &input_items,
			gr_vector_void_star &output_items);
};

} // namespace phasma
} // namespace gr

#endif /* INCLUDED_PHASMA_EIGENVALUE_SIGNAL_DETECTOR_IMPL_H */

