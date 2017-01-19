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
#include "opencv_predict_impl.h"

namespace gr {
namespace phasma {

opencv_predict::sptr opencv_predict::make(const size_t input_multiplier,
		const size_t classifier_type, const size_t npredictors,
		const size_t ninport, const std::string filename) {
	return gnuradio::get_initial_sptr(
			new opencv_predict_impl(input_multiplier, classifier_type,
					npredictors, ninport, filename));
}

/*
 * The private constructor
 */
opencv_predict_impl::opencv_predict_impl(const size_t input_multiplier,
		const size_t classifier_type, const size_t npredictors,
		const size_t ninport, const std::string filename) :
		gr::sync_block("opencv_predict",
				gr::io_signature::make(1, ninport, sizeof(gr_complex)),
				gr::io_signature::make(0, 0, 0)), d_npredictors(npredictors), d_classifier_type(
				classifier_type), d_input_multiplier(input_multiplier), d_ninport(
				ninport), d_filename(filename) {
	
	message_port_register_out(pmt::intern("classification"));
	set_output_multiple(d_input_multiplier * d_npredictors);
	
	switch (d_classifier_type) {
	case RANDOM_FOREST: {
		d_model = cv::ml::RTrees::load<cv::ml::RTrees>(
				d_filename);
		std::cout << "Is trained: " << d_model->isTrained() << std::endl;
		std::cout << "Is classifier: " << d_model->isClassifier() << std::endl;
	}
		break;
	default: {
		PHASMA_ERROR("Unsupported ML classifier");
	}
		break;
	}
	
	if(d_model.empty()) {
		PHASMA_ERROR("Could not read the classifier ", d_filename);
	}
	
	d_input = (gr_complex*) malloc(d_npredictors * sizeof(gr_complex));

}

/*
 * Our virtual destructor.
 */
opencv_predict_impl::~opencv_predict_impl() {
}

int opencv_predict_impl::work(int noutput_items,
		gr_vector_const_void_star &input_items,
		gr_vector_void_star &output_items) {
	const gr_complex *in = (const gr_complex *) input_items[0];

	size_t available_observations = noutput_items / d_npredictors;

	for (int i = 0; i < available_observations; i++) {
		for (int n = 0; n < d_ninport; n++) {

			in = (const gr_complex*) input_items[n];

			memcpy(d_input, &in[i * d_npredictors],
					d_npredictors * sizeof(gr_complex));

			/* Insert new dataset row */
			/* TODO: Handle complex */
			if (d_predictors.empty() && d_labels.empty()) {
				d_predictors = cv::Mat(1, d_npredictors, CV_32F, d_input);
				d_labels = cv::Mat(1, 1, CV_32F, 1);
			}
			
			d_data = cv::ml::TrainData::create(d_predictors, cv::ml::ROW_SAMPLE,
					d_labels);
			d_data->setTrainTestSplitRatio(0);
			
			cv::Mat predict_labels;
			std::cout << d_model->predict(d_data->getSamples()) << std::endl;
			

			d_predictors.pop_back();
			d_labels.pop_back();

		}

	}
	return noutput_items;
}

} /* namespace phasma */
} /* namespace gr */

