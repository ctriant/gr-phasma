/* -*- c++ -*- */
/* 
 * Copyright 2017 Kostis Triantafyllakis.
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
#include <volk/volk.h>
#include <pmt/pmt.h>
#include "opencv_predict_impl.h"

namespace gr {
namespace phasma {

opencv_predict::sptr 
opencv_predict::make(const size_t classifier_type,
		const size_t data_type, const size_t npredictors, const size_t nlabels,
		const std::string filename) {
	return gnuradio::get_initial_sptr(
			new opencv_predict_impl(classifier_type, data_type, npredictors,
					nlabels, filename));
}

/*
 * The private constructor
 */
opencv_predict_impl::opencv_predict_impl(const size_t classifier_type,
		const size_t data_type, const size_t npredictors, const size_t nlabels,
		const std::string filename) :
		gr::block("opencv_predict", gr::io_signature::make(0, 0, 0),
				gr::io_signature::make(0, 0, 0)), d_classifier_type(
				classifier_type), d_data_type(data_type), d_npredictors(
				npredictors), d_labels (cv::Mat (1, nlabels, CV_32F)), d_nlabels(nlabels),
				d_predictors(cv::Mat(1, npredictors, CV_32F)), d_filename(
				filename) {

	message_port_register_in(pmt::mp("in"));
	message_port_register_out(pmt::mp("classification"));

	switch (d_classifier_type) {
	case RANDOM_FOREST: {
		d_model = cv::Algorithm::load<cv::ml::RTrees>(d_filename);
	}
		break;
	default: {
		PHASMA_ERROR("Unsupported ML classifier");
	}
		break;
	}

	if (d_model.empty()) {
		PHASMA_ERROR("Could not read the classifier ", d_filename);
	}

	// TODO: Fix hard-coded value. Mind buffer sizes cause it's a mess.
	d_input = new gr_complex[d_npredictors * 1000];
	d_input_re = (float*)volk_malloc(sizeof(float)*d_npredictors * 1000, 32);
	d_input_imag = (float*)volk_malloc(sizeof(float)*d_npredictors * 1000, 32);
	
	/*  */
	d_trigger_thread =
			boost::shared_ptr<boost::thread>(
					new boost::thread(
							boost::bind(
									&opencv_predict_impl::msg_handler_trigger,
									this)));
}

/*
 * Our virtual destructor.
 */
opencv_predict_impl::~opencv_predict_impl() {
	delete [] d_input;
	volk_free(d_input_re);
	volk_free(d_input_imag);
}

void 
opencv_predict_impl::msg_handler_trigger() {
	
	pmt::pmt_t msg;
	pmt::pmt_t tuple;
	size_t curr_sig = 0;
	size_t available_samples = 0;
	size_t available_observations = 0;
	void* data;
	std::vector<float> predictions;

	while (true) {
		try {
			// Access the message queue
			msg = delete_head_blocking(pmt::mp("in"));
			tuple = pmt::vector_ref(msg, curr_sig);
			std::cout<< pmt::tuple_ref(tuple, 0) << std::endl;
			/* TODO: Handle complex */
			switch (d_data_type) {
			case COMPLEX:
				data = (gr_complex *)pmt::blob_data(pmt::tuple_ref(tuple, 1));
				available_samples = pmt::blob_length(pmt::tuple_ref(tuple, 1))/sizeof(gr_complex);
				volk_32fc_deinterleave_32f_x2_a(d_input_re, d_input_imag, (gr_complex *)data,
						available_samples);
				
				// Finally we handle floats
				available_samples = available_samples * 2;
				
				// Convert iq samples into a interleaved vector
				for(size_t s=0; s<available_samples-1; s++) {
					d_input[s] = d_input_re[s];
					d_input[s+1] = d_input_re[s];
				}
				break;
			case FLOAT:
				data = (float *)pmt::blob_data(pmt::tuple_ref(tuple, 1));
				available_samples = pmt::blob_length(pmt::tuple_ref(tuple, 1))/sizeof(float);;
				memcpy(d_input, data, pmt::blob_length(pmt::tuple_ref(tuple, 1)));
				break;
			}
			
			/**
			 * Iterate through all available observations of data provided by
			 * the incoming tuple message
			 */
			available_observations = available_samples / d_npredictors;
			for (size_t i = 0; i < available_observations; i++) {
				/* Insert new dataset row */
				d_predictors = cv::Mat(1, d_npredictors, CV_32F, d_input);
				memcpy(d_predictors.ptr(), &d_input[i*d_npredictors],
						d_npredictors * sizeof(float));
				d_labels.at<float>(0, 0) = 0;
				d_data = cv::ml::TrainData::create(d_predictors,
						cv::ml::ROW_SAMPLE, d_labels);
				d_data->setTrainTestSplitRatio(0);

				cv::Mat predict_labels;
				switch (d_classifier_type) {
				case RANDOM_FOREST: {
					predictions.push_back(
							reinterpret_cast<cv::Ptr<cv::ml::RTrees>&>(d_model)->predict(
									d_data->getSamples(), predict_labels));
					break;
				}
				default: {
					break;
				}
				}
			}
			pmt::pmt_t pmt_msg = pmt::make_dict();
			pmt_msg = pmt::dict_add(pmt_msg, pmt::string_to_symbol("Predicted"),
					pmt::from_float(get_most_freq_decision(&predictions)));
			message_port_pub(pmt::intern("classification"), pmt_msg);
			predictions.clear();
			// Go catch the next tuple of the incoming vector message
			curr_sig++;
		} catch (pmt::out_of_range&) {
			/* You are out of range so break */
			std::cout<< "Out of range" << std::endl;
			curr_sig = 0;
		}
	}
}

float
opencv_predict_impl::get_most_freq_decision(std::vector<float>* predictions){
	size_t max = 0;
	size_t tmp = 0;
	float max_idx = 0;
	
	for (size_t i=0; i<d_nlabels; i++) {
		tmp = std::count(predictions->begin(), predictions->end(), i);
		if (tmp > max) {
			max = tmp;
			max_idx = i;
		}
	}
	return max_idx;
}


} /* namespace phasma */
} /* namespace gr */

