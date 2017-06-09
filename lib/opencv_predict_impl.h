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

#ifndef INCLUDED_PHASMA_OPENCV_PREDICT_IMPL_H
#define INCLUDED_PHASMA_OPENCV_PREDICT_IMPL_H

#include <phasma/api.h>
#include <phasma/log.h>
#include <phasma/opencv_predict.h>
#include <phasma/featuresets/dummy_featureset.h>
#include <phasma/featuresets/raw_iq_featureset.h>
#include <phasma/featuresets/jaga.h>
#include <opencv2/ml.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <pthread.h>

namespace gr {
namespace phasma {

class opencv_predict_impl: public opencv_predict {
private:
	size_t d_npredictors;
	size_t d_nlabels;
	size_t d_classifier_type;
	size_t d_data_type;

	gr_complex* d_input;
	float* d_input_re;
	float* d_input_imag;
	
	cv::Mat d_predictors;
	cv::Ptr<cv::ml::TrainData> d_data;

	cv::Ptr<cv::ml::StatModel> d_model;
	cv::Mat d_labels;

	const std::string d_filename;
	
	featureset::jaga* d_featurset;

	boost::shared_ptr<boost::thread> d_trigger_thread;
	
	Json::Value d_root;
	
	const std::string d_metafile;

	void
	print_opencv_mat(cv::Mat* mat);

	std::string
	decode_decision(int decision);

public:
	opencv_predict_impl(const size_t classifier_type, const size_t data_type,
			const size_t npredictors, const size_t nlabels, 
			const std::string filename, const std::string metafile);
	~opencv_predict_impl();

	void 
	msg_handler_trigger();
};

} // namespace phasma
} // namespace gr

#endif /* INCLUDED_PHASMA_OPENCV_PREDICT_IMPL_H */

