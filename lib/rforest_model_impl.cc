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
#include <volk/volk.h>
#include <phasma/log.h>
#include "rforest_model_impl.h"

namespace gr {
namespace phasma {

rforest_model::sptr rforest_model::make(const size_t npredictors,
		const size_t nobservations, size_t ninport, const size_t max_depth,
		const size_t min_sample_count, const size_t regression_accu,
		const uint8_t use_surrogates, const size_t max_categories,
		const uint8_t calc_var_importance, const size_t active_var_count,
		const size_t max_iter, const std::string filename) {
	return gnuradio::get_initial_sptr(
			new rforest_model_impl(npredictors, nobservations, ninport,
					max_depth, min_sample_count, regression_accu,
					use_surrogates, max_categories, calc_var_importance,
					active_var_count, max_iter, filename));
}

/*
 * The private constructor
 */
rforest_model_impl::rforest_model_impl(const size_t npredictors,
		const size_t nobservations, const size_t ninport,
		const size_t max_depth, const size_t min_sample_count,
		const size_t regression_accu, const uint8_t use_surrogates,
		const size_t max_categories, const uint8_t calc_var_importance,
		const size_t active_var_count, const size_t max_iter,
		const std::string filename) :
		gr::sync_block("rforest_model",
				gr::io_signature::make(1, ninport, sizeof(gr_complex)),
				gr::io_signature::make(0, 0, 0)), d_npredictors(
				npredictors * 2), // if complex multiply by 2 for I/Q predictors
		d_nobservations(nobservations), d_remaining(nobservations), d_ninport(
				ninport), d_max_depth(max_depth), d_min_sample_count(
				min_sample_count), d_regression_accu(regression_accu), d_use_surrogates(
				use_surrogates), d_max_categories(max_categories), d_calc_var_importance(
				calc_var_importance), d_active_var_count(active_var_count), d_max_iter(
				max_iter), d_filename(filename) {
	set_output_multiple(100 * d_npredictors);
	d_input = (gr_complex*) malloc(d_npredictors * sizeof(gr_complex));
	d_rfmodel = cv::ml::RTrees::create();
}

/*
 * Our virtual destructor.
 */
rforest_model_impl::~rforest_model_impl() {

}

static void train_and_print_errs(cv::Ptr<cv::ml::StatModel> model,
		const cv::Ptr<cv::ml::TrainData>& data) {
	bool ok = model->train(data);
	if (!ok) {
		PHASMA_ERROR("Training failed\n");
	} else {
		PHASMA_LOG_INFO("Train error: %f\n",
				model->calcError(data, false, cv::noArray()));
		PHASMA_LOG_INFO("Test error: %f\n", model->calcError(data, true, cv::noArray()));
	}
}

int rforest_model_impl::work(int noutput_items,
		gr_vector_const_void_star &input_items,
		gr_vector_void_star &output_items) {
	const gr_complex *in;
	size_t available_observations = noutput_items / d_npredictors;

	if (d_nobservations % d_ninport) {
		PHASMA_WARN(
				"Number of requested dataset observation should be multiple of number of inputs.");
	}

	PHASMA_DEBUG("Available samples: %d\n",noutput_items); 
	PHASMA_DEBUG("Available Observations: %d\n",available_observations); 
	PHASMA_DEBUG("Requested Observations: %d\n",d_nobservations);

	for (int i = 0; i < available_observations; i++) {
		for (int n = 0; n < d_ninport; n++) {

			in = (const gr_complex*) input_items[n];

			memcpy(d_input, &in[i * d_npredictors],
					d_npredictors * sizeof(gr_complex));

			/* Insert new dataset row */
			/* TODO: Handle complex */
			if (d_predictors.empty() && d_labels.empty()) {
				d_predictors = cv::Mat(1, d_npredictors, CV_32F, d_input);
				d_labels = cv::Mat(1, 1, CV_32F, n);
			} else {
				cv::vconcat(cv::Mat(1, d_npredictors, CV_32F, d_input),
						d_predictors, d_predictors);
				cv::vconcat(cv::Mat(1, 1, CV_32F, n), d_labels, d_labels);
			}
			d_remaining--;

			if (d_remaining == 0) {
				// TODO: Train model
				PHASMA_LOG_INFO("\n\n====== Dataset creation =====\n");
				d_train_data = cv::ml::TrainData::create(d_predictors,
						cv::ml::ROW_SAMPLE, d_labels);
				d_train_data->setTrainTestSplitRatio(0.85);

				PHASMA_LOG_INFO("Test/Train: %d/%d\n",d_train_data->getNTestSamples(),d_train_data->getNTrainSamples());

				PHASMA_LOG_INFO("====== Random Forest training =====\n");
				d_rfmodel->setMaxDepth(d_max_depth);
				d_rfmodel->setMinSampleCount(d_min_sample_count);
				d_rfmodel->setRegressionAccuracy(d_regression_accu);
				d_rfmodel->setUseSurrogates(d_use_surrogates);
				d_rfmodel->setMaxCategories(d_max_categories);
				d_rfmodel->setPriors(cv::Mat());
				d_rfmodel->setCalculateVarImportance(d_calc_var_importance);
				d_rfmodel->setActiveVarCount(d_active_var_count);
				d_rfmodel->setTermCriteria(
						cv::TermCriteria(cv::TermCriteria::MAX_ITER, d_max_iter,
								0));
				d_train_data->shuffleTrainTest();
				train_and_print_errs(d_rfmodel, d_train_data);
				d_rfmodel->save(d_filename);
				PHASMA_LOG_INFO("====== Model exported as xml =====\n");
				return WORK_DONE;
			}
		}
	}
	return noutput_items;
}

} /* namespace phasma */
} /* namespace gr */

