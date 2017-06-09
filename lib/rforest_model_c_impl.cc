/* -*- c++ -*- */
/* 
 * Copyright 2017 Kostis Triantafyllakis - ctriant.
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
#include "rforest_model_c_impl.h"

namespace gr
{
  namespace phasma
  {

    rforest_model_c::sptr
    rforest_model_c::make (const size_t npredictors, const size_t nobservations,
			 size_t ninport, const std::vector<uint16_t> &labels,
			 const size_t max_depth, const size_t min_sample_count,
			 const size_t regression_accu,
			 const uint8_t use_surrogates,
			 const size_t max_categories,
			 const uint8_t calc_var_importance,
			 const size_t active_var_count, const size_t max_iter,
			 const std::string filename)
    {
      return gnuradio::get_initial_sptr (
	  new rforest_model_c_impl (npredictors, nobservations, ninport, labels,
				  max_depth, min_sample_count, regression_accu,
				  use_surrogates, max_categories,
				  calc_var_importance, active_var_count,
				  max_iter, filename));
    }

    /*
     * The private constructor
     */
    rforest_model_c_impl::rforest_model_c_impl (const size_t npredictors,
					    const size_t nobservations,
					    const size_t ninport,
					    const std::vector<uint16_t> &labels,
					    const size_t max_depth,
					    const size_t min_sample_count,
					    const size_t regression_accu,
					    const uint8_t use_surrogates,
					    const size_t max_categories,
					    const uint8_t calc_var_importance,
					    const size_t active_var_count,
					    const size_t max_iter,
					    const std::string filename) :
	    gr::sync_block (
		"rforest_model_c",
		gr::io_signature::make (1, ninport, sizeof(gr_complex)),
		gr::io_signature::make (0, 0, 0)),
	    d_npredictors (npredictors),
	    d_nobservations (nobservations),
	    d_remaining (nobservations),
	    d_ninport (ninport),
	    d_max_depth (max_depth),
	    d_min_sample_count (min_sample_count),
	    d_regression_accu (regression_accu),
	    d_use_surrogates (use_surrogates),
	    d_max_categories (max_categories),
	    d_calc_var_importance (calc_var_importance),
	    d_active_var_count (active_var_count),
	    d_max_iter (max_iter),
	    d_filename (filename),
	    d_port_label (d_ninport)
    {
      set_output_multiple (100 * d_npredictors);
      d_input = (gr_complex*) malloc (d_npredictors * sizeof(gr_complex));
      d_rfmodel = cv::ml::RTrees::create ();
      d_featurset = new featureset::jaga(d_npredictors);
      bind_port_label (labels);
    }

    /*
     * Our virtual destructor.
     */
    rforest_model_c_impl::~rforest_model_c_impl ()
    {

    }

    static void
    train_and_print_errs (cv::Ptr<cv::ml::StatModel> model,
			  const cv::Ptr<cv::ml::TrainData>& data)
    {
      bool ok = model->train (data);
      if (!ok) {
	PHASMA_ERROR("Training failed\n");
      }
      else {
	PHASMA_LOG_INFO("Train error: %f\n",
			model->calcError (data, false, cv::noArray ()));
	PHASMA_LOG_INFO("Test error: %f\n",
			model->calcError (data, true, cv::noArray ()));
      }
    }

    int
    rforest_model_c_impl::work (int noutput_items,
			      gr_vector_const_void_star &input_items,
			      gr_vector_void_star &output_items)
    {
      const gr_complex *in;
      size_t available_observations = noutput_items / d_npredictors;

      if (d_nobservations % d_ninport) {
	PHASMA_WARN(
	    "Number of requested dataset observation should be multiple of number of inputs.");
      }

      for (size_t i = 0; i < available_observations; i++) {
	for (size_t n = 0; n < d_ninport; n++) {

	  in = (const gr_complex*) input_items[n];
	  memcpy (d_input, &in[i * d_npredictors],
		  d_npredictors * sizeof(gr_complex));

	  d_featurset->generate((const gr_complex*)d_input);

	  /* Insert new dataset row */
	  if (d_predictors.empty () && d_labels.empty ()) {
	    std::cout << "hey" << std::endl;
	    d_predictors = cv::Mat (1, d_featurset->get_features_num(), CV_32FC1, d_featurset->get_outbuf());
	    d_labels = cv::Mat (1, 1, CV_32F, d_port_label[n]);
	  }
	  else {
	    cv::vconcat (cv::Mat (1, d_featurset->get_features_num(), CV_32FC1, d_featurset->get_outbuf()), d_predictors,
			 d_predictors);
	    cv::vconcat (cv::Mat (1, 1, CV_32F, d_port_label[n]), d_labels,
			 d_labels);
	  }
	  d_remaining--;
	  PHASMA_LOG_INFO("Sample: %f %f %f\n", d_predictors.at<float>(i,0), d_predictors.at<float>(i,1), d_labels.at<float>(0));
	  if (d_remaining == 0) {
	    // TODO: Train model
	    PHASMA_LOG_INFO("====== Dataset creation =====\n");
	    cv::Mat var_types (1, d_featurset->get_features_num() + 1, CV_8UC1,
			       cv::Scalar (cv::ml::VAR_ORDERED));
	    var_types.at<uchar> (d_featurset->get_features_num()) = cv::ml::VAR_CATEGORICAL;
	    d_train_data = cv::ml::TrainData::create (d_predictors,
						      cv::ml::ROW_SAMPLE,
						      d_labels, cv::noArray (),
						      cv::noArray (),
						      cv::noArray (),
						      var_types);
//	    d_train_data->shuffleTrainTest();
	    d_train_data->setTrainTestSplitRatio (0.95);

//	    cv::Mat test_samples = d_train_data->getTestSamples();
//	    cv::Mat train_samples = d_train_data->getTrainSamples();
//	    cv::Mat test_labels = d_train_data->getResponses();
//	    cv::Mat samples = d_train_data->getSamples();
//
//	    PHASMA_DEBUG("Test samples:");
//	    print_opencv_mat(&samples);
//	    PHASMA_DEBUG("Test responses:");
//	    print_opencv_mat(&test_labels);

	    PHASMA_LOG_INFO("Test/Train: %d/%d\n",
			    d_train_data->getNTestSamples (),
			    d_train_data->getNTrainSamples ());

	    PHASMA_LOG_INFO("====== Random Forest training =====\n");
	    d_rfmodel->setMaxDepth (d_max_depth);
	    d_rfmodel->setMinSampleCount (d_min_sample_count);
	    d_rfmodel->setRegressionAccuracy (0.01);
	    d_rfmodel->setUseSurrogates (d_use_surrogates);
	    d_rfmodel->setMaxCategories (d_max_categories);
	    d_rfmodel->setPriors (cv::Mat ());
	    d_rfmodel->setCalculateVarImportance (d_calc_var_importance);
	    d_rfmodel->setActiveVarCount (d_active_var_count);
	    d_rfmodel->setTermCriteria (
		cv::TermCriteria (cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, d_max_iter, 0.1));
	    train_and_print_errs (d_rfmodel, d_train_data);
	    d_rfmodel->save (d_filename);
	    PHASMA_LOG_INFO("====== Model exported as xml =====\n");
	    return WORK_DONE;
	  }
	}
      }
      return noutput_items;
    }

    void
    rforest_model_c_impl::bind_port_label (const std::vector<uint16_t> &labels)
    {
      d_port_label = labels;
    }

    void
    rforest_model_c_impl::print_opencv_mat (cv::Mat* mat)
    {
      for (size_t idr = 0; idr < (size_t)mat->rows; idr++) {
	for (size_t idc = 0; idc < (size_t)mat->cols; idc++) {
	  printf ("%f ", mat->at<float> (idr, idc));
	}
	printf ("\n");
      }
    }

  } /* namespace phasma */
} /* namespace gr */

