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
#include <gnuradio/math.h>
#include <algorithm>
#include <numeric>
#include <volk/volk.h>
#include <phasma/log.h>
#include "rforest_model_impl.h"

namespace gr
{
  namespace phasma
  {

    rforest_model::sptr
    rforest_model::make (const size_t npredictors, const size_t nobservations,
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
	  new rforest_model_impl (npredictors, nobservations, ninport, labels,
				  max_depth, min_sample_count, regression_accu,
				  use_surrogates, max_categories,
				  calc_var_importance, active_var_count,
				  max_iter, filename));
    }

    /*
     * The private constructor
     */
    rforest_model_impl::rforest_model_impl (const size_t npredictors,
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
		"rforest_model",
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
      bind_port_label (labels);
    }

    /*
     * Our virtual destructor.
     */
    rforest_model_impl::~rforest_model_impl ()
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
    rforest_model_impl::work (int noutput_items,
			      gr_vector_const_void_star &input_items,
			      gr_vector_void_star &output_items)
    {
      const gr_complex *in;
      size_t available_observations = noutput_items / d_npredictors;

      if (d_nobservations % d_ninport) {
	PHASMA_WARN(
	    "Number of requested dataset observation should be multiple of number of inputs.");
      }

      double sum;
      double mean;
      double sq_sum;
      double stdev;
      double stdev_diff;

      float d_tmp_pred[4];
      std::vector<float> d_tmp_angle (d_npredictors);
      std::vector<float> d_tmp_i (d_npredictors);
      std::vector<float> d_tmp_q (d_npredictors);
      std::vector<float> d_tmp_angle_diff;
      std::vector<float> d_tmp_i_diff;
      std::vector<float> d_tmp_q_diff;

      for (size_t i = 0; i < available_observations; i++) {
	for (size_t n = 0; n < d_ninport; n++) {

	  in = (const gr_complex*) input_items[n];

	  memcpy (d_input, &in[i * d_npredictors],
		  d_npredictors * sizeof(gr_complex));

	  for (size_t s = 0; s < d_npredictors; s++) {
	    d_tmp_angle[s] = gr::fast_atan2f (d_input[s]);
	    d_tmp_i[s] = d_input[s].imag ();
	    d_tmp_q[s] = d_input[s].real ();
	  }
	  sum = std::accumulate (d_tmp_angle.begin (), d_tmp_angle.end (), 0.0);
	  mean = sum / d_tmp_angle.size ();

	  //Angle std
	  std::vector<double> diff (d_tmp_angle.size ());
	  std::transform (d_tmp_angle.begin (), d_tmp_angle.end (),
			  diff.begin (), [mean](double x) {return x - mean;});
	  sq_sum = std::inner_product (diff.begin (), diff.end (),
				       diff.begin (), 0.0);
	  stdev = std::sqrt (sq_sum / d_tmp_angle.size ());
	  d_tmp_pred[0] = stdev;

	  //I std
	  sum = std::accumulate (d_tmp_i.begin (), d_tmp_i.end (), 0.0);
	  mean = sum / d_tmp_i.size ();
	  std::vector<double> diff3 (d_tmp_i.size ());
	  std::transform (d_tmp_i.begin (), d_tmp_i.end (), diff3.begin (),
			  [mean](double x) {return x - mean;});
	  sq_sum = std::inner_product (diff3.begin (), diff3.end (),
				       diff3.begin (), 0.0);
	  stdev = std::sqrt (sq_sum / d_tmp_i.size ());
	  d_tmp_pred[2] = stdev;

	  //Q std
	  sum = std::accumulate (d_tmp_q.begin (), d_tmp_q.end (), 0.0);
	  mean = sum / d_tmp_q.size ();
	  std::vector<double> diff4 (d_tmp_q.size ());
	  std::transform (d_tmp_q.begin (), d_tmp_q.end (), diff4.begin (),
			  [mean](double x) {return x - mean;});
	  sq_sum = std::inner_product (diff4.begin (), diff4.end (),
				       diff4.begin (), 0.0);
	  stdev = std::sqrt (sq_sum / d_tmp_q.size ());
	  d_tmp_pred[3] = stdev;

	  //Angle diff std
	  for (size_t s = 0; s < d_npredictors; s = s + 2) {
	    d_tmp_angle_diff.push_back (
		gr::fast_atan2f (d_input[s + 1])
		    - gr::fast_atan2f (d_input[s]));
	    d_tmp_i_diff.push_back (
		d_input[s + 1].imag () - d_input[s].imag ());
	    d_tmp_q_diff.push_back (
		d_input[s + 1].real () - d_input[s].real ());
	  }
	  sum = std::accumulate (d_tmp_angle_diff.begin (),
				 d_tmp_angle_diff.end (), 0.0);
	  mean = sum / d_tmp_angle_diff.size ();

	  std::vector<double> diff2 (d_tmp_angle_diff.size ());
	  std::transform (d_tmp_angle_diff.begin (), d_tmp_angle_diff.end (),
			  diff2.begin (), [mean](double x) {return x - mean;});
	  sq_sum = std::inner_product (diff2.begin (), diff2.end (),
				       diff2.begin (), 0.0);
	  stdev_diff = std::sqrt (sq_sum / d_tmp_angle_diff.size ());
	  d_tmp_pred[1] = stdev_diff;
//
//	  //I diff std
//	  sum = std::accumulate (d_tmp_i_diff.begin (), d_tmp_i_diff.end (),
//				 0.0);
//	  mean = sum / d_tmp_i_diff.size ();
//
//	  std::vector<double> diff5 (d_tmp_i_diff.size ());
//	  std::transform (d_tmp_i_diff.begin (), d_tmp_i_diff.end (),
//			  diff5.begin (), [mean](double x) {return x - mean;});
//	  sq_sum = std::inner_product (diff5.begin (), diff5.end (),
//				       diff5.begin (), 0.0);
//	  stdev_diff = std::sqrt (sq_sum / d_tmp_i_diff.size ());
//	  d_tmp_pred[4] = stdev_diff;
//
//	  //Q diff std
//	  sum = std::accumulate (d_tmp_q_diff.begin (), d_tmp_q_diff.end (),
//				 0.0);
//	  mean = sum / d_tmp_q_diff.size ();
//
//	  std::vector<double> diff6 (d_tmp_q_diff.size ());
//	  std::transform (d_tmp_q_diff.begin (), d_tmp_q_diff.end (),
//			  diff6.begin (), [mean](double x) {return x - mean;});
//	  sq_sum = std::inner_product (diff6.begin (), diff6.end (),
//				       diff6.begin (), 0.0);
//	  stdev_diff = std::sqrt (sq_sum / d_tmp_q_diff.size ());
//	  d_tmp_pred[5] = stdev_diff;
//
	  d_tmp_angle_diff.clear ();
//	  d_tmp_i_diff.clear ();
//	  d_tmp_q_diff.clear ();

	  /* Insert new dataset row */
	  if (d_predictors.empty () && d_labels.empty ()) {
	    d_predictors = cv::Mat (1, 4, CV_32F, d_tmp_pred);
	    d_labels = cv::Mat (1, 1, CV_32F, d_port_label[n]);
	  }
	  else {
	    cv::vconcat (cv::Mat (1, 4, CV_32F, d_tmp_pred), d_predictors,
			 d_predictors);
	    cv::vconcat (cv::Mat (1, 1, CV_32F, d_port_label[n]), d_labels,
			 d_labels);
	  }
	  d_remaining--;

	  if (d_remaining == 0) {
	    // TODO: Train model
	    PHASMA_LOG_INFO("====== Dataset creation =====\n");
	    cv::Mat var_types (1, 4 + 1, CV_8UC1,
			       cv::Scalar (cv::ml::VAR_ORDERED));
	    var_types.at<uchar> (4) = cv::ml::VAR_CATEGORICAL;
	    d_train_data = cv::ml::TrainData::create (d_predictors,
						      cv::ml::ROW_SAMPLE,
						      d_labels, cv::noArray (),
						      cv::noArray (),
						      cv::noArray (),
						      var_types);
	    d_train_data->setTrainTestSplitRatio (0.85);

	    PHASMA_LOG_INFO("Test/Train: %d/%d\n",
			    d_train_data->getNTestSamples (),
			    d_train_data->getNTrainSamples ());

	    PHASMA_LOG_INFO("====== Random Forest training =====\n");
	    d_rfmodel->setMaxDepth (d_max_depth);
	    d_rfmodel->setMinSampleCount (d_min_sample_count);
	    d_rfmodel->setRegressionAccuracy (d_regression_accu);
	    d_rfmodel->setUseSurrogates (d_use_surrogates);
	    d_rfmodel->setMaxCategories (d_max_categories);
	    d_rfmodel->setPriors (cv::Mat ());
	    d_rfmodel->setCalculateVarImportance (d_calc_var_importance);
	    d_rfmodel->setActiveVarCount (d_active_var_count);
	    d_rfmodel->setTermCriteria (
		cv::TermCriteria (cv::TermCriteria::MAX_ITER, d_max_iter, 0));
	    d_train_data->shuffleTrainTest ();
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
    rforest_model_impl::bind_port_label (const std::vector<uint16_t> &labels)
    {
      d_port_label = labels;
    }

  } /* namespace phasma */
} /* namespace gr */

