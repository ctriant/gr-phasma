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
#include <gnuradio/math.h>
#include <algorithm>
#include <numeric>
#include <pmt/pmt.h>
#include <json/json.h>
#include "opencv_predict_impl.h"

namespace gr
{
  namespace phasma
  {

    opencv_predict::sptr
    opencv_predict::make (const size_t classifier_type, const size_t data_type,
			  const size_t npredictors, const size_t nlabels,
			  const std::string filename)
    {
      return gnuradio::get_initial_sptr (
	  new opencv_predict_impl (classifier_type, data_type, npredictors,
				   nlabels, filename));
    }

    /*
     * The private constructor
     */
    opencv_predict_impl::opencv_predict_impl (const size_t classifier_type,
					      const size_t data_type,
					      const size_t npredictors,
					      const size_t nlabels,
					      const std::string filename) :
	    gr::block ("opencv_predict", gr::io_signature::make (0, 0, 0),
		       gr::io_signature::make (0, 0, 0)),
	    d_classifier_type (classifier_type),
	    d_data_type (data_type),
	    d_npredictors (npredictors),
	    d_labels (cv::Mat (1, nlabels, CV_32F)),
	    d_nlabels (nlabels),
	    d_predictors (cv::Mat (1, npredictors, CV_32F)),
	    d_filename (filename)
    {

      message_port_register_in (pmt::mp ("in"));
      message_port_register_out (pmt::mp ("classification"));

      switch (d_classifier_type)
	{
	case RANDOM_FOREST:
	  {
	    d_model = cv::Algorithm::load<cv::ml::RTrees> (d_filename);
	  }
	  break;
	default:
	  {
	    PHASMA_ERROR("Unsupported ML classifier");
	  }
	  break;
	}

      if (d_model.empty ()) {
	PHASMA_ERROR("Could not read the classifier ", d_filename);
      }

      /*  */
      d_trigger_thread = boost::shared_ptr<boost::thread> (
	  new boost::thread (
	      boost::bind (&opencv_predict_impl::msg_handler_trigger, this)));
    }

    /*
     * Our virtual destructor.
     */
    opencv_predict_impl::~opencv_predict_impl ()
    {
    }

    void
    opencv_predict_impl::msg_handler_trigger ()
    {

      pmt::pmt_t msg;
      pmt::pmt_t tuple;
      size_t curr_sig = 0;
      size_t available_samples = 0;
      size_t available_observations = 0;
      void* data;
      std::vector<float> predictions;
      std::vector<float> d_tmp_angle;
      std::vector<float> d_tmp_angle_diff;
      while (true) {
	try {
	  // Access the message queue
	  msg = delete_head_blocking (pmt::mp ("in"));
	  tuple = pmt::vector_ref (msg, curr_sig);
	  float d_tmp_pred[2];
	  switch (d_data_type)
	    {
	    case COMPLEX: {
	      data = (gr_complex *) pmt::blob_data (pmt::tuple_ref (tuple, 1));
	      available_samples = pmt::blob_length (pmt::tuple_ref (tuple, 1))
		  / sizeof(gr_complex);

	      //Angle std
	      for (size_t s = 0; s < available_samples; s++) {
		d_tmp_angle.push_back(gr::fast_atan2f (((gr_complex*)data)[s]));
	      }
	      double sum = std::accumulate (d_tmp_angle.begin (),
					    d_tmp_angle.end (), 0.0);
	      double mean = sum / d_tmp_angle.size ();

	      std::vector<double> diff (d_tmp_angle.size ());
	      std::transform (d_tmp_angle.begin (), d_tmp_angle.end (),
			      diff.begin (),
			      [mean](double x) {return x - mean;});
	      double sq_sum = std::inner_product (diff.begin (), diff.end (),
						  diff.begin (), 0.0);
	      double stdev = std::sqrt (sq_sum / d_tmp_angle.size ());
	      d_tmp_pred[0] = stdev;

	      //Angle diff std
	      for (size_t s = 0; s < available_samples; s = s + 2) {
		d_tmp_angle_diff.push_back (
		    gr::fast_atan2f (((gr_complex*)data)[s + 1])
			- gr::fast_atan2f (((gr_complex*)data)[s]));
	      }
	      sum = std::accumulate (d_tmp_angle_diff.begin (),
				     d_tmp_angle_diff.end (), 0.0);
	      mean = sum / d_tmp_angle_diff.size ();

	      std::vector<double> diff2 (d_tmp_angle_diff.size ());
	      std::transform (d_tmp_angle_diff.begin (),
			      d_tmp_angle_diff.end (), diff2.begin (),
			      [mean](double x) {return x - mean;});
	      sq_sum = std::inner_product (diff2.begin (), diff2.end (),
					   diff2.begin (), 0.0);
	      double stdev_diff = std::sqrt (sq_sum / d_tmp_angle_diff.size ());
	      d_tmp_pred[1] = stdev_diff;
	      d_tmp_angle_diff.clear ();
	      d_tmp_angle.clear();
	    }
	      break;
	    case FLOAT:
	      data = (float *) pmt::blob_data (pmt::tuple_ref (tuple, 1));
	      available_samples = pmt::blob_length (pmt::tuple_ref (tuple, 1))
		  / sizeof(float);
	      break;
	    }

	  /**
	   * Iterate through all available observations of data provided by
	   * the incoming tuple message
	   */
	  available_observations = 1;
	  for (size_t i = 0; i < available_observations; i++) {
	    /* Insert new dataset row */
	    d_predictors = cv::Mat (1, 2, CV_32F, d_tmp_pred);
	    std::cout << "Predictors: " << d_predictors << std::endl;
//	    memcpy (d_predictors.ptr (), &d_input[i * d_npredictors],
//		    d_npredictors * sizeof(float));
	    d_labels.at<float> (0, 0) = 0;
	    d_data = cv::ml::TrainData::create (d_predictors,
						cv::ml::ROW_SAMPLE, d_labels);
	    d_data->setTrainTestSplitRatio (0);

	    cv::Mat predict_labels;
	    switch (d_classifier_type)
	      {
	      case RANDOM_FOREST:
		{
		  predictions.push_back (
		      reinterpret_cast<cv::Ptr<cv::ml::RTrees>&> (d_model)->predict (
			  d_data->getSamples(), predict_labels));
		  break;
		}
	      default:
		{
		  break;
		}
	      }
	  }
	  Json::Value root;
	  Json::Reader reader;
	  bool parsedSuccess = reader.parse (
	      pmt::symbol_to_string (pmt::tuple_ref (tuple, 0)), root, false);

	  if (not parsedSuccess) {
	    // Report failures and their locations 
	    // in the document.
	    std::cout << "Failed to parse JSON" << std::endl
		<< reader.getFormatedErrorMessages () << std::endl;
	    return;
	  }

	  // Let's extract the array contained 
	  // in the root object
	  std::string final_dec = get_most_freq_decision (&predictions);
	  root["annotation"]["core:comment"] = final_dec;
	  for (size_t l = 0; l < predictions.size (); l++) {
	    std::cout << "Predictions: " << predictions[l] << std::endl;
	  }
	  std::cout << "Final: " << final_dec << std::endl;
	  message_port_pub (pmt::intern ("classification"),
			    pmt::string_to_symbol (root.toStyledString ()));
	  predictions.clear ();
	  // Go catch the next tuple of the incoming vector message
	  curr_sig++;
	}
	catch (pmt::out_of_range&) {
	  /* You are out of range so break */
	  curr_sig = 0;
	}
      }
    }

    std::string
    opencv_predict_impl::get_most_freq_decision (
	std::vector<float>* predictions)
    {
      size_t max = 0;
      size_t tmp = 0;
      float max_idx = 0;

      for (size_t i = 1; i <= d_nlabels; i++) {
	tmp = std::count (predictions->begin (), predictions->end (), i);
	if (tmp > max) {
	  max = tmp;
	  max_idx = i;
	}
      }

      switch ((int) max_idx)
	{
	case BPSK:
	  return "BPSK";
	  break;
	case QPSK:
	  return "QPSK";
	  break;
	case GMSK:
	  return "GMSK";
	  break;
	case QAM:
	  return "QAM";
	  break;
	default:
	  return "UNKNOWN";
	}
    }

  } /* namespace phasma */
} /* namespace gr */

