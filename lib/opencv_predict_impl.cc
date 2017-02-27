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
#include <pmt/pmt.h>
#include "opencv_predict_impl.h"

namespace gr
{
  namespace phasma
  {

    opencv_predict::sptr
    opencv_predict::make (const size_t input_multiplier,
			  const size_t classifier_type,
			  const size_t npredictors, const size_t ninport,
			  const std::vector<uint16_t> &labels,
			  const std::string filename)
    {
      return gnuradio::get_initial_sptr (
	  new opencv_predict_impl (input_multiplier, classifier_type,
				   npredictors, ninport, labels, filename));
    }

    /*
     * The private constructor
     */
    opencv_predict_impl::opencv_predict_impl (const size_t input_multiplier,
					      const size_t classifier_type,
					      const size_t npredictors,
					      const size_t ninport,
					      const std::vector<uint16_t> &labels,
					      const std::string filename) :
	    gr::sync_block (
		"opencv_predict",
		gr::io_signature::make (1, ninport, sizeof(float)),
		gr::io_signature::make (0, 0, 0)),
	    d_npredictors (npredictors),
	    d_classifier_type (classifier_type),
	    d_input_multiplier (input_multiplier),
	    d_ninport (ninport),
	    d_predictors (cv::Mat (1, npredictors, CV_32F)),
	    d_labels (cv::Mat (1, ninport, CV_32F)),
	    d_filename (filename),
	    d_port_label(d_ninport)
    {

      message_port_register_out (pmt::intern ("classification"));
      set_output_multiple (d_input_multiplier * d_npredictors);

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

      d_input = (float*) malloc (d_npredictors * sizeof(float));
      bind_port_label(labels);

    }

    /*
     * Our virtual destructor.
     */
    opencv_predict_impl::~opencv_predict_impl ()
    {
    }

    int
    opencv_predict_impl::work (int noutput_items,
			       gr_vector_const_void_star &input_items,
			       gr_vector_void_star &output_items)
    {
      const float *in = (const float *) input_items[0];

      size_t available_observations = noutput_items / d_npredictors;

      for (size_t i = 0; i < available_observations; i++) {
	for (size_t n = 0; n < d_ninport; n++) {

	  in = (const float*) input_items[n];

	  /* Insert new dataset row */
	  /* TODO: Handle complex */
	  d_predictors = cv::Mat (1, d_npredictors, CV_32F, d_input);
	  memcpy (d_predictors.ptr (), &in[i * d_npredictors],
		  d_npredictors * sizeof(float));
	  d_labels.at<float> (0, 0) = d_port_label[n];
	  d_data = cv::ml::TrainData::create (d_predictors, cv::ml::ROW_SAMPLE,
					      d_labels);
	  d_data->setTrainTestSplitRatio (0);

	  cv::Mat predict_labels;
	  pmt::pmt_t pmt_msg = pmt::make_dict ();
	  switch (d_classifier_type)
	    {
	    case RANDOM_FOREST:
	      {
		reinterpret_cast<cv::Ptr<cv::ml::RTrees>&> (d_model)->predict (
		    d_data->getSamples (), predict_labels);
		pmt_msg = pmt::dict_add (
		    pmt_msg, pmt::string_to_symbol ("Original"),
		    pmt::from_float (d_labels.at<float> (0)));
		pmt_msg = pmt::dict_add (
		    pmt_msg, pmt::string_to_symbol ("Predicted"),
		    pmt::from_float (predict_labels.at<float> (0)));
		message_port_pub (pmt::intern ("classification"), pmt_msg);
		break;
	      }
	    default:
	      {
		break;
	      }
	    }
	}
      }
      return noutput_items;
    }

    void
    opencv_predict_impl::bind_port_label (
	const std::vector<uint16_t> &labels)
    {
      d_port_label = labels;
    }

  } /* namespace phasma */
} /* namespace gr */

