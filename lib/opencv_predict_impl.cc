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
#include <phasma/utils/sigMF.h>
#include "opencv_predict_impl.h"

namespace gr
{
  namespace phasma
  {

    opencv_predict::sptr
    opencv_predict::make (const size_t classifier_type, const size_t data_type,
			  const size_t npredictors, const size_t nlabels,
			  const std::string filename,
			  const std::string metafile)
    {
      return gnuradio::get_initial_sptr (
	  new opencv_predict_impl (classifier_type, data_type, npredictors,
				   nlabels, filename, metafile));
    }

    /*
     * The private constructor
     */
    opencv_predict_impl::opencv_predict_impl (const size_t classifier_type,
					      const size_t data_type,
					      const size_t npredictors,
					      const size_t nlabels,
					      const std::string filename,
					      const std::string metafile) :
	    gr::block ("opencv_predict", gr::io_signature::make (0, 0, 0),
		       gr::io_signature::make (0, 0, 0)),
	    d_classifier_type (classifier_type),
	    d_data_type (data_type),
	    d_npredictors (npredictors),
	    d_labels (cv::Mat (1, nlabels, CV_32F)),
	    d_nlabels (nlabels),
	    d_predictors (cv::Mat (1, npredictors, CV_32F)),
	    d_filename (filename),
	    d_metafile (metafile)
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
      int decision;

      double sum;
      double mean;
      double sq_sum;
      double stdev;
      double stdev_diff;

      std::vector<float> d_tmp_angle;
      std::vector<float> d_tmp_i;
      std::vector<float> d_tmp_q;
      std::vector<float> d_tmp_angle_diff;
      std::vector<float> d_tmp_i_diff;
      std::vector<float> d_tmp_q_diff;

      while (true) {
	try {
	  // Access the message queue
	  msg = delete_head_blocking (pmt::mp ("in"));
	  tuple = pmt::vector_ref (msg, curr_sig);
	  float d_tmp_pred[4];
	  switch (d_data_type)
	    {
	    case COMPLEX:
	      {
		data = (gr_complex *) pmt::blob_data (
		    pmt::tuple_ref (tuple, 1));
		available_samples = pmt::blob_length (pmt::tuple_ref (tuple, 1))
		    / sizeof(gr_complex);

		for (size_t s = 0; s < available_samples; s++) {
		  d_tmp_angle.push_back (
		      gr::fast_atan2f (((gr_complex*) data)[s]));
		  d_tmp_i.push_back (((gr_complex*) data)[s].imag ());
		  d_tmp_q.push_back (((gr_complex*) data)[s].imag ());
		}

		/* Standard deviation of complex angle */
		sum = std::accumulate (d_tmp_angle.begin (), d_tmp_angle.end (),
				       0.0);
		mean = sum / d_tmp_angle.size ();

		std::vector<double> diff (d_tmp_angle.size ());
		std::transform (d_tmp_angle.begin (), d_tmp_angle.end (),
				diff.begin (),
				[mean](double x) {return x - mean;});
		sq_sum = std::inner_product (diff.begin (), diff.end (),
					     diff.begin (), 0.0);
		stdev = std::sqrt (sq_sum / d_tmp_angle.size ());
		d_tmp_pred[0] = stdev;

		//I std
		sum = std::accumulate (d_tmp_i.begin (), d_tmp_i.end (), 0.0);
		mean = sum / d_tmp_i.size ();
		std::vector<double> diff3 (d_tmp_i.size ());
		std::transform (d_tmp_i.begin (), d_tmp_i.end (),
				diff3.begin (),
				[mean](double x) {return x - mean;});
		sq_sum = std::inner_product (diff3.begin (), diff3.end (),
					     diff3.begin (), 0.0);
		stdev = std::sqrt (sq_sum / d_tmp_i.size ());
		d_tmp_pred[2] = stdev;

		//Q std
		sum = std::accumulate (d_tmp_q.begin (), d_tmp_q.end (), 0.0);
		mean = sum / d_tmp_q.size ();
		std::vector<double> diff4 (d_tmp_q.size ());
		std::transform (d_tmp_q.begin (), d_tmp_q.end (),
				diff4.begin (),
				[mean](double x) {return x - mean;});
		sq_sum = std::inner_product (diff4.begin (), diff4.end (),
					     diff4.begin (), 0.0);
		stdev = std::sqrt (sq_sum / d_tmp_q.size ());
		d_tmp_pred[3] = stdev;

		/* Standard deviation of complex angle difference */
		//Angle diff std
		for (size_t s = 0; s < d_npredictors; s = s + 2) {
		  d_tmp_angle_diff.push_back (
		      gr::fast_atan2f (
			  ((gr_complex*) data)[s + 1]
			      - gr::fast_atan2f (((gr_complex*) data)[s])));
		  d_tmp_i_diff.push_back (
		      ((gr_complex*) data)[s + 1].imag ()
			  - ((gr_complex*) data)[s].imag ());
		  d_tmp_q_diff.push_back (
		      ((gr_complex*) data)[s + 1].real ()
			  - ((gr_complex*) data)[s].real ());
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
		stdev_diff = std::sqrt (sq_sum / d_tmp_angle_diff.size ());
		d_tmp_pred[1] = stdev_diff;
//
//		//I diff std
//		sum = std::accumulate (d_tmp_i_diff.begin (),
//				       d_tmp_i_diff.end (), 0.0);
//		mean = sum / d_tmp_i_diff.size ();
//
//		std::vector<double> diff5 (d_tmp_i_diff.size ());
//		std::transform (d_tmp_i_diff.begin (), d_tmp_i_diff.end (),
//				diff5.begin (),
//				[mean](double x) {return x - mean;});
//		sq_sum = std::inner_product (diff5.begin (), diff5.end (),
//					     diff5.begin (), 0.0);
//		stdev_diff = std::sqrt (sq_sum / d_tmp_i_diff.size ());
//		d_tmp_pred[4] = stdev_diff;
//
//		//Q diff std
//		sum = std::accumulate (d_tmp_q_diff.begin (),
//				       d_tmp_q_diff.end (), 0.0);
//		mean = sum / d_tmp_q_diff.size ();
//
//		std::vector<double> diff6 (d_tmp_q_diff.size ());
//		std::transform (d_tmp_q_diff.begin (), d_tmp_q_diff.end (),
//				diff6.begin (),
//				[mean](double x) {return x - mean;});
//		sq_sum = std::inner_product (diff6.begin (), diff6.end (),
//					     diff6.begin (), 0.0);
//		stdev_diff = std::sqrt (sq_sum / d_tmp_q_diff.size ());
//		d_tmp_pred[5] = stdev_diff;

		d_tmp_angle_diff.clear ();
		//d_tmp_i_diff.clear ();
		//d_tmp_q_diff.clear ();
		d_tmp_angle.clear ();
		d_tmp_q.clear ();
		d_tmp_i.clear ();
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
	    d_predictors = cv::Mat (1, 4, CV_32F, d_tmp_pred);
	    d_labels.at<float> (0, 0) = 0;
	    d_data = cv::ml::TrainData::create (d_predictors,
						cv::ml::ROW_SAMPLE, d_labels);
	    d_data->setTrainTestSplitRatio (0);

	    cv::Mat predict_labels;
	    switch (d_classifier_type)
	      {
	      case RANDOM_FOREST:
		{
		  decision =
		      reinterpret_cast<cv::Ptr<cv::ml::RTrees>&> (d_model)->predict (
			  d_data->getSamples (), predict_labels);
		  break;
		}
	      default:
		{
		  break;
		}
	      }
	  }

	  std::string final_dec = decode_decision (decision);

	  sigMF meta_msg = sigMF ("cf32", "./", "1.1.0");
	  ;
	  sigMF meta = sigMF ("cf32", "./", "1.1.0");
	  ;

	  meta_msg.parse_string (
	      pmt::symbol_to_string (pmt::tuple_ref (tuple, 0)), final_dec);

	  meta.parse_file (d_metafile);
	  meta.add_annotation (meta_msg.get_annotation ()[0]);
	  //meta.add_capture(meta_msg.get_capture()[0]);

	  /* Append output file */
	  std::ofstream outfile;
	  outfile.open (d_metafile, std::ios_base::in);
	  outfile << meta.toJSON ();
	  outfile.close ();

	  message_port_pub (
	      pmt::intern ("classification"),
	      pmt::string_to_symbol (meta_msg.getRoot ().toStyledString ()));
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
    opencv_predict_impl::decode_decision (int decision)
    {
      switch ((int) decision)
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
	case QAM16:
	  return "16QAM";
	  break;
	case PSK8:
	  return "8PSK";
	  break;
	case QAM64:
	  return "64QAM";
	  break;
	default:
	  return "UNKNOWN";
	}
    }

  } /* namespace phasma */
} /* namespace gr */

