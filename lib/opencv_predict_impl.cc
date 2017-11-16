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
                          const size_t history_size, bool debug_mode,
                          size_t active_mod, const std::vector<size_t> &labels,
                          const std::string filename,
                          const std::string metafile)
    {
      return gnuradio::get_initial_sptr (
          new opencv_predict_impl (classifier_type, data_type, npredictors,
                                   nlabels, history_size, debug_mode,
                                   active_mod, labels, filename, metafile));
    }

    /*
     * The private constructor
     */
    opencv_predict_impl::opencv_predict_impl (const size_t classifier_type,
                                              const size_t data_type,
                                              const size_t npredictors,
                                              const size_t nlabels,
                                              const size_t history_size,
                                              bool debug_mode,
                                              size_t active_mod,
                                              const std::vector<size_t> &labels,
                                              const std::string filename,
                                              const std::string metafile) :
            gr::block ("opencv_predict", gr::io_signature::make (0, 0, 0),
                       gr::io_signature::make (0, 0, 0)),
            classifier (history_size, nlabels, labels),
            d_classifier_type (classifier_type),
            d_data_type (data_type),
            d_npredictors (npredictors),
            d_nlabels (nlabels),
            d_labels (cv::Mat (1, nlabels, CV_32F)),
            d_history_size (history_size),
            d_debug_mode (debug_mode),
            d_active_mod (active_mod),
            d_predictors (cv::Mat (1, npredictors, CV_32F)),
            d_filename (filename),
            d_metafile (metafile)
    {

      message_port_register_in (pmt::mp ("in"));
      message_port_register_out (pmt::mp ("classification"));
      message_port_register_out (pmt::mp ("print"));

      if (d_debug_mode && labels.size () <= 0) {
        throw std::runtime_error ("opencv_predict: Prediction labels not set");
      }

      switch (d_classifier_type)
        {
        case RANDOM_FOREST:
          {
            d_model = cv::Algorithm::load < cv::ml::RTrees > (d_filename);
          }
          break;
        case ANN:
          {
            d_model = cv::Algorithm::load < cv::ml::ANN_MLP > (d_filename);
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

      d_featurset = new featureset::jaga (d_npredictors);

      /*  */
      d_trigger_thread = boost::shared_ptr < boost::thread
          > (new boost::thread (
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
      pmt::pmt_t print_msg;
      pmt::pmt_t tuple;
      size_t curr_sig = 0;
      size_t available_samples = 0;
      size_t available_observations = 0;
      void* data;
      float decision;

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
          switch (d_data_type)
            {
            case COMPLEX:
              {
                data = (gr_complex *) pmt::blob_data (
                    pmt::tuple_ref (tuple, 1));
                available_samples = pmt::blob_length (pmt::tuple_ref (tuple, 1))
                    / sizeof(gr_complex);
                if (available_samples < d_npredictors) {
                  PHASMA_WARN(
                      "openCV predict: Extracted samples less than predictors");
                  d_featurset->set_samples_num (available_samples);
                }
                // TODO: What if extracted samples more than predictors?
                d_featurset->generate ((gr_complex*) data);
              }
              break;
            case FLOAT:
              data = (float *) pmt::blob_data (pmt::tuple_ref (tuple, 1));
              available_samples = pmt::blob_length (pmt::tuple_ref (tuple, 1))
                  / sizeof(float);
              break;
            }

          /**
           * TODO: Iterate through all available observations of data provided by
           * the incoming tuple message
           */
          available_observations = 1;
          for (size_t i = 0; i < available_observations; i++) {
            /* Insert new dataset row */
            d_predictors = cv::Mat (1, d_featurset->get_features_num (),
                                    CV_32FC1, d_featurset->get_outbuf ());
            d_labels.at<float> (0, 0) = 0;

            d_data = cv::ml::TrainData::create (d_predictors,
                                                cv::ml::ROW_SAMPLE, d_labels);

            cv::Mat train_samples = d_data->getTrainSamples ();

            cv::Mat predict_labels;
            switch (d_classifier_type)
              {
              case RANDOM_FOREST:
                {
                  decision =
                      reinterpret_cast<cv::Ptr<cv::ml::RTrees>&> (d_model)->predict (
                          d_predictors, predict_labels);
                  break;
                }
              case ANN:
                {
                  decision =
                      reinterpret_cast<cv::Ptr<cv::ml::ANN_MLP>&> (d_model)->predict (
                          d_predictors, predict_labels);
                  break;
                }
              default:
                {
                  break;
                }
              }
            record_prediction ((int) decision, d_active_mod);
            calculate_confussion_matrix ();
          }

          std::string final_dec = decode_decision (decision);

          sigMF meta_msg = sigMF ("cf32", "./", "1.1.0");
          sigMF meta = sigMF ("cf32", "./", "1.1.0");

          meta_msg.parse_string (
              pmt::symbol_to_string (pmt::tuple_ref (tuple, 0)), final_dec);

          meta.parse_file (d_metafile);
          meta.add_annotation (meta_msg.get_annotation ()[0]);

          /* Append output file */
          std::ofstream outfile;
          outfile.open (d_metafile, std::ios_base::in);
          outfile << meta.toJSON ();
          outfile.close ();

          message_port_pub (
              pmt::intern ("classification"),
              pmt::string_to_symbol (meta_msg.getRoot ().toStyledString ()));

          print_msg = pmt::make_dict ();
          print_msg = pmt::dict_add (
              print_msg,
              pmt::intern ("CNFN_MATRIX_UPD"),
              pmt::make_blob (
                  &d_confusion_matrix,
                  d_labels_num * d_labels_num * sizeof(std::pair<int, float>)));
          message_port_pub (pmt::mp ("print"), print_msg);

          // Go catch the next tuple of the incoming vector message
          curr_sig++;
        }
        catch (pmt::out_of_range&) {
          /* You are out of range so break */
          curr_sig = 0;
        }
      }
    }

    void
    opencv_predict_impl::print_opencv_mat (cv::Mat* mat)
    {
      for (size_t idr = 0; idr < (size_t) mat->rows; idr++) {
        for (size_t idc = 0; idc < (size_t) mat->cols; idc++) {
          printf ("%f ", mat->at<float> (idr, idc));
        }
        printf ("\n");
      }
    }

    void
    opencv_predict_impl::set_active_mod (size_t active_mod)
    {
      d_active_mod = active_mod;
    }

  } /* namespace phasma */
} /* namespace gr */

