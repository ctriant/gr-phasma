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
#include "ann_model_c_impl.h"

namespace gr
{
  namespace phasma
  {

    ann_model_c::sptr
    ann_model_c::make (const size_t npredictors, const size_t nobservations,
                       size_t ninport, const std::vector<uint16_t> &labels,
                       size_t activation_func, double activation_a,
                       double activation_b,
                       const std::vector<uint16_t> &layer_sizes,
                       size_t train_method, const std::string filename)
    {
      return gnuradio::get_initial_sptr (
          new ann_model_c_impl (npredictors, nobservations, ninport, labels,
                                activation_func, activation_a, activation_b,
                                layer_sizes, train_method, filename));
    }

    /*
     * The private constructor
     */
    ann_model_c_impl::ann_model_c_impl (
        const size_t npredictors, const size_t nobservations, size_t ninport,
        const std::vector<uint16_t> &labels, size_t activation_func,
        double activation_a, double activation_b,
        const std::vector<uint16_t> &layer_sizes, size_t train_method,
        const std::string filename) :
            gr::sync_block (
                "ann_model_c",
                gr::io_signature::make (1, ninport, sizeof(gr_complex)),
                gr::io_signature::make (0, 0, 0)),
            d_npredictors (npredictors),
            d_nobservations (nobservations),
            d_remaining (nobservations),
            d_ninport (ninport),
            d_activation_func (activation_func),
            d_activation_a (activation_a),
            d_activation_b (activation_b),
            d_train_method (train_method),
            d_filename (filename),
            d_port_label (d_ninport)
    {
      set_output_multiple (100 * d_npredictors);
      d_input = (gr_complex*) malloc (d_npredictors * sizeof(gr_complex));
      d_annmodel = cv::ml::ANN_MLP::create ();
      d_featurset = new featureset::jaga (d_npredictors);
      bind_port_label (labels);
    }

    /*
     * Our virtual destructor.
     */
    ann_model_c_impl::~ann_model_c_impl ()
    {
//      cv::Mat mat = d_rfmodel->getVarImportance();
//      for (size_t idr = 0; idr < (size_t)mat.rows; idr++) {
//      	for (size_t idc = 0; idc < (size_t)mat.cols; idc++) {
//      	  printf ("%f ", mat.at<float> (idr, idc));
//      	}
//      	printf ("\n");
//            }
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
        PHASMA_LOG_INFO("Train error: %lf\n",
                        model->calcError (data, false, cv::noArray ()));PHASMA_LOG_INFO(
            "Test error: %lf\n", model->calcError (data, true, cv::noArray ()));
      }
    }

    int
    ann_model_c_impl::work (int noutput_items,
                            gr_vector_const_void_star &input_items,
                            gr_vector_void_star &output_items)
    {
      gr_complex *in;
      size_t available_observations = noutput_items / d_npredictors;
      size_t labels_num = std::set<size_t>( d_port_label.begin(), d_port_label.end() ).size();
      if (d_nobservations % d_ninport) {
        PHASMA_WARN(
            "Number of requested dataset observation should be multiple of number of inputs.");
      }

      cv::Mat tmp_label = cv::Mat::zeros (1, labels_num, CV_32FC1);

      for (size_t i = 0; i < available_observations; i++) {
        for (size_t n = 0; n < d_ninport; n++) {
          in = (gr_complex*) input_items[n];

          memcpy (d_input, &in[i * d_npredictors],
                  d_npredictors * sizeof(gr_complex));

          d_featurset->generate ((const gr_complex*) d_input);
          /* Insert new dataset row */
          if (d_predictors.empty () && d_labels.empty ()) {
            d_predictors = cv::Mat (1, d_featurset->get_features_num (),
                                    CV_32FC1, d_featurset->get_outbuf ());
            d_labels = cv::Mat::zeros (1, labels_num, CV_32FC1);
            d_labels.at<float> (
                0,
                d_port_label[n]) = 1.f;

          }
          else {
            cv::vconcat (
                cv::Mat (1, d_featurset->get_features_num (), CV_32FC1,
                         d_featurset->get_outbuf ()),
                d_predictors, d_predictors);

            for (size_t z=0; z < labels_num; z++) {
              tmp_label.at<float> (0, z) = 0;
            }
            tmp_label.at<float> (
                0,
                d_port_label[n]) = 1.f;
            cv::vconcat (tmp_label, d_labels, d_labels);
          }
          d_remaining--;

          if (d_remaining == 0) {
            for (size_t i = 0; i < d_nobservations; i++) {
              PHASMA_DEBUG("Sample: %f %f %f %f %f %f %f %f\n",
                           d_predictors.at<float> (i, 0),
                           d_predictors.at<float> (i, 1),
                           d_predictors.at<float> (i, 2),
                           d_predictors.at<float> (i, 3),
                           d_predictors.at<float> (i, 4),
                           d_predictors.at<float> (i, 5),
                           d_labels.at<float> (i, 0),
                           d_labels.at<float> (i, 1));
            }
            // TODO: Train model
            PHASMA_LOG_INFO("====== Dataset creation =====\n");
//            cv::Mat var_types (1, d_featurset->get_features_num() + 1, CV_8UC1,
//                                           cv::Scalar (cv::ml::VAR_ORDERED));
            d_train_data = cv::ml::TrainData::create (d_predictors,
                                                      cv::ml::ROW_SAMPLE,
                                                      d_labels);
            d_layer_sizes_mat = cv::Mat (1, 3, CV_32SC1);
            d_layer_sizes_mat.at<int> (0) = d_featurset->get_features_num ();
            d_layer_sizes_mat.at<int> (1) = 8;
            d_layer_sizes_mat.at<int> (2) = labels_num;
            d_annmodel->setLayerSizes (d_layer_sizes_mat);
            PHASMA_LOG_INFO("====== Model training =====\n");
            d_annmodel->setActivationFunction (d_activation_func,
                                               d_activation_a, d_activation_b);
            d_annmodel->setTermCriteria (
                cv::TermCriteria (
                    cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 300,
                    FLT_EPSILON));
            d_annmodel->setTrainMethod (d_train_method, 0.001);
            train_and_print_errs (d_annmodel, d_train_data);
            d_annmodel->save (d_filename);
            PHASMA_LOG_INFO("====== Model exported as xml =====\n");
            return WORK_DONE;
          }
        }
      }
      return d_npredictors;
    }

    void
    ann_model_c_impl::bind_port_label (const std::vector<uint16_t> &labels)
    {
      d_port_label = labels;
    }

    void
    ann_model_c_impl::print_opencv_mat (cv::Mat* mat)
    {
      for (size_t idr = 0; idr < (size_t) mat->rows; idr++) {
        for (size_t idc = 0; idc < (size_t) mat->cols; idc++) {
          printf ("%f ", mat->at<float> (idr, idc));
        }
        printf ("\n");
      }
    }

  } /* namespace phasma */
} /* namespace gr */

