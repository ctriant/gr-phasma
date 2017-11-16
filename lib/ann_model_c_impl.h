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

#ifndef INCLUDED_PHASMA_ANN_MODEL_C_IMPL_H
#define INCLUDED_PHASMA_ANN_MODEL_C_IMPL_H

#include <phasma/ann_model_c.h>
#include <phasma/featuresets/dummy_featureset.h>
#include <phasma/featuresets/raw_iq_featureset.h>
#include <phasma/featuresets/jaga.h>
#include <opencv2/ml.hpp>
#include <opencv2/core.hpp>

namespace gr
{
  namespace phasma
  {

    class ann_model_c_impl : public ann_model_c
    {
    private:

      size_t d_npredictors;
      size_t d_nobservations;
      size_t d_remaining;
      size_t d_ninport;


      size_t d_activation_func;
      double d_activation_a;
      double d_activation_b;
      const std::vector<uint16_t> d_layer_sizes_vec;
      size_t d_train_method;

      std::string d_filename;

      cv::Mat d_predictors;
      cv::Mat d_labels;
      cv::Mat d_layer_sizes_mat;
      cv::Ptr<cv::ml::TrainData> d_train_data;

      /**
       * Random forest model
       */
      cv::Ptr<cv::ml::ANN_MLP> d_annmodel;

      featureset::jaga* d_featurset;

      /**
       * Auxiliary buffer
       */
      gr_complex* d_input;

      std::vector<uint16_t> d_port_label;

      void
      bind_port_label (const std::vector<uint16_t> &labels);

      void
      print_opencv_mat (cv::Mat* mat);

    public:
      ann_model_c_impl (const size_t npredictors, const size_t nobservations,
                        size_t ninport, const std::vector<uint16_t> &labels,
                        size_t activation_func, double activation_a,
                        double activation_b,
                        const std::vector<uint16_t> &layer_sizes,
                        size_t train_method, const std::string filename);

      ~ann_model_c_impl ();

      // Where all the action really happens
      int
      work (int noutput_items, gr_vector_const_void_star &input_items,
            gr_vector_void_star &output_items);
    };

  } // namespace phasma
} // namespace gr

#endif /* INCLUDED_PHASMA_ANN_MODEL_C_IMPL_H */

