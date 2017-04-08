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

#ifndef INCLUDED_PHASMA_RFOREST_MODEL_IMPL_H
#define INCLUDED_PHASMA_RFOREST_MODEL_IMPL_H

#include <phasma/rforest_model.h>
#include <phasma/featuresets/dummy_featureset.h>
#include <phasma/featuresets/raw_iq_featureset.h>
#include <gnuradio/fft/fft.h>
#include <gnuradio/fft/window.h>
#include <opencv2/ml.hpp>
#include <opencv2/core.hpp>

namespace gr
{
  namespace phasma
  {

    class rforest_model_impl : public rforest_model
    {
    private:

      size_t d_npredictors;
      size_t d_nobservations;
      size_t d_remaining;
      size_t d_ninport;
      size_t d_max_depth;
      size_t d_min_sample_count;
      size_t d_regression_accu;
      size_t d_max_categories;
      size_t d_active_var_count;
      size_t d_max_iter;

      uint8_t d_use_surrogates;
      uint8_t d_calc_var_importance;

      std::string d_filename;

      cv::Mat d_predictors;
      cv::Mat d_labels;
      cv::Ptr<cv::ml::TrainData> d_train_data;

      /**
       * Random forest model
       */
      cv::Ptr<cv::ml::RTrees> d_rfmodel;

      featureset::dummy_featureset* d_featurset;

      /**
       * Auxiliary buffer
       */
      gr_complex* d_input;

      std::vector<uint16_t> d_port_label;

      void
      bind_port_label(const std::vector<uint16_t> &labels);

      void
      print_opencv_mat(cv::Mat* mat);

    public:
      rforest_model_impl (const size_t npredictors, const size_t nobservations,
			  const size_t ninport, const std::vector<uint16_t> &labels,
			  const size_t max_depth,
			  const size_t min_sample_count,
			  const size_t regression_accu,
			  const uint8_t use_surrogates,
			  const size_t max_categories,
			  const uint8_t calc_var_importance,
			  const size_t active_var_count, const size_t max_iter,
			  const std::string filename);

      ~rforest_model_impl ();

      // Where all the action really happens
      int
      work (int noutput_items, gr_vector_const_void_star &input_items,
	    gr_vector_void_star &output_items);
    };

  } // namespace phasma
} // namespace gr

#endif /* INCLUDED_PHASMA_RFOREST_MODEL_IMPL_H */

