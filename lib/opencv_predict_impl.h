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

#ifndef INCLUDED_PHASMA_OPENCV_PREDICT_IMPL_H
#define INCLUDED_PHASMA_OPENCV_PREDICT_IMPL_H

#include <phasma/api.h>
#include <phasma/log.h>
#include <phasma/opencv_predict.h>
#include <opencv2/ml.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>

namespace gr
{
  namespace phasma
  {

    class opencv_predict_impl : public opencv_predict
    {
    private:
      size_t d_npredictors;
      size_t d_input_multiplier;
      size_t d_ninport;
      size_t d_classifier_type;
      
      float* d_input;
      
      cv::Mat d_predictors;
      cv::Mat d_labels;
      cv::Ptr<cv::ml::TrainData> d_data;

      cv::Ptr<cv::ml::StatModel> d_model;

      const std::string d_filename;
      std::vector<uint16_t> d_port_label;

      void
      bind_port_label(const std::vector<uint16_t> &labels);

    public:
      opencv_predict_impl (const size_t input_multiplier,
			   const size_t classifier_type,
			   const size_t npredictors, const size_t d_ninport,
			   const std::vector<uint16_t> &labels,
			   const std::string filename);
      ~opencv_predict_impl ();

      int
      work (int noutput_items, gr_vector_const_void_star &input_items,
	    gr_vector_void_star &output_items);
    };

  } // namespace phasma
} // namespace gr

#endif /* INCLUDED_PHASMA_OPENCV_PREDICT_IMPL_H */

