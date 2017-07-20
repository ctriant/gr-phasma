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
#include <phasma/featuresets/dummy_featureset.h>
#include <phasma/featuresets/raw_iq_featureset.h>
#include <phasma/featuresets/jaga.h>
#include <opencv2/ml.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <phasma/utils/ncurses_utils.h>
#include <pthread.h>

namespace gr
{
  namespace phasma
  {

    class opencv_predict_impl : public opencv_predict
    {
    private:
      size_t d_npredictors;
      size_t d_nlabels;
      const size_t d_history_size;
      size_t d_debug_mode;
      size_t d_active_mod;
      size_t d_classifier_type;
      size_t d_data_type;
      gr_complex* d_input;
      float* d_input_re;
      float* d_input_imag;

      cv::Mat d_predictors;
      cv::Ptr<cv::ml::TrainData> d_data;

      cv::Ptr<cv::ml::StatModel> d_model;
      cv::Mat d_labels;

      const std::string d_filename;

      featureset::jaga* d_featurset;

      boost::shared_ptr<boost::thread> d_trigger_thread;
      boost::shared_ptr<boost::thread> d_print_thread;
      boost::condition_variable d_start_print_cond;
      boost::mutex d_start_print_mtx;

      bool d_running;

      Json::Value d_root;

      const std::string d_metafile;

      std::vector<size_t> d_classes;

      void
      print_opencv_mat (cv::Mat* mat);

      void
      update_confusion_matrix_screen ();

    public:
      opencv_predict_impl (const size_t classifier_type, const size_t data_type,
			   const size_t npredictors, const size_t nlabels,
			   const size_t history_size, bool debug_mode,
			   size_t active_mod, const std::vector<size_t> &labels,
			   const std::string filename,
			   const std::string metafile);
      ~opencv_predict_impl ();

      void
      msg_handler_trigger ();

      void
      set_labels (const std::vector<size_t> &labels);

      bool
      stop ();

      void
      set_active_mod (size_t active_mod);

      void
      print_thread ();

    };

  } // namespace phasma
} // namespace gr

#endif /* INCLUDED_PHASMA_OPENCV_PREDICT_IMPL_H */

