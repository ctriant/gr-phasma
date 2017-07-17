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
#include <phasma/classifier.h>
#include <iostream>
#include <iomanip>

namespace gr
{
  namespace phasma
  {

    classifier::classifier (size_t hist_size, size_t labels_num,
			    const std::vector<size_t> &labels) :
	    d_pred_history (hist_size),
	    d_labels_num (labels_num),
	    d_confusion_matrix (labels_num)
    {
      if (labels.size () != labels_num) {
	throw std::runtime_error (
	    "classifier: Prediction labels incorrectly set");
      }

      d_labels = labels;

      for (size_t i = 0; i < d_labels_num; i++) {
	d_confusion_matrix[i] = std::vector<std::pair<int, float>> (
	    d_labels_num);
      }
    }

    classifier::~classifier ()
    {
    }

    void
    classifier::record_prediction (int predicted, int actual)
    {
      d_pred_history.push_back (std::make_pair (predicted, actual));
    }

    float
    classifier::calc_pred_accuracy ()
    {
      size_t counter = 0;
      for (size_t i = 0; i < d_pred_history.size (); i++) {
	if (d_pred_history[i].first == d_pred_history[i].second) {
	  counter++;
	}
      }
      return (float) counter / d_pred_history.size ();
    }

    void
    classifier::calculate_confussion_matrix ()
    {
      size_t sum = 0;
      for (size_t i = 0; i < d_labels.size (); i++) {
	for (size_t z = 0; z < d_labels.size (); z++) {
	  for (size_t j = 0; j < d_pred_history.size (); j++) {
	    if (d_pred_history[j].second == d_labels[i] &&
		d_pred_history[j].first == d_labels[z]) {
	      sum ++;
	    }
	  }
	  std::pair<int, float> p = std::make_pair(d_labels[i], (float) sum / d_pred_history.size ());
	  d_confusion_matrix[i][z] = p;
	  sum = 0;
	}
      }
    }

    void
    classifier::print_confussion_matrix ()
    {
      std::cout << "Confusion matrix" << std::endl;
      std::cout << "================" << std::endl;
      for (size_t i = 0; i < d_labels.size (); i++) {
	std::cout << decode_decision(d_labels[i]) << " | ";
	for (size_t j = 0; j < d_labels.size (); j++) {
	  std::cout << decode_decision(d_labels[j]) << ":" << std::setprecision (2)
	      << d_confusion_matrix[i][j].second << "\t\t";
	}
	std::cout << std::endl;
      }
    }

    void
    classifier::clear_pred_history ()
    {
      d_pred_history.clear ();
    }

    std::string
    classifier::decode_decision (int decision)
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
	case GFSK:
	  return "GFSK";
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

