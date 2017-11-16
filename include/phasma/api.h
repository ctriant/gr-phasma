/*
 * Copyright 2011 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_PHASMA_API_H
#define INCLUDED_PHASMA_API_H

#include <gnuradio/attributes.h>
#include <phasma/log.h>

#ifdef gnuradio_phasma_EXPORTS
#  define PHASMA_API __GR_ATTR_EXPORT
#else
#  define PHASMA_API __GR_ATTR_IMPORT
#endif

typedef enum
{
  RANDOM_FOREST = 0,
  ANN
} opencv_ml_classifier_type_t;

typedef enum
{
  INT = 0,
  FLOAT,
  DOUBLE,
  COMPLEX,
  STRING,
  INT_VEC,
  FLOAT_VEC,
  DOUBLE_VEC,
  COMPLEX_VEC,
} phasma_data_type_t;

typedef enum
{
  BPSK = 0,
  QPSK,
  PSK8,
  GMSK,
  GFSK,
  AM,
  FM,
  FSK,
  OQPSK,
  PSK16,
  PSK32,
  QAM16,
  QAM32,
  QAM64,
  QAM128,
  QAM256,
  FSK2,
  FSK4,
  FKS8,
  ASK,
  SPREAD_SPECTRUM,
} phasma_mod_type_t;

typedef std::vector<std::vector<std::pair<int, float>>> phasma_confusion_matrix_type_t;

#endif /* INCLUDED_PHASMA_API_H */
