/* -*- c++ -*- */

#define PHASMA_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "phasma_swig_doc.i"

%{
#include "phasma/rforest_model_c.h"
#include "phasma/rforest_model_f.h"
#include "phasma/eigenvalue_signal_detector.h"
#include "phasma/opencv_predict.h"
#include "phasma/signal_separator.h"
#include "phasma/sigmf_debug.h"
#include "phasma/ncurses_console.h"
%}


%include "phasma/rforest_model_c.h"
GR_SWIG_BLOCK_MAGIC2(phasma, rforest_model_c);
%include "phasma/rforest_model_f.h"
GR_SWIG_BLOCK_MAGIC2(phasma, rforest_model_f);
%include "phasma/eigenvalue_signal_detector.h"
GR_SWIG_BLOCK_MAGIC2(phasma, eigenvalue_signal_detector);
%include "phasma/opencv_predict.h"
GR_SWIG_BLOCK_MAGIC2(phasma, opencv_predict);
%include "phasma/signal_separator.h"
GR_SWIG_BLOCK_MAGIC2(phasma, signal_separator);
%include "phasma/sigmf_debug.h"
GR_SWIG_BLOCK_MAGIC2(phasma, sigmf_debug);
%include "phasma/ncurses_console.h"
GR_SWIG_BLOCK_MAGIC2(phasma, ncurses_console);
