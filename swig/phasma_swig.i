/* -*- c++ -*- */

#define PHASMA_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "phasma_swig_doc.i"

%{
#include "phasma/rforest_model.h"
#include "phasma/eigenvalue_signal_detector.h"
#include "phasma/opencv_predict.h"
#include "phasma/signal_extractor.h"
%}


%include "phasma/rforest_model.h"
GR_SWIG_BLOCK_MAGIC2(phasma, rforest_model);
%include "phasma/eigenvalue_signal_detector.h"
GR_SWIG_BLOCK_MAGIC2(phasma, eigenvalue_signal_detector);
%include "phasma/opencv_predict.h"
GR_SWIG_BLOCK_MAGIC2(phasma, opencv_predict);
%include "phasma/signal_extractor.h"
GR_SWIG_BLOCK_MAGIC2(phasma, signal_extractor);
