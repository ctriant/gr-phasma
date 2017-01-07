/* -*- c++ -*- */

#define PHASMA_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "phasma_swig_doc.i"

%{
#include "phasma/rforest_model.h"
%}


%include "phasma/rforest_model.h"
GR_SWIG_BLOCK_MAGIC2(phasma, rforest_model);
