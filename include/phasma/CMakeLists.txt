# Copyright 2011,2012 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

########################################################################
# Install public header files
########################################################################
install(FILES
    utils/sigMF.h
    utils/ncurses_utils.h
    api.h
    log.h
    rforest_model_c.h
    rforest_model_f.h
    ann_model_c.h
    eigenvalue_signal_detector.h 
    opencv_predict.h
    signal_separator.h
    message_stream.h
    featuresets/dummy_featureset.h
    featuresets/raw_iq_featureset.h 
    featuresets/jaga.h
    classifier.h
    ncurses_console.h DESTINATION include/phasma
)
