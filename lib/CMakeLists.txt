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
# Setup library
########################################################################
include(GrPlatform) #define LIB_SUFFIX

include_directories(${Boost_INCLUDE_DIR}
                    ${ARMADILLO_INCLUDE_DIRS}
                    ${JSONCPP_INCLUDE_DIR})
                    
link_directories(${Boost_LIBRARY_DIRS})

include_directories(${ARMADILLO_INCLUDE_DIRS})
list(APPEND phasma_sources
    utils/sigMF.cc
    rforest_model_c_impl.cc
    rforest_model_f_impl.cc
    eigenvalue_signal_detector_impl.cc
    opencv_predict_impl.cc
    signal_separator_impl.cc
    message_stream_impl.cc
    featuresets/dummy_featureset.cc
    featuresets/raw_iq_featureset.cc 
    featuresets/jaga.cc
    classifier.cc
    ncurses_console_impl.cc
    )

set(phasma_sources "${phasma_sources}" PARENT_SCOPE)
if(NOT phasma_sources)
	MESSAGE(STATUS "No C++ sources... skipping lib/")
	return()
endif(NOT phasma_sources)                                         

list(APPEND phasma_libs ${Boost_LIBRARIES} 
                            ${OpenCV_LIBS}
                            ${ARMADILLO_LIBRARIES}
                            ${GNURADIO_ALL_LIBRARIES}
                            ${JSONCPP_LIBRARY}
                            ${CURSES_LIBRARIES}
)

add_library(gnuradio-phasma SHARED ${phasma_sources})

target_link_libraries(gnuradio-phasma ${phasma_libs})
                       
set_target_properties(gnuradio-phasma PROPERTIES DEFINE_SYMBOL "gnuradio_phasma_EXPORTS")

if(APPLE)
    set_target_properties(gnuradio-phasma PROPERTIES
        INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
    )
endif(APPLE)

########################################################################
# Install built library files
########################################################################
install(TARGETS gnuradio-phasma
    LIBRARY DESTINATION lib${LIB_SUFFIX} # .so/.dylib file
    ARCHIVE DESTINATION lib${LIB_SUFFIX} # .lib file
    RUNTIME DESTINATION bin              # .dll file
)
