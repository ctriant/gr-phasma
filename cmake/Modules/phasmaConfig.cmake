INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_PHASMA phasma)

FIND_PATH(
    PHASMA_INCLUDE_DIRS
    NAMES phasma/api.h
    HINTS $ENV{PHASMA_DIR}/include
        ${PC_PHASMA_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    PHASMA_LIBRARIES
    NAMES gnuradio-phasma
    HINTS $ENV{PHASMA_DIR}/lib
        ${PC_PHASMA_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(PHASMA DEFAULT_MSG PHASMA_LIBRARIES PHASMA_INCLUDE_DIRS)
MARK_AS_ADVANCED(PHASMA_LIBRARIES PHASMA_INCLUDE_DIRS)

