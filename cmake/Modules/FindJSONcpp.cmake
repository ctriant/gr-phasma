# - Try to find Jsoncpp
# Once done, this will define
#
#  Jsoncpp_FOUND - system has Jsoncpp
#  Jsoncpp_INCLUDE_DIRS - the Jsoncpp include directories
#  Jsoncpp_LIBRARIES - link these to use Jsoncpp

INCLUDE(FindPkgConfig)

# Use pkg-config to get hints about paths
PKG_CHECK_MODULES(JSONCPP "jsoncpp")

FIND_PATH(JSONCPP_INCLUDE_DIR
    NAMES json
    ${CMAKE_INSTALL_PREFIX}/include
    PATHS
    /usr/local/include
    /usr/include
)

FIND_LIBRARY(JSONCPP_LIBRARY
    NAMES jsoncpp
    ${CMAKE_INSTALL_PREFIX}/lib
    ${CMAKE_INSTALL_PREFIX}/lib64
    PATHS
    /usr/lib64/
    /usr/local/lib64/
    ${JSONCPP_INCLUDE_DIRS}/../lib
    /usr/local/lib
    /usr/lib
)
LIST(APPEND JSONCPP_LIBRARY ${CMAKE_DL_LIBS})

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(JSONCPP DEFAULT_MSG JSONCPP_LIBRARY JSONCPP_INCLUDE_DIR)
MARK_AS_ADVANCED(JSONCPP_LIBRARY JSONCPP_INCLUDE_DIR)