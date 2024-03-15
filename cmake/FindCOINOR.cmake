# modeled after FindCOIN.cmake in the lemon project

# Written by: Matthew Gidden Last updated: 12/17/12

# This cmake file is designed to locate coin-related dependencies on a
# filesystem.
#
# If the coin dependencies were installed in a non-standard directory, e.g.
# installed from source perhaps, then the user can provide a prefix hint via the
# COINOR_ROOT_DIR cmake variable: $> cmake ../src
# -DCOINOR_ROOT_DIR=/path/to/coin/root

# To date, this install requires the following dev versions of the respective
# coin libraries: * coinor-libCbc-dev * coinor-libClp-dev *
# coinor-libcoinutils-dev * coinor-libOsi-dev

#
# Get the root directory hint if provided
#
if(NOT DEFINED COINOR_ROOT_DIR)
    set(COINOR_ROOT_DIR "$ENV{COINOR_ROOT_DIR}")
    message("\tCOINOR Root Dir: ${COINOR_INCLUDE_DIR}")
endif(NOT DEFINED COINOR_ROOT_DIR)
message(STATUS "COINOR_ROOT_DIR hint is : ${COINOR_ROOT_DIR}")

#
# Find the path based on a required header file
#
message(STATUS "Coin multiple library dependency status:")
find_path(
    COINOR_INCLUDE_DIR coin/CbcModel.hpp
    HINTS "${COINOR_INCLUDE_DIR}"
    HINTS "${COINOR_ROOT_DIR}/include"
    HINTS /usr/
    HINTS /usr/include/
    HINTS /usr/local/
    HINTS /usr/local/include/
    HINTS /usr/coin/
    HINTS /usr/coin-Cbc/
    HINTS /usr/local/coin/
    HINTS /usr/local/coin-Cbc/)
set(COINOR_INCLUDE_DIR ${COINOR_INCLUDE_DIR}/coin)
message("\tCOINOR Include Dir: ${COINOR_INCLUDE_DIR}")

#
# Find all coin library dependencies
#
find_library(
    COINOR_CBC_LIBRARY
    NAMES Cbc libCbc # libCbc.so.0
    HINTS "${COINOR_INCLUDE_DIR}/../../lib/"
    HINTS "${COINOR_ROOT_DIR}/lib")
message("\tCOINOR CBC: ${COINOR_CBC_LIBRARY}")

find_library(
    COINOR_CBC_SOLVER_LIBRARY
    NAMES CbcSolver libCbcSolver libCbcSolver.so.0
    HINTS ${COINOR_INCLUDE_DIR}/../../lib/
    HINTS "${COINOR_ROOT_DIR}/lib")
message("\tCOINOR CBC solver: ${COINOR_CBC_SOLVER_LIBRARY}")

find_library(
    COINOR_CGL_LIBRARY
    NAMES Cgl libCgl libCgl.so.0
    HINTS ${COINOR_INCLUDE_DIR}/../../lib/
    HINTS "${COINOR_ROOT_DIR}/lib")
message("\tCOINOR CGL: ${COINOR_CGL_LIBRARY}")

find_library(
    COINOR_CLP_LIBRARY
    NAMES Clp libClp # libClp.so.0
    HINTS ${COINOR_INCLUDE_DIR}/../../lib/
    HINTS "${COINOR_ROOT_DIR}/lib")
message("\tCOINOR CLP: ${COINOR_CLP_LIBRARY}")

find_library(
    COINOR_COINOR_UTILS_LIBRARY
    NAMES CoinUtils libCoinUtils # libCoinUtils.so.0
    HINTS ${COINOR_INCLUDE_DIR}/../../lib/
    HINTS "${COINOR_ROOT_DIR}/lib")
message("\tCOINOR UTILS: ${COINOR_COINOR_UTILS_LIBRARY}")

find_library(
    COINOR_OSI_LIBRARY
    NAMES Osi libOsi # libOsi.so.0
    HINTS ${COINOR_INCLUDE_DIR}/../../lib/
    HINTS "${COINOR_ROOT_DIR}/lib")
message("\tCOINOR OSI: ${COINOR_OSI_LIBRARY}")

#
# Not required by cbc v2.5, but required by later versions
#
# FIND_LIBRARY(COINOR_OSI_CBC_LIBRARY NAMES OsiCbc libOsiCbc libOsiCbc.so.0 HINTS
# ${COINOR_INCLUDE_DIR}/../../lib/ HINTS "${COINOR_ROOT_DIR}/lib" ) MESSAGE("\tCOINOR
# OSI CBC: ${COINOR_OSI_CBC_LIBRARY}")

find_library(
    COINOR_OSI_CLP_LIBRARY
    NAMES OsiClp libOsiClp libOsiClp.so.0
    HINTS ${COINOR_INCLUDE_DIR}/../../lib/
    HINTS "${COINOR_ROOT_DIR}/lib")
message("\tCOINOR OSI CLP: ${COINOR_OSI_CLP_LIBRARY}")

find_library(
    COINOR_ZLIB_LIBRARY
    NAMES z libz libz.so.1
    HINTS ${COINOR_ROOT_DIR}/lib
    HINTS "${COINOR_ROOT_DIR}/lib")
message("\tCOINOR ZLIB: ${COINOR_ZLIB_LIBRARY}")

find_library(
    COINOR_BZ2_LIBRARY
    NAMES bz2 libz2 libz2.so.1
    HINTS ${COINOR_ROOT_DIR}/lib
    HINTS "${COINOR_ROOT_DIR}/lib")
message("\tCOINOR BZ2: ${COINOR_BZ2_LIBRARY}")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    COINOR
    DEFAULT_MSG
    COINOR_INCLUDE_DIR
    COINOR_CBC_LIBRARY
    COINOR_CBC_SOLVER_LIBRARY
    COINOR_CGL_LIBRARY
    COINOR_CLP_LIBRARY
    COINOR_COINOR_UTILS_LIBRARY
    COINOR_OSI_LIBRARY
    # Not required by cbc v2.5, but required by later versions
    # COINOR_OSI_CBC_LIBRARY
    COINOR_OSI_CLP_LIBRARY
    COINOR_ZLIB_LIBRARY
    COINOR_BZ2_LIBRARY)

#
# Set all required cmake variables based on our findings
#
if(COINOR_FOUND)
    set(COINOR_INCLUDE_DIRS ${COINOR_INCLUDE_DIR})
    set(COINOR_CLP_LIBRARIES
        "${COINOR_CLP_LIBRARY};${COINOR_COINOR_UTILS_LIBRARY};${COINOR_ZLIB_LIBRARY}")
    if(COINOR_ZLIB_LIBRARY)
        set(COINOR_CLP_LIBRARIES "${COINOR_CLP_LIBRARIES};${COINOR_ZLIB_LIBRARY}")
    endif(COINOR_ZLIB_LIBRARY)
    if(COINOR_BZ2_LIBRARY)
        set(COINOR_CLP_LIBRARIES "${COINOR_CLP_LIBRARIES};${COINOR_BZ2_LIBRARY}")
    endif(COINOR_BZ2_LIBRARY)
    # Not required by cbc v2.5, but required by later versions in which case,
    # the lower line should be commented out and this line used
    # SET(COINOR_CBC_LIBRARIES
    # "${COINOR_CBC_LIBRARY};${COINOR_CBC_SOLVER_LIBRARY};${COINOR_CGL_LIBRARY};${COINOR_OSI_LIBRARY};${COINOR_OSI_CBC_LIBRARY};${COINOR_OSI_CLP_LIBRARY};${COINOR_CLP_LIBRARIES}")
    set(COINOR_CBC_LIBRARIES
        "${COINOR_CBC_LIBRARY};${COINOR_CBC_SOLVER_LIBRARY};${COINOR_CGL_LIBRARY};${COINOR_OSI_LIBRARY};${COINOR_OSI_CLP_LIBRARY};${COINOR_CLP_LIBRARIES}"
    )
    set(COINOR_LIBRARIES ${COINOR_CBC_LIBRARIES})
endif(COINOR_FOUND)

#
# Report a synopsis of our findings
#
if(COINOR_INCLUDE_DIRS)
    message(STATUS "Found COINOR Include Dirs: ${COINOR_INCLUDE_DIRS}")
else(COINOR_INCLUDE_DIRS)
    message(STATUS "COINOR Include Dirs NOT FOUND")
endif(COINOR_INCLUDE_DIRS)
