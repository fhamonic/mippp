# - Try to find SCIP
# See http://scip.zib.de/ for more information on SCIP
#
# Once done, this will define
#
#  SCIP_INCLUDE_DIRS   - where to find scip/scip.h, etc.
#  SCIP_LIBRARIES      - List of libraries when using scip.
#  SCIP_FOUND          - True if scip found.
#
#  SCIP_VERSION        - The version of scip found (x.y.z)
#  SCIP_VERSION_MAJOR  - The major version of scip
#  SCIP_VERSION_MINOR  - The minor version of scip
#  SCIP_VERSION_PATCH  - The patch version of scip
#
# Variables used by this module, they can change the default behaviour and
# need to be set before calling find_package:
#
# SCIP_LPS             - Set to SPX to force SOPLEX as the LP-Solver and
#                        to CPX for CPLEX. If not set, first CPLEX is tested,
#                        if this is not available SOPLEX is tried.
# SCIP_ROOT            - The preferred installation prefix for searching for
#                        Scip.  Set this if the module has problems finding
#                        the proper SCIP installation. SCIP_ROOT is also
#                        available as an environment variable.
#
# Author:
# Wolfgang A. Welz <welz@math.tu-berlin.de>
#
# Distributed under the Boost Software License, Version 1.0.
# (See accompanying file LICENSE.txt or copy at
# http://www.boost.org/LICENSE_1_0.txt)

include(LibFindMacros)

# Dependencies
libfind_package(SCIP ZLIB)
#libfind_package(SCIP Readline)

MESSAGE(STATUS "Check for working SCIP installation")

# If SCIP_ROOT is not set, look for the environment variable
if(NOT SCIP_ROOT AND NOT "$ENV{SCIP_ROOT}" STREQUAL "")
  set(SCIP_ROOT $ENV{SCIP_ROOT})
endif()

set(_SCIP_SEARCHES)

# Search SCIP_ROOT first if it is set.
if(SCIP_ROOT)
  set(_SCIP_SEARCH_ROOT PATHS ${SCIP_ROOT} NO_DEFAULT_PATH)
  list(APPEND _SCIP_SEARCHES _SCIP_SEARCH_ROOT)
endif()

# Normal search.
set(_SCIP_SEARCH_NORMAL
  PATHS ""
)
list(APPEND _SCIP_SEARCHES _SCIP_SEARCH_NORMAL)

# Try each search configuration.
foreach(search ${_SCIP_SEARCHES})
  FIND_PATH(SCIP_INCLUDE_DIR NAMES scip/scip.h ${${search}} PATH_SUFFIXES src include)
  FIND_LIBRARY(SCIP_LIBRARY NAMES scip ${${search}} PATH_SUFFIXES lib)
  FIND_LIBRARY(OBJSCIP_LIBRARY NAMES objscip ${${search}} PATH_SUFFIXES lib)
  FIND_LIBRARY(NLPI_LIBRARY NAMES nlpi ${${search}} PATH_SUFFIXES lib)
  # now we still need an lp solver
  FIND_LIBRARY(LPISPX_LIBRARY NAMES lpispx ${${search}} PATH_SUFFIXES lib)
  FIND_LIBRARY(LPICPX_LIBRARY NAMES lpicpx ${${search}} PATH_SUFFIXES lib)
endforeach()

IF(SCIP_INCLUDE_DIR AND EXISTS "${SCIP_INCLUDE_DIR}/scip/def.h")
  FILE(STRINGS "${SCIP_INCLUDE_DIR}/scip/def.h" SCIP_DEF_H REGEX "^#define SCIP_VERSION +[0-9]+")
  STRING(REGEX REPLACE "^#define SCIP_VERSION +([0-9]+).*" "\\1" SVER ${SCIP_DEF_H})

  STRING(REGEX REPLACE "([0-9]).*" "\\1" SCIP_VERSION_MAJOR ${SVER})
  STRING(REGEX REPLACE "[0-9]([0-9]).*" "\\1" SCIP_VERSION_MINOR ${SVER})
  STRING(REGEX REPLACE "[0-9][0-9]([0-9]).*" "\\1" SCIP_VERSION_PATCH ${SVER})
  SET(SCIP_VERSION "${SCIP_VERSION_MAJOR}.${SCIP_VERSION_MINOR}.${SCIP_VERSION_PATCH}")
ENDIF()

# Now check LP-Solver dependencies
IF(SCIP_LIBRARY)
  IF(NOT SCIP_LPS STREQUAL "SPX" AND NOT LPI_LIBRARIES AND LPICPX_LIBRARY)
    FIND_PACKAGE(CPLEX QUIET)
    IF(CPLEX_FOUND)
      SET(LPI_LIBRARIES ${LPICPX_LIBRARY} ${CPLEX_LIBRARIES})
      MESSAGE(STATUS "  using CPLEX ${CPLEX_VERSION} as the LP-Solver")
    ELSE()
      MESSAGE(STATUS "  CPLEX not found")
    ENDIF()    
  ENDIF(NOT SCIP_LPS STREQUAL "SPX" AND NOT LPI_LIBRARIES AND LPICPX_LIBRARY)

  IF(NOT SCIP_LPS STREQUAL "CPX" AND NOT LPI_LIBRARIES AND LPISPX_LIBRARY)
    FIND_PACKAGE(SOPLEX QUIET)
    IF(SOPLEX_FOUND)
      SET(LPI_LIBRARIES ${LPISPX_LIBRARY} ${SOPLEX_LIBRARIES})
      MESSAGE(STATUS "  using SOPLEX ${SOPLEX_VERSION} as the LP-Solver")
    ELSE()
      MESSAGE(STATUS "  SOPLEX not found")
    ENDIF()    
  ENDIF(NOT SCIP_LPS STREQUAL "CPX" AND NOT LPI_LIBRARIES AND LPISPX_LIBRARY)

  IF(NOT LPI_LIBRARIES)
    MESSAGE(FATAL_ERROR "SCIP requires either SOPLEX or CPLEX as an LP-Solver.\nPlease make sure that SCIP is configured for the correct solver and that the corresponding solver-library is installed. You may also set SCIP_LPS, SOPLEX_ROOT, CPLEX_ROOT appropriately.") 
  ENDIF()
ENDIF()

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(SCIP_PROCESS_INCLUDES SCIP_INCLUDE_DIR ZLIB_INCLUDE_DIRS)
set(SCIP_PROCESS_LIBS SCIP_LIBRARY OBJSCIP_LIBRARY NLPI_LIBRARY LPI_LIBRARIES ZLIB_LIBRARIES)
libfind_process(SCIP)