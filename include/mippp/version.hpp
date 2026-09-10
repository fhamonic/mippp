#pragma once

// Single source of truth for the MIP++ version number: CMakeLists.txt and
// conanfile.py parse these macros, do not change their formatting.
#define MIPPP_VERSION_MAJOR 1
#define MIPPP_VERSION_MINOR 0
#define MIPPP_VERSION_PATCH 0

#define MIPPP_VERSION                                          \
    (MIPPP_VERSION_MAJOR * 10000 + MIPPP_VERSION_MINOR * 100 + \
     MIPPP_VERSION_PATCH)
