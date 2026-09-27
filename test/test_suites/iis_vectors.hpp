#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "iis_oracle.hpp"

namespace mippp::iis_oracle {

// Numbers adapted from HiGHS 1.12.0, check/TestIis.cpp, under the MIT license
// in test/data/iis/HIGHS-LICENSE.txt:
// https://github.com/ERGO-Code/HiGHS/blob/755a8e027a99a8d4ecf153a8dde4b2a767cdf384/check/TestIis.cpp
// Upstream builds the upper variant of the empty row by changing the bounds of
// the lower one.
//
// A vector may have several IISs, of the listed sizes counted in sides: check
// an answer with is_iis, never against one membership.
struct published_vector {
    const char * name;
    linear_system system;
    std::vector<std::size_t> iis_sizes;
};

inline std::vector<published_vector> published_vectors() {
    const auto none = std::nullopt;
    return {{"lp-incompatible-bounds",
             {{{0., 1.}, {0., 1.}, {0., -1.}},
              {{{{1, 1.}, {2, 1.}}, 1., 0.}, {{{0, 1.}, {2, 1.}}, 0., 1.}}},
             {2, 3}},
            {"lp-empty-infeasible-row/lower",
             {{{0., none}, {0., none}},
              {{{{0, 2.}, {1, 1.}}, none, 8.},
               {{}, 1., 2.},
               {{{0, 1.}, {1, 3.}}, none, 9.}}},
             {1}},
            {"lp-empty-infeasible-row/upper",
             {{{0., none}, {0., none}},
              {{{{0, 2.}, {1, 1.}}, none, 8.},
               {{}, -2., -1.},
               {{{0, 1.}, {1, 3.}}, none, 9.}}},
             {1}},
            {"lp-get-iis",
             {{{0., none}, {0., none}},
              {{{{0, 2.}, {1, 1.}}, none, 8.},
               {{{0, 1.}, {1, 3.}}, none, 9.},
               {{{0, 1.}, {1, 1.}}, none, -2.}}},
             {3}}};
}

}  // namespace mippp::iis_oracle
