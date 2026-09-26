#pragma once

#include <vector>

namespace mippp {

// Owning Farkas multipliers in original model row/column order. Each finite
// side is separate: a ranged row (or fixed variable) can contribute either
// side. An absent side has zero weight. Unlike a row ray, this representation
// can identify bound contributions without reconstructing them from A^T y.
// Numerical values remain hints until their support is revalidated by a solve.
template <typename Scalar>
struct linear_infeasibility_certificate {
    std::vector<Scalar> row_lower;
    std::vector<Scalar> row_upper;
    std::vector<Scalar> variable_lower;
    std::vector<Scalar> variable_upper;
};

}  // namespace mippp
