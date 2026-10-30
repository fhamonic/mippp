#pragma once

#include <algorithm>
#include <cstddef>
#include <limits>
#include <optional>
#include <ranges>
#include <utility>
#include <vector>

#include "mippp/mapping.hpp"
#include "mippp/model_concepts.hpp"

namespace mippp {
namespace colgen {

// Wentges smoothing of the duals of a restricted master problem. Pricing
// reads the separation point
//     pi_sep = a * pi_center + (1 - a) * pi_out
// where pi_out holds the duals of the last master solve and pi_center, the
// stability center, the separation point with the best Lagrangian value so
// far. Better(x, y) tells whether the Lagrangian value x is better than y:
// std::greater<> when the master minimizes (the values are lower bounds),
// std::less<> when it maximizes (upper bounds).
template <typename Model, typename Better>
class dual_smoother {
public:
    using constraint = model_constraint_t<Model>;
    using scalar = model_scalar_t<Model>;

private:
    // indexed by constraint uid and of the same size: a row first seen after
    // the center was taken enters the center with a zero dual, which leaves
    // the center's Lagrangian value unchanged
    std::vector<scalar> _center;
    std::vector<scalar> _out;
    scalar _alpha;
    scalar _sep_alpha = scalar{0};
    std::size_t _num_mispricings = 0;
    scalar _master_value = scalar{0};
    // the master part of the center's Lagrangian value, see update()
    scalar _center_master_part = scalar{0};
    std::optional<scalar> _center_value;
    [[no_unique_address]] Better _better;

public:
    explicit dual_smoother(scalar alpha, Better better = {})
        : _alpha(alpha), _better(std::move(better)) {}

    // after an optimal solve of the master: copies the duals of the given
    // rows, all the rows the pricing reads, and starts a new pricing round
    template <std::ranges::input_range R, typename Duals>
    void set_out_point(R && constraints, const Duals & duals,
                       scalar master_value) {
        for(auto && c : constraints) {
            const std::size_t i = c.uid();
            if(i >= _out.size()) {
                _out.resize(i + 1);
                _center.resize(i + 1);
            }
            _out[i] = duals[c];
        }
        _master_value = master_value;
        _num_mispricings = 0;
        _sep_alpha = _center_value ? _alpha : scalar{0};
    }

    // computed on access, reads the current state: do not call a mutator
    // while pricing reads it
    [[nodiscard]] auto separation_point() const {
        return maps::function([this](const constraint & c) {
            const std::size_t i = c.uid();
            return _sep_alpha * _center[i] + (1 - _sep_alpha) * _out[i];
        });
    }
    [[nodiscard]] auto center() const {
        return maps::function(
            [this](const constraint & c) { return _center[c.uid()]; });
    }

    // pricing_part: the pricing's share of the Lagrangian value at the
    // separation point (see the design note); moves the center there if
    // that value is better. Valid only after a complete pricing round.
    bool update(scalar pricing_part) {
        // the master part is convex (concave) in the duals when the master
        // maximizes (minimizes) and master_value lies on its safe side at
        // pi_out, so interpolating never crosses the Lagrangian function
        const scalar master_part =
            _sep_alpha * _center_master_part + (1 - _sep_alpha) * _master_value;
        const scalar value = master_part + pricing_part;
        if(_center_value && !_better(value, *_center_value)) return false;
        for(std::size_t i = 0; i < _center.size(); ++i)
            _center[i] = _sep_alpha * _center[i] + (1 - _sep_alpha) * _out[i];
        _center_master_part = master_part;
        _center_value = value;
        return true;
    }

    // after a mispricing: the weight 1 - k(1 - alpha) of the k-th pricing
    // attempt on this pi_out, false if the separation point already was
    // pi_out
    bool reduce_alpha() {
        if(_sep_alpha == scalar{0}) return false;
        ++_num_mispricings;
        const scalar next =
            scalar{1} -
            static_cast<scalar>(_num_mispricings + 1) * (scalar{1} - _alpha);
        // a rounding residue would cost one more pricing round at ~pi_out
        _sep_alpha =
            next > 4 * std::numeric_limits<scalar>::epsilon() ? next : 0;
        return true;
    }

    [[nodiscard]] scalar alpha() const noexcept { return _alpha; }
    void set_alpha(scalar alpha) noexcept { _alpha = alpha; }
    [[nodiscard]] scalar separation_alpha() const noexcept {
        return _sep_alpha;
    }
    [[nodiscard]] std::size_t num_mispricings() const noexcept {
        return _num_mispricings;
    }
    [[nodiscard]] const std::optional<scalar> & center_value() const noexcept {
        return _center_value;
    }
    // after any change of the master other than new columns or new rows
    void reset() noexcept {
        _center_value.reset();
        _sep_alpha = scalar{0};
        _num_mispricings = 0;
    }
};

}  // namespace colgen
}  // namespace mippp
