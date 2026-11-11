#pragma once

#include <concepts>
#include <variant>

#include "mippp/model_concepts.hpp"
#include "mippp/utility/variant.hpp"

namespace mippp::detail {

// What set_lp_algorithm() takes: A itself must be an alternative. A setter
// taking the tags by value would convert a refinement to the base it lists
// and run interior_point_without_crossover with a crossover.
template <typename A, typename Variant>
concept settable_lp_algorithm = variant_with_alternative<Variant, A> &&
                                !std::same_as<A, lp_algorithm::other>;

template <typename A, typename Variant, typename Knobs, typename Encode>
constexpr bool decodes_as_lp_algorithm(Variant & algorithm, const Knobs & knobs,
                                       Encode & encode) {
    if constexpr(std::same_as<A, lp_algorithm::other>) {
        return false;
    } else {
        if(!(encode(A{}) == knobs)) return false;
        algorithm = A{};
        return true;
    }
}

// The tag whose encoding is knobs, and what `otherwise` holds when no
// alternative encodes to them. Decoding through the encoder keeps
// set_lp_algorithm(get_lp_algorithm()) from changing a knob: a decoder of
// its own that named a tag for knobs the setter writes differently would
// make restoring a saved algorithm reset them.
template <typename... As, typename Knobs, typename Encode>
[[nodiscard]] constexpr std::variant<As...> decode_lp_algorithm(
    std::variant<As...> otherwise, const Knobs & knobs, Encode encode) {
    (void)(decodes_as_lp_algorithm<As>(otherwise, knobs, encode) || ...);
    return otherwise;
}

}  // namespace mippp::detail
