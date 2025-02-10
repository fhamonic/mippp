#ifndef MIPPP_RANGE_V3_COMPATIBILITY_HPP
#define MIPPP_RANGE_V3_COMPATIBILITY_HPP

#include <ranges>

#include <range/v3/range/concepts.hpp>
#include <range/v3/view/concat.hpp>

////////////////////////////// std into RANGE-V3 //////////////////////////////

template <typename _Tp>
constexpr bool ::ranges::enable_view<std::ranges::ref_view<_Tp>> =
    std::ranges::viewable_range<std::ranges::ref_view<_Tp>>;

template <typename _Vp>
constexpr bool ::ranges::enable_view<std::ranges::single_view<_Vp>> =
    std::ranges::viewable_range<std::ranges::single_view<_Vp>>;

template <typename _Tp, typename _Fp>
constexpr bool ::ranges::enable_view<std::ranges::transform_view<_Tp, _Fp>> =
    std::ranges::viewable_range<std::ranges::transform_view<_Tp, _Fp>>;

template <typename _Tp>
constexpr bool ::ranges::enable_view<std::ranges::join_view<_Tp>> =
    std::ranges::viewable_range<std::ranges::join_view<_Tp>>;

template <typename _Tp>
constexpr bool ::ranges::enable_borrowed_range<std::ranges::ref_view<_Tp>> =
    true;

////////////////////////////// RANGE-V3 into std //////////////////////////////

template <typename... Rngs>
constexpr bool std::ranges::enable_view<::ranges::concat_view<Rngs...>> =
    ::ranges::viewable_range<::ranges::concat_view<Rngs...>>;

template <typename _Tp>
constexpr bool std::ranges::enable_borrowed_range<::ranges::ref_view<_Tp>> =
    true;

#endif  // MIPPP_RANGE_V3_COMPATIBILITY_HPP