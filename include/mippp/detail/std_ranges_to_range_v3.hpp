#ifndef MIPPP_STD_RANGES_TO_RANGE_V3_HPP
#define MIPPP_STD_RANGES_TO_RANGE_V3_HPP

#include <ranges>

#include <range/v3/range/concepts.hpp>

////////////////////////////// std into RANGE-V3 //////////////////////////////

#define COMMA ,
#define STD_RANGES_TO_RANGE_V3(templates, range_t)            \
    template <templates>                                      \
    constexpr bool ::ranges::enable_view<range_t> =           \
        std::ranges::viewable_range<range_t>;                 \
    template <templates>                                      \
    constexpr bool ::ranges::enable_borrowed_range<range_t> = \
        std::ranges::borrowed_range<range_t>;

STD_RANGES_TO_RANGE_V3(typename _Tp, std::ranges::empty_view<_Tp>)
STD_RANGES_TO_RANGE_V3(typename _Tp, std::ranges::single_view<_Tp>)
STD_RANGES_TO_RANGE_V3(typename _Wp COMMA typename _Bp,
                       std::ranges::iota_view<_Wp COMMA _Bp>)
STD_RANGES_TO_RANGE_V3(typename _Vp COMMA typename _Cp COMMA typename _Tp,
                       std::ranges::basic_istream_view<_Vp COMMA _Cp COMMA _Tp>)
STD_RANGES_TO_RANGE_V3(typename _Tp, std::ranges::ref_view<_Tp>)
STD_RANGES_TO_RANGE_V3(typename _Tp, std::ranges::owning_view<_Tp>)
// STD_RANGES_TO_RANGE_V3(typename _Tp, std::ranges::as_rvalue_view<_Tp>)
STD_RANGES_TO_RANGE_V3(typename _Tp COMMA typename _Fp,
                       std::ranges::transform_view<_Tp COMMA _Fp>)
STD_RANGES_TO_RANGE_V3(typename _Tp COMMA typename _Fp,
                       std::ranges::filter_view<_Tp COMMA _Fp>)
STD_RANGES_TO_RANGE_V3(typename _Tp, std::ranges::take_view<_Tp>)
STD_RANGES_TO_RANGE_V3(typename _Tp COMMA typename _Fp,
                       std::ranges::take_while_view<_Tp COMMA _Fp>)
STD_RANGES_TO_RANGE_V3(typename _Tp, std::ranges::drop_view<_Tp>)
STD_RANGES_TO_RANGE_V3(typename _Tp COMMA typename _Fp,
                       std::ranges::drop_while_view<_Tp COMMA _Fp>)
STD_RANGES_TO_RANGE_V3(typename _Tp, std::ranges::join_view<_Tp>)
// STD_RANGES_TO_RANGE_V3(typename _Tp COMMA typename _Fp,
//                        std::ranges::join_with_view<_Tp COMMA _Fp>)
STD_RANGES_TO_RANGE_V3(typename _Tp COMMA typename _Fp,
                       std::ranges::lazy_split_view<_Tp COMMA _Fp>)
STD_RANGES_TO_RANGE_V3(typename _Tp COMMA typename _Fp,
                       std::ranges::split_view<_Tp COMMA _Fp>)
// STD_RANGES_TO_RANGE_V3(typename... _Rngs,
//                        std::ranges::concat_view<_Rngs...>)
STD_RANGES_TO_RANGE_V3(typename _Tp, std::ranges::common_view<_Tp>)
STD_RANGES_TO_RANGE_V3(typename _Tp, std::ranges::reverse_view<_Tp>)
// STD_RANGES_TO_RANGE_V3(typename _Tp, std::ranges::const_view<_Tp>)
STD_RANGES_TO_RANGE_V3(typename _Tp COMMA std::size_t _N,
                       std::ranges::elements_view<_Tp COMMA _N>)
// STD_RANGES_TO_RANGE_V3(typename... _Rngs,
//                        std::ranges::zip_view<_Rngs...>)
// STD_RANGES_TO_RANGE_V3(typename _Fp COMMA typename... _Rngs,
//                        std::ranges::zip_view<_Fp, _Rngs...>)
// STD_RANGES_TO_RANGE_V3(typename _Tp COMMA std::size_t _N,
//                        std::ranges::adjacent_view<_Tp COMMA _N>)

// TODO: other c++23 and c++26 views

#endif // MIPPP_STD_RANGES_TO_RANGE_V3_HPP