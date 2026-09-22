#pragma once

#include <cstddef>
#include <iterator>
#include <ranges>
#include <tuple>
#include <type_traits>
#include <utility>
#include <version>

namespace mippp::detail {

// MIPPP_PORTABLE_RANGE_SHAPES forces the fallback where the standard adaptor
// exists, so a libstdc++ build can exercise what libc++ users get.
#if defined(__cpp_lib_ranges_cartesian_product) && \
    !defined(MIPPP_PORTABLE_RANGE_SHAPES)
template <typename... Vs>
using cartesian_product_view = std::ranges::cartesian_product_view<Vs...>;

template <std::ranges::viewable_range R1, std::ranges::viewable_range R2>
constexpr auto cartesian_product(R1 && r1, R2 && r2) {
    return std::views::cartesian_product(std::forward<R1>(r1),
                                         std::forward<R2>(r2));
}
#else

// Binary std::views::cartesian_product for standard libraries without it
// (libc++ as of LLVM 21). The second operand is walked once per element of
// the first, hence forward_range on it.
template <std::ranges::view V1, std::ranges::view V2>
    requires std::ranges::input_range<V1> && std::ranges::forward_range<V2>
class cartesian_product_view
    : public std::ranges::view_interface<cartesian_product_view<V1, V2> > {
private:
    V1 _first;
    V2 _second;

    template <bool Const>
    using base_t = std::conditional_t<Const, const cartesian_product_view,
                                      cartesian_product_view>;

    template <bool Const>
    class iterator {
        using Parent = base_t<Const>;
        using FirstBase = std::conditional_t<Const, const V1, V1>;
        using SecondBase = std::conditional_t<Const, const V2, V2>;

        std::ranges::iterator_t<FirstBase> _first_it{};
        std::ranges::sentinel_t<FirstBase> _first_end{};
        std::ranges::iterator_t<SecondBase> _second_begin{};
        std::ranges::iterator_t<SecondBase> _second_it{};
        std::ranges::sentinel_t<SecondBase> _second_end{};
        // an empty second operand empties the product; a flag rather than
        // moving _first_it to its end, which non-common ranges cannot do
        bool _exhausted = false;

    public:
        using iterator_concept =
            std::conditional_t<std::ranges::forward_range<FirstBase>,
                               std::forward_iterator_tag,
                               std::input_iterator_tag>;
        using iterator_category = std::input_iterator_tag;
        using difference_type = std::ptrdiff_t;
        using value_type = std::tuple<std::ranges::range_value_t<FirstBase>,
                                      std::ranges::range_value_t<SecondBase> >;
        using reference =
            std::tuple<std::ranges::range_reference_t<FirstBase>,
                       std::ranges::range_reference_t<SecondBase> >;

        iterator() = default;

        constexpr iterator(Parent & parent)
            : _first_it(std::ranges::begin(parent._first))
            , _first_end(std::ranges::end(parent._first))
            , _second_begin(std::ranges::begin(parent._second))
            , _second_it(_second_begin)
            , _second_end(std::ranges::end(parent._second))
            , _exhausted(_second_begin == _second_end) {}

        constexpr reference operator*() const {
            return reference(*_first_it, *_second_it);
        }

        constexpr iterator & operator++() {
            if(++_second_it == _second_end) {
                _second_it = _second_begin;
                ++_first_it;
            }
            return *this;
        }

        constexpr void operator++(int) { ++(*this); }
        constexpr iterator operator++(int)
            requires std::forward_iterator<std::ranges::iterator_t<FirstBase> >
        {
            auto tmp = *this;
            ++(*this);
            return tmp;
        }

        friend constexpr bool operator==(const iterator & it,
                                         std::default_sentinel_t) noexcept {
            return it._exhausted || it._first_it == it._first_end;
        }
        friend constexpr bool operator==(const iterator & lhs,
                                         const iterator & rhs)
            requires std::equality_comparable<
                std::ranges::iterator_t<FirstBase> >
        {
            return lhs._first_it == rhs._first_it &&
                   lhs._second_it == rhs._second_it;
        }
    };

public:
    cartesian_product_view()
        requires std::default_initializable<V1> &&
                     std::default_initializable<V2>
    = default;

    constexpr cartesian_product_view(V1 first, V2 second)
        : _first(std::move(first)), _second(std::move(second)) {}

    constexpr auto begin() { return iterator<false>(*this); }

    constexpr auto begin() const
        requires std::ranges::range<const V1> && std::ranges::range<const V2>
    {
        return iterator<true>(*this);
    }

    constexpr auto end() const noexcept { return std::default_sentinel; }
    constexpr auto end() noexcept { return std::default_sentinel; }

    constexpr auto size()
        requires std::ranges::sized_range<V1> && std::ranges::sized_range<V2>
    {
        return std::ranges::size(_first) * std::ranges::size(_second);
    }
    constexpr auto size() const
        requires std::ranges::sized_range<const V1> &&
                 std::ranges::sized_range<const V2>
    {
        return std::ranges::size(_first) * std::ranges::size(_second);
    }
};

template <typename R1, typename R2>
cartesian_product_view(R1 &&, R2 &&)
    -> cartesian_product_view<std::views::all_t<R1>, std::views::all_t<R2> >;

template <std::ranges::viewable_range R1, std::ranges::viewable_range R2>
constexpr auto cartesian_product(R1 && r1, R2 && r2) {
    return cartesian_product_view(std::forward<R1>(r1), std::forward<R2>(r2));
}
#endif

}  // namespace mippp::detail
