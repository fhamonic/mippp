#ifndef RANGES_TEST_HELPER_HPP
#define RANGES_TEST_HELPER_HPP

#include <initializer_list>
#include <ranges>

#include <range/v3/view/zip.hpp>

template <typename R1, typename R2>
void ASSERT_EQ_RANGES(R1 && r1, R2 && r2) {
    ASSERT_EQ(std::ranges::distance(r1), std::ranges::distance(r2));
    for(const auto & [e1, e2] : ranges::views::zip(r1, r2)) {
        ASSERT_EQ(e1, e2);
    }
}

template <typename R1, typename T>
requires std::same_as<std::ranges::range_value_t<R1>, T>
void ASSERT_EQ_RANGES(R1 && r1, std::initializer_list<T> l) {
    ASSERT_EQ(std::ranges::size(r1), l.size());
    for(const auto & [e1, e2] : ranges::views::zip(r1, l)) {
        ASSERT_EQ(e1, e2);
    }
}

#endif //RANGES_TEST_HELPER_HPP