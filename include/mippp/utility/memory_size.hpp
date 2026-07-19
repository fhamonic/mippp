#pragma once

#include <ratio>
#include <type_traits>

namespace mippp {

template <typename Rep, typename Ratio = std::ratio<1>>
struct memory_size {
    Rep count;
    constexpr explicit memory_size(Rep c) : count(c) {}
    template <typename Rep2, typename Ratio2,
              typename CF = std::ratio_divide<Ratio2, Ratio>>
        requires(std::is_floating_point_v<Rep> ||
                 (CF::den == 1 && !std::is_floating_point_v<Rep2>))
    constexpr memory_size(memory_size<Rep2, Ratio2> other)
        : count(static_cast<Rep>(other.count) * CF::num / CF::den) {}
};

using bytes = memory_size<std::size_t>;
using kilobytes = memory_size<std::size_t, std::kilo>;
using megabytes = memory_size<std::size_t, std::mega>;
using gigabytes = memory_size<std::size_t, std::giga>;

using kibi = std::ratio<1024>;
using mebi = std::ratio<1024LL * 1024>;
using gibi = std::ratio<1024LL * 1024 * 1024>;

using kibibytes = memory_size<std::size_t, kibi>;
using mebibytes = memory_size<std::size_t, mebi>;
using gibibytes = memory_size<std::size_t, gibi>;

}  // namespace mippp