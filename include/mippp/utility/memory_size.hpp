#pragma once

#include <cstddef>
#include <ratio>
#include <type_traits>

namespace mippp {

// Mirrors std::chrono::duration: the rep/period member types, the value read
// through count(), and the same implicit-conversion rule -- a conversion is
// allowed only when it cannot lose precision, unless the target
// representation is floating point.
template <typename Rep, typename Ratio = std::ratio<1>>
class memory_size {
public:
    using rep = Rep;
    using period = Ratio;

private:
    // value-initialized, unlike chrono's uninitialized rep: a memory limit
    // read from a default-constructed object is a number, not garbage
    Rep _count{};

public:
    constexpr memory_size() = default;
    constexpr memory_size(const memory_size &) = default;
    constexpr memory_size(memory_size &&) = default;
    constexpr memory_size & operator=(const memory_size &) = default;
    constexpr memory_size & operator=(memory_size &&) = default;

    constexpr explicit memory_size(Rep c) : _count(c) {}

    template <typename Rep2, typename Ratio2,
              typename CF = std::ratio_divide<Ratio2, Ratio>>
        requires(std::is_floating_point_v<Rep> ||
                 (CF::den == 1 && !std::is_floating_point_v<Rep2>))
    constexpr memory_size(memory_size<Rep2, Ratio2> other)
        : _count(static_cast<Rep>(other.count()) * CF::num / CF::den) {}

    [[nodiscard]] constexpr Rep count() const noexcept { return _count; }
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
