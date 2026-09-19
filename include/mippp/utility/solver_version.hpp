#pragma once

#include <compare>
#include <string>

namespace mippp {

// A solver release number, ordered numerically component by component;
// absent components are 0, so {13} is 13.0.0 and {1, 8} < {1, 10}.
struct solver_version {
    int major = 0, minor = 0, patch = 0;

    friend constexpr auto operator<=>(
        const solver_version &, const solver_version &) noexcept = default;
};

// trailing zero components are left out: {1, 8, 0} -> "1.8", {13} -> "13"
inline std::string to_string(const solver_version & v) {
    std::string s = std::to_string(v.major);
    if(v.minor != 0 || v.patch != 0) s += '.' + std::to_string(v.minor);
    if(v.patch != 0) s += '.' + std::to_string(v.patch);
    return s;
}

}  // namespace mippp
