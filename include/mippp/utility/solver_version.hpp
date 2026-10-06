#pragma once

#include <array>
#include <compare>
#include <cstddef>
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

// Releases a backend has driven through the full suite, as a half-open
// interval: {{10}, {14}} is every 10.x.y up to 13.x.y. Both bounds come
// from a recorded pass -- a row of docs/solvers/compatibility.md or, for the
// solvers the matrix cannot obtain or license, a maintainer's run noted on
// that page; `before` is the first release not covered, never "anything
// newer". `<solver>_api::validated_versions` is an array of these, and so is
// the wider `supported_versions` a backend may declare, whose bounds come from
// partial passes: the lp_model core passed, and only members the release lacks
// threw feature_unavailable_error.
struct solver_version_range {
    solver_version from, before;

    constexpr bool contains(const solver_version & v) const noexcept {
        return from <= v && v < before;
    }
};

template <std::size_t N>
constexpr bool contains(const std::array<solver_version_range, N> & ranges,
                        const solver_version & v) noexcept {
    for(const auto & r : ranges)
        if(r.contains(v)) return true;
    return false;
}

template <std::size_t N>
constexpr bool is_validated(const std::array<solver_version_range, N> & ranges,
                            const solver_version & v) noexcept {
    static_assert(N > 0, "a wrapper validated for nothing is not a claim");
    return contains(ranges, v);
}

// whether every range of `inner` lies inside one range of `outer`
template <std::size_t N, std::size_t M>
constexpr bool covers(
    const std::array<solver_version_range, N> & outer,
    const std::array<solver_version_range, M> & inner) noexcept {
    for(const auto & i : inner) {
        bool inside = false;
        for(const auto & o : outer)
            inside = inside || (o.from <= i.from && i.before <= o.before);
        if(!inside) return false;
    }
    return true;
}

// How far a backend vouches for a release, see `<solver>_api::support_of`.
// validated: in validated_versions, where every shared suite passed. partial:
// in supported_versions only, where models build and solve and the members
// the release lacks throw feature_unavailable_error. untested: in neither, or
// no release number at all. All three load.
enum class release_support : unsigned char { validated, partial, untested };

inline std::string to_string(release_support support) {
    switch(support) {
        case release_support::validated:
            return "validated";
        case release_support::partial:
            return "partial";
        default:
            return "untested";
    }
}

template <std::size_t N>
std::string to_string(const std::array<solver_version_range, N> & ranges) {
    std::string s;
    for(const auto & r : ranges) {
        if(!s.empty()) s += " or ";
        s += ">= " + to_string(r.from) + " and < " + to_string(r.before);
    }
    return s;
}

}  // namespace mippp
