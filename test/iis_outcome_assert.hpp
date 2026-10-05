#pragma once

#include <gtest/gtest.h>

#include <concepts>
#include <ostream>
#include <sstream>
#include <string>
#include <string_view>
#include <variant>

#include "mippp/utility/iis_outcome.hpp"

namespace mippp::iis_outcome {

namespace print_detail {
// One overload per tag: a missing one would name the tag by its base.
inline std::string_view name_of(const irreducible &) { return "irreducible"; }
inline std::string_view name_of(const feasible &) { return "feasible"; }
inline std::string_view name_of(const incomplete &) { return "incomplete"; }
inline std::string_view name_of(const inconclusive_trial &) {
    return "inconclusive_trial";
}
inline std::string_view name_of(const stopped &) { return "stopped"; }
inline std::string_view name_of(const interrupted &) { return "interrupted"; }
inline std::string_view name_of(const limit_reached &) {
    return "limit_reached";
}
inline std::string_view name_of(const time_limit &) { return "time_limit"; }
inline std::string_view name_of(const trial_limit &) { return "trial_limit"; }
inline std::string_view name_of(const iteration_limit &) {
    return "iteration_limit";
}
inline std::string_view name_of(const node_limit &) { return "node_limit"; }
inline std::string_view name_of(const memory_limit &) { return "memory_limit"; }

inline std::string_view flag_text(bool conflict) {
    return conflict ? " (conflict held)" : " (no conflict)";
}

// The completed tags fix the flag, so only an undecided one shows it.
template <std::derived_from<any> Tag>
void print(const Tag & tag, std::ostream * os) {
    *os << name_of(tag);
    if constexpr(std::derived_from<Tag, incomplete>)
        *os << flag_text(tag.conflict_available);
}
}  // namespace print_detail

// Found by ADL from GoogleTest's printer of a variant's alternative. Exact
// overloads: its catch-all PrintTo template beats a conversion to a base.
inline void PrintTo(const irreducible & t, std::ostream * os) {
    print_detail::print(t, os);
}
inline void PrintTo(const feasible & t, std::ostream * os) {
    print_detail::print(t, os);
}
inline void PrintTo(const incomplete & t, std::ostream * os) {
    print_detail::print(t, os);
}
inline void PrintTo(const inconclusive_trial & t, std::ostream * os) {
    print_detail::print(t, os);
}
inline void PrintTo(const stopped & t, std::ostream * os) {
    print_detail::print(t, os);
}
inline void PrintTo(const interrupted & t, std::ostream * os) {
    print_detail::print(t, os);
}
inline void PrintTo(const limit_reached & t, std::ostream * os) {
    print_detail::print(t, os);
}
inline void PrintTo(const time_limit & t, std::ostream * os) {
    print_detail::print(t, os);
}
inline void PrintTo(const trial_limit & t, std::ostream * os) {
    print_detail::print(t, os);
}
inline void PrintTo(const iteration_limit & t, std::ostream * os) {
    print_detail::print(t, os);
}
inline void PrintTo(const node_limit & t, std::ostream * os) {
    print_detail::print(t, os);
}
inline void PrintTo(const memory_limit & t, std::ostream * os) {
    print_detail::print(t, os);
}

}  // namespace mippp::iis_outcome

namespace iis_outcome_assert_detail {
template <typename Outcome>
std::string describe(const Outcome & o) {
    std::ostringstream out;
    std::visit(
        [&](const auto & tag) {
            mippp::iis_outcome::print_detail::print(tag, &out);
        },
        o);
    return out.str();
}
}  // namespace iis_outcome_assert_detail

// The exact tag, as is<Tag>: a Tag the path does not list does not compile,
// and a base such as stopped misses the tags under it.
template <typename Tag, mippp::variant_of<mippp::iis_outcome::any> Outcome>
::testing::AssertionResult outcome_is(const Outcome & o)
    requires mippp::variant_with_alternative<Outcome, Tag>
{
    if(mippp::is<Tag>(o)) return ::testing::AssertionSuccess();
    return ::testing::AssertionFailure()
           << "the outcome is " << iis_outcome_assert_detail::describe(o)
           << ", not " << mippp::iis_outcome::print_detail::name_of(Tag{});
}

// The exact tag and whether it holds a conflict.
template <typename Tag, mippp::variant_of<mippp::iis_outcome::any> Outcome>
::testing::AssertionResult outcome_is(const Outcome & o, bool conflict)
    requires mippp::variant_with_alternative<Outcome, Tag>
{
    if(mippp::is<Tag>(o) &&
       mippp::iis_outcome::conflict_available(o) == conflict)
        return ::testing::AssertionSuccess();
    return ::testing::AssertionFailure()
           << "the outcome is " << iis_outcome_assert_detail::describe(o)
           << ", not " << mippp::iis_outcome::print_detail::name_of(Tag{})
           << mippp::iis_outcome::print_detail::flag_text(conflict);
}
