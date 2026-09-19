#include <gtest/gtest.h>

#include <variant>

#include "mippp/utility/status.hpp"

// the one include is the point: the status vocabulary is public API

using namespace mippp;

using SV = std::variant<status::unknown, status::optimal,
                        status::infeasible_or_unbounded, status::infeasible,
                        status::unbounded, status::time_limit>;

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Concepts /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

static_assert(variant_of<SV, status::any>);
static_assert(variant_of<const SV &, status::any>);
static_assert(!variant_of<std::variant<int, status::optimal>, status::any>);

static_assert(variant_with_alternative<SV, status::infeasible_or_unbounded>);
static_assert(
    !variant_with_alternative<SV, status::primal_and_dual_infeasible>);
// exact alternative, not a base of one
static_assert(!variant_with_alternative<SV, status::completed>);

static_assert(variant_containing_a<SV, status::completed>);
static_assert(variant_containing_a<SV, status::limit_reached>);
static_assert(!variant_containing_a<SV, status::node_limit>);

// is<S> / is_a<S> reject a tag the variant cannot answer for
template <typename S, typename V>
concept is_callable = requires(const V & v) { mippp::is<S>(v); };
template <typename S, typename V>
concept is_a_callable = requires(const V & v) { mippp::is_a<S>(v); };
static_assert(is_callable<status::infeasible_or_unbounded, SV>);
static_assert(!is_callable<status::primal_and_dual_infeasible, SV>);
static_assert(!is_callable<status::completed, SV>);
static_assert(is_a_callable<status::completed, SV>);
static_assert(!is_a_callable<status::node_limit, SV>);

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Queries /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

TEST(status, is_matches_the_exact_tag_only) {
    SV r = status::infeasible{};
    ASSERT_TRUE(is<status::infeasible>(r));
    ASSERT_FALSE(is<status::infeasible_or_unbounded>(r));
    ASSERT_FALSE(is<status::unbounded>(r));
}

TEST(status, is_a_matches_the_whole_branch) {
    SV r = status::infeasible{};
    ASSERT_TRUE(is_a<status::infeasible>(r));
    ASSERT_TRUE(is_a<status::infeasible_or_unbounded>(r));
    ASSERT_TRUE(is_a<status::completed>(r));
    ASSERT_TRUE(is_a<status::any>(r));
    ASSERT_FALSE(is_a<status::unbounded>(r));
    ASSERT_FALSE(is_a<status::stopped>(r));
}

TEST(status, branches_are_proofs_not_a_partition) {
    SV r = status::time_limit{};
    ASSERT_FALSE(is_a<status::completed>(r));
    ASSERT_FALSE(is_a<status::infeasible>(r));
    ASSERT_TRUE(is_a<status::limit_reached>(r));
}

TEST(status, solution_available_follows_the_tag_flag) {
    ASSERT_TRUE(status::solution_available(SV{status::optimal{}}));
    ASSERT_FALSE(status::solution_available(SV{status::infeasible{}}));
    ASSERT_FALSE(status::solution_available(SV{status::unknown{}}));
    ASSERT_FALSE(status::solution_available(SV{status::time_limit{}}));
    ASSERT_TRUE(status::solution_available(SV{status::time_limit{true}}));
}

TEST(status, queries_are_constexpr) {
    static_assert(is<status::optimal>(SV{status::optimal{}}));
    static_assert(is_a<status::completed>(SV{status::unbounded{}}));
    static_assert(status::solution_available(SV{status::optimal{}}));
}
