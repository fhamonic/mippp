#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include "mippp/solvers/cbc/all.hpp"
#include "mippp/solvers/clp/all.hpp"
#include "mippp/solvers/highs/all.hpp"
#include "mippp/utility/linear_iis.hpp"

// Numerical data adapted from HiGHS 1.12.0, check/TestIis.cpp.
// See data/iis/README.md and data/iis/HIGHS-LICENSE.txt for provenance.
namespace {
using namespace mippp::iis;

struct test_vector {
    const char * name;
    linear_system<> system;
    std::vector<std::size_t> iis_sizes;
};

std::vector<test_vector> published_vectors() {
    const auto free = std::nullopt;
    return {
        {"lp-incompatible-bounds",
         {{{0., 1.}, {0., 1.}, {0., -1.}},
          {{{{1, 1.}, {2, 1.}}, 1., 0.},
           {{{0, 1.}, {2, 1.}}, 0., 1.}}},
         {2, 3}},
        {"lp-empty-infeasible-row/lower",
         {{{0., free}, {0., free}},
          {{{{0, 2.}, {1, 1.}}, free, 8.},
           {{}, 1., 2.},
           {{{0, 1.}, {1, 3.}}, free, 9.}}},
         {1}},
        {"lp-empty-infeasible-row/upper",
         {{{0., free}, {0., free}},
          {{{{0, 2.}, {1, 1.}}, free, 8.},
           {{}, -2., -1.},
           {{{0, 1.}, {1, 3.}}, free, 9.}}},
         {1}},
        {"lp-get-iis",
         {{{0., free}, {0., free}},
          {{{{0, 2.}, {1, 1.}}, free, 8.},
           {{{0, 1.}, {1, 3.}}, free, 9.},
           {{{0, 1.}, {1, 1.}}, free, -2.}}},
         {3}}};
}

std::vector<member> all_members(const linear_system<> & system) {
    std::vector<member> members;
    for(std::size_t i = 0; i < system.variables.size(); ++i) {
        if(system.variables[i].lower)
            members.push_back({member_kind::variable_lower, i});
        if(system.variables[i].upper)
            members.push_back({member_kind::variable_upper, i});
    }
    for(std::size_t i = 0; i < system.rows.size(); ++i) {
        if(system.rows[i].lower) members.push_back({member_kind::row_lower, i});
        if(system.rows[i].upper) members.push_back({member_kind::row_upper, i});
    }
    return members;
}

// Independent Fourier-Motzkin elimination, restricted to these tiny continuous
// fixtures. Rows are a*x <= b. No solver or production IIS helper is used.
// The integer/dyadic fixture coefficients stay small and exactly representable;
// this is not a general numerical LP solver or an integer-feasibility oracle.
bool feasible(const linear_system<> & system, std::span<const member> subset) {
    const auto n = system.variables.size();
    std::vector<std::vector<double>> rows;
    for(auto [kind, index] : subset) {
        std::vector<double> row(n + 1, 0.);
        const bool lower = kind == member_kind::row_lower ||
                           kind == member_kind::variable_lower;
        const double sign = lower ? -1. : 1.;
        if(kind == member_kind::row_lower || kind == member_kind::row_upper) {
            const auto & source = system.rows.at(index);
            for(auto [column, value] : source.terms)
                row.at(column) += sign * value;
            row[n] = sign * (lower ? source.lower.value() : source.upper.value());
        } else {
            const auto & source = system.variables.at(index);
            row.at(index) = sign;
            row[n] = sign * (lower ? source.lower.value() : source.upper.value());
        }
        rows.push_back(std::move(row));
    }
    for(std::size_t col = 0; col < n; ++col) {
        std::vector<std::vector<double>> next;
        for(const auto & row : rows)
            if(row[col] == 0.) next.push_back(row);
        for(const auto & positive : rows) {
            if(positive[col] <= 0.) continue;
            for(const auto & negative : rows) {
                if(negative[col] >= 0.) continue;
                std::vector<double> row(n + 1, 0.);
                for(std::size_t j = col + 1; j <= n; ++j)
                    row[j] = positive[j] * -negative[col] +
                             negative[j] * positive[col];
                next.push_back(std::move(row));
            }
        }
        rows = std::move(next);
    }
    return std::ranges::all_of(rows, [n](const auto & row) {
        return row[n] >= 0.;
    });
}

void verify_iis(const test_vector & input, const linear_result & answer) {
    ASSERT_TRUE(answer.reduction.proven_infeasible());
    ASSERT_TRUE(answer.reduction.irreducible);
    EXPECT_EQ(answer.reduction.reason, termination::completed);
    const auto candidates = all_members(input.system);
    for(auto m : answer.members) {
        ASSERT_NE(std::ranges::find(candidates, m), candidates.end());
        ASSERT_EQ(std::ranges::count(answer.members, m), 1);
    }
    EXPECT_NE(std::ranges::find(input.iis_sizes, answer.members.size()),
              input.iis_sizes.end());
    EXPECT_FALSE(feasible(input.system, answer.members));
    for(std::size_t i = 0; i < answer.members.size(); ++i) {
        auto remaining = answer.members;
        remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(i));
        EXPECT_TRUE(feasible(input.system, remaining));
    }
}

TEST(IisVectors, IndependentOracleAndPublishedConflicts) {
    for(const auto & input : published_vectors()) {
        SCOPED_TRACE(input.name);
        const auto candidates = all_members(input.system);
        EXPECT_FALSE(feasible(input.system, candidates));
        EXPECT_TRUE(feasible(input.system, {}));
        // Enumerate every subset to validate the known size independently of
        // deletion order. Several different IISs may be valid for one model.
        std::size_t conflicts = 0;
        for(std::size_t mask = 0; mask < (std::size_t{1} << candidates.size());
            ++mask) {
            std::vector<member> subset;
            for(std::size_t i = 0; i < candidates.size(); ++i)
                if(mask & (std::size_t{1} << i)) subset.push_back(candidates[i]);
            if(feasible(input.system, subset)) continue;
            bool minimal = true;
            for(std::size_t i = 0; i < subset.size(); ++i) {
                auto remaining = subset;
                remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(i));
                minimal &= feasible(input.system, remaining);
            }
            if(minimal) {
                ++conflicts;
                EXPECT_NE(std::ranges::find(input.iis_sizes, subset.size()),
                          input.iis_sizes.end());
            }
        }
        EXPECT_GT(conflicts, 0u);
    }
}

template <typename Model>
class PublishedIis : public ::testing::Test {};
using Backends = ::testing::Types<mippp::highs_lp, mippp::highs_milp,
                                  mippp::clp_lp, mippp::cbc_milp>;
TYPED_TEST_SUITE(PublishedIis, Backends);

template <linear_policy Policy, typename Model>
void check_vectors() {
    for(auto input : published_vectors()) {
        SCOPED_TRACE(input.name);
        // Positive scaling preserves feasible subsets. Reversing rows checks
        // that returned identities refer to input rows, not traversal order.
        for(double scale : {1., 0.25, 4.}) {
            auto transformed = input;
            for(auto & row : transformed.system.rows) {
                for(auto & [column, value] : row.terms) value *= scale;
                if(row.lower) *row.lower *= scale;
                if(row.upper) *row.upper *= scale;
            }
            std::ranges::reverse(transformed.system.rows);
            SCOPED_TRACE(scale);
            const auto answer = compute_linear_iis<Policy>(
                transformed.system, [] { return Model{}; },
                linear_options{.limits = {.initial_batch_size = 2},
                               .order = rows_first_order{}});
            verify_iis(transformed, answer);
        }
    }
}

TYPED_TEST(PublishedIis, RebuiltDeletion) {
    check_vectors<linear_policy{}, TypeParam>();
}
TYPED_TEST(PublishedIis, RetainedDeletion) {
    check_vectors<linear_policy{.deletion = deletion_strategy::reuse}, TypeParam>();
}
TYPED_TEST(PublishedIis, RebuiltElasticity) {
    check_vectors<linear_policy{.elasticity = elasticity_strategy::rebuild},
                  TypeParam>();
}
TYPED_TEST(PublishedIis, RetainedElasticityAndDeletion) {
    check_vectors<linear_policy{.elasticity = elasticity_strategy::reuse,
                               .deletion = deletion_strategy::reuse}, TypeParam>();
}

TYPED_TEST(PublishedIis, IntegerBoundsWithFeasibleRelaxation) {
    // HiGHS TestMipSolver.cpp: x + 2y >= 1, x >= 0,
    // 1/4 <= y <= 3/4, with y integer. Its original objective is unbounded
    // on the LP relaxation; IIS extraction instead uses a zero objective.
    linear_system<> system;
    system.variables = {{0., std::nullopt, false}, {0.25, 0.75, true}};
    system.rows = {{{{0, 1.}, {1, 2.}}, 1., std::nullopt}};
    auto factory = [] { return TypeParam{}; };
    if constexpr(mippp::milp_model<TypeParam>) {
        auto check = [&]<linear_policy Policy>() {
            const auto answer = compute_linear_iis<Policy>(system, factory);
            ASSERT_TRUE(answer.reduction.irreducible);
            ASSERT_TRUE(answer.reduction.proven_infeasible());
            ASSERT_EQ(answer.members.size(), 2u);
            for(auto kind : {member_kind::variable_lower,
                             member_kind::variable_upper})
                EXPECT_NE(std::ranges::find(answer.members, member{kind, 1}),
                          answer.members.end());
            // Independent proof: no integer is in [1/4,3/4]. Dropping the
            // lower side admits (x,y)=(1,0), dropping the upper admits (0,1).
            EXPECT_GT(std::ceil(0.25), std::floor(0.75));
            EXPECT_EQ(answer.elasticity_calls, 0u);
        };
        check.template operator()<linear_policy{}>();
        check.template operator()<linear_policy{
            .elasticity = elasticity_strategy::reuse,
            .deletion = deletion_strategy::reuse}>();
    }
    const auto relaxed = compute_linear_iis<linear_policy{
        .analyzed_domain = domain::lp_relaxation}>(system, factory);
    EXPECT_EQ(relaxed.reduction.initial_status, feasibility::feasible);
    EXPECT_TRUE(relaxed.members.empty());
    // A one-bound repair admits the integer witness (0,1).
    system.variables[1].upper = 1.;
    if constexpr(mippp::milp_model<TypeParam>) {
        const auto repaired = compute_linear_iis(system, factory);
        EXPECT_EQ(repaired.reduction.initial_status, feasibility::feasible);
        EXPECT_TRUE(repaired.members.empty());
    }
}
}  // namespace
