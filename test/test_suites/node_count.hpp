#pragma once

#include <gtest/gtest.h>

#include <cstddef>
#include <random>
#include <ranges>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"

namespace mippp {

template <typename T>
struct NodeCountTest : public T {
    using typename T::model_type;
    static_assert(milp_model<model_type>);
    static_assert(has_node_count<model_type>);

    // Cornuejols-Dawande market split: m dense equality rows over 10 (m - 1)
    // binaries, whose slacks keep it feasible and are minimized. The
    // relaxation reaches 0 and no cut lifts it, so proving a positive optimum
    // takes a search.
    static auto build_market_split(model_type & model, std::size_t m) {
        using namespace operators;
        const std::size_t n = 10 * (m - 1);
        std::mt19937 rng(42);
        std::uniform_int_distribution<int> coef(0, 99);
        auto x = model.add_binary_variables(n);
        auto plus = model.add_variables(m);
        auto minus = model.add_variables(m);
        model.set_minimization();
        model.set_objective(xsum(std::views::iota(std::size_t{0}, m),
                                 [&](auto i) { return plus(i) + minus(i); }));
        std::vector<int> row(n);
        for(std::size_t i = 0; i < m; ++i) {
            int total = 0;
            for(int & a : row) total += (a = coef(rng));
            model.add_constraint(xsum(std::views::iota(std::size_t{0}, n),
                                      [&](auto j) { return row[j] * x(j); }) +
                                     plus(i) - minus(i) ==
                                 total / 2);
        }
        return x;
    }
};
TYPED_TEST_SUITE_P(NodeCountTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(NodeCountTest);

TYPED_TEST_P(NodeCountTest, search_explores_nodes) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        TestFixture::build_market_split(model, 3);
        model.solve();
        ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
        ASSERT_GT(model.get_solution_value(), 0.5);
        EXPECT_GT(model.get_node_count(), 1u);
    });
}

// Relaxed after a search, the model is solved as an LP: no node of the
// earlier search may remain.
TYPED_TEST_P(NodeCountTest, integer_free_model_explores_no_node) {
    this->SkipOnLicenseError([this]() {
        constexpr std::size_t m = 3;
        auto model = this->new_model();
        auto x = TestFixture::build_market_split(model, m);
        model.solve();
        ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
        for(std::size_t j = 0; j < 10 * (m - 1); ++j)
            model.set_continuous(x(j));
        model.solve();
        ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
        EXPECT_EQ(model.get_node_count(), 0u);
    });
}

REGISTER_TYPED_TEST_SUITE_P(NodeCountTest, search_explores_nodes,
                            integer_free_model_explores_no_node);

}  // namespace mippp
