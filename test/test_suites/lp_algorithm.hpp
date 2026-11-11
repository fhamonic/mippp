#pragma once

#include <gtest/gtest.h>

#include <array>
#include <concepts>
#include <cstddef>
#include <exception>
#include <iostream>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/utility/iis_by_deletion.hpp"
#include "mippp/utility/solver_exceptions.hpp"
#include "mippp/utility/status.hpp"
#include "mippp/utility/variant.hpp"

#include "iis_cases.hpp"
#include "iis_outcome_assert.hpp"
#include "lp_basis.hpp"
#include "lp_fuzzy_tests.hpp"
#include "lp_status.hpp"

namespace mippp {

namespace lp_algorithm_cases {

// One overload per tag: a missing one would name the tag by its base.
inline const char * name(lp_algorithm::automatic) { return "automatic"; }
inline const char * name(lp_algorithm::simplex) { return "simplex"; }
inline const char * name(lp_algorithm::primal_simplex) {
    return "primal_simplex";
}
inline const char * name(lp_algorithm::dual_simplex) { return "dual_simplex"; }
inline const char * name(lp_algorithm::interior_point) {
    return "interior_point";
}
inline const char * name(lp_algorithm::interior_point_without_crossover) {
    return "interior_point_without_crossover";
}

template <typename M, typename A, typename F>
void run_if_listed(F & f) {
    if constexpr(has_lp_algorithm<M, A>) {
        SCOPED_TRACE(name(A{}));
        f(A{});
    }
}
// f(tag) for every tag M lists, the crossover-less one last
template <typename M, typename F>
void for_each_listed(F f) {
    run_if_listed<M, lp_algorithm::automatic>(f);
    run_if_listed<M, lp_algorithm::simplex>(f);
    run_if_listed<M, lp_algorithm::primal_simplex>(f);
    run_if_listed<M, lp_algorithm::dual_simplex>(f);
    run_if_listed<M, lp_algorithm::interior_point>(f);
    run_if_listed<M, lp_algorithm::interior_point_without_crossover>(f);
}

// Without a crossover a solver ends on an interior point: it is optimal
// within the barrier's tolerance, and on an LP with no optimum the run may
// end without a verdict, as failed on Xpress or numerical_failure on COPT.
template <typename A>
inline constexpr bool crosses_over =
    !std::same_as<A, lp_algorithm::interior_point_without_crossover>;

template <typename A>
double tolerance(double value) {
    return (crosses_over<A> ? 1. : 10.) * lp_basis_cases::tolerance(value);
}

// x - y + z <= -1 and y - x + z <= -1 sum to z <= -1 against z >= 0, and the
// objective grows along x = y, which both rows allow: neither the LP nor its
// dual has a point. A solver that meets the ray first may call the LP
// unbounded, which is all Gurobi means by that status: what no algorithm may
// do is call it optimal, or stop without a verdict.
template <typename M>
void build_primal_and_dual_infeasible(M & model) {
    using namespace operators;
    const auto free = lp_basis_cases::bounds<M>(std::nullopt, std::nullopt);
    const auto x = model.add_variable(free);
    const auto y = model.add_variable(free);
    const auto z = model.add_variable();
    model.set_maximization();
    model.set_objective(x + y);
    model.add_constraint(x - y + z <= -1);
    model.add_constraint(y - x + z <= -1);
}

struct vertex_results {
    double value = 0.;
    std::vector<double> primal, duals, reduced_costs;
};

// The free column of build_vertex is left out: it is in no row and costs
// nothing, so an interior point may leave it anywhere.
template <typename M>
vertex_results read_vertex(M & model, const lp_basis_cases::vertex<M> & v) {
    vertex_results results;
    results.value = model.get_solution_value();
    const std::array columns{v.x1, v.x2, v.x3, v.x4, v.x5, v.w};
    auto solution = model.get_solution();
    for(auto x : columns) results.primal.push_back(solution[x]);
    if constexpr(has_dual_solution<M>) {
        auto duals = model.get_dual_solution();
        for(auto c : std::array{v.le, v.ge, v.eq, v.slack})
            results.duals.push_back(duals[c]);
    }
    if constexpr(has_reduced_costs<M>) {
        auto reduced_costs = model.get_reduced_costs();
        for(auto x : columns) results.reduced_costs.push_back(reduced_costs[x]);
    }
    return results;
}

inline void expect_near(const std::vector<double> & values,
                        const std::vector<double> & expected, double factor) {
    ASSERT_EQ(values.size(), expected.size());
    for(std::size_t i = 0; i < values.size(); ++i)
        EXPECT_NEAR(values[i], expected[i],
                    factor * lp_basis_cases::tolerance(expected[i]))
            << "at index " << i;
}

// max 3 x + 2 y under x + y <= 4, x + 3 y <= 7 and x <= 3.5 over x, y >= 0,
// of optimum 11.5 at (3.5, 0.5). Each later step moves the optimum to a
// vertex the previous basis does not hold.
template <typename M>
struct plan {
    model_variable_t<M> x, y;
    model_constraint_t<M> sum, weighted;
};
template <typename M>
plan<M> build_plan(M & model) {
    using namespace operators;
    const auto x = model.add_variable({.lower_bound = 0, .upper_bound = 3.5});
    const auto y = model.add_variable();
    model.set_maximization();
    model.set_objective(3 * x + 2 * y);
    const auto sum = model.add_constraint(x + y <= 4);
    const auto weighted = model.add_constraint(x + 3 * y <= 7);
    return {x, y, sum, weighted};
}
// x - y <= 2 cuts (3.5, 0.5) off: 11 at (3, 1)
template <typename M>
void cut_plan(M & model, const plan<M> & p) {
    using namespace operators;
    model.add_constraint(p.x - p.y <= 2);
}

// Counts the trials the deletion filter ran under the dual simplex.
template <typename M>
struct trial_probe : M {
    using M::M;
    std::size_t solves = 0;
    std::size_t dual_simplex_solves = 0;

    void solve() {
        ++solves;
        if(is<lp_algorithm::dual_simplex>(this->get_lp_algorithm()))
            ++dual_simplex_solves;
        M::solve();
    }
};

}  // namespace lp_algorithm_cases

template <typename T>
struct LpAlgorithmTest : public T {
    using typename T::model_type;
    static_assert(lp_model<model_type>);
    static_assert(has_lp_algorithm<model_type>);
    // what generic code falls back on, the deletion filter included
    static_assert(has_lp_algorithm<model_type, lp_algorithm::dual_simplex>);
};
TYPED_TEST_SUITE_P(LpAlgorithmTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(LpAlgorithmTest);

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Setting ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

TYPED_TEST_P(LpAlgorithmTest, a_fresh_model_reads_automatic) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        EXPECT_TRUE(is<lp_algorithm::automatic>(model.get_lp_algorithm()));
    });
}

// Every tag after every other: a setter that left a knob of the previous
// tag in place, such as the crossover switch, would read back as another tag
// or as other.
TYPED_TEST_P(LpAlgorithmTest, each_tag_replaces_the_previous_one) {
    this->SkipOnLicenseError([this]() {
        using M = typename TestFixture::model_type;
        auto model = this->new_model();
        lp_algorithm_cases::for_each_listed<M>([&](auto previous) {
            lp_algorithm_cases::for_each_listed<M>([&](auto tag) {
                model.set_lp_algorithm(previous);
                model.set_lp_algorithm(tag);
                EXPECT_TRUE(is<decltype(tag)>(model.get_lp_algorithm()));
            });
        });
    });
}

TYPED_TEST_P(LpAlgorithmTest, setting_it_keeps_the_results_readable) {
    this->SkipOnLicenseError([this]() {
        using M = typename TestFixture::model_type;
        auto model = this->new_model();
        const auto box = lp_status_cases::build_box(model, true, 2., 1.5);
        model.solve();
        lp_algorithm_cases::for_each_listed<M>([&](auto tag) {
            model.set_lp_algorithm(tag);
            EXPECT_TRUE(is_a<status::optimal>(model.get_status()));
            EXPECT_NEAR(model.get_solution_value(), 9., TEST_EPSILON);
            EXPECT_NEAR(model.get_solution()[box.x], 3., TEST_EPSILON);
        });
    });
}

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Solving ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

TYPED_TEST_P(LpAlgorithmTest, status_cases_under_each_algorithm) {
    this->SkipOnLicenseError([this]() {
        using M = typename TestFixture::model_type;
        using lp_status_cases::verdict;
        lp_algorithm_cases::for_each_listed<M>([this](auto tag) {
            using A = decltype(tag);
            for(const auto & c : lp_status_cases::solved_cases<M>()) {
                SCOPED_TRACE(c.name);
                auto model = this->new_model();
                c.build(model);
                model.set_lp_algorithm(tag);
                model.solve();
                lp_status_cases::refine(model);
                if(c.expected == verdict::optimal) {
                    ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
                    EXPECT_NEAR(model.get_solution_value(), c.value,
                                lp_algorithm_cases::tolerance<A>(c.value));
                } else if(!lp_algorithm_cases::crosses_over<A>) {
                    EXPECT_FALSE(is_a<status::optimal>(model.get_status()));
                } else if(c.expected == verdict::unbounded) {
                    EXPECT_TRUE(is_a<status::unbounded>(model.get_status()));
                } else {
                    EXPECT_TRUE(is_a<status::infeasible>(model.get_status()));
                }
            }
        });
    });
}

TYPED_TEST_P(LpAlgorithmTest, primal_and_dual_infeasible_is_not_optimal) {
    this->SkipOnLicenseError([this]() {
        using M = typename TestFixture::model_type;
        lp_algorithm_cases::for_each_listed<M>([this](auto tag) {
            auto model = this->new_model();
            lp_algorithm_cases::build_primal_and_dual_infeasible(model);
            model.set_lp_algorithm(tag);
            model.solve();
            lp_status_cases::refine(model);
            EXPECT_FALSE(is_a<status::optimal>(model.get_status()));
            if constexpr(lp_algorithm_cases::crosses_over<decltype(tag)>) {
                EXPECT_TRUE(
                    is_a<status::infeasible_or_unbounded>(model.get_status()));
            }
        });
    });
}

TYPED_TEST_P(LpAlgorithmTest, constant_row_without_variables_is_not_optimal) {
    this->SkipOnLicenseError([this]() {
        using M = typename TestFixture::model_type;
        lp_algorithm_cases::for_each_listed<M>([this](auto tag) {
            using namespace operators;
            auto model = this->new_model();
            constexpr auto & no_terms =
                empty_linear_expression<model_variable_t<M>, double>;
            model.add_constraint(no_terms >= 1.0);
            model.set_lp_algorithm(tag);
            model.solve();
            EXPECT_FALSE(is_a<status::optimal>(model.get_status()));
        });
    });
}

// The optimum of build_vertex is unique and nondegenerate, so the point, the
// duals and the reduced costs are the same whatever reaches them.
TYPED_TEST_P(LpAlgorithmTest, results_agree_with_the_default_solve) {
    this->SkipOnLicenseError([this]() {
        using M = typename TestFixture::model_type;
        auto reference_model = this->new_model();
        const auto reference_vertex =
            lp_basis_cases::build_vertex(reference_model);
        reference_model.solve();
        ASSERT_TRUE(is_a<status::optimal>(reference_model.get_status()));
        const auto reference =
            lp_algorithm_cases::read_vertex(reference_model, reference_vertex);
        ASSERT_NEAR(reference.value, -8., TEST_EPSILON);

        lp_algorithm_cases::for_each_listed<M>([&, this](auto tag) {
            const double factor =
                lp_algorithm_cases::crosses_over<decltype(tag)> ? 1. : 10.;
            auto model = this->new_model();
            const auto vertex = lp_basis_cases::build_vertex(model);
            model.set_lp_algorithm(tag);
            model.solve();
            ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
            const auto results = lp_algorithm_cases::read_vertex(model, vertex);
            EXPECT_NEAR(results.value, reference.value,
                        factor * lp_basis_cases::tolerance(reference.value));
            lp_algorithm_cases::expect_near(results.primal, reference.primal,
                                            factor);
            lp_algorithm_cases::expect_near(results.duals, reference.duals,
                                            factor);
            lp_algorithm_cases::expect_near(results.reduced_costs,
                                            reference.reduced_costs, factor);
        });
    });
}

// A solve after a new row, a new column and a tighter bound starts from what
// the previous one left: the path column generation takes, which a cold
// solve under each algorithm never reaches.
TYPED_TEST_P(LpAlgorithmTest, re_solves_after_each_change) {
    this->SkipOnLicenseError([this]() {
        using M = typename TestFixture::model_type;
        lp_algorithm_cases::for_each_listed<M>([this](auto tag) {
            using A = decltype(tag);
            const auto expect_optimum = [](M & model, const char * step,
                                           double value) {
                SCOPED_TRACE(step);
                model.solve();
                ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
                EXPECT_NEAR(model.get_solution_value(), value,
                            lp_algorithm_cases::tolerance<A>(value));
            };
            auto model = this->new_model();
            model.set_lp_algorithm(tag);
            const auto plan = lp_algorithm_cases::build_plan(model);
            expect_optimum(model, "first solve", 11.5);
            lp_algorithm_cases::cut_plan(model, plan);
            expect_optimum(model, "after a new row", 11.);
            if constexpr(has_column_generation<M>) {
                // z >= 0, worth 4, takes one unit of both rows: 16 at z = 4
                const auto z =
                    model.add_column({{plan.sum, 1.0}, {plan.weighted, 1.0}},
                                     {.obj_coef = 4.0, .lower_bound = 0.0});
                expect_optimum(model, "after a new column", 16.);
                if constexpr(has_modifiable_variable_bounds<M>) {
                    // z <= 1 leaves 8.5 at (2.5, 0.5) beside its 4
                    model.set_variable_upper_bound(z, 1.0);
                    expect_optimum(model, "after a tighter bound", 12.5);
                }
            }
            EXPECT_TRUE(is<A>(model.get_lp_algorithm()));
        });
    });
}

// Random models and changes, each solve compared with the reference model's:
// the one check here that no hand-picked LP biases.
TYPED_TEST_P(LpAlgorithmTest, random_operations_under_each_algorithm) {
    this->SkipOnLicenseError([this]() {
        using M = typename TestFixture::model_type;
        try {
            clp_api::load();
        } catch(const std::exception & e) {
            GTEST_SKIP() << "the dumb_lp reference model requires Clp: "
                         << e.what();
        }
        const unsigned long seed =
            fuzzy_detail::env_or("MIPPP_FUZZ_SEED", std::random_device{}());
        const std::size_t num_rounds =
            fuzzy_detail::env_or("MIPPP_FUZZ_ROUNDS", 40ul);
        ::testing::Test::RecordProperty("fuzz_seed", std::to_string(seed));
        std::cout << "[   FUZZ   ] MIPPP_FUZZ_SEED=" << seed
                  << " MIPPP_FUZZ_ROUNDS=" << num_rounds << std::endl;
        lp_algorithm_cases::for_each_listed<M>([&, this](auto tag) {
            auto model = this->new_model();
            model.set_lp_algorithm(tag);
            lp_fuzzy_state_machine<M> machine(std::move(model), dumb_lp(),
                                              seed);
            try {
                machine.run(num_rounds);
            } catch(const std::exception &) {
                machine.print_trace();
                throw;
            }
            EXPECT_TRUE(is<decltype(tag)>(machine.tested().get_lp_algorithm()));
        });
    });
}

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// Basis /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

TYPED_TEST_P(LpAlgorithmTest, an_interior_point_ends_with_a_basis) {
    using M = typename TestFixture::model_type;
    if constexpr(!has_lp_basis<M> ||
                 !has_lp_algorithm<M, lp_algorithm::interior_point>) {
        GTEST_SKIP() << "no interior point, or no basis";
    } else {
        this->SkipOnLicenseError([this]() {
            auto model = this->new_model();
            lp_basis_cases::build_dense(model);
            model.set_lp_algorithm(lp_algorithm::interior_point{});
            model.solve();
            ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
            EXPECT_NO_THROW((void)model.get_basis());
        });
    }
}

// On the dense LP, which presolve leaves whole: a model presolve solves by
// itself never reaches the interior point.
TYPED_TEST_P(LpAlgorithmTest, without_a_crossover_there_is_no_basis) {
    using M = typename TestFixture::model_type;
    if constexpr(!has_lp_basis<M> ||
                 !has_lp_algorithm<
                     M, lp_algorithm::interior_point_without_crossover>) {
        GTEST_SKIP() << "no crossover-less interior point, or no basis";
    } else {
        this->SkipOnLicenseError([this]() {
            using tag = lp_algorithm::interior_point_without_crossover;
            auto reference = this->new_model();
            lp_basis_cases::build_dense(reference);
            reference.solve();
            ASSERT_TRUE(is_a<status::optimal>(reference.get_status()));
            const double value = reference.get_solution_value();

            auto model = this->new_model();
            const auto x = lp_basis_cases::build_dense(model);
            model.set_lp_algorithm(tag{});
            model.solve();
            ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
            EXPECT_NEAR(model.get_solution_value(), value,
                        lp_algorithm_cases::tolerance<tag>(value));
            EXPECT_NO_THROW((void)model.get_solution()[x.front()]);
            if constexpr(has_dual_solution<M>) {
                EXPECT_NO_THROW((void)model.get_dual_solution());
            }
            if constexpr(has_reduced_costs<M>) {
                EXPECT_NO_THROW((void)model.get_reduced_costs());
            }
            EXPECT_THROW((void)model.get_basis(), solver_error);
        });
    }
}

// The solver still holds the basis of the first solve, which the new row cut
// off: read back after the interior solve, it would be taken for its end
// point.
TYPED_TEST_P(LpAlgorithmTest,
             an_earlier_basis_is_not_read_without_a_crossover) {
    using M = typename TestFixture::model_type;
    if constexpr(!has_lp_basis<M> ||
                 !has_lp_algorithm<
                     M, lp_algorithm::interior_point_without_crossover>) {
        GTEST_SKIP() << "no crossover-less interior point, or no basis";
    } else {
        this->SkipOnLicenseError([this]() {
            using namespace operators;
            auto model = this->new_model();
            const auto x = lp_basis_cases::build_dense(model);
            model.solve();
            ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
            ASSERT_NO_THROW((void)model.get_basis());
            std::optional<model_variable_t<M>> positive;
            double value = 0.;
            {
                auto solution = model.get_solution();
                for(auto v : x) {
                    if(solution[v] <= 1e-3) continue;
                    positive = v;
                    value = solution[v];
                    break;
                }
            }
            ASSERT_TRUE(positive.has_value());
            model.add_constraint(*positive <= value / 2);
            model.set_lp_algorithm(
                lp_algorithm::interior_point_without_crossover{});
            model.solve();
            ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
            EXPECT_THROW((void)model.get_basis(), solver_error);
        });
    }
}

// An interior point starts from no basis: one written before it is ignored,
// not refused.
TYPED_TEST_P(LpAlgorithmTest, an_interior_point_solves_after_set_basis) {
    using M = typename TestFixture::model_type;
    if constexpr(!has_lp_basis_warm_start<M> ||
                 !has_lp_algorithm<M, lp_algorithm::interior_point>) {
        GTEST_SKIP() << "no interior point, or no set_basis";
    } else {
        this->SkipOnLicenseError([this]() {
            auto solved = this->new_model();
            lp_basis_cases::build_dense(solved);
            solved.solve();
            ASSERT_TRUE(is_a<status::optimal>(solved.get_status()));
            const double value = solved.get_solution_value();

            auto model = this->new_model();
            lp_basis_cases::build_dense(model);
            model.set_basis(solved.get_basis());
            model.set_lp_algorithm(lp_algorithm::interior_point{});
            model.solve();
            ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
            EXPECT_NEAR(model.get_solution_value(), value,
                        lp_basis_cases::tolerance(value));
        });
    }
}

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// IIS //////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// The native routines solve on their own: under whatever the model is set
// to, the answer is an IIS and the setting is the caller's again.
TYPED_TEST_P(LpAlgorithmTest, compute_iis_under_each_algorithm) {
    using M = typename TestFixture::model_type;
    if constexpr(!has_iis<M>) {
        GTEST_SKIP() << "no native IIS";
    } else {
        this->SkipOnLicenseError([this]() {
            const iis_cases::iis_case c = iis_cases::four_row_conflict_case();
            lp_algorithm_cases::for_each_listed<M>([&, this](auto tag) {
                auto model = this->new_model();
                const auto built = iis_cases::build(model, c);
                model.set_lp_algorithm(tag);
                const auto iis = model.compute_iis();
                const auto answer = iis_oracle::read_answer(
                    iis, built.variables, built.constraints);
                ASSERT_TRUE(iis_cases::outcome_is_expected(iis.get_outcome(),
                                                           c.expected_outcome))
                    << ::testing::PrintToString(answer);
                EXPECT_TRUE(iis_oracle::is_iis(c.system, answer))
                    << ::testing::PrintToString(answer);
                EXPECT_TRUE(is<decltype(tag)>(model.get_lp_algorithm()));
            });
        });
    }
}

// The filter re-solves one model many times, each trial a small change from
// the last: under an interior point every one would start cold, and without
// a crossover an infeasible trial may end without a verdict. Its trials run
// the dual simplex then, and only then.
TYPED_TEST_P(LpAlgorithmTest, deletion_filter_trials_under_each_algorithm) {
    using M = typename TestFixture::model_type;
    using probe = lp_algorithm_cases::trial_probe<M>;
    if constexpr(!iis_by_deletion_model<probe>) {
        GTEST_SKIP() << "no deletion filter";
    } else {
        this->SkipOnLicenseError([this]() {
            const iis_cases::iis_case c = iis_cases::four_row_conflict_case();
            lp_algorithm_cases::for_each_listed<M>([&, this](auto tag) {
                using A = decltype(tag);
                probe model(*this->api);
                const auto built = iis_cases::build(model, c);
                model.set_lp_algorithm(tag);
                const auto iis = compute_iis_by_deletion(model);
                const auto answer = iis_oracle::read_answer(
                    iis, built.variables, built.constraints);
                ASSERT_TRUE(iis_cases::outcome_is_expected(iis.get_outcome(),
                                                           c.expected_outcome))
                    << ::testing::PrintToString(answer);
                EXPECT_TRUE(iis_oracle::is_iis(c.system, answer))
                    << ::testing::PrintToString(answer);
                EXPECT_GT(model.solves, 0u);
                if constexpr(std::derived_from<A,
                                               lp_algorithm::interior_point> ||
                             std::same_as<A, lp_algorithm::dual_simplex>) {
                    EXPECT_EQ(model.dual_simplex_solves, model.solves);
                } else {
                    EXPECT_EQ(model.dual_simplex_solves, 0u);
                }
                EXPECT_TRUE(is<A>(model.get_lp_algorithm()));
            });
        });
    }
}

REGISTER_TYPED_TEST_SUITE_P(
    LpAlgorithmTest, a_fresh_model_reads_automatic,
    each_tag_replaces_the_previous_one, setting_it_keeps_the_results_readable,
    status_cases_under_each_algorithm,
    primal_and_dual_infeasible_is_not_optimal,
    constant_row_without_variables_is_not_optimal,
    results_agree_with_the_default_solve, re_solves_after_each_change,
    random_operations_under_each_algorithm, an_interior_point_ends_with_a_basis,
    without_a_crossover_there_is_no_basis,
    an_earlier_basis_is_not_read_without_a_crossover,
    an_interior_point_solves_after_set_basis, compute_iis_under_each_algorithm,
    deletion_filter_trials_under_each_algorithm);

}  // namespace mippp
