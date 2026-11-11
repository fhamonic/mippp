#pragma once

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <ranges>
#include <set>
#include <type_traits>
#include <utility>
#include <vector>

#include "lazy_constraints.hpp"

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/utility/iis_by_deletion.hpp"
#include "mippp/utility/iis_outcome.hpp"

namespace mippp {

namespace detail {

// A multidimensional knapsack: binaries, three capacity rows, profits close
// to the weights so that the relaxation is fractional and a tree is needed.
// Cover inequalities are the user cuts: valid for every feasible binary
// point, and violated by fractional ones.
struct cover_knapsack {
    static constexpr std::size_t num_items = 20;
    static constexpr std::size_t num_rows = 3;
    static constexpr double offset = 7;

    static constexpr double weight(const std::size_t r, const std::size_t i) {
        return static_cast<double>(10 + (r * 37 + i * 91 + r * i * 13) % 41);
    }
    static constexpr double profit(const std::size_t i) {
        double total = static_cast<double>((i * 7) % 5);
        for(std::size_t r = 0; r < num_rows; ++r) total += weight(r, i);
        return total;
    }
    static constexpr double capacity(const std::size_t r) {
        double total = 0;
        for(std::size_t i = 0; i < num_items; ++i) total += weight(r, i);
        return std::floor(total / 2);
    }
    // the best profit over the feasible binary points, offset left out
    static double best_profit() {
        static const double best = [] {
            double value = 0;
            for(unsigned mask = 0; mask < (1u << num_items); ++mask) {
                std::array<double, num_rows> load{};
                double total = 0;
                for(std::size_t i = 0; i < num_items; ++i) {
                    if(((mask >> i) & 1u) == 0) continue;
                    total += profit(i);
                    for(std::size_t r = 0; r < num_rows; ++r)
                        load[r] += weight(r, i);
                }
                bool feasible = true;
                for(std::size_t r = 0; r < num_rows; ++r)
                    feasible = feasible && load[r] <= capacity(r);
                if(feasible) value = std::max(value, total);
            }
            return value;
        }();
        return best;
    }

    // The items of a cover of row r that the point violates, by the usual
    // greedy: items by increasing 1 - x until their weight exceeds the
    // capacity. Empty when that cover is not violated.
    template <typename Value>
    static std::vector<std::size_t> violated_cover(const std::size_t r,
                                                   Value && value) {
        std::array<std::size_t, num_items> order;
        std::iota(order.begin(), order.end(), std::size_t{0});
        std::ranges::sort(order, {}, [&](const std::size_t i) {
            return std::pair{1 - value(i), i};
        });
        std::vector<std::size_t> cover;
        double load = 0, slack = 0;
        for(const std::size_t i : order) {
            cover.push_back(i);
            load += weight(r, i);
            slack += 1 - value(i);
            if(load > capacity(r)) break;
        }
        if(load <= capacity(r) || slack >= 1 - 1e-4) cover.clear();
        return cover;
    }
};

// The model, with `num_fixed` columns fixed at 0 created before the items:
// the solver's presolve removes them, so that its columns are not the
// model's. With `remove_a_variable`, a variable created first is removed
// once the others exist, where the model can: the items then sit one column
// lower than their handles say, on the backends that renumber. Returns the
// item variables.
template <typename Model>
auto build_cover_knapsack(Model & model, const std::size_t num_fixed,
                          const bool maximize,
                          const bool remove_a_variable = false) {
    using namespace operators;
    using knapsack = cover_knapsack;
    const auto items = std::views::iota(std::size_t{0}, knapsack::num_items);
    [[maybe_unused]] auto removed = model.add_variable();
    for(std::size_t k = 0; k < num_fixed; ++k) {
        auto fixed = model.add_binary_variable();
        model.add_constraint(fixed <= 0);
    }
    auto x = model.add_binary_variables(knapsack::num_items);
    if constexpr(has_remove_variable<Model>) {
        if(remove_a_variable) model.remove_variable(removed);
    }
    if(maximize) {
        model.set_maximization();
        model.set_objective(xsum(items,
                                 [&](const std::size_t i) {
                                     return knapsack::profit(i) * x(i);
                                 }) +
                            knapsack::offset);
    } else {
        model.set_minimization();
        model.set_objective(xsum(items,
                                 [&](const std::size_t i) {
                                     return -knapsack::profit(i) * x(i);
                                 }) -
                            knapsack::offset);
    }
    for(std::size_t r = 0; r < knapsack::num_rows; ++r)
        model.add_constraint(xsum(items, [&, r](const std::size_t i) {
                                 return knapsack::weight(r, i) * x(i);
                             }) <= knapsack::capacity(r));
    return x;
}

// Solves the knapsack with `offer(handle, x, cover)` called on every cover a
// relaxation point violates, and checks the optimum.
template <typename Test, typename Offer>
void solve_with_cover_cuts(Test & test, const std::size_t num_fixed,
                           Offer && offer,
                           const bool remove_a_variable = false) {
    using knapsack = cover_knapsack;
    auto model = test.new_model();
    auto x = build_cover_knapsack(model, num_fixed, true, remove_a_variable);
    int num_calls = 0, num_covers = 0;
    model.set_node_relaxation_callback([&](auto & handle) {
        ++num_calls;
        auto point = handle.get_solution();
        for(std::size_t r = 0; r < knapsack::num_rows; ++r) {
            const auto cover = knapsack::violated_cover(
                r, [&](const std::size_t i) { return point[x(i)]; });
            if(cover.empty()) continue;
            ++num_covers;
            offer(handle, x, cover);
        }
    });
    model.solve();

    // a hook that never runs, or never finds a cover, passes any comparison
    // of optima
    ASSERT_GE(num_calls, 1);
    ASSERT_GE(num_covers, 1);
    ASSERT_TRUE(is<status::optimal>(model.get_status()));
    ASSERT_NEAR(model.get_solution_value(),
                knapsack::best_profit() + knapsack::offset, TEST_EPSILON);
}

}  // namespace detail

///////////////////////////////////////////////////////////////////////////////
////////////////////////// Node-relaxation callback ///////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T>
struct NodeRelaxationCallbackTest : public T {
    using typename T::model_type;
    static_assert(milp_model<model_type>);
    static_assert(has_node_relaxation_callback<model_type>);
};
TYPED_TEST_SUITE_P(NodeRelaxationCallbackTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(NodeRelaxationCallbackTest);

// A solver calls back on nodes whose relaxation it could not solve too, and
// what it then reads is garbage that a separator still finds cuts in: the
// point must be one of the relaxation, and the value its objective.
TYPED_TEST_P(NodeRelaxationCallbackTest, reads_points_of_the_relaxation) {
    this->SkipOnLicenseError([this]() {
        using knapsack = detail::cover_knapsack;
        for(const bool maximize : {true, false}) {
            auto model = this->new_model();
            auto x = detail::build_cover_knapsack(model, 2, maximize, true);
            int num_calls = 0, num_invalid = 0;
            model.set_node_relaxation_callback([&](auto & handle) {
                ++num_calls;
                auto point = handle.get_solution();
                double value = knapsack::offset;
                std::array<double, knapsack::num_rows> load{};
                for(std::size_t i = 0; i < knapsack::num_items; ++i) {
                    const double v = point[x(i)];
                    if(!(v >= -TEST_EPSILON && v <= 1 + TEST_EPSILON))
                        ++num_invalid;
                    value += knapsack::profit(i) * v;
                    for(std::size_t r = 0; r < knapsack::num_rows; ++r)
                        load[r] += knapsack::weight(r, i) * v;
                }
                for(std::size_t r = 0; r < knapsack::num_rows; ++r)
                    if(!(load[r] <= knapsack::capacity(r) + 1e-4))
                        ++num_invalid;
                if(!maximize) value = -value;
                if(!(std::fabs(handle.get_solution_value() - value) <= 1e-4))
                    ++num_invalid;
            });
            model.solve();

            ASSERT_GE(num_calls, 1);
            ASSERT_EQ(num_invalid, 0);
            ASSERT_TRUE(is<status::optimal>(model.get_status()));
            const double best = knapsack::best_profit() + knapsack::offset;
            ASSERT_NEAR(model.get_solution_value(), maximize ? best : -best,
                        TEST_EPSILON);
        }
    });
}

TYPED_TEST_P(NodeRelaxationCallbackTest, a_kept_point_stays_readable) {
    this->SkipOnLicenseError([this]() {
        using knapsack = detail::cover_knapsack;
        using handle_type =
            node_relaxation_callback_handle_t<typename TestFixture::model_type>;
        using point_type =
            decltype(std::declval<handle_type &>().get_solution());
        auto model = this->new_model();
        auto x = detail::build_cover_knapsack(model, 0, true);
        struct kept_point {
            point_type point;
            std::array<double, knapsack::num_items> values;
        };
        std::vector<kept_point> points;
        model.set_node_relaxation_callback([&](auto & handle) {
            auto point = handle.get_solution();
            std::array<double, knapsack::num_items> values;
            for(std::size_t i = 0; i < knapsack::num_items; ++i)
                values[i] = point[x(i)];
            points.emplace_back(std::move(point), values);
        });
        model.solve();

        ASSERT_FALSE(points.empty());
        for(auto & [point, values] : points)
            for(std::size_t i = 0; i < knapsack::num_items; ++i)
                ASSERT_DOUBLE_EQ(point[x(i)], values[i]);
    });
}

// Registered by the setter, the native user data would point at the moved-from
// model.
TYPED_TEST_P(NodeRelaxationCallbackTest, a_moved_model_keeps_its_callback) {
    this->SkipOnLicenseError([this]() {
        using knapsack = detail::cover_knapsack;
        auto model = this->new_model();
        detail::build_cover_knapsack(model, 0, true);
        int num_calls = 0;
        model.set_node_relaxation_callback([&](auto &) { ++num_calls; });
        auto moved = std::move(model);
        moved.solve();
        ASSERT_GE(num_calls, 1);
        ASSERT_NEAR(moved.get_solution_value(),
                    knapsack::best_profit() + knapsack::offset, TEST_EPSILON);
    });
}

TYPED_TEST_P(NodeRelaxationCallbackTest, a_second_callback_replaces_the_first) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        detail::build_cover_knapsack(model, 0, true);
        int first = 0, second = 0;
        model.set_node_relaxation_callback([&](auto &) { ++first; });
        model.set_node_relaxation_callback([&](auto &) { ++second; });
        model.solve();
        ASSERT_EQ(first, 0);
        ASSERT_GE(second, 1);
    });
}

// The optimum a first solve found stays one under any valid cut, so the
// solver may answer without calling back: only the answer is pinned.
TYPED_TEST_P(NodeRelaxationCallbackTest, a_callback_set_after_a_solve_is_fine) {
    this->SkipOnLicenseError([this]() {
        using knapsack = detail::cover_knapsack;
        auto model = this->new_model();
        detail::build_cover_knapsack(model, 0, true);
        model.solve();
        const double best = knapsack::best_profit() + knapsack::offset;
        ASSERT_NEAR(model.get_solution_value(), best, TEST_EPSILON);
        model.set_node_relaxation_callback([](auto &) {});
        model.solve();
        ASSERT_TRUE(is<status::optimal>(model.get_status()));
        ASSERT_NEAR(model.get_solution_value(), best, TEST_EPSILON);
    });
}

// What an algorithm solving variants of the model does around its solves: a
// cut valid for the model need not be valid for them.
TYPED_TEST_P(NodeRelaxationCallbackTest, a_released_callback_does_not_run) {
    this->SkipOnLicenseError([this]() {
        using knapsack = detail::cover_knapsack;
        auto model = this->new_model();
        detail::build_cover_knapsack(model, 0, true);
        int num_calls = 0;
        model.set_node_relaxation_callback([&](auto &) { ++num_calls; });
        auto released = model.release_node_relaxation_callback();
        model.solve();
        ASSERT_EQ(num_calls, 0);
        const double best = knapsack::best_profit() + knapsack::offset;
        ASSERT_NEAR(model.get_solution_value(), best, TEST_EPSILON);

        // a change, so that the solver has something to search again
        model.set_objective_offset(knapsack::offset + 1);
        model.set_node_relaxation_callback(std::move(released));
        model.solve();
        ASSERT_GE(num_calls, 1);
        ASSERT_NEAR(model.get_solution_value(), best + 1, TEST_EPSILON);
    });
}

// Nothing promises a call on a model without integer variables: only the
// answer is pinned.
TYPED_TEST_P(NodeRelaxationCallbackTest, an_integer_free_model_solves) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x = model.add_variable({.upper_bound = 4});
        auto y = model.add_variable({.upper_bound = 4});
        model.set_maximization();
        model.set_objective(x + 2 * y);
        model.add_constraint(x + y <= 5);
        model.set_node_relaxation_callback([](auto &) {});
        model.solve();
        ASSERT_TRUE(is<status::optimal>(model.get_status()));
        ASSERT_NEAR(model.get_solution_value(), 9, TEST_EPSILON);
    });
}

// What a backend reads for the callback before the search (the objective, a
// column count) has nothing to read here.
TYPED_TEST_P(NodeRelaxationCallbackTest, an_empty_model_solves) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        model.set_node_relaxation_callback([](auto &) {});
        model.solve();
        ASSERT_DOUBLE_EQ(model.get_solution_value(), 0.0);
    });
}

REGISTER_TYPED_TEST_SUITE_P(
    NodeRelaxationCallbackTest, reads_points_of_the_relaxation,
    a_kept_point_stays_readable, a_moved_model_keeps_its_callback,
    a_second_callback_replaces_the_first, a_callback_set_after_a_solve_is_fine,
    a_released_callback_does_not_run, an_integer_free_model_solves,
    an_empty_model_solves);

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// User cuts //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T>
struct UserCutsTest : public T {
    using typename T::model_type;
    using handle_type = node_relaxation_callback_handle_t<model_type>;
    static_assert(milp_model<model_type>);
    static_assert(has_node_relaxation_callback<model_type>);
    static_assert(has_user_cuts<handle_type, model_type>);
    // on the handle of the relaxation, not on the model
    static_assert(!has_user_cuts<model_type>);
};
TYPED_TEST_SUITE_P(UserCutsTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(UserCutsTest);

TYPED_TEST_P(UserCutsTest, valid_cuts_keep_the_optimum) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        detail::solve_with_cover_cuts(
            *this, 0, [](auto & handle, auto && x, const auto & cover) {
                handle.add_user_cut(xsum(cover, x) <=
                                    static_cast<double>(cover.size()) - 1);
            });
    });
}

TYPED_TEST_P(UserCutsTest, valid_cuts_keep_the_optimum_distinct_variables) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        detail::solve_with_cover_cuts(
            *this, 0, [](auto & handle, auto && x, const auto & cover) {
                handle.add_user_cut(
                    distinct_variables,
                    xsum(cover, x) <= static_cast<double>(cover.size()) - 1);
            });
    });
}

// The solver's presolve drops the fixed columns, so the cut's columns are
// not at the places the solver's own columns are: a cut handed over by
// position lands on other items and cuts the optimum off.
TYPED_TEST_P(UserCutsTest, cuts_follow_their_variables_through_presolve) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        detail::solve_with_cover_cuts(
            *this, 5, [](auto & handle, auto && x, const auto & cover) {
                handle.add_user_cut(xsum(cover, x) <=
                                    static_cast<double>(cover.size()) - 1);
            });
    });
}

TYPED_TEST_P(UserCutsTest, cuts_follow_their_variables_after_a_removal) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        detail::solve_with_cover_cuts(
            *this, 0,
            [](auto & handle, auto && x, const auto & cover) {
                handle.add_user_cut(xsum(cover, x) <=
                                    static_cast<double>(cover.size()) - 1);
            },
            true);
    });
}

// The cover cut with each item written as `2 x - x`, then as `-x + 2 x`. A
// solver handed repeated columns keeps the first coefficient or the last, and
// reads one of the two forms as `2 sum x <= |C| - 1`, which cuts the optimum
// off. The cancelling pair leaves a zero behind, which must not be passed on
// either.
TYPED_TEST_P(UserCutsTest, repeated_and_cancelling_terms_are_folded) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        detail::solve_with_cover_cuts(
            *this, 0, [](auto & handle, auto && x, const auto & cover) {
                const auto other =
                    x((cover.front() + 1) % detail::cover_knapsack::num_items);
                const double rhs = static_cast<double>(cover.size()) - 1;
                const auto twice = [&](const std::size_t i) {
                    return 2 * x(i);
                };
                const auto once = [&](const std::size_t i) {
                    return -1 * x(i);
                };
                handle.add_user_cut(xsum(cover, twice) + other +
                                        xsum(cover, once) - other <=
                                    rhs);
                handle.add_user_cut(xsum(cover, once) + other +
                                        xsum(cover, twice) - other <=
                                    rhs);
            });
    });
}

// The units a separator works in must not matter: past 1e9, solvers handed
// the cut as written ignore it or lose the optimum.
TYPED_TEST_P(UserCutsTest, the_scale_of_a_cut_changes_nothing) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        for(const double scale : {1e-8, 1e-3, 1e6, 1e12})
            detail::solve_with_cover_cuts(
                *this, 0,
                [scale](auto & handle, auto && x, const auto & cover) {
                    handle.add_user_cut(
                        xsum(cover, [&](const std::size_t i) {
                            return scale * x(i);
                        }) <= scale * (static_cast<double>(cover.size()) - 1));
                });
    });
}

// A separator may offer every cut it knows at every call. Those the point
// satisfies must not reach the solver: one held at the point keeps some
// solvers separating the same node forever.
TYPED_TEST_P(UserCutsTest, cuts_the_point_satisfies_are_harmless) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        using knapsack = detail::cover_knapsack;
        const auto items =
            std::views::iota(std::size_t{0}, knapsack::num_items);
        detail::solve_with_cover_cuts(
            *this, 0, [&](auto & handle, auto && x, const auto & cover) {
                handle.add_user_cut(xsum(items, x) <=
                                    static_cast<double>(knapsack::num_items));
                handle.add_user_cut(xsum(items, x) >= 0);
                for(std::size_t r = 0; r < knapsack::num_rows; ++r)
                    handle.add_user_cut(
                        xsum(items, [&, r](const std::size_t i) {
                            return knapsack::weight(r, i) * x(i);
                        }) <= knapsack::capacity(r));
                handle.add_user_cut(xsum(cover, x) <=
                                    static_cast<double>(cover.size()) - 1);
            });
    });
}

// y + z is 1 at every integer point of this model, and anything from 0.5 to
// 1.5 at a fractional one: the equality is a valid cut. The knapsack beside
// it is what keeps the search from ending in presolve, before any call.
TYPED_TEST_P(UserCutsTest, takes_an_equality) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        using knapsack = detail::cover_knapsack;
        const auto items =
            std::views::iota(std::size_t{0}, knapsack::num_items);
        auto model = this->new_model();
        auto x = detail::build_cover_knapsack(model, 0, true);
        auto y = model.add_binary_variable();
        auto z = model.add_binary_variable();
        model.add_constraint(2 * y + 2 * z >= 1);
        model.add_constraint(2 * y + 2 * z <= 3);
        model.set_objective(xsum(items,
                                 [&](const std::size_t i) {
                                     return knapsack::profit(i) * x(i);
                                 }) +
                            5 * y + 3 * z + knapsack::offset);
        int num_calls = 0;
        model.set_node_relaxation_callback([&](auto & handle) {
            ++num_calls;
            handle.add_user_cut(y + z == 1);
        });
        model.solve();

        ASSERT_GE(num_calls, 1);
        ASSERT_TRUE(is<status::optimal>(model.get_status()));
        ASSERT_NEAR(model.get_solution_value(),
                    knapsack::best_profit() + knapsack::offset + 5,
                    TEST_EPSILON);
    });
}

// With a row asking for 18 items, the knapsack is infeasible, and every
// conflict holds a capacity row. A cover cut repeats part of the capacity row
// it comes from: offered in a trial that relaxed that row, it keeps the trial
// infeasible, and the filter ends on the 18-item row and the bounds, which
// have a solution. Whether a solver calls back in a trial is its own
// business, and on this model none of Gurobi, CPLEX and COPT does: what the
// test pins on them is that the callback is taken away and put back.
TYPED_TEST_P(UserCutsTest, the_deletion_filter_runs_without_the_cuts) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        using knapsack = detail::cover_knapsack;
        const auto items =
            std::views::iota(std::size_t{0}, knapsack::num_items);
        auto model = this->new_model();
        auto x = detail::build_cover_knapsack(model, 0, true);
        auto demand = model.add_constraint(xsum(items, x) >= 18);
        int num_calls = 0;
        model.set_node_relaxation_callback([&](auto & handle) {
            ++num_calls;
            auto point = handle.get_solution();
            for(std::size_t r = 0; r < knapsack::num_rows; ++r) {
                const auto cover = knapsack::violated_cover(
                    r, [&](const std::size_t i) { return point[x(i)]; });
                if(cover.empty()) continue;
                handle.add_user_cut(xsum(cover, x) <=
                                    static_cast<double>(cover.size()) - 1);
            }
        });
        const auto iis = compute_iis_by_deletion(model);

        ASSERT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
        ASSERT_GE(iis.num_constraint_members(), 2u);
        ASSERT_EQ(num_calls, 0);

        // the callback is back on the model once the filter returns
        if constexpr(has_modifiable_constraint_bounds<
                         typename TestFixture::model_type>)
            model.set_constraint_lower_bound(demand, 0);
        else
            model.set_constraint_rhs(demand, 0);
        model.solve();
        ASSERT_GE(num_calls, 1);
        ASSERT_NEAR(model.get_solution_value(),
                    knapsack::best_profit() + knapsack::offset, TEST_EPSILON);
    });
}

// Lazy constraints for feasibility and user cuts for strength, from one
// separator that keeps state and locks nothing: the callbacks of a model never
// run at the same time. The solve is too short for this test to catch a
// solver that lets them, which CPLEX does natively: it pins the use, and that
// one generic lambda serves both handles.
TYPED_TEST_P(UserCutsTest, one_separator_serves_candidates_and_relaxations) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        using model_type = typename TestFixture::model_type;
        if constexpr(has_candidate_solution_callback<model_type> &&
                     has_lazy_constraints<
                         candidate_solution_callback_handle_t<model_type>,
                         model_type>) {
            using cities = detail::eight_cities;
            auto model = this->new_model();
            auto x = cities::build(model);
            bool in_a_callback = false;
            int num_overlaps = 0, num_rows = 0;
            std::set<std::vector<std::size_t>> eliminated;
            const auto separate = [&](auto & handle) {
                if(in_a_callback) ++num_overlaps;
                in_a_callback = true;
                auto point = handle.get_solution();
                for(const auto & piece : cities::pieces(
                        [&](const std::size_t a) { return point[x(a)]; })) {
                    eliminated.insert(piece);
                    ++num_rows;
                    const auto inside = cities::arcs_inside(piece);
                    const double size = static_cast<double>(piece.size());
                    if constexpr(has_user_cuts<
                                     std::remove_cvref_t<decltype(handle)>,
                                     model_type>)
                        handle.add_user_cut(xsum(inside, x) <= size - 1);
                    else
                        handle.add_lazy_constraint(xsum(inside, x) <= size - 1);
                }
                in_a_callback = false;
            };
            model.set_candidate_solution_callback(separate);
            model.set_node_relaxation_callback(separate);
            if constexpr(has_progress_callback<model_type>)
                model.set_progress_callback([&](auto &) {
                    if(in_a_callback) ++num_overlaps;
                });
            model.solve();

            ASSERT_EQ(num_overlaps, 0);
            ASSERT_GE(num_rows, 1);
            ASSERT_FALSE(eliminated.empty());
            ASSERT_TRUE(is<status::optimal>(model.get_status()));
            ASSERT_NEAR(model.get_solution_value(), cities::shortest_tour(),
                        TEST_EPSILON);
        }
    });
}

REGISTER_TYPED_TEST_SUITE_P(UserCutsTest, valid_cuts_keep_the_optimum,
                            valid_cuts_keep_the_optimum_distinct_variables,
                            cuts_follow_their_variables_through_presolve,
                            cuts_follow_their_variables_after_a_removal,
                            repeated_and_cancelling_terms_are_folded,
                            the_scale_of_a_cut_changes_nothing,
                            cuts_the_point_satisfies_are_harmless,
                            takes_an_equality,
                            the_deletion_filter_runs_without_the_cuts,
                            one_separator_serves_candidates_and_relaxations);

}  // namespace mippp
