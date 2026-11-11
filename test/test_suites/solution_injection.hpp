#pragma once

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <ranges>

#include "mippp/linear_constraint.hpp"
#include "mippp/mapping.hpp"
#include "mippp/model_concepts.hpp"

namespace mippp {

template <typename T>
struct SolutionInjectionTest : public T {
    using typename T::model_type;
    using handle_type = candidate_solution_callback_handle_t<model_type>;
    static_assert(milp_model<model_type>);
    static_assert(has_candidate_solution_callback<model_type>);
    static_assert(has_lazy_constraints<handle_type, model_type>);
    static_assert(has_solution_injection<handle_type, model_type>);
    // on the handle, not on the model
    static_assert(!has_solution_injection<model_type>);

    // a rounding heuristic hands integers over
    struct integer_point {
        int operator[](const model_variable_t<model_type> &) const;
    };
    static_assert(requires(handle_type & handle) {
        handle.add_solution(integer_point{});
    });
};
TYPED_TEST_SUITE_P(SolutionInjectionTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(SolutionInjectionTest);

namespace detail {

// Facility location whose assignment cost sits behind one epigraph variable,
// the shape of a Benders master. A candidate priced below the cost of its open
// facilities is answered with the optimality cut that is tight there, so the
// same facilities at their true cost are feasible: that point is the repair.
struct epigraph_location {
    static constexpr std::size_t num_facilities = 8;
    static constexpr std::size_t num_customers = 12;
    using facility_set = std::array<bool, num_facilities>;

    static constexpr double opening_cost(const std::size_t i) {
        return static_cast<double>(9 + (5 * i) % 7);
    }
    static constexpr double assignment_cost(const std::size_t i,
                                            const std::size_t j) {
        return static_cast<double>(1 + (7 * i + 13 * j + i * j) % 17);
    }
    static constexpr double nearest(const facility_set & open,
                                    const std::size_t j) {
        double best = std::numeric_limits<double>::infinity();
        for(std::size_t i = 0; i < num_facilities; ++i)
            if(open[i]) best = std::min(best, assignment_cost(i, j));
        return best;
    }
    static constexpr double assignment_cost(const facility_set & open) {
        double total = 0;
        for(std::size_t j = 0; j < num_customers; ++j)
            total += nearest(open, j);
        return total;
    }
    // of facility i in the cut `eta + sum_i coefficient_i y_i >= cost(open)`,
    // valid for every set with a facility open
    static constexpr double cut_coefficient(const facility_set & open,
                                            const std::size_t i) {
        double coefficient = 0;
        for(std::size_t j = 0; j < num_customers; ++j)
            coefficient +=
                std::max(nearest(open, j) - assignment_cost(i, j), 0.0);
        return coefficient;
    }
    static constexpr double optimum() {
        double best = std::numeric_limits<double>::infinity();
        for(unsigned mask = 1; mask < (1u << num_facilities); ++mask) {
            facility_set open{};
            double value = 0;
            for(std::size_t i = 0; i < num_facilities; ++i) {
                open[i] = ((mask >> i) & 1u) != 0;
                if(open[i]) value += opening_cost(i);
            }
            best = std::min(best, value + assignment_cost(open));
        }
        return best;
    }
    // every facility open: feasible, and no optimum
    static constexpr double all_open_value() {
        facility_set open;
        open.fill(true);
        double value = assignment_cost(open);
        for(std::size_t i = 0; i < num_facilities; ++i)
            value += opening_cost(i);
        return value;
    }
};
static_assert(epigraph_location::optimum() <
              epigraph_location::all_open_value());

// configure(model) runs before the solve; accepted(handle, point, Y, eta) on
// a candidate that needs no cut; repair(handle, point, eta, cost) on one just
// answered with a cut, its binaries rounded.
template <typename Test, typename Configure, typename Accepted, typename Repair>
void solve_epigraph_location(Test & test, const bool remove_a_variable,
                             Configure && configure, Accepted && accepted,
                             Repair && repair) {
    using namespace operators;
    using location = epigraph_location;
    using model_type = typename Test::model_type;
    auto model = test.new_model();
    // Removed below: the variables after it then sit one column lower than
    // their handles say, on the backends that renumber.
    [[maybe_unused]] auto removed = model.add_variable();
    auto Y = model.add_binary_variables(location::num_facilities);
    auto eta = model.add_variable();
    if constexpr(has_remove_variable<model_type>) {
        if(remove_a_variable) model.remove_variable(removed);
    }
    const auto facilities =
        std::views::iota(std::size_t{0}, location::num_facilities);
    model.set_minimization();
    model.set_objective(xsum(facilities,
                             [&](const std::size_t i) {
                                 return location::opening_cost(i) * Y(i);
                             }) +
                        eta);
    model.add_constraint(xsum(facilities, Y) >= 1);

    const auto open_facilities = [&](auto && point) {
        location::facility_set open;
        for(const std::size_t i : facilities) open[i] = point[Y(i)] > 0.5;
        return open;
    };
    int num_cuts = 0;
    model.set_candidate_solution_callback([&](auto & handle) {
        auto point = handle.get_solution();
        const auto open = open_facilities(point);
        const double cost = location::assignment_cost(open);
        if(point[eta] >= cost - TEST_EPSILON) {
            accepted(handle, point, Y, eta);
            return;
        }
        ++num_cuts;
        handle.add_lazy_constraint(
            eta + xsum(facilities,
                       [&](const std::size_t i) {
                           return location::cut_coefficient(open, i) * Y(i);
                       }) >=
            cost);
        for(const std::size_t i : facilities) point[Y(i)] = open[i] ? 1 : 0;
        repair(handle, point, eta, cost);
    });
    configure(model);
    model.solve();

    ASSERT_TRUE(is<status::optimal>(model.get_status()));
    ASSERT_GE(num_cuts, 1);
    ASSERT_NEAR(model.get_solution_value(), location::optimum(), TEST_EPSILON);
    const auto solution = model.get_solution();
    ASSERT_NEAR(solution[eta],
                location::assignment_cost(open_facilities(solution)),
                TEST_EPSILON);
}

inline constexpr auto nothing = [](auto &&...) {};
// the candidate, read, corrected in place and handed back
inline constexpr auto repair_in_place = [](auto & handle, auto & point,
                                           const auto eta, const double cost) {
    point[eta] = cost;
    handle.add_solution(point);
};

}  // namespace detail

// What a test can pin is the optimum: whether and when a solver uses a point
// is its own business, and Gurobi and CPLEX show it to the callback again
// where COPT does not. A point handed over wrongly is caught where it is
// taken unchecked, on COPT, and silently dropped elsewhere.
TYPED_TEST_P(SolutionInjectionTest, repairs_a_rejected_candidate) {
    this->SkipOnLicenseError([this]() {
        detail::solve_epigraph_location(*this, false, detail::nothing,
                                        detail::nothing,
                                        detail::repair_in_place);
    });
}

TYPED_TEST_P(SolutionInjectionTest, repairs_after_a_variable_was_removed) {
    this->SkipOnLicenseError([this]() {
        detail::solve_epigraph_location(*this, true, detail::nothing,
                                        detail::nothing,
                                        detail::repair_in_place);
    });
}

TYPED_TEST_P(SolutionInjectionTest, takes_a_point_computed_on_the_fly) {
    this->SkipOnLicenseError([this]() {
        using variable = model_variable_t<typename TestFixture::model_type>;
        detail::solve_epigraph_location(
            *this, false, detail::nothing, detail::nothing,
            [](auto & handle, auto & point, const auto eta, const double cost) {
                handle.add_solution(
                    maps::function([&](const variable & v) -> double {
                        return v == eta ? cost : point[v];
                    }));
            });
    });
}

// The solver drops what does not improve its incumbent: a feasible point
// worse than the candidate just accepted must leave the answer alone.
TYPED_TEST_P(SolutionInjectionTest, a_worse_point_changes_nothing) {
    this->SkipOnLicenseError([this]() {
        using location = detail::epigraph_location;
        location::facility_set all_open;
        all_open.fill(true);
        const double all_open_cost = location::assignment_cost(all_open);
        int num_worse = 0;
        detail::solve_epigraph_location(
            *this, false, detail::nothing,
            [&](auto & handle, auto & point, auto && Y, const auto eta) {
                for(std::size_t i = 0; i < location::num_facilities; ++i)
                    point[Y(i)] = 1;
                point[eta] = all_open_cost;
                handle.add_solution(point);
                ++num_worse;
            },
            detail::repair_in_place);
        ASSERT_GE(num_worse, 1);
    });
}

// A solver that takes a point at once may report its new incumbent before
// the injecting call returns. The progress callback must then wait, or it
// runs in the middle of the candidate callback.
TYPED_TEST_P(SolutionInjectionTest, stays_apart_from_the_progress_callback) {
    this->SkipOnLicenseError([this]() {
        if constexpr(has_progress_callback<typename TestFixture::model_type>) {
            bool in_candidate_callback = false;
            int num_progress = 0, num_nested = 0, num_repairs = 0;
            detail::solve_epigraph_location(
                *this, false,
                [&](auto & model) {
                    model.set_progress_callback([&](auto &) {
                        ++num_progress;
                        if(in_candidate_callback) ++num_nested;
                    });
                },
                detail::nothing,
                [&](auto & handle, auto & point, const auto eta,
                    const double cost) {
                    in_candidate_callback = true;
                    detail::repair_in_place(handle, point, eta, cost);
                    in_candidate_callback = false;
                    ++num_repairs;
                });
            ASSERT_GE(num_repairs, 1);
            ASSERT_GE(num_progress, 1);
            ASSERT_EQ(num_nested, 0);
        }
    });
}

REGISTER_TYPED_TEST_SUITE_P(SolutionInjectionTest, repairs_a_rejected_candidate,
                            repairs_after_a_variable_was_removed,
                            takes_a_point_computed_on_the_fly,
                            a_worse_point_changes_nothing,
                            stays_apart_from_the_progress_callback);

}  // namespace mippp
