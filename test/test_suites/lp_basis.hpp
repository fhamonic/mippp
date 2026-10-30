#pragma once

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <optional>
#include <random>
#include <ranges>
#include <variant>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/utility/solver_exceptions.hpp"
#include "mippp/utility/status.hpp"
#include "mippp/utility/variant.hpp"

namespace mippp {

namespace lp_basis_cases {

// min -3 x1 + x4 - 2 x5 + 2 w, with x1 in [0, 2], x2 = 1, f free and in no
// row, the others >= 0:
//     le:     x1 + x3 + x5      <= 7     tight, dual -1
//     ge:     x3 - x5           >= 1     tight, dual 1
//     eq:     x2 + x4           == 3     dual 1
//     slack:  x3 + x4 + x5 + w  <= 100
// Its optimum x = (2, 1, 3, 2, 2, 0, 0), of value -8, is unique (but for f)
// and nondegenerate: every backend ends on the same basis.
template <typename M>
struct vertex {
    model_variable_t<M> x1, x2, x3, x4, x5, w, f;
    model_constraint_t<M> le, ge, eq, slack;
};

template <typename M>
model_variable_params_t<M> bounds(std::optional<double> lower,
                                  std::optional<double> upper) {
    model_variable_params_t<M> params = M::default_variable_params;
    params.lower_bound = lower;
    params.upper_bound = upper;
    return params;
}

template <typename M>
model_variable_params_t<M> costing(double obj_coef) {
    model_variable_params_t<M> params = M::default_variable_params;
    params.obj_coef = obj_coef;
    return params;
}

template <typename M>
vertex<M> build_vertex(M & model, bool maximize = false) {
    using namespace operators;
    const auto x1 = model.add_variable(bounds<M>(0.0, 2.0));
    const auto x2 = model.add_variable(bounds<M>(1.0, 1.0));
    const auto x3 = model.add_variable();
    const auto x4 = model.add_variable();
    const auto x5 = model.add_variable();
    const auto w = model.add_variable();
    const auto f = model.add_variable(bounds<M>(std::nullopt, std::nullopt));
    if(maximize) {
        model.set_maximization();
        model.set_objective(3 * x1 - x4 + 2 * x5 - 2 * w);
    } else {
        model.set_minimization();
        model.set_objective(-3 * x1 + x4 - 2 * x5 + 2 * w);
    }
    const auto le = model.add_constraint(x1 + x3 + x5 <= 7);
    const auto ge = model.add_constraint(x3 - x5 >= 1);
    const auto eq = model.add_constraint(x2 + x4 == 3);
    const auto slack = model.add_constraint(x3 + x4 + x5 + w <= 100);
    return {x1, x2, x3, x4, x5, w, f, le, ge, eq, slack};
}

template <typename M, typename LE>
model_constraint_t<M> add_ranged_row(M & model, LE && lhs, double lower,
                                     double upper) {
    using namespace operators;
    if constexpr(has_ranged_constraints<M>) {
        return model.add_ranged_constraint(std::forward<LE>(lhs), lower, upper);
    } else {
        auto c = model.add_constraint(std::forward<LE>(lhs) >= lower);
        model.set_constraint_upper_bound(c, upper);
        return c;
    }
}

// Dense rows of positive coefficients over x >= 0, as IterationLimitTest's:
// presolve leaves them whole, and a cold solve needs many iterations.
template <typename M>
std::vector<model_variable_t<M>> build_dense(M & model) {
    using namespace operators;
    constexpr std::size_t size = 50;
    std::mt19937 rng(7);
    std::uniform_int_distribution<int> coef(1, 1000);
    auto columns = std::views::iota(std::size_t{0}, size);
    std::vector<model_variable_t<M>> x;
    for(std::size_t j = 0; j < size; ++j) x.push_back(model.add_variable());
    std::vector<int> row(size);
    for(int & a : row) a = coef(rng);
    model.set_maximization();
    model.set_objective(xsum(columns, [&](auto j) { return row[j] * x[j]; }));
    for(std::size_t i = 0; i < size; ++i) {
        for(int & a : row) a = coef(rng);
        model.add_constraint(xsum(columns, [&](auto j) {
                                 return row[j] * x[j];
                             }) <= 1000.0 * static_cast<double>(size));
    }
    return x;
}

// x, y >= 0, x + y >= 5, x + 2 y <= 3: infeasible, as presolve proves
// before any simplex run
template <typename M>
void build_infeasible(M & model) {
    using namespace operators;
    const auto x = model.add_variable();
    const auto y = model.add_variable();
    model.add_constraint(x + y >= 5);
    model.add_constraint(x + 2 * y <= 3);
}

inline double tolerance(double value) {
    return 1e-6 * std::max(1.0, std::abs(value));
}

template <typename Status>
bool is_basic(const Status & status) {
    return is_a<basis_status::basic>(status);
}

// A source of the caller's own making: every status through a refinement
// tag of the caller, which writes as the tag it refines.
struct own_lower : basis_status::nonbasic_at_lower_bound {};
struct own_upper : basis_status::nonbasic_at_upper_bound {};
struct own_basic : basis_status::basic {};
using own_status =
    std::variant<basis_status::nonbasic, own_basic, own_lower, own_upper>;

template <typename B>
struct own_basis {
    const B * basis;
    template <typename Handle>
    own_status get_status(Handle e) const {
        const auto status = basis->get_status(e);
        if(is_a<basis_status::basic>(status)) return own_basic{};
        if(is_a<basis_status::nonbasic_at_lower_bound>(status))
            return own_lower{};
        if(is_a<basis_status::nonbasic_at_upper_bound>(status))
            return own_upper{};
        return basis_status::nonbasic{};
    }
};

// Records whether set_basis asked for the status of a variable the model no
// longer holds: a source mapping handles may have none for it.
template <typename B, typename Variable>
struct live_only_basis {
    const B * basis;
    std::vector<Variable> live;
    bool * read_a_dead_variable;
    auto get_status(Variable v) const {
        if(std::ranges::none_of(live, [&](Variable w) { return w == v; }))
            *read_a_dead_variable = true;
        return basis->get_status(v);
    }
    template <typename Constraint>
        requires(!std::same_as<Constraint, Variable>)
    auto get_status(Constraint c) const {
        return basis->get_status(c);
    }
};

// every entity basic: one basic per entity, far more than the rows
struct all_basic {
    template <typename Handle>
    std::variant<basis_status::basic, basis_status::nonbasic> get_status(
        Handle) const {
        return basis_status::basic{};
    }
};

}  // namespace lp_basis_cases

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Reading ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T>
struct LpBasisTest : public T {
    using typename T::model_type;
    static_assert(has_lp_basis<model_type>);
    static_assert(has_modifiable_lp_basis<model_type>);
};
TYPED_TEST_SUITE_P(LpBasisTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(LpBasisTest);

// The side of a row is the side of its activity, whatever the solver's slack
// convention; a fixed entity may read nonbasic_fixed or a side, and a free
// one nonbasic_free or superbasic.
TYPED_TEST_P(LpBasisTest, reads_the_side_of_every_entity) {
    this->SkipOnLicenseError([this]() {
        for(const bool maximize : {false, true}) {
            auto model = this->new_model();
            const auto v = lp_basis_cases::build_vertex(model, maximize);
            model.solve();
            ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
            ASSERT_NEAR(model.get_solution_value(), maximize ? 8.0 : -8.0,
                        TEST_EPSILON);
            const auto basis = model.get_basis();
            ASSERT_TRUE(is<basis_status::nonbasic_at_upper_bound>(
                basis.get_status(v.x1)));
            ASSERT_TRUE(
                is_a<basis_status::nonbasic_at_bound>(basis.get_status(v.x2)));
            ASSERT_TRUE(is<basis_status::basic>(basis.get_status(v.x3)));
            ASSERT_TRUE(is<basis_status::basic>(basis.get_status(v.x4)));
            ASSERT_TRUE(is<basis_status::basic>(basis.get_status(v.x5)));
            ASSERT_TRUE(is<basis_status::nonbasic_at_lower_bound>(
                basis.get_status(v.w)));
            ASSERT_TRUE(is_a<basis_status::superbasic>(basis.get_status(v.f)));
            ASSERT_TRUE(is<basis_status::nonbasic_at_upper_bound>(
                basis.get_status(v.le)));
            ASSERT_TRUE(is<basis_status::nonbasic_at_lower_bound>(
                basis.get_status(v.ge)));
            ASSERT_TRUE(
                is_a<basis_status::nonbasic_at_bound>(basis.get_status(v.eq)));
            ASSERT_TRUE(is<basis_status::basic>(basis.get_status(v.slack)));
        }
    });
}

// one basic entity per row, after a solve as in any simplex basis
TYPED_TEST_P(LpBasisTest, holds_one_basic_entity_per_row) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        const auto v = lp_basis_cases::build_vertex(model);
        model.solve();
        const auto basis = model.get_basis();
        std::size_t num_basic = 0;
        for(const auto x : {v.x1, v.x2, v.x3, v.x4, v.x5, v.w, v.f})
            num_basic += lp_basis_cases::is_basic(basis.get_status(x));
        for(const auto c : {v.le, v.ge, v.eq, v.slack})
            num_basic += lp_basis_cases::is_basic(basis.get_status(c));
        ASSERT_EQ(num_basic, model.num_constraints());
    });
}

// min -x + y + z, x in [0, 10], y, z >= 0:
//     r1:  1 <= x + y <= 3   tight at its upper side
//     r2:  2 <= z + y <= 8   tight at its lower side
TYPED_TEST_P(LpBasisTest, reads_the_side_of_a_ranged_row) {
    using M = typename TestFixture::model_type;
    if constexpr(!has_ranged_constraints<M> &&
                 !has_modifiable_constraint_bounds<M>) {
        GTEST_SKIP() << "no ranged rows";
    } else {
        this->SkipOnLicenseError([this]() {
            using namespace operators;
            auto model = this->new_model();
            const auto x =
                model.add_variable(lp_basis_cases::bounds<M>(0.0, 10.0));
            const auto y = model.add_variable();
            const auto z = model.add_variable();
            model.set_minimization();
            model.set_objective(-x + y + z);
            const auto r1 =
                lp_basis_cases::add_ranged_row(model, x + y, 1.0, 3.0);
            const auto r2 =
                lp_basis_cases::add_ranged_row(model, z + y, 2.0, 8.0);
            model.solve();
            ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
            ASSERT_NEAR(model.get_solution_value(), -1.0, TEST_EPSILON);
            const auto basis = model.get_basis();
            ASSERT_TRUE(is<basis_status::basic>(basis.get_status(x)));
            ASSERT_TRUE(
                is<basis_status::nonbasic_at_lower_bound>(basis.get_status(y)));
            ASSERT_TRUE(is<basis_status::basic>(basis.get_status(z)));
            ASSERT_TRUE(is<basis_status::nonbasic_at_upper_bound>(
                basis.get_status(r1)));
            ASSERT_TRUE(is<basis_status::nonbasic_at_lower_bound>(
                basis.get_status(r2)));
        });
    }
}

// taken by value: a column added later reads nonbasic and a row basic, and
// the entries of the others stay
TYPED_TEST_P(LpBasisTest, reads_entities_created_later_as_its_defaults) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        const auto v = lp_basis_cases::build_vertex(model);
        model.solve();
        const auto basis = model.get_basis();
        const auto y = model.add_variable();
        const auto c = model.add_constraint(y + v.x3 <= 50);
        ASSERT_TRUE(is<basis_status::nonbasic>(basis.get_status(y)));
        ASSERT_TRUE(is<basis_status::basic>(basis.get_status(c)));
        ASSERT_TRUE(
            is<basis_status::nonbasic_at_upper_bound>(basis.get_status(v.x1)));
        ASSERT_TRUE(
            is<basis_status::nonbasic_at_upper_bound>(basis.get_status(v.le)));
    });
}

// without columns the only square basis has every row basic, on every
// backend and without a solve
TYPED_TEST_P(LpBasisTest, model_without_columns_reads_every_row_basic) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        constexpr auto & no_terms =
            empty_linear_expression<model_variable_t<decltype(model)>, double>;
        const auto c = model.add_constraint(no_terms <= 1.0);
        model.solve();
        const auto basis = model.get_basis();
        ASSERT_TRUE(is<basis_status::basic>(basis.get_status(c)));
    });
}

REGISTER_TYPED_TEST_SUITE_P(LpBasisTest, reads_the_side_of_every_entity,
                            holds_one_basic_entity_per_row,
                            reads_the_side_of_a_ranged_row,
                            reads_entities_created_later_as_its_defaults,
                            model_without_columns_reads_every_row_basic);

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Warm start ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// A basis is used when the solve that starts from it reaches the optimum
// within one iteration, where a cold one cannot. A limit of 0 would not do:
// several solvers stop there before checking optimality. Without an
// iteration limit, the suite checks the answer only, and each backend's own
// tests count the native iterations.
template <typename T>
struct LpBasisWarmStartTest : public T {
    using typename T::model_type;
    static_assert(has_lp_basis_warm_start<model_type>);

    static void limit_to_one_iteration([[maybe_unused]] model_type & model) {
        if constexpr(has_iteration_limit<model_type>)
            model.set_iteration_limit(1);
    }
};
TYPED_TEST_SUITE_P(LpBasisWarmStartTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(LpBasisWarmStartTest);

TYPED_TEST_P(LpBasisWarmStartTest, starts_an_identical_model_at_the_optimum) {
    using M = typename TestFixture::model_type;
    this->SkipOnLicenseError([this]() {
        auto a = this->new_model();
        lp_basis_cases::build_dense(a);
        a.solve();
        ASSERT_TRUE(is_a<status::optimal>(a.get_status()));
        const double optimum = a.get_solution_value();

        auto b = this->new_model();
        lp_basis_cases::build_dense(b);
        b.set_basis(a.get_basis());
        TestFixture::limit_to_one_iteration(b);
        b.solve();
        ASSERT_TRUE(is_a<status::optimal>(b.get_status()));
        ASSERT_NEAR(b.get_solution_value(), optimum,
                    lp_basis_cases::tolerance(optimum));

        if constexpr(has_iteration_limit<M>) {
            auto cold = this->new_model();
            lp_basis_cases::build_dense(cold);
            TestFixture::limit_to_one_iteration(cold);
            cold.solve();
            ASSERT_FALSE(is_a<status::optimal>(cold.get_status()));
        }
    });
}

// The column added after the snapshot is unattractive and the row loose, so
// the extended basis, column nonbasic and row basic, is optimal.
TYPED_TEST_P(LpBasisWarmStartTest, extends_a_basis_to_entities_added_later) {
    using M = typename TestFixture::model_type;
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto a = this->new_model();
        lp_basis_cases::build_dense(a);
        a.solve();
        ASSERT_TRUE(is_a<status::optimal>(a.get_status()));
        const double optimum = a.get_solution_value();

        auto b = this->new_model();
        const auto x = lp_basis_cases::build_dense(b);
        const auto z = b.add_variable(lp_basis_cases::costing<M>(-1.0));
        b.add_constraint(z + x[0] + x[1] <= 1e6);
        b.set_basis(a.get_basis());
        TestFixture::limit_to_one_iteration(b);
        b.solve();
        ASSERT_TRUE(is_a<status::optimal>(b.get_status()));
        ASSERT_NEAR(b.get_solution_value(), optimum,
                    lp_basis_cases::tolerance(optimum));
    });
}

// The cold start of a model never solved is a slack basis too, so only the
// answer is checked: replaces_the_basis_of_a_solved_model sees the write.
TYPED_TEST_P(LpBasisWarmStartTest,
             accepts_a_default_basis_on_a_model_never_solved) {
    using M = typename TestFixture::model_type;
    this->SkipOnLicenseError([this]() {
        auto a = this->new_model();
        lp_basis_cases::build_dense(a);
        a.solve();
        const double optimum = a.get_solution_value();

        auto b = this->new_model();
        lp_basis_cases::build_dense(b);
        b.set_basis(model_basis_t<M>{});
        b.solve();
        ASSERT_TRUE(is_a<status::optimal>(b.get_status()));
        ASSERT_NEAR(b.get_solution_value(), optimum,
                    lp_basis_cases::tolerance(optimum));
    });
}

TYPED_TEST_P(LpBasisWarmStartTest, accepts_a_basis_of_the_callers_making) {
    this->SkipOnLicenseError([this]() {
        auto a = this->new_model();
        lp_basis_cases::build_dense(a);
        a.solve();
        const double optimum = a.get_solution_value();
        const auto basis = a.get_basis();

        auto b = this->new_model();
        lp_basis_cases::build_dense(b);
        b.set_basis(lp_basis_cases::own_basis<decltype(basis)>{&basis});
        TestFixture::limit_to_one_iteration(b);
        b.solve();
        ASSERT_TRUE(is_a<status::optimal>(b.get_status()));
        ASSERT_NEAR(b.get_solution_value(), optimum,
                    lp_basis_cases::tolerance(optimum));
    });
}

// get_status() reports the last solve, as after any other change
// set_basis() counts as a change of the model: Gurobi, CPLEX and COPT drop
// their solution when a basis is written
TYPED_TEST_P(LpBasisWarmStartTest, set_basis_resets_the_status) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        lp_basis_cases::build_dense(model);
        model.solve();
        ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
        model.set_basis(model.get_basis());
        EXPECT_TRUE(is<status::unknown>(model.get_status()));
        EXPECT_THROW((void)model.get_solution(), solve_required_error);
        EXPECT_THROW((void)model.get_basis(), solve_required_error);
        model.solve();
        ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
        EXPECT_NO_THROW((void)model.get_basis());
    });
}

// a basis the solver cannot use costs a cold start, never an error
TYPED_TEST_P(LpBasisWarmStartTest, an_invalid_basis_still_solves) {
    this->SkipOnLicenseError([this]() {
        auto a = this->new_model();
        lp_basis_cases::build_dense(a);
        a.solve();
        const double optimum = a.get_solution_value();

        auto b = this->new_model();
        lp_basis_cases::build_dense(b);
        b.set_basis(lp_basis_cases::all_basic{});
        b.solve();
        ASSERT_TRUE(is_a<status::optimal>(b.get_status()));
        ASSERT_NEAR(b.get_solution_value(), optimum,
                    lp_basis_cases::tolerance(optimum));
    });
}

// a basis written over a solved model replaces the one the solver holds
TYPED_TEST_P(LpBasisWarmStartTest, replaces_the_basis_of_a_solved_model) {
    using M = typename TestFixture::model_type;
    if constexpr(!has_iteration_limit<M>) {
        GTEST_SKIP() << "no iteration limit";
    } else {
        this->SkipOnLicenseError([this]() {
            auto model = this->new_model();
            lp_basis_cases::build_dense(model);
            model.solve();
            ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
            model.set_basis(model_basis_t<M>{});
            TestFixture::limit_to_one_iteration(model);
            model.solve();
            ASSERT_FALSE(is_a<status::optimal>(model.get_status()));
        });
    }
}

// The removed column is nonbasic at the optimum, as the eviction strategies
// of column generation pick them, and the column added next may reuse its id.
TYPED_TEST_P(LpBasisWarmStartTest, survives_the_removal_of_a_nonbasic_column) {
    using M = typename TestFixture::model_type;
    if constexpr(!has_remove_variable<M>) {
        GTEST_SKIP() << "no remove_variable";
    } else {
        this->SkipOnLicenseError([this]() {
            using namespace operators;
            const auto change = [](M & model, auto removed) {
                model.remove_variable(removed);
                const auto y =
                    model.add_variable(lp_basis_cases::costing<M>(1.0));
                model.add_constraint(y <= 1000.0);
            };
            auto model = this->new_model();
            const auto x = lp_basis_cases::build_dense(model);
            model.solve();
            ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
            const auto basis = model.get_basis();
            const auto removed = std::ranges::find_if(x, [&](auto v) {
                return !lp_basis_cases::is_basic(basis.get_status(v));
            });
            ASSERT_NE(removed, x.end());
            const auto removed_index = removed - x.begin();
            change(model, *removed);
            model.set_basis(basis);
            model.solve();
            ASSERT_TRUE(is_a<status::optimal>(model.get_status()));

            auto cold = this->new_model();
            const auto cold_x = lp_basis_cases::build_dense(cold);
            change(cold, cold_x[static_cast<std::size_t>(removed_index)]);
            cold.solve();
            ASSERT_NEAR(model.get_solution_value(), cold.get_solution_value(),
                        lp_basis_cases::tolerance(cold.get_solution_value()));
        });
    }
}

// Removing a column other than the last shifts the solver's indices away from
// the handles on HiGHS, Gurobi and CPLEX: a basis keyed by handle still lands
// on the right columns, both ways.
TYPED_TEST_P(LpBasisWarmStartTest, follows_handles_across_a_removal) {
    using M = typename TestFixture::model_type;
    if constexpr(!has_remove_variable<M> || !has_iteration_limit<M>) {
        GTEST_SKIP() << "no remove_variable or no iteration limit";
    } else {
        this->SkipOnLicenseError([this]() {
            auto a = this->new_model();
            const auto xa = lp_basis_cases::build_dense(a);
            a.solve();
            ASSERT_TRUE(is_a<status::optimal>(a.get_status()));
            const auto basis = a.get_basis();
            const auto removed =
                std::find_if(xa.begin(), xa.end() - 1, [&](auto v) {
                    return !lp_basis_cases::is_basic(basis.get_status(v));
                });
            ASSERT_NE(removed, xa.end() - 1);
            const auto index = static_cast<std::size_t>(removed - xa.begin());

            auto b = this->new_model();
            const auto xb = lp_basis_cases::build_dense(b);
            b.remove_variable(xb[index]);
            b.set_basis(basis);
            TestFixture::limit_to_one_iteration(b);
            b.solve();
            ASSERT_TRUE(is_a<status::optimal>(b.get_status()));
            ASSERT_NEAR(b.get_solution_value(), a.get_solution_value(),
                        lp_basis_cases::tolerance(a.get_solution_value()));
            const auto read = b.get_basis();
            for(std::size_t j = 0; j < xb.size(); ++j) {
                if(j == index) continue;
                ASSERT_EQ(read.get_status(xb[j]).index(),
                          basis.get_status(xa[j]).index());
            }
        });
    }
}

TYPED_TEST_P(LpBasisWarmStartTest, never_reads_a_removed_variable) {
    using M = typename TestFixture::model_type;
    if constexpr(!has_remove_variable<M> || !has_enumerable_variables<M>) {
        GTEST_SKIP() << "no remove_variable";
    } else {
        this->SkipOnLicenseError([this]() {
            auto model = this->new_model();
            const auto x = lp_basis_cases::build_dense(model);
            model.solve();
            const auto basis = model.get_basis();
            model.remove_variable(x[3]);
            bool read_a_dead_variable = false;
            const auto live = model.variables();
            model.set_basis(
                lp_basis_cases::live_only_basis<decltype(basis),
                                                model_variable_t<M>>{
                    &basis, {live.begin(), live.end()}, &read_a_dead_variable});
            ASSERT_FALSE(read_a_dead_variable);
            model.solve();
            ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
        });
    }
}

REGISTER_TYPED_TEST_SUITE_P(
    LpBasisWarmStartTest, starts_an_identical_model_at_the_optimum,
    extends_a_basis_to_entities_added_later,
    accepts_a_default_basis_on_a_model_never_solved,
    accepts_a_basis_of_the_callers_making, set_basis_resets_the_status,
    an_invalid_basis_still_solves, replaces_the_basis_of_a_solved_model,
    survives_the_removal_of_a_nonbasic_column, follows_handles_across_a_removal,
    never_reads_a_removed_variable);

}  // namespace mippp
