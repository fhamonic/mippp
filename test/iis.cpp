#include <gtest/gtest.h>

#include <algorithm>
#include <bit>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <stop_token>
#include <thread>

#include "mippp/algorithm/deletion_filter.hpp"
#include "mippp/solvers/cbc/all.hpp"
#include "mippp/solvers/clp/all.hpp"
#include "mippp/solvers/copt/all.hpp"
#include "mippp/solvers/cplex/all.hpp"
#include "mippp/solvers/glpk/all.hpp"
#include "mippp/solvers/gurobi/all.hpp"
#include "mippp/solvers/highs/all.hpp"
#include "mippp/solvers/mosek/all.hpp"
#include "mippp/solvers/scip/all.hpp"
#include "mippp/solvers/soplex/all.hpp"
#include "mippp/solvers/xpress/all.hpp"
#include "mippp/utility/linear_iis.hpp"

using namespace mippp::iis;

namespace {
// Composition-based compile probe: these optional methods satisfy capability
// concepts, but instantiating their bodies is an error. Default-policy calls
// must compile without instantiating native extraction or retained workspaces.
struct policy_probe_model {
    mippp::clp_lp model;
    using variable = mippp::model_variable_t<mippp::clp_lp>;
    using params = mippp::model_variable_params_t<mippp::clp_lp>;
    static constexpr auto default_variable_params =
        mippp::clp_lp::default_variable_params;
    auto add_variable(params p = default_variable_params) {
        return model.add_variable(p);
    }
    template <typename... Args>
    decltype(auto) add_variables(Args &&... args) {
        return model.add_variables(std::forward<Args>(args)...);
    }
    template <typename... Args>
    decltype(auto) set_maximization(Args &&... args) {
        return model.set_maximization(std::forward<Args>(args)...);
    }
    template <typename... Args>
    decltype(auto) set_minimization(Args &&... args) {
        return model.set_minimization(std::forward<Args>(args)...);
    }
    template <typename... Args>
    decltype(auto) set_objective_offset(Args &&... args) {
        return model.set_objective_offset(std::forward<Args>(args)...);
    }
    template <typename... Args>
    decltype(auto) set_objective(Args &&... args) {
        return model.set_objective(std::forward<Args>(args)...);
    }
    template <typename... Args>
    decltype(auto) add_constraint(Args &&... args) {
        return model.add_constraint(std::forward<Args>(args)...);
    }
    template <typename... Args>
    decltype(auto) add_constraints(Args &&... args) {
        return model.add_constraints(std::forward<Args>(args)...);
    }
    template <typename... Args>
    decltype(auto) num_variables(Args &&... args) {
        return model.num_variables(std::forward<Args>(args)...);
    }
    template <typename... Args>
    decltype(auto) num_constraints(Args &&... args) {
        return model.num_constraints(std::forward<Args>(args)...);
    }
    double infinity() const noexcept { return model.infinity(); }
    bool is_infinite(double value) const noexcept {
        return model.is_infinite(value);
    }
    template <typename... Args>
    decltype(auto) solve(Args &&... args) {
        return model.solve(std::forward<Args>(args)...);
    }
    template <typename... Args>
    decltype(auto) get_status(Args &&... args) {
        return model.get_status(std::forward<Args>(args)...);
    }
    template <typename... Args>
    decltype(auto) get_solution_value(Args &&... args) {
        return model.get_solution_value(std::forward<Args>(args)...);
    }
    template <typename... Args>
    decltype(auto) get_solution(Args &&... args) {
        return model.get_solution(std::forward<Args>(args)...);
    }
    template <typename T = void>
    std::optional<std::vector<double>> get_infeasibility_ray() {
        static_assert(!std::same_as<T, T>,
                      "Disabled native path was instantiated");
        return {};
    }
    template <typename T = void>
    void set_variable_lower_bound(variable, double) {
        static_assert(!std::same_as<T, T>,
                      "Disabled update path was instantiated");
    }
    template <typename T = void>
    void set_variable_upper_bound(variable, double) {
        static_assert(!std::same_as<T, T>,
                      "Disabled update path was instantiated");
    }
    template <typename T = void>
    double get_variable_lower_bound(variable) {
        static_assert(!std::same_as<T, T>,
                      "Disabled bound-read path was instantiated");
        return 0;
    }
    template <typename T = void>
    double get_variable_upper_bound(variable) {
        static_assert(!std::same_as<T, T>,
                      "Disabled bound-read path was instantiated");
        return 0;
    }
};
static_assert(mippp::lp_model<policy_probe_model>);
static_assert(has_infeasibility_ray<policy_probe_model>);
static_assert(mippp::iis::detail::has_deletion_updates<policy_probe_model>);
static_assert(valid_linear_policy<linear_policy{}>);
static_assert(!valid_linear_policy<linear_policy{.prune_bounds = true}>);
static_assert(!valid_linear_policy<linear_policy{.order_by_weight = true}>);
static_assert(!valid_linear_policy<linear_policy{
                  .elasticity = static_cast<elasticity_strategy>(99)}>);
static_assert(!valid_linear_policy<linear_policy{
                  .deletion = static_cast<deletion_strategy>(99)}>);
static_assert(!valid_linear_policy<linear_policy{
                  .analyzed_domain = static_cast<domain>(99)}>);
static_assert(std::is_empty_v<
              decltype(mippp::iis::detail::native_seed_data<false>{}.weights)>);
}  // namespace

TEST(ClpRay, DefaultPolicyDoesNotInstantiateDisabledFeatures) {
    linear_system<> system;
    system.variables = {{std::nullopt, 1., false}};
    system.rows = {{{{0, 1.}}, 2., std::nullopt}};
    const auto answer = compute_linear_iis(
        system, [] { return policy_probe_model{}; },
        {.elastic = {.violation_tolerance =
                         std::numeric_limits<double>::quiet_NaN()},
         .native = {.relative_tolerance = -1}});
    EXPECT_TRUE(answer.reduction.irreducible);
    EXPECT_EQ(answer.members.size(), 2u);
    EXPECT_FALSE(answer.deletion_model_reused);
    EXPECT_FALSE(answer.native_seed_used);
    EXPECT_EQ(answer.elasticity_calls, 0u);
}

TEST(ClpRay, CohesiveOptionsOwnMoveOnlyComparatorState) {
    linear_system<> system;
    system.variables = {{std::nullopt, 1., false}};
    system.rows = {{{{0, 1.}}, 2., std::nullopt}};
    auto comparisons = std::make_unique<unsigned>(0);
    auto * count = comparisons.get();
    auto order = [state = std::move(comparisons)](member a, member b) {
        ++*state;
        return rows_first_order{}(a, b);
    };
    auto config =
        linear_options{.limits = {.max_solves = 10}, .order = std::move(order)};
    static_assert(!std::copy_constructible<decltype(config)>);
    // Check the counter during extraction: options owns and destroys its state.
    unsigned calls_seen = 0;
    auto factory = [&] {
        calls_seen = *count;
        return mippp::clp_lp{};
    };
    const auto answer = compute_linear_iis(system, factory, std::move(config));
    EXPECT_TRUE(answer.reduction.irreducible);
    EXPECT_GT(calls_seen, 0u);
}

TEST(DeletionFilter, PolicyValidatesOnlyActiveParametersBeforeLoadingSolver) {
    unsigned calls = 0;
    auto factory = [&] {
        ++calls;
        return mippp::clp_lp{};
    };
    linear_system<> system;
    constexpr linear_policy native{.native_seed = true};
    constexpr linear_policy elastic{.elasticity = elasticity_strategy::reuse};
    EXPECT_THROW((void)compute_linear_iis<native>(
                     system, factory, {.native = {.relative_tolerance = -1}}),
                 std::invalid_argument);
    EXPECT_THROW(
        (void)compute_linear_iis<elastic>(
            system, factory,
            {.elastic = {.violation_tolerance =
                             std::numeric_limits<double>::quiet_NaN()}}),
        std::invalid_argument);
    EXPECT_EQ(calls, 0u);
}

// Exercise structural alternatives without a runtime-config compatibility
// layer.
template <typename F>
void for_each_bool(F && test) {
    test.template operator()<false>();
    test.template operator()<true>();
}

TEST(DeletionFilter, PhaseBudgetRejectsOverspendingAndPreservesDeadline) {
    mippp::iis::detail::phase_budget budget(
        {.max_solves = 5, .time_limit = std::chrono::seconds(2)});
    const auto deadline = budget.limits().deadline;
    EXPECT_EQ(budget.remaining(1).max_solves, 1u);
    budget.consume(2);
    EXPECT_EQ(budget.remaining().max_solves, 3u);
    EXPECT_EQ(budget.remaining().deadline, deadline);
    EXPECT_THROW(budget.consume(4), std::logic_error);
    EXPECT_EQ(budget.used(), 2u);
    budget.consume(3);
    EXPECT_EQ(budget.remaining().max_solves, 0u);
    EXPECT_THROW(budget.consume(1), std::logic_error);
    mippp::iis::detail::phase_budget unlimited({});
    unlimited.consume(std::numeric_limits<std::size_t>::max());
    EXPECT_EQ(unlimited.remaining().max_solves, 0u);
    EXPECT_THROW(unlimited.consume(1), std::logic_error);
}

namespace {
struct fake_ray_provider {
    mippp::model_variable_t<mippp::clp_lp>
    add_variable();  // type deduction only
    std::size_t rows = 1, columns = 1;
    std::vector<double> ray{1.};
    auto num_constraints() const { return rows; }
    auto num_variables() const { return columns; }
    std::optional<std::vector<double>> get_infeasibility_ray() { return ray; }
};
struct fake_full_certificate_provider {
    mippp::model_variable_t<mippp::clp_lp> add_variable();
    mippp::linear_infeasibility_certificate<double> certificate{
        {1.}, {0.}, {0.}, {1.}};
    std::size_t num_constraints() const { return 1; }
    std::size_t num_variables() const { return 1; }
    std::optional<mippp::linear_infeasibility_certificate<double>>
    get_infeasibility_certificate() {
        return certificate;
    }
};
struct fake_mosek_cleanup {
    using Env = mippp::mosek::impl::v1::MSKenv_t;
    using Task = mippp::mosek::impl::v1::MSKtask_t;
    std::vector<char> & released;
    int deletetask(Task *) const {
        released.push_back('t');
        return -1;
    }
    int deleteenv(Env *) const {
        released.push_back('e');
        return -1;
    }
};
}  // namespace

TEST(DeletionFilter, PreparedSystemAndMalformedSeedProviderNeedNoSolver) {
    linear_system<> input;
    input.variables = {{std::nullopt, 1., false}};
    input.rows = {{{{0, 1.}}, 2., std::nullopt}};
    mippp::iis::detail::prepared_linear_system prepared(input, domain::original,
                                                        false);
    ASSERT_EQ(prepared.candidates.size(), 2u);
    EXPECT_EQ(prepared.candidates[0], (member{member_kind::variable_upper, 0}));
    EXPECT_EQ(prepared.inequalities().size(), 2u);
    fake_ray_provider provider;
    const std::vector<std::size_t> active{0, 1};
    auto capture = [&] {
        return mippp::iis::detail::collect_native_seed<true, true>(
            provider, active, prepared, {});
    };
    ASSERT_TRUE(capture().members);
    EXPECT_EQ(capture().members->size(), 2u);
    provider.rows = 100;  // formerly could underflow bounds.reserve()
    EXPECT_FALSE(capture().members);
    provider.rows = 1;
    provider.columns = 0;
    EXPECT_FALSE(capture().members);
    provider.columns = 1;
    provider.ray = {1., 2.};
    EXPECT_FALSE(capture().members);
    provider.ray = {std::numeric_limits<double>::quiet_NaN()};
    EXPECT_FALSE(capture().members);
    input.rows[0].terms[0].first = 2;
    EXPECT_THROW((mippp::iis::detail::prepared_linear_system(
                     input, domain::original, false)),
                 std::invalid_argument);
}

TEST(DeletionFilter, MosekCleanupUnwindsPartialConstructionWithoutThrowing) {
    using namespace mippp::mosek::impl::v1;
    int storage;
    for(unsigned stage : {0u, 1u, 2u}) {
        std::vector<char> released;
        fake_mosek_cleanup api{released};
        MSKenv_t env = nullptr;
        MSKtask_t task = nullptr;
        try {
            resource_detail::mosek_handle_guard guard(api, env, task);
            if(stage >= 1) env = reinterpret_cast<MSKenv_t>(&storage);
            if(stage >= 2) task = reinterpret_cast<MSKtask_t>(&storage);
            throw std::runtime_error("original construction failure");
        } catch(const std::runtime_error & error) {
            EXPECT_STREQ(error.what(), "original construction failure");
        }
        EXPECT_EQ(env, nullptr);
        EXPECT_EQ(task, nullptr);
        const std::vector<char> expected = stage == 2
                                               ? std::vector<char>{'t', 'e'}
                                           : stage == 1 ? std::vector<char>{'e'}
                                                        : std::vector<char>{};
        EXPECT_EQ(released,
                  expected);  // error returns never prevent env cleanup
    }
    static_assert(std::is_nothrow_destructible_v<mippp::mosek_lp>);
}

TEST(DeletionFilter, FullCertificateProviderRejectsMalformedArrays) {
    linear_system<> input;
    input.variables = {{std::nullopt, 1., false}};
    input.rows = {{{{0, 1.}}, 2., std::nullopt}};
    mippp::iis::detail::prepared_linear_system prepared(input, domain::original,
                                                        false);
    const std::vector<std::size_t> active{0, 1};
    fake_full_certificate_provider provider;
    auto capture = [&] {
        return mippp::iis::detail::collect_native_seed<false, true>(
            provider, active, prepared, {});
    };
    ASSERT_TRUE(capture().members);
    EXPECT_EQ(capture().weights, (std::vector<long double>{1., 1.}));
    provider.certificate.variable_upper.clear();
    EXPECT_FALSE(capture().members);
    provider.certificate.variable_upper = {
        std::numeric_limits<double>::quiet_NaN()};
    EXPECT_FALSE(capture().members);
    provider.certificate.variable_upper = {0.};
    provider.certificate.row_lower = {0.};
    EXPECT_FALSE(capture().members);
    EXPECT_TRUE(capture().weights.empty());
}

TEST(DeletionFilter, InternalContinuationPreservesProofAndLimits) {
    auto oracle = [](std::span<const std::size_t>) {
        return feasibility::infeasible;
    };
    const auto known = deletion_filter(3, oracle, {.max_solves = 1});
    ASSERT_TRUE(known.proven_infeasible());
    for(auto reason : {termination::solve_limit, termination::time_limit,
                       termination::cancelled}) {
        options opts;
        std::stop_source stop;
        if(reason == termination::solve_limit) opts.max_solves = 1;
        if(reason == termination::time_limit)
            opts.deadline = std::chrono::steady_clock::now();
        if(reason == termination::cancelled) {
            stop.request_stop();
            opts.stop = stop.get_token();
        }
        auto no_calls = [](std::span<const std::size_t>) {
            ADD_FAILURE() << "Stopped continuation must not invoke the oracle";
            return feasibility::unknown;
        };
        const auto answer = mippp::iis::detail::deletion_filter_impl(
            known, no_calls, opts, input_order{});
        EXPECT_EQ(answer.members, known.members);
        EXPECT_TRUE(answer.proven_infeasible());
        EXPECT_FALSE(answer.irreducible);
        EXPECT_EQ(answer.solve_count, 1u);
        EXPECT_EQ(answer.reason, reason);
    }
    auto unknown = [](std::span<const std::size_t> subset) {
        EXPECT_EQ(subset.size(), 2u);  // never recheck the proven full set
        return feasibility::unknown;
    };
    const auto answer = mippp::iis::detail::deletion_filter_impl(
        known, unknown, {}, input_order{});
    EXPECT_TRUE(answer.proven_infeasible());
    EXPECT_FALSE(answer.irreducible);
    EXPECT_EQ(answer.reason, termination::indeterminate);
    EXPECT_EQ(answer.solve_count, 4u);
}

TEST(DeletionFilter, RaySupportRejectsMalformedAndNormalizesMagnitude) {
    const std::vector<double> ray{0., -4e100, 2e100, 1e80};
    ASSERT_TRUE(ray_support<double>(ray));
    EXPECT_EQ(*ray_support<double>(ray), (std::vector<std::size_t>{1, 2}));
    EXPECT_EQ(*ray_support<double>(ray, 0),
              (std::vector<std::size_t>{1, 2, 3}));
    const std::vector<double> zero{0., 0.};
    EXPECT_FALSE(ray_support<double>(zero));
    for(double bad : {std::numeric_limits<double>::infinity(),
                      std::numeric_limits<double>::quiet_NaN()}) {
        const std::vector<double> invalid{1., bad};
        EXPECT_FALSE(ray_support<double>(invalid));
        EXPECT_THROW((void)ray_support<double>(ray, bad),
                     std::invalid_argument);
    }
    EXPECT_THROW((void)ray_support<double>(ray, -1), std::invalid_argument);
}

namespace {
linear_system<> clp_ray_system() {
    linear_system<> system;
    system.variables = {{std::nullopt, 1., false}, {-100., 100., false}};
    // The lower side of a ranged row conflicts with a variable upper bound.
    // Other rows/bounds are redundant, but must retain their original IDs.
    system.rows = {{{{0, 1.}}, 2., 20.}};
    for(unsigned i = 0; i < 12; ++i)
        system.rows.push_back({{{1, 1.}}, std::nullopt, 200. + i});
    return system;
}
}  // namespace

TEST(DeletionFilter, RayColumnsUseSignedFullRayAndRejectMalformedInput) {
    std::vector<std::vector<std::pair<std::size_t, double>>> rows{
        {{0, 1.}, {1, 2.}, {1, -1.}}, {{0, 2.}, {1, 2.}}};
    auto terms = [&](std::size_t r) -> const auto & { return rows[r]; };
    for(const auto & ray : {std::vector<double>{2., -1.}, {-2e200, 1e200}}) {
        auto columns = ray_column_magnitudes<double>(3, ray, terms);
        ASSERT_TRUE(columns);
        EXPECT_EQ(*columns, (std::vector<long double>{0., 0., 0.}));
    }
    const std::vector<double> ray{2., 1.};
    auto columns = ray_column_magnitudes<double>(3, ray, terms);
    ASSERT_TRUE(columns);
    EXPECT_EQ(*columns, (std::vector<long double>{2., 2., 0.}));
    EXPECT_FALSE(ray_column_magnitudes<double>(1, ray, terms));
    rows[0][0].second = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(ray_column_magnitudes<double>(3, ray, terms));
    const std::vector<double> zero{0., 0.};
    EXPECT_FALSE(ray_column_magnitudes<double>(3, zero, terms));
    const std::vector<double> invalid{0.,
                                      std::numeric_limits<double>::quiet_NaN()};
    EXPECT_FALSE(ray_column_magnitudes<double>(3, invalid, terms));
}

TEST(ClpRay, ColumnSupportShrinksSeedAndWeightOrderingPreservesProof) {
    auto system = clp_ray_system();
    for(unsigned i = 0; i < 40; ++i)
        system.variables.push_back({-10., 10., false});
    auto factory = [] { return mippp::clp_lp{}; };
    auto baseline =
        compute_linear_iis<linear_policy{.native_seed = true}>(system, factory);
    for_each_bool([&]<bool order>() {
        for_each_bool([&]<bool reuse>() {
            auto answer = compute_linear_iis<linear_policy{
                .deletion = (reuse ? deletion_strategy::reuse
                                   : deletion_strategy::rebuild),
                .native_seed = true,
                .prune_bounds = true,
                .order_by_weight = order}>(system, factory);
            ASSERT_TRUE(answer.reduction.irreducible);
            EXPECT_TRUE(answer.native_seed_used);
            EXPECT_EQ(answer.members.size(), 2u);
            EXPECT_EQ(answer.native_seed_size, 2u);
            EXPECT_LT(answer.reduction.solve_count,
                      baseline.reduction.solve_count);
            for(auto m : baseline.members)
                EXPECT_NE(
                    std::find(answer.members.begin(), answer.members.end(), m),
                    answer.members.end());
        });
    });
    for(std::size_t budget = 0; budget < 7; ++budget) {
        auto answer =
            compute_linear_iis<linear_policy{.native_seed = true,
                                             .prune_bounds = true,
                                             .order_by_weight = true}>(
                system, factory,
                linear_options{.limits = {.max_solves = budget}});
        EXPECT_LE(answer.reduction.solve_count, budget);
        EXPECT_EQ(answer.reduction.proven_infeasible(), budget > 0);
    }
    // Aggressive thresholding destroys support. Its failed validation still
    // falls back to the known full set under the new ordering/reuse options.
    auto rejected =
        compute_linear_iis<linear_policy{.deletion = deletion_strategy::reuse,
                                         .native_seed = true,
                                         .prune_bounds = true,
                                         .order_by_weight = true}>(
            system, factory,
            linear_options{.native = {.relative_tolerance = 1.},
                           .order = bounds_first_order{}});
    EXPECT_TRUE(rejected.reduction.irreducible);
    EXPECT_FALSE(rejected.native_seed_used);
    EXPECT_EQ(rejected.members.size(), 2u);
}

TEST(ClpRay, ExtractsOwnedRowRayWithoutAnotherSolve) {
    mippp::clp_lp model;
    EXPECT_FALSE(model.get_infeasibility_ray());
    const auto x =
        model.add_variable({.lower_bound = std::nullopt, .upper_bound = 1.});
    std::vector<std::pair<decltype(x), double>> terms{{x, 1.}};
    model.add_constraint(mippp::operators::operator>=(
        mippp::linear_expression_view(terms, 0.), 2.));
    model.solve();
    ASSERT_TRUE(
        std::holds_alternative<mippp::status::infeasible>(model.get_status()));
    const auto ray = model.get_infeasibility_ray();
    ASSERT_TRUE(ray);
    ASSERT_EQ(ray->size(), 1u);
    EXPECT_NE((*ray)[0], 0.);
    // Mutating/re-solving the model cannot invalidate the returned copy.
    model.set_variable_upper_bound(x, 3.);
    model.solve();
    EXPECT_FALSE(model.get_infeasibility_ray());
    EXPECT_NE((*ray)[0], 0.);
}

TEST(ClpRay, SmallerSeedPreservesBoundsAndOriginalRowSides) {
    const auto system = clp_ray_system();
    auto factory = [] { return mippp::clp_lp{}; };
    const auto baseline = compute_linear_iis(system, factory);
    for_each_bool([&]<bool elastic>() {
        const auto answer = compute_linear_iis<linear_policy{
            .elasticity = (elastic ? elasticity_strategy::reuse
                                   : elasticity_strategy::off),
            .native_seed = true}>(system, factory,
                                  linear_options{.order = rows_first_order{}});
        ASSERT_TRUE(answer.native_seed_used);
        EXPECT_LT(answer.native_seed_size, 17u);
        EXPECT_EQ(answer.elasticity_calls, 0u);
        EXPECT_LT(answer.reduction.solve_count, baseline.reduction.solve_count);
        ASSERT_TRUE(answer.reduction.irreducible);
        ASSERT_EQ(answer.members.size(), 2u);
        EXPECT_NE(std::find(answer.members.begin(), answer.members.end(),
                            member{member_kind::variable_upper, 0}),
                  answer.members.end());
        EXPECT_NE(std::find(answer.members.begin(), answer.members.end(),
                            member{member_kind::row_lower, 0}),
                  answer.members.end());
    });
}

TEST(ClpRay, ThresholdedInvalidSeedFallsBackAndBudgetsStayShared) {
    const auto system = clp_ray_system();
    for_each_bool([&]<bool elastic>() {
        for(double tolerance : {1e-9, 1.}) {
            for(std::size_t budget = 0; budget < 12; ++budget) {
                std::size_t calls = 0;
                auto factory = [&] {
                    ++calls;
                    return mippp::clp_lp{};
                };
                const auto answer = compute_linear_iis<linear_policy{
                    .elasticity = (elastic ? elasticity_strategy::reuse
                                           : elasticity_strategy::off),
                    .native_seed = true}>(
                    system, factory,
                    linear_options{.limits = {.max_solves = budget,
                                              .initial_batch_size = 2},
                                   .native = {.relative_tolerance = tolerance},
                                   .order = bounds_first_order{}});
                EXPECT_LE(answer.reduction.solve_count, budget);
                EXPECT_EQ(calls + answer.elasticity_reoptimizations,
                          answer.reduction.solve_count);
                EXPECT_EQ(answer.reduction.proven_infeasible(), budget != 0);
                if(tolerance == 1.) {
                    EXPECT_FALSE(answer.native_seed_used);
                }
                if(answer.reduction.irreducible)
                    EXPECT_EQ(answer.members.size(), 2u);
                else
                    EXPECT_EQ(answer.reduction.reason,
                              termination::solve_limit);
            }
        }
        // A tolerance of one deliberately drops every row from the ray. The
        // bounds-only proposal is feasible and MUST NOT become a certificate.
        const auto answer = compute_linear_iis<linear_policy{
            .elasticity = (elastic ? elasticity_strategy::rebuild
                                   : elasticity_strategy::off),
            .native_seed = true}>(
            system, [] { return mippp::clp_lp{}; },
            linear_options{.native = {.relative_tolerance = 1.}});
        EXPECT_FALSE(answer.native_seed_used);
        EXPECT_TRUE(answer.reduction.irreducible);
        EXPECT_EQ(answer.members.size(), 2u);
        if(!elastic) {
            const auto baseline =
                compute_linear_iis(system, [] { return mippp::clp_lp{}; });
            // Exactly one extra call validates (and rejects) the native seed;
            // returning to the unchanged full set must not verify it again.
            EXPECT_EQ(answer.reduction.solve_count,
                      baseline.reduction.solve_count + 1);
        }
    });
}

TEST(ClpRay, MissingRayAndZeroDeadlineFallBackSafely) {
    linear_system<> system;
    system.variables = {
        {2., 1., false}};  // certified without a native solve/ray
    auto factory = [] { return mippp::clp_lp{}; };
    auto answer =
        compute_linear_iis<linear_policy{.native_seed = true}>(system, factory);
    EXPECT_TRUE(answer.reduction.irreducible);
    EXPECT_FALSE(answer.native_seed_used);
    answer = compute_linear_iis<linear_policy{.native_seed = true}>(
        system, factory,
        linear_options{.limits = {.time_limit = std::chrono::seconds(0)}});
    EXPECT_EQ(answer.reduction.solve_count, 0u);
    EXPECT_EQ(answer.reduction.reason, termination::time_limit);
    EXPECT_FALSE(answer.reduction.proven_infeasible());
}

TEST(ClpRay, CancellationDuringSeedVerificationKeepsFullProof) {
    std::stop_source stop;
    unsigned constructions = 0;
    auto factory = [&] {
        if(++constructions == 2) stop.request_stop();
        return mippp::clp_lp{};
    };
    const auto answer = compute_linear_iis<linear_policy{
        .elasticity = elasticity_strategy::rebuild, .native_seed = true}>(
        clp_ray_system(), factory,
        linear_options{.limits = {.stop = stop.get_token()}});
    EXPECT_EQ(constructions, 2u);
    EXPECT_EQ(answer.reduction.solve_count, 2u);
    EXPECT_EQ(answer.reduction.reason, termination::cancelled);
    EXPECT_TRUE(answer.reduction.proven_infeasible());
    EXPECT_FALSE(answer.reduction.irreducible);
    EXPECT_FALSE(answer.native_seed_used);
    EXPECT_EQ(answer.members.size(), 17u);  // seed was never verified
}

namespace {
mippp::soplex_lp soplex_ray_model() {
    mippp::soplex_lp model;
    // Certificate availability is not guaranteed when a simplifier establishes
    // infeasibility. Opt out in this test factory, never inside the accessor.
    model.native_api().setIntParam(model.native_model(), 10,
                                   0);  // SIMPLIFIER_OFF
    return model;
}
}  // namespace

TEST(SoPlexRay, OptionalEntryPointsAndOwnedRay) {
    std::optional<std::vector<double>> copy;
    {
        auto model = soplex_ray_model();
        ASSERT_NE(model.native_api().hasDualFarkas, nullptr);
        ASSERT_NE(model.native_api().getDualFarkasReal, nullptr);
        EXPECT_FALSE(model.get_infeasibility_ray());
        const auto x = model.add_variable(
            {.lower_bound = std::nullopt, .upper_bound = 1.});
        std::vector<std::pair<decltype(x), double>> terms{{x, 1.}};
        model.add_constraint(mippp::operators::operator>=(
            mippp::linear_expression_view(terms, 0.), 2.));
        model.solve();
        ASSERT_TRUE(std::holds_alternative<mippp::status::infeasible>(
            model.get_status()));
        // Moving an already solved model must preserve the status gate too.
        auto moved = std::move(model);
        copy = moved.get_infeasibility_ray();
        ASSERT_TRUE(copy);
        ASSERT_EQ(copy->size(), 1u);
        EXPECT_NE((*copy)[0], 0.);
        const auto & api = moved.native_api();
        double sentinel = 123.;
        EXPECT_EQ(api.getDualFarkasReal(moved.native_model(), &sentinel, 0), 0);
        EXPECT_EQ(sentinel, 123.);
        EXPECT_EQ(api.getDualFarkasReal(moved.native_model(), nullptr, 1), 0);
        EXPECT_EQ(api.getDualFarkasReal(nullptr, &sentinel, 1), 0);
        EXPECT_EQ(api.hasDualFarkas(nullptr), 0);
    }
    EXPECT_NE((*copy)[0], 0.);  // no borrowed solver allocation
}

TEST(SoPlexRay, VerifiedSeedAndThresholdFallback) {
    const auto system = clp_ray_system();
    for_each_bool([&]<bool elastic>() {
        auto answer = compute_linear_iis<linear_policy{
            .elasticity = (elastic ? elasticity_strategy::rebuild
                                   : elasticity_strategy::off),
            .native_seed = true}>(system, soplex_ray_model,
                                  linear_options{.order = rows_first_order{}});
        EXPECT_TRUE(answer.native_seed_used);
        EXPECT_LT(answer.native_seed_size, 17u);
        EXPECT_TRUE(answer.reduction.irreducible);
        EXPECT_EQ(answer.members.size(), 2u);
        EXPECT_EQ(answer.elasticity_calls, 0u);
        answer = compute_linear_iis<linear_policy{
            .elasticity = (elastic ? elasticity_strategy::rebuild
                                   : elasticity_strategy::off),
            .native_seed = true}>(
            system, soplex_ray_model,
            linear_options{.native = {.relative_tolerance = 1.},
                           .order = bounds_first_order{}});
        EXPECT_FALSE(answer.native_seed_used);
        EXPECT_TRUE(answer.reduction.irreducible);
        EXPECT_EQ(answer.members.size(), 2u);
    });
}

TEST(SoPlexRay, SharedBudgetAndDefaultPresolveRemainSafe) {
    for(std::size_t budget = 0; budget < 9; ++budget) {
        std::size_t calls = 0;
        auto factory = [&] {
            ++calls;
            return soplex_ray_model();
        };
        const auto answer =
            compute_linear_iis<linear_policy{.native_seed = true}>(
                clp_ray_system(), factory,
                linear_options{.limits = {.max_solves = budget}});
        EXPECT_LE(answer.reduction.solve_count, budget);
        EXPECT_EQ(answer.reduction.solve_count, calls);
        EXPECT_EQ(answer.reduction.proven_infeasible(), budget != 0);
        if(answer.reduction.irreducible) {
            EXPECT_EQ(answer.members.size(), 2u);
        }
    }
    // Do not require a certificate from presolve; only correctness and
    // fallback.
    const auto answer = compute_linear_iis<linear_policy{
        .elasticity = elasticity_strategy::rebuild, .native_seed = true}>(
        clp_ray_system(), [] { return mippp::soplex_lp{}; });
    EXPECT_TRUE(answer.reduction.irreducible);
    EXPECT_EQ(answer.members.size(), 2u);
}

TEST(SoPlexRay, ColumnScreeningAndWeightedOrder) {
    const auto answer = compute_linear_iis<linear_policy{
        .native_seed = true, .prune_bounds = true, .order_by_weight = true}>(
        clp_ray_system(), soplex_ray_model);
    EXPECT_TRUE(answer.native_seed_used);
    EXPECT_EQ(answer.native_seed_size, 2u);
    EXPECT_TRUE(answer.reduction.irreducible);
    EXPECT_EQ(answer.members.size(), 2u);
}

TEST(SoPlexStock, MissingSymbolsFallBackWithoutBreakingLibraryLoading) {
    const auto path = std::getenv("MIPPP_SOPLEX_STOCK_LIBRARY");
    ASSERT_NE(path, nullptr)
        << "Select this suite with an unmodified SoPlex library";
    const auto & api = mippp::soplex_api::load(path);
    ASSERT_EQ(api.hasDualFarkas, nullptr);
    ASSERT_EQ(api.getDualFarkasReal, nullptr);
    const auto answer = compute_linear_iis<linear_policy{.native_seed = true}>(
        clp_ray_system(), [&] { return mippp::soplex_lp(api); });
    EXPECT_TRUE(answer.reduction.irreducible);
    EXPECT_EQ(answer.members.size(), 2u);
    EXPECT_FALSE(answer.native_seed_used);
}

namespace {
mippp::mosek_lp mosek_certificate_model(int optimizer) {
    using namespace mippp::mosek::impl::v1;
    mippp::mosek_lp model;
    const auto & api = model.native_api();
    const auto task = model.native_model().second;
    api._check(api.putintparam(task, MSK_IPAR_OPTIMIZER, optimizer));
    api._check(api.putintparam(task, MSK_IPAR_NUM_THREADS, 1));
    api._check(api.putintparam(task, MSK_IPAR_PRESOLVE_USE, 0));
    if(optimizer == MSK_OPTIMIZER_INTPNT)
        api._check(api.putintparam(task, MSK_IPAR_INTPNT_BASIS,
                                   0));  // no basic solution
    return model;
}

mippp::mosek::impl::v1::MSKint32t count_mosek_optimizations(
    mippp::mosek::impl::v1::MSKtask_t, void * count,
    mippp::mosek::impl::v1::MSKcallbackcodee caller, const double *,
    const mippp::mosek::impl::v1::MSKint32t *,
    const mippp::mosek::impl::v1::MSKint64t *) {
    if(caller == mippp::mosek::impl::v1::MSK_CALLBACK_BEGIN_OPTIMIZER)
        ++*static_cast<unsigned *>(count);
    return 0;
}
}  // namespace

TEST(MosekCertificate, SimplexAndInteriorPointOwnAllFourMultiplierArrays) {
    using namespace mippp::mosek::impl::v1;
    for(int optimizer : {MSK_OPTIMIZER_PRIMAL_SIMPLEX,
                         MSK_OPTIMIZER_DUAL_SIMPLEX, MSK_OPTIMIZER_INTPNT}) {
        auto model = mosek_certificate_model(optimizer);
        EXPECT_FALSE(model.get_infeasibility_certificate());
        const auto x = model.add_variable(
            {.lower_bound = std::nullopt, .upper_bound = 1.});
        model.add_variable({.lower_bound = -100., .upper_bound = 100.});
        std::vector<std::pair<decltype(x), double>> terms{{x, 1.}};
        model.add_constraint(mippp::operators::operator>=(
            mippp::linear_expression_view(terms, 0.), 2.));
        unsigned optimizations = 0;
        model.native_api()._check(model.native_api().putcallbackfunc(
            model.native_model().second, count_mosek_optimizations,
            &optimizations));
        model.solve();
        EXPECT_EQ(optimizations, 1u);
        ASSERT_TRUE(std::holds_alternative<mippp::status::infeasible>(
            model.get_status()));
        const auto certificate = model.get_infeasibility_certificate();
        ASSERT_TRUE(certificate);
        ASSERT_EQ(certificate->row_lower.size(), 1u);
        ASSERT_EQ(certificate->row_upper.size(), 1u);
        ASSERT_EQ(certificate->variable_lower.size(), 2u);
        ASSERT_EQ(certificate->variable_upper.size(), 2u);
        EXPECT_GT(certificate->row_lower[0], 0.);
        EXPECT_GT(certificate->variable_upper[0], 0.);
        EXPECT_EQ(optimizations,
                  1u);  // status/certificate reads cannot optimize
        if(optimizer == MSK_OPTIMIZER_INTPNT) {
            MSKbooleant basic = 1;
            model.native_api()._check(model.native_api().solutiondef(
                model.native_model().second, MSK_SOL_BAS, &basic));
            EXPECT_EQ(basic, 0);
        }
        model.set_variable_upper_bound(x, 3.);
        model.solve();
        EXPECT_EQ(optimizations, 2u);
        EXPECT_TRUE(
            std::holds_alternative<mippp::status::optimal>(model.get_status()));
        EXPECT_FALSE(model.get_infeasibility_certificate());
        EXPECT_GE(model.get_solution()[x], 2. - 1e-6);
        EXPECT_LE(model.get_solution()[x], 3. + 1e-6);
        EXPECT_GT(certificate->variable_upper[0],
                  0.);  // independent of re-solve
    }
}

TEST(MosekCertificate, BoundAwareSeedAndInvalidSupportFallback) {
    using namespace mippp::mosek::impl::v1;
    for(int optimizer : {MSK_OPTIMIZER_PRIMAL_SIMPLEX, MSK_OPTIMIZER_INTPNT}) {
        SCOPED_TRACE(optimizer);
        auto factory = [&] { return mosek_certificate_model(optimizer); };
        const auto system = clp_ray_system();
        // An interior-point certificate can legitimately be dense. A coarser
        // support threshold is a heuristic, made safe by mandatory
        // revalidation.
        const double tolerance = optimizer == MSK_OPTIMIZER_INTPNT ? .01 : 1e-9;
        const auto answer = compute_linear_iis<linear_policy{
            .elasticity = elasticity_strategy::reuse, .native_seed = true}>(
            system, factory,
            linear_options{.native = {.relative_tolerance = tolerance},
                           .order = rows_first_order{}});
        ASSERT_TRUE(answer.native_seed_used);
        EXPECT_EQ(answer.native_seed_size,
                  2u);  // redundant variable bounds excluded
        ASSERT_TRUE(answer.reduction.irreducible);
        EXPECT_EQ(answer.members.size(), 2u);
        EXPECT_EQ(answer.elasticity_calls, 0u);
        EXPECT_NE(std::find(answer.members.begin(), answer.members.end(),
                            member{member_kind::row_lower, 0}),
                  answer.members.end());
        EXPECT_NE(std::find(answer.members.begin(), answer.members.end(),
                            member{member_kind::variable_upper, 0}),
                  answer.members.end());
        const auto fallback = compute_linear_iis<linear_policy{
            .elasticity = elasticity_strategy::rebuild, .native_seed = true}>(
            system, factory,
            linear_options{.native = {.relative_tolerance = 1.},
                           .order = bounds_first_order{}});
        EXPECT_FALSE(fallback.native_seed_used);
        EXPECT_TRUE(fallback.reduction.irreducible);
        EXPECT_EQ(fallback.members.size(), 2u);
        const auto defaults =
            compute_linear_iis<linear_policy{.native_seed = true}>(system,
                                                                   factory);
        EXPECT_TRUE(
            defaults.reduction.irreducible);  // dense support may fall back
        EXPECT_EQ(defaults.members.size(), 2u);
    }
}

TEST(MosekCertificate, SharedBudgetsAndCancellationPreserveEvidence) {
    using namespace mippp::mosek::impl::v1;
    for(std::size_t budget = 0; budget < 7; ++budget) {
        std::size_t constructions = 0;
        auto factory = [&] {
            ++constructions;
            return mosek_certificate_model(MSK_OPTIMIZER_INTPNT);
        };
        const auto answer =
            compute_linear_iis<linear_policy{.native_seed = true}>(
                clp_ray_system(), factory,
                linear_options{
                    .limits = {.max_solves = budget,
                               .time_limit = std::chrono::seconds(30)},
                    .native = {.relative_tolerance = .01}});
        EXPECT_EQ(answer.reduction.solve_count, constructions);
        EXPECT_LE(constructions, budget);
        EXPECT_EQ(answer.reduction.proven_infeasible(), budget != 0);
        if(answer.reduction.irreducible) {
            EXPECT_EQ(answer.members.size(), 2u);
        }
    }
    std::stop_source stop;
    unsigned calls = 0;
    const auto answer = compute_linear_iis<linear_policy{.native_seed = true}>(
        clp_ray_system(),
        [&] {
            if(++calls == 2) stop.request_stop();
            return mosek_certificate_model(MSK_OPTIMIZER_INTPNT);
        },
        linear_options{.limits = {.stop = stop.get_token()},
                       .native = {.relative_tolerance = .01}});
    EXPECT_EQ(calls, 2u);
    EXPECT_EQ(answer.reduction.reason, termination::cancelled);
    EXPECT_TRUE(answer.reduction.proven_infeasible());
    EXPECT_FALSE(answer.native_seed_used);
    EXPECT_FALSE(answer.reduction.irreducible);
    EXPECT_EQ(answer.members.size(), 17u);
}

TEST(MosekCertificate, WeightedOrderWithFullCertificateAndRetainedDeletion) {
    using namespace mippp::mosek::impl::v1;
    auto factory = [] {
        return mosek_certificate_model(MSK_OPTIMIZER_PRIMAL_SIMPLEX);
    };
    for(std::size_t budget : {1u, 3u, 30u}) {
        const auto answer = compute_linear_iis<linear_policy{
            .deletion = deletion_strategy::reuse,
            .native_seed = true,
            .prune_bounds = true,
            .order_by_weight = true}>(
            clp_ray_system(), factory,
            linear_options{.limits = {.max_solves = budget},
                           .order = rows_first_order{}});
        EXPECT_TRUE(answer.reduction.proven_infeasible());
        EXPECT_LE(answer.reduction.solve_count, budget);
        if(budget == 30) {
            EXPECT_TRUE(answer.reduction.irreducible);
            EXPECT_EQ(answer.members.size(), 2u);
        }
    }
}

TEST(MosekCertificate, MilpWrapperOptimizesOnceAndCanReadContinuousSolutions) {
    mippp::mosek_milp model;
    const auto x = model.add_variable(
        {.obj_coef = 1., .lower_bound = 0., .upper_bound = 1.});
    model.set_maximization();
    unsigned optimizations = 0;
    model.native_api()._check(model.native_api().putcallbackfunc(
        model.native_model().second, count_mosek_optimizations,
        &optimizations));
    model.solve();
    EXPECT_EQ(optimizations, 1u);
    EXPECT_TRUE(
        std::holds_alternative<mippp::status::optimal>(model.get_status()));
    EXPECT_NEAR(model.get_solution()[x], 1., 1e-6);
    EXPECT_NEAR(model.get_solution_value(), 1., 1e-6);
    EXPECT_EQ(optimizations, 1u);
}

TEST(DeletionFilter, RelativeTimeLimitNormalizationAndValidation) {
    using namespace std::chrono;
    const auto now = steady_clock::now();
    auto opts = detail::normalize_limits({.time_limit = 250ms}, now);
    EXPECT_EQ(opts.deadline, now + 250ms);
    EXPECT_EQ(detail::normalize_limits(opts, now + 100ms).deadline,
              opts.deadline);
    EXPECT_EQ(detail::normalize_limits(
                  {.deadline = now + 50ms, .time_limit = 250ms}, now)
                  .deadline,
              now + 50ms);
    EXPECT_EQ(
        detail::normalize_limits({.time_limit = duration<double>::max()}, now)
            .deadline,
        steady_clock::time_point::max());
    auto oracle = [](auto) {
        ADD_FAILURE();
        return feasibility::unknown;
    };
    auto answer = deletion_filter(1, oracle, {.time_limit = 0s});
    EXPECT_EQ(answer.reason, termination::time_limit);
    EXPECT_EQ(answer.solve_count, 0u);
    EXPECT_THROW((void)deletion_filter(1, oracle, {.time_limit = -1s}),
                 std::invalid_argument);
    EXPECT_THROW(
        (void)deletion_filter(1, oracle,
                              {.time_limit = duration<double>(
                                   std::numeric_limits<double>::quiet_NaN())}),
        std::invalid_argument);
}

TEST(DeletionFilter, StopReasonPriorityAndExactCompletion) {
    const auto now = std::chrono::steady_clock::now();
    std::stop_source source;
    options opts{.max_solves = 0, .deadline = now, .stop = source.get_token()};
    EXPECT_EQ(detail::stop_reason(opts, 0, now), termination::time_limit);
    source.request_stop();
    EXPECT_EQ(detail::stop_reason(opts, 0, now), termination::cancelled);
    // Completing the proof on the last permitted call is success, not a limit.
    const auto answer =
        deletion_filter(1,
                        [](auto ids) {
                            return ids.empty() ? feasibility::feasible
                                               : feasibility::infeasible;
                        },
                        {.max_solves = 2});
    EXPECT_TRUE(answer.irreducible);
    EXPECT_EQ(answer.reason, termination::completed);
    EXPECT_EQ(answer.solve_count, 2u);
}

TEST(DeletionFilter, UnknownOnLastCallReportsExhaustedBudget) {
    for(bool ordered : {false, true}) {
        auto oracle = [](auto ids) {
            return ids.empty() ? feasibility::unknown : feasibility::infeasible;
        };
        auto answer = ordered ? deletion_filter(1, oracle, {.max_solves = 2},
                                                std::less<std::size_t>{})
                              : deletion_filter(1, oracle, {.max_solves = 2});
        EXPECT_TRUE(answer.proven_infeasible());
        EXPECT_FALSE(answer.irreducible);
        EXPECT_EQ(answer.members.size(), 1u);
        EXPECT_EQ(answer.reason, termination::solve_limit);
    }
}

TEST(DeletionFilter, DeadlineExpiringInsideOraclePreservesProof) {
    for(bool final_proof : {false, true}) {
        // Leave enough time for the initial proof on a busy CI runner. The
        // second oracle call still holds the deadline until it expires.
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(1);
        const auto answer =
            deletion_filter(1,
                            [&](auto ids) {
                                if(!ids.empty()) return feasibility::infeasible;
                                std::this_thread::sleep_until(deadline);
                                // Some sleep_until implementations can wake a
                                // fraction early relative to steady_clock.
                                while(std::chrono::steady_clock::now() < deadline)
                                    std::this_thread::yield();
                                return final_proof ? feasibility::feasible
                                                   : feasibility::unknown;
                            },
                            {.deadline = deadline});
        // The clock check above forces expiry without asserting elapsed time.
        EXPECT_TRUE(answer.proven_infeasible());
        EXPECT_EQ(answer.irreducible, final_proof);
        EXPECT_EQ(answer.reason, final_proof ? termination::completed
                                             : termination::time_limit);
    }
}

TEST(ElasticityFilter, UnknownAfterCancellationReportsCancellation) {
    std::stop_source source;
    const auto answer = elasticity_filter(1,
                                          [&](auto) -> elastic_trial {
                                              source.request_stop();
                                              return {};
                                          },
                                          {.stop = source.get_token()});
    EXPECT_EQ(answer.reason, termination::cancelled);
    EXPECT_FALSE(answer.proven_infeasible);
    EXPECT_EQ(answer.solve_count, 1u);
    EXPECT_EQ(elasticity_filter(1, [](auto) -> elastic_trial { return {}; },
                                {.max_solves = 1})
                  .reason,
              termination::solve_limit);
    EXPECT_EQ(elasticity_filter(1,
                                [](auto) -> elastic_trial {
                                    ADD_FAILURE();
                                    return {};
                                },
                                {.time_limit = std::chrono::seconds(0)})
                  .solve_count,
              0u);
}

namespace {
// A plain capability stub, not a solver subclass. An explicit clock argument
// makes fractional forwarding and warm re-solve checks deterministic.
struct timed_model {
    std::chrono::duration<double> limit{100};
    unsigned writes = 0;
    void set_time_limit(std::chrono::duration<double> value) {
        limit = value;
        ++writes;
    }
    auto get_time_limit() { return limit; }
    std::variant<mippp::status::time_limit> get_status();
};
static_assert(mippp::has_time_limit<timed_model>);
struct optional_timed_model : timed_model {
    bool available = false;
    bool time_limit_available() const { return available; }
    void set_time_limit(std::chrono::duration<double> value) {
        if(!available) throw std::runtime_error("missing time-limit setter");
        timed_model::set_time_limit(value);
    }
};
}  // namespace

TEST(DeletionFilter, OptionalNativeDeadlineFallsBackWithoutCallingSetter) {
    using namespace std::chrono;
    const auto now = steady_clock::now();
    options opts{.deadline = now + 250ms};
    work_statistics stats;
    optional_timed_model model;
    EXPECT_FALSE(detail::native_time_limit_available(model));
    EXPECT_TRUE(detail::prepare_iis_solve(model, opts, stats, now));
    EXPECT_EQ(model.writes, 0u);
    EXPECT_FALSE(stats.observed_solver_time_limit);
    EXPECT_FALSE(detail::prepare_iis_solve(model, opts, stats, opts.deadline));
    EXPECT_THROW(model.set_time_limit(1s), std::runtime_error);
    model.available = true;
    EXPECT_TRUE(detail::native_time_limit_available(model));
    EXPECT_TRUE(detail::prepare_iis_solve(model, opts, stats, now));
    EXPECT_EQ(model.writes, 1u);
    EXPECT_DOUBLE_EQ(model.limit.count(), .25);
}

TEST(DeletionFilter, NativeDeadlineSetterFailuresAreNotHidden) {
    struct failing_model : timed_model {
        void set_time_limit(std::chrono::duration<double>) {
            throw std::runtime_error("native setter failed");
        }
    } model;
    work_statistics stats;
    const auto now = std::chrono::steady_clock::now();
    EXPECT_THROW(
        detail::prepare_iis_solve(
            model, {.deadline = now + std::chrono::seconds(1)}, stats, now),
        std::runtime_error);
}

TEST(DeletionFilter, NativeDeadlineForwardingPreservesTighterCaps) {
    work_statistics stats;
    using namespace std::chrono;
    const auto now = steady_clock::now();
    options opts{.deadline = now + 750ms};
    timed_model model;
    ASSERT_TRUE(detail::prepare_iis_solve(model, opts, stats, now));
    EXPECT_DOUBLE_EQ(model.limit.count(), .75);
    ASSERT_TRUE(detail::prepare_iis_solve(model, opts, stats, now + 500ms));
    EXPECT_DOUBLE_EQ(model.limit.count(), .25);
    model.limit = 10ms;
    ASSERT_TRUE(detail::prepare_iis_solve(model, opts, stats, now + 600ms));
    EXPECT_DOUBLE_EQ(model.limit.count(), .01);
    const auto writes = model.writes;
    EXPECT_FALSE(detail::prepare_iis_solve(model, opts, stats, opts.deadline));
    EXPECT_EQ(model.writes, writes);
    ASSERT_TRUE(detail::prepare_iis_solve(model, {}, stats, now));
    EXPECT_EQ(model.writes,
              writes);  // unlimited means leave factory settings alone
    struct unsupported {
    } fallback;
    EXPECT_TRUE(detail::prepare_iis_solve(fallback, opts, stats, now));
    EXPECT_FALSE(
        detail::prepare_iis_solve(fallback, opts, stats, opts.deadline));
    std::stop_source stop;
    stop.request_stop();
    EXPECT_FALSE(detail::prepare_iis_solve(model, {.stop = stop.get_token()},
                                           stats, now));
    EXPECT_EQ(model.writes, writes);
}

TEST(ElasticityFilter, AccumulatesViolationsUntilInfeasibility) {
    // The first violated side is feasible alone. Only after a second solve
    // identifies the other side can the next trial prove their contradiction.
    auto oracle = [](std::span<const std::size_t> hard) -> elastic_trial {
        if(hard.empty()) return {feasibility::feasible, {2}};
        if(hard.size() == 1) return {feasibility::feasible, {5}};
        return {feasibility::infeasible, {}};
    };
    auto answer = elasticity_filter(8, oracle);
    ASSERT_TRUE(answer.proven_infeasible);
    EXPECT_EQ(answer.members, (std::vector<std::size_t>{2, 5}));
    EXPECT_EQ(answer.solve_count, 3u);
    auto limited = elasticity_filter(8, oracle, {.max_solves = 2});
    EXPECT_FALSE(limited.proven_infeasible);
    EXPECT_EQ(limited.reason, termination::solve_limit);
}

TEST(ElasticityFilter, UnknownAndNoProgressCannotCertifySeed) {
    EXPECT_FALSE(elasticity_filter(3, [](auto) -> elastic_trial {
                     return {};
                 }).proven_infeasible);
    EXPECT_FALSE(elasticity_filter(3, [](auto) -> elastic_trial {
                     return {feasibility::feasible, {}};
                 }).proven_infeasible);
    auto malformed = [](auto) -> elastic_trial {
        return {feasibility::feasible, {9}};
    };
    EXPECT_THROW((void)elasticity_filter(3, malformed), std::invalid_argument);
    auto repeated = [](auto) -> elastic_trial {
        return {feasibility::feasible, {0}};
    };
    EXPECT_THROW((void)elasticity_filter(3, repeated), std::invalid_argument);
}

TEST(ElasticityFilter, CancellationAndDeadline) {
    auto oracle = [](auto) -> elastic_trial {
        ADD_FAILURE() << "Oracle must not run after cancellation/deadline";
        return {};
    };
    std::stop_source stop;
    stop.request_stop();
    EXPECT_EQ(elasticity_filter(1, oracle, {.stop = stop.get_token()}).reason,
              termination::cancelled);
    EXPECT_EQ(elasticity_filter(1, oracle,
                                {.deadline = std::chrono::steady_clock::now()})
                  .reason,
              termination::time_limit);
}

TEST(DeletionFilter, ExhaustiveMonotoneOracles) {
    // Every family of conflicts over four candidates. The oracle declares a
    // set infeasible when it contains any selected conflict. Includes empty
    // conflicts, overlapping conflicts and feasible initial systems.
    // Four candidates have 16 subsets, hence 2^16 families. Distinct families
    // can encode the same monotone oracle; that redundancy is cheap at this
    // size and avoids depending on a separate minimal-conflict enumerator.
    for(unsigned family = 0; family < (1u << 16); ++family) {
        auto oracle = [family](std::span<const std::size_t> ids) {
            unsigned mask = 0;
            for(auto id : ids) mask |= 1u << id;
            for(unsigned conflict = 0; conflict < 16; ++conflict)
                if((family & (1u << conflict)) && (mask & conflict) == conflict)
                    return feasibility::infeasible;
            return feasibility::feasible;
        };
        for(auto batch : {0u, 1u, 2u, 3u, 4u, 32u}) {
            for(bool prioritized : {false, true}) {
                SCOPED_TRACE(::testing::Message()
                             << "family=" << family << " batch=" << batch);
                const auto answer =
                    prioritized ? deletion_filter(4, oracle,
                                                  {.initial_batch_size = batch},
                                                  std::greater<std::size_t>{})
                                : deletion_filter(
                                      4, oracle, {.initial_batch_size = batch});
                if(batch <= 1) {
                    ASSERT_LE(answer.solve_count, 5u);
                }
                if(family == 0) {
                    ASSERT_EQ(answer.initial_status, feasibility::feasible);
                    ASSERT_FALSE(answer.irreducible);
                    continue;
                }
                ASSERT_TRUE(answer.proven_infeasible());
                ASSERT_TRUE(answer.irreducible);
                ASSERT_EQ(oracle(answer.members), feasibility::infeasible);
                // Check the defining property directly, without assuming a
                // particular IIS or traversal order: every single-member
                // deletion must be feasible.
                for(std::size_t i = 0; i < answer.members.size(); ++i) {
                    auto subset = answer.members;
                    subset.erase(subset.begin() +
                                 static_cast<std::ptrdiff_t>(i));
                    ASSERT_EQ(oracle(subset), feasibility::feasible);
                }
            }
        }
    }
}

TEST(DeletionFilter, PriorityIsPreservedAfterSuccessfulRemoval) {
    std::vector<std::size_t> removed;
    std::vector<std::size_t> previous{0, 1, 2, 3, 4};
    auto oracle = [&](std::span<const std::size_t> ids) {
        if(ids.size() < previous.size()) {
            for(auto id : previous)
                if(std::find(ids.begin(), ids.end(), id) == ids.end())
                    removed.push_back(id);
        }
        previous.assign(ids.begin(), ids.end());
        return ids.empty() ? feasibility::feasible : feasibility::infeasible;
    };
    auto order = [state = std::make_unique<int>(0)](std::size_t a,
                                                    std::size_t b) {
        ++*state;
        return a > b;
    };
    const auto answer = deletion_filter(5, oracle, {}, std::move(order));
    EXPECT_EQ(removed, (std::vector<std::size_t>{4, 3, 2, 1, 0}));
    EXPECT_EQ(answer.members, (std::vector<std::size_t>{0}));
    EXPECT_TRUE(answer.irreducible);
}

TEST(DeletionFilter, OrderingChangesSelectedConflictWithoutChangingGuarantees) {
    auto oracle = [](std::span<const std::size_t> ids) {
        const auto has = [&](auto id) {
            return std::find(ids.begin(), ids.end(), id) != ids.end();
        };
        return (has(0) && has(1)) || (has(2) && has(3))
                   ? feasibility::infeasible
                   : feasibility::feasible;
    };
    auto first = deletion_filter(4, oracle, {}, std::less<std::size_t>{});
    auto last = deletion_filter(4, oracle, {}, std::greater<std::size_t>{});
    std::sort(first.members.begin(), first.members.end());
    std::sort(last.members.begin(), last.members.end());
    EXPECT_EQ(first.members, (std::vector<std::size_t>{2, 3}));
    EXPECT_EQ(last.members, (std::vector<std::size_t>{0, 1}));
    EXPECT_TRUE(first.irreducible);
    EXPECT_TRUE(last.irreducible);
}

TEST(DeletionFilter, OrderedUnknownAndLimitsRetainProof) {
    auto oracle = [](auto ids) {
        return ids.size() == 4 ? feasibility::infeasible : feasibility::unknown;
    };
    auto answer = deletion_filter(4, oracle, {}, std::greater<std::size_t>{});
    EXPECT_TRUE(answer.proven_infeasible());
    EXPECT_FALSE(answer.irreducible);
    EXPECT_EQ(answer.members.size(), 4u);
    answer = deletion_filter(4, oracle, {.max_solves = 2},
                             std::greater<std::size_t>{});
    EXPECT_EQ(answer.reason, termination::solve_limit);
    EXPECT_TRUE(answer.proven_infeasible());
    EXPECT_EQ(answer.members.size(), 4u);
}

TEST(DeletionFilter, EquivalentPrioritiesAndNoSortingAfterBudgetExhaustion) {
    std::size_t comparisons = 0;
    auto equal = [&](std::size_t, std::size_t) {
        ++comparisons;
        return false;
    };
    auto oracle = [](auto ids) {
        return ids.empty() ? feasibility::feasible : feasibility::infeasible;
    };
    auto stopped = deletion_filter(4, oracle, {.max_solves = 1}, equal);
    EXPECT_EQ(comparisons, 0u);
    EXPECT_TRUE(stopped.proven_infeasible());
    auto complete = deletion_filter(4, oracle, {}, equal);
    EXPECT_EQ(complete.members, (std::vector<std::size_t>{3}));
    EXPECT_TRUE(complete.irreducible);
}

TEST(DeletionFilter, BatchingReducesCallsForSparseConflict) {
    // Only IDs 17 and 900 matter; the other 1022 assumptions can be discarded.
    auto oracle = [](std::span<const std::size_t> ids) {
        return std::find(ids.begin(), ids.end(), 17u) != ids.end() &&
                       std::find(ids.begin(), ids.end(), 900u) != ids.end()
                   ? feasibility::infeasible
                   : feasibility::feasible;
    };
    auto single = deletion_filter(1024, oracle);
    auto batched = deletion_filter(1024, oracle, {.initial_batch_size = 64});
    ASSERT_TRUE(single.irreducible);
    ASSERT_TRUE(batched.irreducible);
    std::sort(single.members.begin(), single.members.end());
    std::sort(batched.members.begin(), batched.members.end());
    EXPECT_EQ(single.members, batched.members);
    EXPECT_LT(batched.solve_count, single.solve_count / 4);
    RecordProperty("single_calls", single.solve_count);
    RecordProperty("batched_calls", batched.solve_count);
}

TEST(DeletionFilter, UnknownBatchCanStillProduceCertifiedIis) {
    // The initial set is infeasible; two-at-a-time removals are inconclusive.
    // Singles subsequently establish that all four members are necessary.
    auto oracle = [](std::span<const std::size_t> ids) {
        if(ids.size() == 4) return feasibility::infeasible;
        if(ids.size() == 2) return feasibility::unknown;
        return feasibility::feasible;
    };
    auto answer = deletion_filter(4, oracle, {.initial_batch_size = 2});
    EXPECT_TRUE(answer.irreducible);
    EXPECT_EQ(answer.members.size(), 4u);
    EXPECT_EQ(answer.reason, termination::completed);
}

TEST(DeletionFilter, BatchBudgetKeepsOnlyProvenDeletions) {
    auto oracle = [](std::span<const std::size_t> ids) {
        return std::find(ids.begin(), ids.end(), 7u) != ids.end()
                   ? feasibility::infeasible
                   : feasibility::feasible;
    };
    for(std::size_t budget = 0; budget < 16; ++budget) {
        auto answer = deletion_filter(
            8, oracle, {.max_solves = budget, .initial_batch_size = 4});
        EXPECT_LE(answer.solve_count, budget);
        if(answer.proven_infeasible()) {
            EXPECT_EQ(oracle(answer.members), feasibility::infeasible);
        }
        if(answer.irreducible) {
            ASSERT_EQ(answer.members.size(), 1u);
            EXPECT_EQ(answer.members.front(), 7u);
        } else {
            EXPECT_EQ(answer.reason, termination::solve_limit);
        }
    }
}

TEST(DeletionFilter, CancellationAfterUnknownBatchRestoresFullSet) {
    std::stop_source source;
    std::size_t calls = 0;
    auto oracle = [&](auto) {
        if(++calls == 1) return feasibility::infeasible;
        source.request_stop();
        return feasibility::unknown;
    };
    auto answer = deletion_filter(
        8, oracle, {.stop = source.get_token(), .initial_batch_size = 4});
    EXPECT_TRUE(answer.proven_infeasible());
    EXPECT_FALSE(answer.irreducible);
    EXPECT_EQ(answer.members.size(), 8u);
    EXPECT_EQ(answer.reason, termination::cancelled);
    EXPECT_EQ(answer.solve_count, 2u);
}

TEST(DeletionFilter, UnknownTrialPreservesProvenConflict) {
    auto oracle = [](std::span<const std::size_t> ids) {
        return ids.size() == 3 ? feasibility::infeasible : feasibility::unknown;
    };
    auto answer = deletion_filter(3, oracle);
    EXPECT_TRUE(answer.proven_infeasible());
    EXPECT_FALSE(answer.irreducible);
    EXPECT_EQ(answer.members.size(), 3u);
    EXPECT_EQ(answer.reason, termination::indeterminate);
}

TEST(DeletionFilter, InitialUnknownIsNotAConflict) {
    auto answer = deletion_filter(2, [](auto) { return feasibility::unknown; });
    EXPECT_FALSE(answer.proven_infeasible());
    EXPECT_EQ(answer.solve_count, 1u);
}

TEST(DeletionFilter, LimitsAndCancellation) {
    auto oracle = [](auto) { return feasibility::infeasible; };
    auto answer = deletion_filter(3, oracle, {.max_solves = 2});
    EXPECT_TRUE(answer.proven_infeasible());
    EXPECT_FALSE(answer.irreducible);
    EXPECT_EQ(answer.members.size(), 2u);
    EXPECT_EQ(answer.reason, termination::solve_limit);
    answer = deletion_filter(3, oracle, {.max_solves = 0});
    EXPECT_EQ(answer.solve_count, 0u);
    EXPECT_FALSE(answer.proven_infeasible());
    answer = deletion_filter(3, oracle,
                             {.deadline = std::chrono::steady_clock::now()});
    EXPECT_EQ(answer.reason, termination::time_limit);
    EXPECT_EQ(answer.solve_count, 0u);
    std::stop_source source;
    source.request_stop();
    answer = deletion_filter(3, oracle, {.stop = source.get_token()});
    EXPECT_EQ(answer.reason, termination::cancelled);
    EXPECT_EQ(answer.solve_count, 0u);
}

TEST(DeletionFilter, MoveOnlyOracleAndExceptions) {
    auto oracle = [state = std::make_unique<int>(0)](auto subset) {
        ++*state;
        return subset.empty() ? feasibility::feasible : feasibility::infeasible;
    };
    EXPECT_TRUE(deletion_filter(1, std::move(oracle)).irreducible);
    auto throws = [](auto) -> feasibility {
        throw std::runtime_error("oracle");
    };
    EXPECT_THROW((void)deletion_filter(1, throws), std::runtime_error);
}

TEST(DeletionFilter, EmptyCandidateSet) {
    auto answer =
        deletion_filter(0, [](auto) { return feasibility::infeasible; });
    EXPECT_TRUE(answer.irreducible);  // fixed background itself is infeasible
    EXPECT_TRUE(answer.members.empty());
    EXPECT_EQ(answer.solve_count, 1u);
}

TEST(DeletionFilter, CancellationDuringOracleKeepsLastProvenSubset) {
    std::stop_source source;
    auto oracle = [&](auto) {
        source.request_stop();
        return feasibility::infeasible;
    };
    auto answer = deletion_filter(3, oracle, {.stop = source.get_token()});
    EXPECT_TRUE(answer.proven_infeasible());
    EXPECT_FALSE(answer.irreducible);
    EXPECT_EQ(answer.solve_count, 1u);
    EXPECT_EQ(answer.members.size(), 3u);
    EXPECT_EQ(answer.reason, termination::cancelled);
}

template <typename Model>
class LinearIis : public ::testing::Test {};
using Backends =
    ::testing::Types<mippp::highs_lp, mippp::highs_milp, mippp::clp_lp,
                     mippp::cbc_milp, mippp::gurobi_milp, mippp::cplex_milp,
                     mippp::copt_milp, mippp::glpk_milp, mippp::mosek_milp,
                     mippp::scip_milp, mippp::soplex_lp, mippp::xpress_milp,
                     mippp::mosek_lp>;
TYPED_TEST_SUITE(LinearIis, Backends);

TYPED_TEST(LinearIis, FactoryLoggingIsPreservedAcrossIisWorkspaces) {
    if constexpr(mippp::has_verbosity<TypeParam>) {
        // Check the setting at every native solve, not just at construction.
        // Backend log emission itself is covered by the shared VerbosityTest.
        struct logging_model : TypeParam {
            bool expected_verbose = false;
            void solve() {
                EXPECT_EQ(this->is_verbose(), expected_verbose);
                TypeParam::solve();
            }
        };
        linear_system<> system;
        system.variables = {{std::nullopt, 1., false}};
        system.rows = {{{{0, 1.}}, 2., std::nullopt}};
        for_each_bool([&]<bool warm>() {
            for(bool verbose : {false, true}) {
                auto factory = [&] {
                    logging_model model;
                    EXPECT_FALSE(model.is_verbose());
                    model.expected_verbose = verbose;
                    if(verbose) model.set_verbose(true);
                    return model;
                };
                const auto answer = compute_linear_iis<linear_policy{
                    .elasticity = warm ? elasticity_strategy::reuse
                                       : elasticity_strategy::rebuild,
                    .deletion = warm ? deletion_strategy::reuse
                                     : deletion_strategy::rebuild}>(system,
                                                                    factory);
                EXPECT_TRUE(answer.reduction.irreducible);
                EXPECT_GT(answer.statistics.elastic.solver_runs, 0u);
            }
        });
    }
}

TYPED_TEST(LinearIis,
           RuntimeMissingTimeLimitPreservesExtractionAndDiagnostics) {
    // Simulate a library without its optional setter while using a real solver
    // for feasibility. No ABI mutation or licensed solver is needed for HiGHS.
    struct model_without_timer : TypeParam {
        bool time_limit_available() const { return false; }
        void set_time_limit(std::chrono::duration<double>) {
            throw std::runtime_error("optional setter must not be called");
        }
    };
    linear_system<> system;
    system.variables = {{std::nullopt, 1., false}};
    system.rows = {{{{0, 1.}}, 2., std::nullopt}};
    for_each_bool([&]<bool warm>() {
        const auto answer = compute_linear_iis<linear_policy{
            .elasticity = warm ? elasticity_strategy::reuse
                               : elasticity_strategy::rebuild,
            .deletion =
                warm ? deletion_strategy::reuse : deletion_strategy::rebuild}>(
            system, [] { return model_without_timer{}; },
            linear_options{.limits = {.time_limit = std::chrono::seconds(30)}});
        EXPECT_TRUE(answer.reduction.irreducible);
        EXPECT_FALSE(answer.diagnostics.solver_time_limit_supported);
        EXPECT_FALSE(answer.statistics.rebuild.observed_solver_time_limit);
        EXPECT_FALSE(answer.statistics.deletion.observed_solver_time_limit);
        EXPECT_FALSE(answer.statistics.elastic.observed_solver_time_limit);
    });
}

// Tests deliberately do not catch exceptions: missing or broken libraries
// must fail this explicitly selected integration executable.
TYPED_TEST(LinearIis, RemovesRedundantRowsAndBounds) {
    linear_system<> system;
    system.variables = {{-100., 100., false}};
    system.rows = {{{{0, 1.}}, 2., std::nullopt},
                   {{{0, 1.}}, std::nullopt, 1.},
                   {{{0, 1.}}, std::nullopt, 50.}};
    auto answer = compute_linear_iis(system, [] { return TypeParam{}; });
    ASSERT_TRUE(answer.reduction.irreducible);
    ASSERT_EQ(answer.members.size(), 2u);
    EXPECT_NE(std::find(answer.members.begin(), answer.members.end(),
                        member{member_kind::row_lower, 0}),
              answer.members.end());
    EXPECT_NE(std::find(answer.members.begin(), answer.members.end(),
                        member{member_kind::row_upper, 1}),
              answer.members.end());
}

TYPED_TEST(LinearIis, NativeSeedOptionIsOptionalAcrossBackends) {
    auto system = clp_ray_system();
    const auto answer = compute_linear_iis<linear_policy{.native_seed = true}>(
        system, [] { return TypeParam{}; });
    EXPECT_TRUE(answer.reduction.irreducible);
    EXPECT_EQ(answer.members.size(), 2u);
    if constexpr(!has_infeasibility_ray<TypeParam> &&
                 !has_infeasibility_certificate<TypeParam>) {
        EXPECT_FALSE(answer.native_seed_used);
        EXPECT_EQ(answer.native_seed_size, 0u);
    }
}

TYPED_TEST(LinearIis, TimeLimitsAndCancellationDoNotLoadOrSolveUnnecessarily) {
    linear_system<> system;
    system.variables = {{std::nullopt, std::nullopt, false}};
    system.rows = {{{{0, 1.}}, 2., std::nullopt},
                   {{{0, 1.}}, std::nullopt, 1.}};
    std::size_t constructions = 0;
    std::stop_source stop;
    auto factory = [&] {
        ++constructions;
        stop.request_stop();  // simulate cancellation during model construction
        return TypeParam{};
    };
    auto answer = compute_linear_iis<linear_policy{
        .elasticity = elasticity_strategy::reuse}>(
        system, factory,
        linear_options{.limits = {.time_limit = std::chrono::seconds(0)}});
    EXPECT_EQ(constructions, 0u);
    EXPECT_EQ(answer.reduction.solve_count, 0u);
    EXPECT_EQ(answer.reduction.reason, termination::time_limit);
    answer = compute_linear_iis(
        system, factory, linear_options{.limits = {.stop = stop.get_token()}});
    EXPECT_EQ(constructions, 1u);
    EXPECT_EQ(answer.reduction.solve_count, 1u);
    EXPECT_EQ(answer.reduction.reason, termination::cancelled);
    EXPECT_FALSE(answer.reduction.proven_infeasible());
    EXPECT_TRUE(answer.members.empty());
}

TYPED_TEST(LinearIis, NativeTimeLimitCapabilityAndPipelineBudgets) {
    work_statistics stats;
    using namespace std::chrono;
    if constexpr(mippp::has_time_limit<TypeParam>) {
        TypeParam model;
        const auto now = steady_clock::now();
        if(detail::native_time_limit_available(model)) {
            model.set_time_limit(duration<double>(5));
            ASSERT_TRUE(detail::prepare_iis_solve(
                model, {.deadline = now + 250ms}, stats, now));
            EXPECT_NEAR(duration<double>(model.get_time_limit()).count(), .25,
                        1e-6);
            model.set_time_limit(duration<double>(.125));
            ASSERT_TRUE(detail::prepare_iis_solve(
                model, {.deadline = now + 250ms}, stats, now));
            EXPECT_NEAR(duration<double>(model.get_time_limit()).count(), .125,
                        1e-6);
        } else {
            EXPECT_TRUE(detail::prepare_iis_solve(
                model, {.deadline = now + 250ms}, stats, now));
            EXPECT_FALSE(stats.observed_solver_time_limit);
        }
    }
    linear_system<> system;
    system.variables = {{std::nullopt, std::nullopt, false}};
    system.rows = {{{{0, 1.}}, 2., std::nullopt},
                   {{{0, 1.}}, std::nullopt, 1.},
                   {{{0, 1.}}, std::nullopt, 10.}};
    for_each_bool([&]<bool warm>() {
        for(std::size_t budget = 0; budget < 12; ++budget) {
            std::size_t constructions = 0;
            auto factory = [&] {
                ++constructions;
                return TypeParam{};
            };
            const auto answer = compute_linear_iis<linear_policy{
                .elasticity = (warm ? elasticity_strategy::reuse
                                    : elasticity_strategy::rebuild)}>(
                system, factory,
                linear_options{.limits = {.max_solves = budget,
                                          .initial_batch_size = 2,
                                          .time_limit = 30s},
                               .order = rows_first_order{}});
            EXPECT_LE(answer.reduction.solve_count, budget);
            EXPECT_EQ(constructions + answer.elasticity_reoptimizations,
                      answer.reduction.solve_count);
            if(budget == 0)
                EXPECT_FALSE(answer.reduction.proven_infeasible());
            else
                EXPECT_TRUE(answer.reduction.proven_infeasible());
            if(answer.reduction.irreducible) {
                EXPECT_EQ(answer.reduction.reason, termination::completed);
                EXPECT_EQ(answer.members.size(), 2u);
            } else
                EXPECT_EQ(answer.reduction.reason, termination::solve_limit);
        }
    });
}

TYPED_TEST(LinearIis, BoundOnlyConflict) {
    // No row participates. Also exercises APIs that reject contradictory
    // variable bounds during model construction, before solve() is possible.
    linear_system<> system;
    system.variables = {{2., 1., false}};
    auto answer = compute_linear_iis(system, [] { return TypeParam{}; });
    ASSERT_TRUE(answer.reduction.irreducible);
    ASSERT_EQ(answer.members.size(), 2u);
}

TYPED_TEST(LinearIis, RangedRowAndVariableBound) {
    linear_system<> system;
    system.variables = {{std::nullopt, 1., false}};
    system.rows = {{{{0, 1.}}, 2., 5.}};
    auto answer = compute_linear_iis(system, [] { return TypeParam{}; });
    ASSERT_TRUE(answer.reduction.irreducible);
    ASSERT_EQ(answer.members.size(), 2u);
    EXPECT_NE(std::find(answer.members.begin(), answer.members.end(),
                        member{member_kind::row_lower, 0}),
              answer.members.end());
}

TYPED_TEST(LinearIis, IntegerOnlyInfeasibilityAndRelaxation) {
    // x == 0.5 is feasible over the reals but impossible over the integers.
    // Dropping either side allows an integer witness (0 or 1), so both sides
    // must be retained in an original-domain IIS.
    linear_system<> system;
    system.variables = {{std::nullopt, std::nullopt, true}};
    system.rows = {{{{0, 1.}}, 0.5, 0.5}};
    auto factory = [] { return TypeParam{}; };
    if constexpr(mippp::milp_model<TypeParam>) {
        auto answer = compute_linear_iis(system, factory);
        ASSERT_TRUE(answer.reduction.irreducible);
        EXPECT_EQ(answer.members.size(), 2u);
    } else {
        EXPECT_THROW((void)compute_linear_iis(system, factory),
                     std::invalid_argument);
    }
    auto relaxed = compute_linear_iis<linear_policy{
        .analyzed_domain = domain::lp_relaxation}>(system, factory);
    EXPECT_EQ(relaxed.reduction.initial_status, feasibility::feasible);
    EXPECT_TRUE(relaxed.members.empty());
    EXPECT_FALSE(relaxed.reduction.irreducible);
}

TYPED_TEST(LinearIis, RejectsInvalidInputAndNonemptyFactory) {
    linear_system<> system;
    system.rows = {{{{0, 1.}}, 1., std::nullopt}};
    EXPECT_THROW((void)compute_linear_iis(system, [] { return TypeParam{}; }),
                 std::invalid_argument);
    system.rows.clear();
    EXPECT_THROW((void)compute_linear_iis(system,
                                          [] {
                                              auto model = TypeParam{};
                                              model.add_variable();
                                              return model;
                                          }),
                 std::invalid_argument);
}

TYPED_TEST(LinearIis, EmptyAndConstantSystems) {
    linear_system<> system;
    auto factory = [] { return TypeParam{}; };
    EXPECT_EQ(compute_linear_iis(system, factory).reduction.initial_status,
              feasibility::feasible);
    system.rows = {{{}, 1., std::nullopt}};
    auto answer = compute_linear_iis(system, factory);
    ASSERT_TRUE(answer.reduction.irreducible);
    EXPECT_EQ(answer.members.size(), 1u);
}

TYPED_TEST(LinearIis, DuplicateTermsAndOriginalIndices) {
    // Removing unrelated variable/row bounds must not renumber the explanation.
    // The repeated coefficient also checks normal expression accumulation.
    linear_system<> system;
    system.variables = {{0., 100., false}, {std::nullopt, 1., false}};
    system.rows = {{{{0, 1.}}, std::nullopt, 50.},
                   {{{1, 0.5}, {1, 0.5}}, 2., std::nullopt}};
    auto answer = compute_linear_iis(system, [] { return TypeParam{}; });
    ASSERT_TRUE(answer.reduction.irreducible);
    ASSERT_EQ(answer.members.size(), 2u);
    EXPECT_NE(std::find(answer.members.begin(), answer.members.end(),
                        member{member_kind::variable_upper, 1}),
              answer.members.end());
    EXPECT_NE(std::find(answer.members.begin(), answer.members.end(),
                        member{member_kind::row_lower, 1}),
              answer.members.end());
}

TYPED_TEST(LinearIis, ScratchReuseResetsBoundsAndRowTerms) {
    linear_system<> system;
    system.variables = {{std::nullopt, 1., false}, {-10., 10., false}};
    // Alternating row widths exercise shrinking/reusing the term buffer. The
    // duplicate coefficients cancel; the empty row must stay empty. Only x's
    // upper bound and row 1's lower side form the conflict.
    system.rows = {{{{0, 1.}, {1, 1.}, {1, -1.}, {0, 1.}}, -100., 100.},
                   {{{0, .5}, {0, .5}}, 2., 20.},
                   {{}, std::nullopt, 0.},
                   {{{1, 1.}}, -20., 20.}};
    auto factory = [] { return TypeParam{}; };
    const std::vector<member> expected{{member_kind::variable_upper, 0},
                                       {member_kind::row_lower, 1}};
    for_each_bool([&]<bool elastic>() {
        for_each_bool([&]<bool warm>() {
            for(std::size_t batch : {1u, 4u}) {
                const auto answer = compute_linear_iis<linear_policy{
                    .elasticity =
                        (elastic ? (warm ? elasticity_strategy::reuse
                                         : elasticity_strategy::rebuild)
                                 : elasticity_strategy::off)}>(
                    system, factory,
                    linear_options{.limits = {.initial_batch_size = batch}});
                ASSERT_TRUE(answer.reduction.irreducible);
                ASSERT_EQ(answer.members.size(), expected.size());
                for(auto m : expected)
                    EXPECT_NE(std::find(answer.members.begin(),
                                        answer.members.end(), m),
                              answer.members.end());
            }
        });
    });
    // Scratch belongs to one extraction only; a subsequent changed system
    // must not inherit bounds, terms, or seed-local IDs from the previous one.
    system.rows[1].lower = 0.;
    EXPECT_EQ(compute_linear_iis(system, factory).reduction.initial_status,
              feasibility::feasible);
}

TYPED_TEST(LinearIis, RetainedDeletionMatchesRebuildAndCountsModels) {
    const auto system = clp_ray_system();
    for_each_bool([&]<bool elastic>() {
        for(std::size_t batch : {1u, 4u}) {
            std::size_t constructions = 0;
            auto factory = [&] {
                ++constructions;
                return TypeParam{};
            };
            auto cold = compute_linear_iis<linear_policy{
                .elasticity = (elastic ? elasticity_strategy::reuse
                                       : elasticity_strategy::off)}>(
                system, factory,
                linear_options{.limits = {.initial_batch_size = batch},
                               .order = bounds_first_order{}});
            constructions = 0;
            auto warm = compute_linear_iis<linear_policy{
                .elasticity = (elastic ? elasticity_strategy::reuse
                                       : elasticity_strategy::off),
                .deletion = deletion_strategy::reuse}>(
                system, factory,
                linear_options{.limits = {.initial_batch_size = batch},
                               .order = bounds_first_order{}});
            ASSERT_TRUE(cold.reduction.irreducible);
            ASSERT_TRUE(warm.reduction.irreducible);
            ASSERT_EQ(warm.members.size(), cold.members.size());
            for(auto m : cold.members)
                EXPECT_NE(
                    std::find(warm.members.begin(), warm.members.end(), m),
                    warm.members.end());
            EXPECT_EQ(constructions + warm.elasticity_reoptimizations +
                          warm.deletion_reoptimizations,
                      warm.reduction.solve_count);
            EXPECT_EQ(warm.deletion_model_reused,
                      mippp::iis::detail::has_deletion_updates<TypeParam>);
            if constexpr(mippp::iis::detail::has_deletion_updates<TypeParam>) {
                EXPECT_GT(warm.deletion_reoptimizations, 0u);
                if(!elastic) {
                    EXPECT_EQ(constructions, 1u);
                }
            }
            if(!elastic) {
                EXPECT_EQ(warm.reduction.solve_count,
                          cold.reduction.solve_count);
            }
        }
    });
}

TYPED_TEST(LinearIis, RetainedDeletionBudgetsAndCancellation) {
    for(std::size_t budget = 0; budget < 8; ++budget) {
        std::size_t constructions = 0;
        auto factory = [&] {
            ++constructions;
            return TypeParam{};
        };
        const auto answer = compute_linear_iis<linear_policy{
            .deletion = deletion_strategy::reuse}>(
            clp_ray_system(), factory,
            linear_options{.limits = {.max_solves = budget}});
        EXPECT_LE(answer.reduction.solve_count, budget);
        EXPECT_EQ(answer.reduction.proven_infeasible(), budget > 0);
        EXPECT_EQ(constructions + answer.deletion_reoptimizations,
                  answer.reduction.solve_count);
        if(budget == 0) {
            EXPECT_EQ(constructions, 0u);
        }
    }
    std::stop_source stop;
    std::size_t constructions = 0;
    auto factory = [&] {
        ++constructions;
        stop.request_stop();
        return TypeParam{};
    };
    const auto answer =
        compute_linear_iis<linear_policy{.deletion = deletion_strategy::reuse}>(
            clp_ray_system(), factory,
            linear_options{.limits = {.stop = stop.get_token()}});
    EXPECT_EQ(constructions, 1u);
    EXPECT_EQ(answer.reduction.reason, termination::cancelled);
    EXPECT_FALSE(answer.reduction.proven_infeasible());
    EXPECT_EQ(answer.deletion_reoptimizations, 0u);
    constructions = 0;
    const auto timed =
        compute_linear_iis<linear_policy{.deletion = deletion_strategy::reuse}>(
            clp_ray_system(), factory,
            linear_options{.limits = {.time_limit = std::chrono::seconds(0)}});
    EXPECT_EQ(constructions, 0u);
    EXPECT_EQ(timed.reduction.reason, termination::time_limit);
}

TYPED_TEST(LinearIis, RetainedDeletionPreservesIntegralityAndConstantRows) {
    auto factory = [] { return TypeParam{}; };
    linear_system<> system;
    system.variables = {{2., 1., false}};
    auto bounds =
        compute_linear_iis<linear_policy{.deletion = deletion_strategy::reuse}>(
            system, factory);
    EXPECT_TRUE(bounds.reduction.irreducible);
    EXPECT_EQ(bounds.members.size(), 2u);
    system.variables.clear();
    system.rows = {{{}, 1., std::nullopt}};
    auto constant =
        compute_linear_iis<linear_policy{.deletion = deletion_strategy::reuse}>(
            system, factory);
    EXPECT_TRUE(constant.reduction.irreducible);
    EXPECT_EQ(constant.members.size(), 1u);
    system.rows.clear();
    auto empty =
        compute_linear_iis<linear_policy{.deletion = deletion_strategy::reuse}>(
            system, factory);
    EXPECT_EQ(empty.reduction.initial_status, feasibility::feasible);
    if constexpr(mippp::milp_model<TypeParam>) {
        system.variables = {{std::nullopt, std::nullopt, true}};
        system.rows = {{{{0, 1.}}, .5, .5}};
        auto integral = compute_linear_iis<linear_policy{
            .deletion = deletion_strategy::reuse}>(system, factory);
        EXPECT_TRUE(integral.reduction.irreducible);
        EXPECT_EQ(integral.members.size(), 2u);
        auto relaxed = compute_linear_iis<linear_policy{
            .analyzed_domain = domain::lp_relaxation,
            .deletion = deletion_strategy::reuse}>(system, factory);
        EXPECT_EQ(relaxed.reduction.initial_status, feasibility::feasible);
    }
}

TEST(ClpRay, RetainedDeletionRestoresRejectedSeedAndSharesBudget) {
    for(double tolerance : {1e-9, 1.}) {
        std::size_t constructions = 0;
        auto factory = [&] {
            ++constructions;
            return mippp::clp_lp{};
        };
        const auto answer = compute_linear_iis<linear_policy{
            .deletion = deletion_strategy::reuse, .native_seed = true}>(
            clp_ray_system(), factory,
            linear_options{.native = {.relative_tolerance = tolerance}});
        EXPECT_TRUE(answer.reduction.irreducible);
        EXPECT_EQ(answer.members.size(), 2u);
        EXPECT_EQ(answer.native_seed_used, tolerance < 1.);
        EXPECT_EQ(
            constructions,
            2u);  // original certificate model + retained feasibility model
        EXPECT_EQ(constructions + answer.deletion_reoptimizations,
                  answer.reduction.solve_count);
    }
}

TEST(ClpRay, RetainedWorkspaceRestoresNonmonotoneSubsets) {
    auto factory = [] { return mippp::clp_lp{}; };
    const std::vector<mippp::iis::detail::linear_inequality<double>> rows{
        {{{0, 1.}}, 2., true}, {{{0, 1.}}, 1., false}};
    work_statistics stats;
    mippp::iis::detail::deletion_workspace<mippp::clp_lp, double> workspace(
        {false}, rows, factory, {}, stats);
    for(const auto & subset : std::vector<std::vector<std::size_t>>{
            {0, 1}, {0}, {1}, {}, {0, 1}, {1}, {0, 1}})
        EXPECT_EQ(workspace(subset), subset.size() == 2
                                         ? feasibility::infeasible
                                         : feasibility::feasible);
    EXPECT_EQ(workspace.reoptimizations(), 6u);
    std::size_t calls = 0;
    auto unknown_trial = [&](std::span<const std::size_t> subset) {
        const auto state = workspace(subset);
        // Simulate an inconclusive first deletion after applying its updates.
        return ++calls == 2 ? feasibility::unknown : state;
    };
    auto answer = deletion_filter(2, unknown_trial);
    EXPECT_TRUE(answer.proven_infeasible());
    EXPECT_FALSE(answer.irreducible);
    EXPECT_EQ(answer.reason, termination::indeterminate);
    EXPECT_EQ(answer.members.size(), 2u);
    EXPECT_EQ(calls, 3u);
    const std::vector<std::size_t> full{0, 1};
    EXPECT_EQ(workspace(full), feasibility::infeasible);
}

TYPED_TEST(LinearIis, BatchedReductionMatchesSingleForUniqueConflict) {
    linear_system<> system;
    system.variables = {{std::nullopt, std::nullopt, false}};
    system.rows = {{{{0, 1.}}, 2., std::nullopt},
                   {{{0, 1.}}, std::nullopt, 1.}};
    for(unsigned i = 0; i < 32; ++i)
        system.rows.push_back({{{0, 1.}}, std::nullopt, 10. + i});
    auto factory = [] { return TypeParam{}; };
    auto single = compute_linear_iis(system, factory);
    auto batched = compute_linear_iis(
        system, factory, linear_options{.limits = {.initial_batch_size = 8}});
    ASSERT_TRUE(single.reduction.irreducible);
    ASSERT_TRUE(batched.reduction.irreducible);
    ASSERT_EQ(batched.members.size(), single.members.size());
    for(auto candidate : single.members)
        EXPECT_NE(std::find(batched.members.begin(), batched.members.end(),
                            candidate),
                  batched.members.end());
    EXPECT_LT(batched.reduction.solve_count, single.reduction.solve_count);
}

TYPED_TEST(LinearIis, ElasticPrefilterFindsAndRevalidatesSmallSeed) {
    linear_system<> system;
    system.variables = {{std::nullopt, std::nullopt, false}};
    system.rows = {{{{0, 1.}}, 2., std::nullopt},
                   {{{0, 1.}}, std::nullopt, 1.}};
    for(unsigned i = 0; i < 32; ++i)
        system.rows.push_back({{{0, 1.}}, std::nullopt, 10. + i});
    auto factory = [] { return TypeParam{}; };
    auto single = compute_linear_iis(system, factory);
    auto filtered = compute_linear_iis<linear_policy{
        .elasticity = elasticity_strategy::rebuild}>(system, factory);
    ASSERT_TRUE(filtered.reduction.irreducible);
    EXPECT_TRUE(filtered.elasticity_seed_used);
    EXPECT_GE(filtered.elasticity_calls, 3u);
    EXPECT_LT(filtered.reduction.solve_count, single.reduction.solve_count);
    ASSERT_EQ(filtered.members.size(), 2u);
    for(auto candidate : single.members)
        EXPECT_NE(std::find(filtered.members.begin(), filtered.members.end(),
                            candidate),
                  filtered.members.end());
}

TYPED_TEST(LinearIis, ElasticBoundsAndEmptyRows) {
    auto factory = [] { return TypeParam{}; };
    linear_system<> system;
    system.variables = {{2., 1., false}};
    auto bounds = compute_linear_iis<linear_policy{
        .elasticity = elasticity_strategy::rebuild}>(system, factory);
    ASSERT_TRUE(bounds.reduction.irreducible);
    EXPECT_TRUE(bounds.elasticity_seed_used);
    EXPECT_EQ(bounds.members.size(), 2u);
    system.variables.clear();
    system.rows = {{{}, 1., std::nullopt}};
    auto empty_row = compute_linear_iis<linear_policy{
        .elasticity = elasticity_strategy::rebuild}>(system, factory);
    EXPECT_TRUE(empty_row.reduction.irreducible);
    EXPECT_EQ(empty_row.members.size(), 1u);
}

TYPED_TEST(LinearIis, ElasticBudgetAndNoProgressFallBackSafely) {
    linear_system<> system;
    system.variables = {{std::nullopt, std::nullopt, false}};
    system.rows = {{{{0, 1.}}, 2., std::nullopt},
                   {{{0, 1.}}, std::nullopt, 1.}};
    auto factory = [] { return TypeParam{}; };
    for(std::size_t budget = 0; budget < 10; ++budget) {
        auto answer = compute_linear_iis<linear_policy{
            .elasticity = elasticity_strategy::rebuild}>(
            system, factory, linear_options{.limits = {.max_solves = budget}});
        EXPECT_LE(answer.reduction.solve_count, budget);
        if(budget > 0) {
            EXPECT_TRUE(answer.reduction.proven_infeasible());
        }
        if(answer.reduction.irreducible) {
            EXPECT_EQ(answer.members.size(), 2u);
        }
    }
    auto capped = compute_linear_iis<linear_policy{
        .elasticity = elasticity_strategy::rebuild}>(
        system, factory, linear_options{.elastic = {.max_solves = 1}});
    EXPECT_TRUE(capped.reduction.irreducible);
    EXPECT_FALSE(capped.elasticity_seed_used);
    // Deliberately hide all violations behind a huge tolerance. The prefilter
    // must be abandoned; the ordinary formulation still proves infeasibility.
    auto stalled = compute_linear_iis<linear_policy{
        .elasticity = elasticity_strategy::rebuild}>(
        system, factory,
        linear_options{.elastic = {.violation_tolerance = 1e6}});
    EXPECT_TRUE(stalled.reduction.irreducible);
    EXPECT_FALSE(stalled.elasticity_seed_used);
}

TYPED_TEST(LinearIis, FallbackReusesFullProofWithExactBudget) {
    linear_system<> system;
    system.variables = {{std::nullopt, std::nullopt, false}};
    system.rows = {{{{0, 1.}}, 2., std::nullopt},
                   {{{0, 1.}}, std::nullopt, 1.},
                   {{{0, 1.}}, std::nullopt, 10.}};
    std::size_t constructions = 0;
    auto factory = [&] {
        ++constructions;
        return TypeParam{};
    };
    auto check = [&](auto order, std::size_t batch) {
        options opts;
        opts.initial_batch_size = batch;
        const auto baseline = compute_linear_iis(
            system, factory, linear_options{.limits = opts, .order = order});
        ASSERT_TRUE(baseline.reduction.irreducible);
        for(std::size_t elastic_cap : {0u, 1u}) {
            constructions = 0;
            opts.max_solves = baseline.reduction.solve_count + elastic_cap;
            const auto answer = compute_linear_iis<linear_policy{
                .elasticity = elasticity_strategy::rebuild}>(
                system, factory,
                linear_options{.limits = opts,
                               .elastic = {.max_solves = elastic_cap},
                               .order = order});
            EXPECT_FALSE(answer.elasticity_seed_used);
            EXPECT_EQ(answer.elasticity_calls, elastic_cap);
            EXPECT_TRUE(answer.reduction.irreducible);
            EXPECT_EQ(answer.reduction.reason, termination::completed);
            EXPECT_EQ(answer.members, baseline.members);
            EXPECT_EQ(answer.reduction.solve_count, opts.max_solves);
            EXPECT_EQ(constructions, answer.reduction.solve_count);
        }
    };
    for(std::size_t batch : {1u, 2u, 4u}) {
        check(input_order{}, batch);
        check([](member a, member b) { return a.index > b.index; }, batch);
    }
}

TYPED_TEST(LinearIis, ElasticityDoesNotSilentlyRelaxIntegers) {
    if constexpr(mippp::milp_model<TypeParam>) {
        linear_system<> system;
        system.variables = {{std::nullopt, std::nullopt, true}};
        system.rows = {{{{0, 1.}}, 0.5, 0.5}};
        auto factory = [] { return TypeParam{}; };
        auto answer = compute_linear_iis<linear_policy{
            .elasticity = elasticity_strategy::reuse}>(system, factory);
        EXPECT_TRUE(answer.reduction.irreducible);
        EXPECT_EQ(answer.elasticity_calls, 0u);
        EXPECT_EQ(answer.elasticity_reoptimizations, 0u);
        auto relaxed = compute_linear_iis<linear_policy{
            .analyzed_domain = domain::lp_relaxation,
            .elasticity = elasticity_strategy::rebuild}>(system, factory);
        EXPECT_EQ(relaxed.reduction.initial_status, feasibility::feasible);
        EXPECT_EQ(relaxed.elasticity_calls, 0u);
    }
}

TYPED_TEST(LinearIis, ElasticWarmStartPreservesIisAndReusesModel) {
    linear_system<> system;
    system.variables = {{std::nullopt, std::nullopt, false}};
    system.rows = {{{{0, 1.}}, 2., std::nullopt},
                   {{{0, 1.}}, std::nullopt, 1.}};
    for(unsigned i = 0; i < 12; ++i)
        system.rows.push_back({{{0, 1.}}, std::nullopt, 10. + i});
    std::size_t constructions = 0;
    auto factory = [&] {
        ++constructions;
        return TypeParam{};
    };
    const auto cold = compute_linear_iis<linear_policy{
        .elasticity = elasticity_strategy::rebuild}>(system, factory);
    EXPECT_EQ(constructions, cold.reduction.solve_count);
    EXPECT_EQ(cold.elasticity_reoptimizations, 0u);
    constructions = 0;
    const auto warm = compute_linear_iis<linear_policy{
        .elasticity = elasticity_strategy::reuse}>(system, factory);
    ASSERT_TRUE(cold.reduction.irreducible);
    ASSERT_TRUE(warm.reduction.irreducible);
    ASSERT_EQ(warm.members.size(), cold.members.size());
    for(auto candidate : cold.members)
        EXPECT_NE(
            std::find(warm.members.begin(), warm.members.end(), candidate),
            warm.members.end());
    EXPECT_TRUE(warm.elasticity_seed_used);
    EXPECT_EQ(constructions + warm.elasticity_reoptimizations,
              warm.reduction.solve_count);
    if constexpr(mippp::has_modifiable_variable_bounds<TypeParam>) {
        ASSERT_GT(warm.elasticity_calls, 1u);
        EXPECT_EQ(warm.elasticity_reoptimizations, warm.elasticity_calls - 1);
    } else {
        EXPECT_EQ(warm.elasticity_reoptimizations, 0u);
    }
}

TYPED_TEST(LinearIis, ElasticWarmStartRespectsBudgetsAndLifetime) {
    linear_system<> system;
    system.variables = {{2., 1., false}};
    for(std::size_t budget = 0; budget < 9; ++budget) {
        std::size_t constructions = 0;
        auto factory = [&] {
            ++constructions;
            return TypeParam{};
        };
        const auto answer = compute_linear_iis<linear_policy{
            .elasticity = elasticity_strategy::reuse}>(
            system, factory, linear_options{.limits = {.max_solves = budget}});
        EXPECT_LE(answer.reduction.solve_count, budget);
        EXPECT_EQ(constructions + answer.elasticity_reoptimizations,
                  answer.reduction.solve_count);
        if(budget == 0) {
            EXPECT_EQ(constructions, 0u);
        } else {
            EXPECT_TRUE(answer.reduction.proven_infeasible());
        }
    }
}

TYPED_TEST(LinearIis, WarmStateDoesNotLeakIntoFallback) {
    linear_system<> system;
    system.variables = {{std::nullopt, std::nullopt, false}};
    system.rows = {{{{0, 1.}}, 2., std::nullopt},
                   {{{0, 1.}}, std::nullopt, 1.}};
    auto factory = [] { return TypeParam{}; };
    const auto answer = compute_linear_iis<linear_policy{
        .elasticity = elasticity_strategy::reuse}>(
        system, factory, linear_options{.elastic = {.max_solves = 2}});
    EXPECT_TRUE(answer.reduction.irreducible);
    EXPECT_FALSE(answer.elasticity_seed_used);
    EXPECT_EQ(answer.members.size(), 2u);
    if constexpr(mippp::has_modifiable_variable_bounds<TypeParam>) {
        EXPECT_EQ(answer.elasticity_reoptimizations, 1u);
    }
}

TYPED_TEST(LinearIis, OrderingPreservesOriginalIdentitiesThroughElasticSeed) {
    linear_system<> system;
    system.variables = {{-100., 100., false}, {std::nullopt, 1., false}};
    system.rows = {{{{0, 1.}}, std::nullopt, 50.},
                   {{{1, 1.}}, 2., std::nullopt}};
    auto factory = [] { return TypeParam{}; };
    for_each_bool([&]<bool elastic>() {
        const auto answer = compute_linear_iis<linear_policy{
            .elasticity = (elastic ? elasticity_strategy::reuse
                                   : elasticity_strategy::off)}>(
            system, factory,
            linear_options{.limits = {.initial_batch_size = 2},
                           .order = rows_first_order{}});
        ASSERT_TRUE(answer.reduction.irreducible);
        ASSERT_EQ(answer.members.size(), 2u);
        EXPECT_NE(std::find(answer.members.begin(), answer.members.end(),
                            member{member_kind::row_lower, 1}),
                  answer.members.end());
        EXPECT_NE(std::find(answer.members.begin(), answer.members.end(),
                            member{member_kind::variable_upper, 1}),
                  answer.members.end());
    });
    const auto fallback = compute_linear_iis<linear_policy{
        .elasticity = elasticity_strategy::rebuild}>(
        system, factory,
        linear_options{.elastic = {.max_solves = 1},
                       .order = bounds_first_order{}});
    EXPECT_TRUE(fallback.reduction.irreducible);
    EXPECT_EQ(fallback.members.size(), 2u);
}
