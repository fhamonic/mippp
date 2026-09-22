#pragma once

#include <cmath>
#include <exception>
#include <functional>
#include <optional>
#include <span>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

#include "mippp/utility/linear_iis_types.hpp"
#include "mippp/utility/cold_model_workspace.hpp"
#include "mippp/utility/native_seed.hpp"
#include "mippp/utility/deletion_workspace.hpp"
#include "mippp/utility/elastic_lp.hpp"
#include "mippp/utility/iis_report.hpp"
#include "mippp/utility/solver_exceptions.hpp"

namespace mippp::iis {

// Policy selects the algorithm and workspaces at compile time; config supplies
// per-extraction limits, tolerances and ordering. Missing optional solver
// capabilities select the corresponding fallback.
template <linear_policy Policy = {}, typename Factory, std::floating_point Scalar,
          linear_candidate_order Order = input_order>
    requires valid_linear_policy<Policy> && std::invocable<Factory &> &&
             lp_model<std::invoke_result_t<Factory &>> &&
             std::same_as<Scalar, model_scalar_t<std::invoke_result_t<Factory &>>>
[[nodiscard]] linear_result compute_linear_iis(
    const linear_system<Scalar> & system, Factory && factory, linear_options<Order> config = {}) {
    detail::budget_clock::time_point started{};
    if constexpr(Policy.measure_time) started = detail::budget_clock::now();
    detail::phase_budget budget(config.limits);
    const auto & opts = budget.limits();
    using Model = std::invoke_result_t<Factory &>;
    constexpr auto mode = Policy.analyzed_domain;
    constexpr bool native_capable = Policy.native_seed &&
        (has_infeasibility_ray<Model> || has_infeasibility_certificate<Model>);
    constexpr bool reuse_deletion = Policy.deletion == deletion_strategy::reuse &&
                                    detail::has_deletion_updates<Model>;
    constexpr bool needs_rebuilt_model = !reuse_deletion || native_capable;
    constexpr bool use_elastic = Policy.elasticity != elasticity_strategy::off;
    constexpr bool weighted = native_capable && Policy.order_by_weight;
    if constexpr(Policy.native_seed) {
        if(!std::isfinite(config.native.relative_tolerance) || config.native.relative_tolerance < 0)
            throw std::invalid_argument("The solver-hint threshold is invalid. " +
                detail::setting_help("native.relative_tolerance",
                    detail::setting_value(config.native.relative_tolerance), "any finite number >= 0 (default 1e-9)") +
                " Smaller values retain more evidence; values >= 1 can discard the entire hint.");
    }
    if constexpr(use_elastic) {
        if(!std::isfinite(config.elastic.violation_tolerance) || config.elastic.violation_tolerance < 0)
            throw std::invalid_argument("The soft-constraint threshold is invalid. " +
                detail::setting_help("elastic.violation_tolerance",
                    detail::setting_value(config.elastic.violation_tolerance), "any finite number >= 0 (default 1e-7)"));
    }
    const detail::prepared_linear_system<Scalar> prepared(system, mode, milp_model<Model>);
    const auto & candidates = prepared.candidates;
    const bool original_mip = prepared.has_integers && mode == domain::original;
    const bool use_native = native_capable && !original_mip;
    linear_statistics statistics;
    statistics.input_variables = system.variables.size();
    statistics.input_rows = system.rows.size();
    statistics.input_candidates = candidates.size();
    linear_diagnostics diagnostics;
    diagnostics.policy = Policy;
    diagnostics.elastic = config.elastic;
    diagnostics.native = config.native;
    diagnostics.max_solves = config.limits.max_solves;
    diagnostics.time_limit_seconds = config.limits.time_limit.count();
    diagnostics.deadline = config.limits.deadline;
    diagnostics.deletion_reuse_supported = detail::has_deletion_updates<Model>;
    diagnostics.elastic_reuse_supported = has_modifiable_variable_bounds<Model>;
    diagnostics.solver_time_limit_supported = has_time_limit<Model> &&
        requires(Model & model, std::chrono::duration<double> t) { model.set_time_limit(t); };
    if constexpr(Policy.native_seed) diagnostics.native_seed = original_mip ? seed_outcome::integer_model :
        (native_capable ? seed_outcome::not_reached : seed_outcome::unsupported);
    if constexpr(use_elastic) diagnostics.elastic_seed = original_mip ? seed_outcome::integer_model : seed_outcome::not_reached;
    bool capture_certificate = use_native;
    bool native_seed_used = false;
    // Disabled features need no buffers or solver storage. Their model methods
    // are not instantiated, so a backend need only support enabled features.
    std::conditional_t<native_capable, detail::native_seed_data<Policy.order_by_weight>,
                       detail::disabled_feature> native_data;
    std::conditional_t<reuse_deletion, std::optional<detail::deletion_workspace<Model, Scalar>>,
                       detail::disabled_feature> deletion_model;
    auto rebuilt_model = [&] {
        if constexpr(needs_rebuilt_model)
            return detail::cold_model_workspace<Factory, Scalar, mode>(prepared, factory, opts, statistics.rebuild);
        else return detail::disabled_feature{};
    }();
    auto check_model = [&](std::span<const std::size_t> active) {
        if constexpr(reuse_deletion) {
            // Read certificates from the original model, without added slack
            // variables. Later checks may use the retained feasibility model.
            if(!capture_certificate) {
                if(!deletion_model) {
                    std::vector<bool> integer_variables;
                    integer_variables.reserve(system.variables.size());
                    for(const auto & v : system.variables)
                        integer_variables.push_back(v.integer && mode == domain::original);
                    deletion_model.emplace(integer_variables, prepared.inequalities(), factory, opts, statistics.deletion);
                }
                return (*deletion_model)(active);
            }
        }
        if constexpr(needs_rebuilt_model) {
            return rebuilt_model(active, [&](auto & model, auto ids) {
                if constexpr(native_capable) {
                    if(capture_certificate) {
                        ++statistics.certificate_queries;
                        native_data = detail::collect_native_seed<Policy.prune_bounds, Policy.order_by_weight>(
                            model, ids, prepared, config.native);
                        diagnostics.native_seed = native_data.outcome;
                    }
                }
            });
        } else {
            // No native provider can set capture_certificate in this specialization.
            // The retained path above always returns.
            throw std::logic_error("Internal conflict-search error: a disabled model-building path was requested. "
                                   "Please report this as a library bug; changing solver settings is not a fix.");
        }
    };
    auto oracle = [&](std::span<const std::size_t> active) {
        const auto state = check_model(active);
        statistics.feasibility_checks.record(state);
        return state;
    };
    std::size_t elasticity_calls = 0;
    bool elasticity_seed_used = false;
    std::size_t elasticity_reoptimizations = 0;
    auto has_weights = [&] {
        if constexpr(weighted) return !native_data.weights.empty();
        else return false;
    };
    auto full_order = [&](std::size_t a, std::size_t b) {
        if constexpr(!std::same_as<Order, input_order>) {
            if(std::invoke(config.order, candidates[a], candidates[b])) return true;
            if(std::invoke(config.order, candidates[b], candidates[a])) return false;
        }
        if constexpr(weighted) {
            if(has_weights() && native_data.weights[a] != native_data.weights[b])
                return native_data.weights[a] < native_data.weights[b];
        }
        return a < b;
    };
    auto full_reduce = [&](options limits) {
        if constexpr(std::same_as<Order, input_order>) {
            if(!weighted || !use_native)
                return deletion_filter(candidates.size(), oracle, limits);
        }
        return deletion_filter(candidates.size(), oracle, limits, full_order);
    };
    auto reduce = [&]() -> result {
        if constexpr(!use_elastic && !native_capable) {
            return full_reduce(opts);
        } else {
            // LP seeds cannot explain integer-only infeasibility.
            if(original_mip) return full_reduce(opts);
            auto known = full_reduce(budget.remaining(1));
            capture_certificate = false;
            if(!known.proven_infeasible() || known.irreducible) return known;
            budget.consume(known.solve_count);
            auto try_seed = [&](const std::vector<std::size_t> & seed, seed_outcome & outcome,
                                std::size_t & verification_calls) -> std::optional<result> {
                std::vector<std::size_t> original;
                bool first_check = true;
                auto seeded_oracle = [&](std::span<const std::size_t> active) {
                    if(first_check) { ++verification_calls; first_check = false; }
                    original.clear();
                    original.reserve(active.size());
                    for(auto id : active) original.push_back(seed[id]);
                    return oracle(original);
                };
                // No policy bypasses initial verification of a proposed seed.
                auto filtered = [&] {
                    if constexpr(std::same_as<Order, input_order>) {
                        if(!has_weights())
                            return deletion_filter(seed.size(), seeded_oracle, budget.remaining());
                    }
                    return deletion_filter(seed.size(), seeded_oracle, budget.remaining(),
                        [&](auto a, auto b) { return full_order(seed[a], seed[b]); });
                }();
                budget.consume(filtered.solve_count);
                outcome = first_check ? seed_outcome::verification_skipped :
                    (filtered.initial_status == feasibility::feasible ? seed_outcome::verification_feasible :
                     filtered.initial_status == feasibility::unknown ? seed_outcome::verification_unknown : seed_outcome::used);
                if(!filtered.proven_infeasible()) return std::nullopt;
                for(auto & id : filtered.members) id = seed[id];
                filtered.solve_count = budget.used();
                return filtered;
            };
            if constexpr(native_capable) {
                if(native_data.members && native_data.members->size() < candidates.size()) {
                    if(auto filtered = try_seed(*native_data.members, diagnostics.native_seed,
                                                statistics.native_verification_calls)) {
                        native_seed_used = true;
                        return std::move(*filtered);
                    }
                }
            }
            if constexpr(use_elastic) {
                const auto inequalities = prepared.inequalities();
                const auto filter_opts = budget.remaining(config.elastic.max_solves);
                auto seed = [&] {
                    if constexpr(Policy.elasticity == elasticity_strategy::reuse &&
                                 has_modifiable_variable_bounds<Model>) {
                        // Lazy construction: a stopped filter never loads a solver.
                        std::optional<detail::elastic_lp_workspace<Model, Scalar>> workspace;
                        auto retained_oracle = [&](std::span<const std::size_t> hard) {
                            if(!workspace)
                                workspace.emplace(system.variables.size(), inequalities, factory,
                                    static_cast<Scalar>(config.elastic.violation_tolerance), opts, statistics.elastic);
                            return (*workspace)(hard);
                        };
                        auto answer = elasticity_filter(candidates.size(), retained_oracle, filter_opts);
                        if(workspace) elasticity_reoptimizations = workspace->reoptimizations();
                        return answer;
                    } else {
                        auto elastic_oracle = [&](std::span<const std::size_t> hard) {
                            return detail::solve_elastic_lp(system.variables.size(), inequalities,
                                hard, factory, static_cast<Scalar>(config.elastic.violation_tolerance), opts, statistics.elastic);
                        };
                        return elasticity_filter(candidates.size(), elastic_oracle, filter_opts);
                    }
                }();
                elasticity_calls = seed.solve_count;
                statistics.elastic_checks = seed.outcomes;
                statistics.elastic_seed_size = seed.members.size();
                diagnostics.elastic_seed = seed.no_progress ? seed_outcome::no_progress :
                    (seed.reason == termination::indeterminate ? seed_outcome::inconclusive : seed_outcome::limit_reached);
                budget.consume(seed.solve_count);
                if(seed.proven_infeasible) {
                    if(auto filtered = try_seed(seed.members, diagnostics.elastic_seed,
                                                statistics.elastic_verification_calls)) {
                        elasticity_seed_used = true;
                        return std::move(*filtered);
                    }
                }
            }
            // The full set is still known to conflict. Restore its active bounds
            // in a retained model and continue without checking it again.
            known.solve_count = budget.used();
            diagnostics.full_set_fallback = true;
            if constexpr(std::same_as<Order, input_order>) {
                if(!has_weights())
                    return detail::deletion_filter_impl(std::move(known), oracle, opts, input_order{});
            }
            return detail::deletion_filter_impl(std::move(known), oracle, opts, full_order);
        }
    };
    auto reduced = [&] {
        try { return reduce(); }
        catch(const license_error & error) {
            const auto message = detail::solver_failure_context(true, diagnostics, statistics) +
                                 "\nSolver details: " + error.what();
            std::throw_with_nested(license_error(message.c_str()));
        } catch(const solver_error & error) {
            const auto message = detail::solver_failure_context(false, diagnostics, statistics) +
                                 "\nSolver details: " + error.what();
            std::throw_with_nested(solver_error(message.c_str()));
        }
    }();
    std::vector<member> members;
    if(reduced.proven_infeasible()) {
        members.reserve(reduced.members.size());
        for(auto id : reduced.members) members.push_back(candidates[id]);
    }
    const auto native_seed_size = [&]() -> std::size_t {
        if constexpr(native_capable) return native_data.members ? native_data.members->size() : 0;
        else return 0;
    }();
    const auto deletion_model_reused = [&] {
        if constexpr(reuse_deletion) return deletion_model.has_value();
        else return false;
    }();
    const auto deletion_reoptimizations = [&]() -> std::size_t {
        if constexpr(reuse_deletion) return deletion_model ? deletion_model->reoptimizations() : 0;
        else return 0;
    }();
    diagnostics.cancellation_requested = opts.stop.stop_requested();
    if constexpr(reuse_deletion) deletion_model.reset(); // include solver cleanup in optional elapsed time
    linear_result answer{
        .reduction = std::move(reduced),
        .members = std::move(members),
        .analyzed_domain = mode,
        .elasticity_calls = elasticity_calls,
        .elasticity_seed_used = elasticity_seed_used,
        .elasticity_reoptimizations = elasticity_reoptimizations,
        .native_seed_used = native_seed_used,
        .native_seed_size = native_seed_size,
        .deletion_model_reused = deletion_model_reused,
        .deletion_reoptimizations = deletion_reoptimizations,
        .statistics = statistics,
        .diagnostics = diagnostics
    };
    if constexpr(Policy.measure_time) answer.statistics.elapsed = detail::budget_clock::now() - started;
    return answer;
}

} // namespace mippp::iis
