#pragma once
#include <limits>
#include <sstream>
#include <string>
#include <string_view>
#include "mippp/utility/linear_iis_types.hpp"

namespace mippp::iis {

namespace detail {
inline std::string solver_failure_context(bool license,
                                          const linear_diagnostics & d,
                                          const linear_statistics & s) {
    std::string message =
        license ? "The solver could not use its license for this problem. "
                  "Check that the license is available, "
                  "valid for this machine, and permits this problem size. "
                  "Library discovery alone does not check licensing."
                : "The solver could not complete the conflict search. Review "
                  "the problem data and the solver details below.";
    message += "\nInput: " + setting_value(s.input_variables) + " variables, " +
               setting_value(s.input_rows) + " rows, " +
               setting_value(s.input_candidates) +
               " separate bound/constraint sides.";
    message += "\n" + setting_help("policy.deletion",
                                   d.policy.deletion == deletion_strategy::reuse
                                       ? "reuse"
                                       : "rebuild",
                                   "rebuild, reuse (compile-time choice)");
    const auto elastic = d.policy.elasticity == elasticity_strategy::off ? "off"
                         : d.policy.elasticity == elasticity_strategy::reuse
                             ? "reuse"
                             : "rebuild";
    message += "\n" + setting_help("policy.elasticity", elastic,
                                   "off, rebuild, reuse (compile-time choice)");
    message +=
        "\nRetained deletion and soft-constraint checks add variables and "
        "constraints. "
        "If the license limits problem size, try deletion=rebuild and "
        "elasticity=off, or use a license/solver "
        "that permits the expanded problem. Rebuilding still splits two-sided "
        "rows, so it cannot guarantee fitting a license limit.";
    return message;
}
}  // namespace detail

inline std::string_view seed_description(seed_outcome outcome) noexcept {
    switch(outcome) {
        case seed_outcome::disabled:
            return "disabled";
        case seed_outcome::not_reached:
            return "not attempted; the search did not reach this step";
        case seed_outcome::unsupported:
            return "not available from this solver model; ordinary checks were "
                   "used";
        case seed_outcome::integer_model:
            return "skipped because fractional-value checks cannot explain "
                   "integer-only conflicts";
        case seed_outcome::unavailable:
            return "the solver did not provide a conflict hint";
        case seed_outcome::invalid_certificate:
            return "the solver hint could not be interpreted safely and was "
                   "ignored";
        case seed_outcome::not_smaller:
            return "the hint did not reduce the candidate set";
        case seed_outcome::no_progress:
            return "no new violated constraints were found; ordinary checks "
                   "were used";
        case seed_outcome::inconclusive:
            return "the soft-constraint pass did not reach a conclusive result";
        case seed_outcome::limit_reached:
            return "stopped at a limit or cancellation before producing a "
                   "verified conflict";
        case seed_outcome::verification_skipped:
            return "a proposed conflict could not be checked before a limit or "
                   "cancellation stopped the search";
        case seed_outcome::verification_feasible:
            return "the proposed conflict could be satisfied, so it was "
                   "rejected";
        case seed_outcome::verification_unknown:
            return "the proposed conflict could not be verified, so it was "
                   "rejected";
        case seed_outcome::used:
            return "a proposed conflict was checked and used for further "
                   "reduction";
    }
    return "unrecognized outcome";
}

inline std::string_view issue_description(solve_issue issue) noexcept {
    switch(issue) {
        case solve_issue::none:
            return "none recorded";
        case solve_issue::time_limit:
            return "the solver reached its own time limit";
        case solve_issue::iteration_limit:
            return "the solver reached its calculation-step limit";
        case solve_issue::node_limit:
            return "the solver reached its search-step limit";
        case solve_issue::solution_limit:
            return "the solver reached its solution-count limit";
        case solve_issue::memory_limit:
            return "the solver ran out of available memory or reached its "
                   "memory limit";
        case solve_issue::other_limit:
            return "the solver reached a limit that it did not identify more "
                   "precisely";
        case solve_issue::interrupted:
            return "the solver was interrupted";
        case solve_issue::numerical:
            return "the solver reported unreliable numerical results";
        case solve_issue::ambiguous:
            return "the solver did not distinguish a conflict from an "
                   "unbounded problem";
        case solve_issue::no_solution:
            return "the solver returned neither a usable solution nor a "
                   "conflict proof";
        case solve_issue::failed:
            return "the solver could not complete the calculation";
    }
    return "unrecognized issue";
}

// Formatting is explicit: no strings, environment reads, logging, or extra
// solver queries in the extraction's successful path. Settings are snapshots
// from this run, not today's values of a caller's modified options object.
struct report_options {
    bool include_members =
        false;  // large conflicts can contain many thousands of sides
};
[[nodiscard]] inline std::string describe_result(const linear_result & answer,
                                                 report_options report = {}) {
    using detail::setting_help;
    using detail::setting_value;
    const auto & r = answer.reduction;
    const auto & d = answer.diagnostics;
    const auto & s = answer.statistics;
    std::ostringstream out;
    if(r.initial_status == feasibility::feasible)
        out << "The supplied problem can be satisfied; no conflict was found.";
    else if(!r.proven_infeasible())
        out << "The search could not determine whether the problem can be "
               "satisfied. No verified conflict is available.";
    else if(r.irreducible) {
        if(answer.members.empty())
            out << "The fixed requirements conflict even without any candidate "
                   "constraints.";
        else
            out << "Found a verified conflict with " << answer.members.size()
                << " bound/constraint sides. Each remaining side is necessary, "
                   "given the fixed requirements.";
        out << " This need not be the smallest possible conflict.";
    } else
        out << "Found a verified conflict with " << answer.members.size()
            << " bound/constraint sides, but not all remaining sides have been "
               "shown to be necessary.";
    if(answer.analyzed_domain == domain::lp_relaxation)
        out << "\nThis result allows fractional variable values; it does not "
               "establish feasibility of the original integer problem.";
    if(report.include_members && r.proven_infeasible()) {
        for(const auto & m : answer.members) {
            const bool row = m.kind == member_kind::row_lower ||
                             m.kind == member_kind::row_upper;
            const bool lower = m.kind == member_kind::row_lower ||
                               m.kind == member_kind::variable_lower;
            out << "\n  " << (row ? "Row " : "Variable ") << m.index << ' '
                << (lower ? "lower" : "upper")
                << " bound (original zero-based index)";
        }
    }

    if(r.reason == termination::solve_limit) {
        out << "\nThe check budget was exhausted. To continue farther, "
               "increase "
            << setting_help(
                   "limits.max_solves", setting_value(d.max_solves),
                   "0 through " +
                       setting_value(std::numeric_limits<std::size_t>::max()) +
                       " (0 prevents checks; the maximum means no practical "
                       "check limit)");
    } else if(r.reason == termination::time_limit) {
        out << "\nThe overall time budget was exhausted. Increase the duration "
               "or choose a later deadline.\n"
            << setting_help("limits.time_limit (seconds)",
                            setting_value(d.time_limit_seconds),
                            "0, any positive duration, or infinity")
            << '\n'
            << setting_help(
                   "limits.deadline",
                   d.deadline == std::chrono::steady_clock::time_point::max()
                       ? "not set"
                       : setting_value(d.deadline.time_since_epoch().count()) +
                             " steady-clock ticks",
                   "a later std::chrono::steady_clock time point, or "
                   "time_point::max() for no absolute deadline");
        if(!d.solver_time_limit_supported)
            out << "\nThis solver model cannot receive the remaining time "
                   "allowance; stopping is checked between solver runs.";
    } else if(r.reason == termination::cancelled) {
        out << "\nCancellation was requested. "
            << setting_help("limits.stop",
                            d.cancellation_requested
                                ? "requested"
                                : "not requested at return",
                            "a default token (no cancellation), or a token "
                            "from a fresh std::stop_source")
            << " A requested token cannot be reset.";
    } else if(r.reason == termination::indeterminate) {
        out << "\nAt least one required check was inconclusive; an "
               "inconclusive check is never treated as a conflict proof.";
    }

    const auto & checks = s.feasibility_checks;
    out << "\nInput: " << s.input_variables << " variables, " << s.input_rows
        << " rows, " << s.input_candidates
        << " separate bound/constraint sides.";
    out << "\nChecks: " << r.solve_count << " total ("
        << answer.elasticity_calls << " soft-constraint checks). "
        << "Ordinary outcomes: " << checks.feasible << " satisfiable, "
        << checks.infeasible << " conflicting, " << checks.unknown
        << " inconclusive."
        << "\nSolver runs: " << s.solver_runs()
        << "; models built: " << s.models_built()
        << "; conflicts proved from bounds alone: "
        << s.rebuild.bound_only_proofs << '.';
    if(s.elapsed) out << " Elapsed: " << s.elapsed->count() << " seconds.";
    const auto & reduction = r.statistics;
    out << "\nFinal reduction pass: " << reduction.batch_checks
        << " group checks, " << reduction.singleton_checks
        << " individual checks; removed " << reduction.removed_by_batches
        << " sides in groups and " << reduction.removed_by_singletons
        << " individually.";
    if(r.proven_infeasible())
        out << " Remaining sides: " << reduction.necessary_members
            << " proven necessary, " << reduction.unresolved_members
            << " inconclusive, "
            << (r.members.size() - reduction.necessary_members -
                reduction.unresolved_members)
            << " not yet tested.";
    auto work = [&](std::string_view name, const work_statistics & w) {
        if(!w.models_built && w.last_solver_issue == solve_issue::none) return;
        out << "\n"
            << name << ": " << w.models_built << " models, " << w.solver_runs
            << " solver runs, " << w.bound_updates << " bound updates, "
            << w.solves_skipped
            << " runs skipped after stopping; largest model "
            << w.peak_variables << " variables / " << w.peak_constraints
            << " constraints.";
        if(w.last_solver_issue != solve_issue::none)
            out << " Last recorded issue during this stage: "
                << issue_description(w.last_solver_issue) << '.';
        if(w.last_solver_issue == solve_issue::time_limit) {
            out << "\nThe factory may set a tighter limit than the overall "
                   "search. "
                << setting_help(
                       "model.set_time_limit (seconds; last value read before "
                       "forwarding the remaining budget)",
                       w.observed_solver_time_limit
                           ? setting_value(*w.observed_solver_time_limit)
                           : "not observed",
                       "a nonnegative duration supported by the selected "
                       "solver model")
                << " An unobserved value is not assumed to be unlimited.";
        }
        if(w.last_solver_issue == solve_issue::numerical)
            out << " Check the units and sizes of coefficients and bounds; do "
                   "not assume that loosening tolerances proves a conflict.";
    };
    work("Rebuilt checks", s.rebuild);
    work("Retained-model checks", s.deletion);
    work("Soft-constraint checks", s.elastic);
    if(d.policy.native_seed) {
        out << "\nSolver-provided hint: " << seed_description(d.native_seed)
            << ". Proposed sides: " << answer.native_seed_size
            << "; verification checks: " << s.native_verification_calls << '.';
        if(d.native_seed == seed_outcome::invalid_certificate ||
           d.native_seed == seed_outcome::verification_feasible ||
           d.native_seed == seed_outcome::verification_unknown ||
           d.native_seed == seed_outcome::not_smaller) {
            out << '\n'
                << setting_help("native.relative_tolerance",
                                setting_value(d.native.relative_tolerance),
                                "any finite number >= 0 (default 1e-9; smaller "
                                "values retain more evidence)")
                << '\n'
                << setting_help("policy.native_seed", "true",
                                "false, true (compile-time choice)")
                << " Hints are optional and are checked before use.";
        }
    }
    if(d.policy.elasticity != elasticity_strategy::off) {
        out << "\nSoft-constraint hint: " << seed_description(d.elastic_seed)
            << ". Accumulated sides: " << s.elastic_seed_size
            << "; verification checks: " << s.elastic_verification_calls << '.';
        if(d.elastic_seed == seed_outcome::no_progress ||
           d.elastic_seed == seed_outcome::limit_reached ||
           d.elastic_seed == seed_outcome::inconclusive) {
            out << '\n'
                << setting_help(
                       "elastic.max_solves",
                       setting_value(d.elastic.max_solves),
                       "0 through " +
                           setting_value(
                               std::numeric_limits<std::size_t>::max()))
                << '\n'
                << setting_help("elastic.violation_tolerance",
                                setting_value(d.elastic.violation_tolerance),
                                "any finite number >= 0 (default 1e-7)")
                << " The overall limits still apply. A high threshold may hide "
                   "small violations.";
        }
    }
    if(d.full_set_fallback)
        out << "\nThe full problem's earlier conflict proof was kept; no "
               "repeated full-problem check was needed.";
    if(d.policy.deletion == deletion_strategy::reuse &&
       !d.deletion_reuse_supported)
        out << "\nThis solver model cannot update/read the required bounds; "
               "checks rebuilt their models. "
            << setting_help("policy.deletion", "reuse",
                            "rebuild, reuse (compile-time choice)");
    if(d.policy.elasticity == elasticity_strategy::reuse &&
       !d.elastic_reuse_supported)
        out << "\nThis solver model cannot update bounds; soft-constraint "
               "checks rebuilt their models. "
            << setting_help("policy.elasticity", "reuse",
                            "off, rebuild, reuse (compile-time choice)");
    return out.str();
}
}  // namespace mippp::iis
