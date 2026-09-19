// Local comparison of fallback algorithms on one open-source solver. No
// commercial backend is loaded, no network request is made, and results are
// never uploaded. Timings include model construction/updates; validation does not.
#include <algorithm>
#include <charconv>
#include <chrono>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "mippp/solvers/highs/all.hpp"
#include "mippp/solvers/clp/all.hpp"
#include "mippp/utility/linear_iis.hpp"

using namespace mippp::iis;

struct problem {
    std::string name;
    linear_system<> system;
    bool feasible = false;
};

std::vector<problem> problems(std::size_t size) {
    std::vector<problem> cases;
    problem sparse{"sparse_rows", {}, false};
    sparse.system.variables = {{std::nullopt, std::nullopt, false}};
    sparse.system.rows = {{{{0, 1.}}, 2., std::nullopt},
                          {{{0, 1.}}, std::nullopt, 1.}};
    for(std::size_t i = 0; i < size; ++i)
        sparse.system.rows.push_back({{{0, 1.}}, std::nullopt, 10. + static_cast<double>(i)});
    cases.push_back(sparse);
    auto feasible = sparse;
    feasible.name = "feasible";
    feasible.feasible = true;
    feasible.system.rows[0].lower = 0.;
    cases.push_back(std::move(feasible));

    auto scaled = sparse;
    scaled.name = "scaled_rows";
    // Same mathematics with different row scales: unit elastic penalties are
    // scale-sensitive, so this case must not be omitted from comparisons.
    for(std::size_t i = 0; i < scaled.system.rows.size(); ++i) {
        const double factor = i % 2 ? 1e3 : 1e-3;
        auto & row = scaled.system.rows[i];
        for(auto & term : row.terms) term.second *= factor;
        if(row.lower) *row.lower *= factor;
        if(row.upper) *row.upper *= factor;
    }
    cases.push_back(std::move(scaled));

    auto disjoint = sparse;
    disjoint.name = "disjoint_conflicts";
    disjoint.system.variables.push_back({std::nullopt, std::nullopt, false});
    disjoint.system.rows.push_back({{{1, 1.}}, 3., std::nullopt});
    disjoint.system.rows.push_back({{{1, 1.}}, std::nullopt, 0.});
    cases.push_back(std::move(disjoint));

    problem bounds{"redundant_bounds", {}, false};
    bounds.system.variables.resize(size + 1, {-100., 100., false});
    bounds.system.variables[0].upper = 1.;
    bounds.system.rows = {{{{0, 1.}}, 2., std::nullopt}};
    cases.push_back(std::move(bounds));

    problem chain{"all_essential_chain", {}, false};
    chain.system.variables.resize(size, {std::nullopt, std::nullopt, false});
    chain.system.rows.push_back({{{0, 1.}}, 1., std::nullopt});
    for(std::size_t i = 1; i < size; ++i)
        chain.system.rows.push_back({{{i, 1.}, {i - 1, -1.}}, 0., std::nullopt});
    chain.system.rows.push_back({{{size - 1, 1.}}, std::nullopt, 0.});
    cases.push_back(std::move(chain));
    return cases;
}

// Verify the defining IIS property in fresh ORIGINAL models, outside timing.
// This deliberately does not compare membership lists: different IISs are valid.
template <typename Factory>
bool verify(const problem & input, const linear_result & answer, Factory & factory) {
    if(input.feasible)
        return answer.reduction.initial_status == feasibility::feasible &&
               answer.members.empty() && !answer.reduction.irreducible;
    if(!answer.reduction.proven_infeasible() || !answer.reduction.irreducible)
        return false;
    auto subset = input.system;
    for(auto & v : subset.variables) { v.lower.reset(); v.upper.reset(); }
    for(auto & row : subset.rows) { row.lower.reset(); row.upper.reset(); }
    auto bound = [](auto & system, member m) -> decltype(auto) {
        switch(m.kind) {
            case member_kind::variable_lower: return (system.variables[m.index].lower);
            case member_kind::variable_upper: return (system.variables[m.index].upper);
            case member_kind::row_lower: return (system.rows[m.index].lower);
            case member_kind::row_upper: return (system.rows[m.index].upper);
        }
        throw std::logic_error("Invalid IIS member kind");
    };
    for(auto m : answer.members) bound(subset, m) = bound(input.system, m);
    auto status_of = [&](const auto & system) {
        return compute_linear_iis(system, factory, {.limits = {.max_solves = 1}}).reduction.initial_status;
    };
    if(status_of(subset) != feasibility::infeasible) return false;
    for(auto m : answer.members) {
        auto & side = bound(subset, m);
        const auto saved = side;
        side.reset();
        const bool feasible = status_of(subset) == feasibility::feasible;
        side = saved;
        if(!feasible) return false;
    }
    return true;
}

std::size_t parse_count(std::string_view text, std::string_view name, std::size_t maximum) {
    std::size_t value = 0;
    const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
    if(error != std::errc{} || end != text.data() + text.size() || value == 0 || value > maximum)
        throw std::invalid_argument(std::string(name) + ": current='" + std::string(text) +
            "'; available=a whole number from 1 through " + std::to_string(maximum) + ".");
    return value;
}

int main(int argc, char ** argv) {
    try {
        if(argc > 5 || (argc > 1 && std::string(argv[1]) == "--help")) {
            std::cout << "Usage: mippp_iis_benchmark [size=128] [repetitions=3] "
                         "[output=iis-benchmark.local.csv] [solver=highs|clp]\n"
                         "Open-source solvers only. Output must not already exist.\n";
            return argc > 5 ? 1 : 0;
        }
        const auto size = argc > 1 ? parse_count(argv[1], "size", 100000) : 128;
        const auto repetitions = argc > 2 ? parse_count(argv[2], "repetitions", 1000) : 3;
        const std::filesystem::path output = argc > 3 ? argv[3] : "iis-benchmark.local.csv";
        const std::string solver_name = argc > 4 ? argv[4] : "highs";
        if(solver_name != "highs" && solver_name != "clp")
            throw std::invalid_argument("solver must be highs or clp");
        if(std::filesystem::exists(output))
            throw std::runtime_error("Refusing to overwrite existing benchmark output");
        // Exclusive creation also protects a file created after the check above.
        std::ofstream csv(output, std::ios::out | std::ios::noreplace);
        if(!csv) throw std::runtime_error("Cannot open benchmark output");
        csv.exceptions(std::ios::failbit | std::ios::badbit);

        auto benchmark = [&](auto & factory, const char * solver, const char * version) {
            csv << "case,size,strategy,iteration,milliseconds,total_calls,elastic_calls,"
                   "members,irreducible,verified,solver,version,elastic_reoptimizations,"
                   "deletion_model_reused,deletion_reoptimizations,native_seed_used,native_seed_size,"
                   "models_built,solver_runs,feasible_checks,infeasible_checks,unknown_checks,"
                   "final_batch_checks,final_singleton_checks,final_batch_removed,final_singleton_removed,"
                   "necessary_members,unresolved_members,native_verification_calls,elastic_verification_calls,"
                   "bound_only_proofs,solves_skipped,bound_updates,peak_variables,peak_constraints\n";
            const char * names[] = {"single", "batched", "elastic", "elastic_batched",
                                   "elastic_warm", "elastic_batched_warm",
                                   "batched_rows_first", "batched_bounds_first",
                                   "deletion_reuse", "deletion_batched_reuse", "all_reuse",
                                   "native", "native_bounds", "native_weights", "native_bounds_weights",
                                   "native_bounds_weights_reuse"};
            const unsigned strategy_count = solver_name == "clp" ? 16 : 11;
            for(const auto & input : problems(size)) {
                // One untimed warm-up for each strategy. Rotate measured execution
                // order across repetitions to reduce systematic ordering effects.
                auto run = [&]<unsigned strategy>() {
                    constexpr linear_policy policy = [] {
                        linear_policy p;
                        if constexpr(strategy >= 11) {
                            p.native_seed = true;
                            p.prune_bounds = strategy == 12 || strategy >= 14;
                            p.order_by_weight = strategy >= 13;
                            p.deletion = strategy == 15 ? deletion_strategy::reuse : deletion_strategy::rebuild;
                        } else if constexpr(strategy >= 8) {
                            p.deletion = deletion_strategy::reuse;
                            if constexpr(strategy == 10) p.elasticity = elasticity_strategy::reuse;
                        } else if constexpr(strategy >= 2 && strategy < 6) {
                            p.elasticity = strategy >= 4 ? elasticity_strategy::reuse : elasticity_strategy::rebuild;
                        }
                        return p;
                    }();
                    options limits;
                    if constexpr(strategy >= 11) limits.initial_batch_size = 1;
                    else if constexpr(strategy >= 8) limits.initial_batch_size = strategy == 8 ? 1 : 64;
                    else if constexpr(strategy >= 6) limits.initial_batch_size = 64;
                    else limits.initial_batch_size = strategy % 2 ? 64 : 1;
                    if constexpr(strategy == 6)
                        return compute_linear_iis<policy>(input.system, factory,
                            linear_options{.limits = limits, .order = rows_first_order{}});
                    else if constexpr(strategy == 7)
                        return compute_linear_iis<policy>(input.system, factory,
                            linear_options{.limits = limits, .order = bounds_first_order{}});
                    else return compute_linear_iis<policy>(input.system, factory, {.limits = limits});
                };
                // Runtime benchmark selection dispatches ONCE at the boundary.
                // The library itself sees one fixed structural policy per call.
                auto dispatch = [&](unsigned strategy) -> linear_result {
                    switch(strategy) {
                        case 0: return run.template operator()<0>();
                        case 1: return run.template operator()<1>();
                        case 2: return run.template operator()<2>();
                        case 3: return run.template operator()<3>();
                        case 4: return run.template operator()<4>();
                        case 5: return run.template operator()<5>();
                        case 6: return run.template operator()<6>();
                        case 7: return run.template operator()<7>();
                        case 8: return run.template operator()<8>();
                        case 9: return run.template operator()<9>();
                        case 10: return run.template operator()<10>();
                        case 11: return run.template operator()<11>();
                        case 12: return run.template operator()<12>();
                        case 13: return run.template operator()<13>();
                        case 14: return run.template operator()<14>();
                        case 15: return run.template operator()<15>();
                        default: throw std::logic_error("Invalid benchmark strategy");
                    }
                };
                for(unsigned strategy = 0; strategy < strategy_count; ++strategy) (void)dispatch(strategy);
                for(std::size_t repetition = 0; repetition < repetitions; ++repetition) {
                    for(unsigned step = 0; step < strategy_count; ++step) {
                        const auto strategy = static_cast<unsigned>((step + repetition) % strategy_count);
                        const auto start = std::chrono::steady_clock::now();
                        const auto answer = dispatch(strategy);
                        const auto elapsed = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - start).count();
                        const bool verified = verify(input, answer, factory);
                        csv << input.name << ',' << size << ',' << names[strategy] << ','
                            << repetition << ',' << std::setprecision(9) << elapsed << ','
                            << answer.reduction.solve_count << ',' << answer.elasticity_calls
                            << ',' << answer.members.size() << ',' << answer.reduction.irreducible
                            << ',' << verified << ',' << solver << ',' << version
                            << ',' << answer.elasticity_reoptimizations
                            << ',' << answer.deletion_model_reused
                            << ',' << answer.deletion_reoptimizations
                            << ',' << answer.native_seed_used << ',' << answer.native_seed_size;
                        const auto & s = answer.statistics;
                        const auto & r = answer.reduction.statistics;
                        csv << ',' << s.models_built() << ',' << s.solver_runs()
                            << ',' << s.feasibility_checks.feasible << ',' << s.feasibility_checks.infeasible
                            << ',' << s.feasibility_checks.unknown << ',' << r.batch_checks << ',' << r.singleton_checks
                            << ',' << r.removed_by_batches << ',' << r.removed_by_singletons
                            << ',' << r.necessary_members << ',' << r.unresolved_members
                            << ',' << s.native_verification_calls << ',' << s.elastic_verification_calls
                            << ',' << s.rebuild.bound_only_proofs
                            << ',' << (s.rebuild.solves_skipped + s.deletion.solves_skipped + s.elastic.solves_skipped)
                            << ',' << (s.deletion.bound_updates + s.elastic.bound_updates)
                            << ',' << std::max({s.rebuild.peak_variables, s.deletion.peak_variables, s.elastic.peak_variables})
                            << ',' << std::max({s.rebuild.peak_constraints, s.deletion.peak_constraints, s.elastic.peak_constraints})
                            << '\n';
                        if(!verified) throw std::runtime_error("IIS verification failed: " + input.name);
                    }
                }
            }
        };
        // Load/configure once outside timing. Compare strategies WITHIN a
        // backend; do not interpret separate runs as a solver ranking.
        if(solver_name == "clp") {
            const auto & api = mippp::clp_api::load();
            // Benchmark-only diagnostics, resolved from that exact library;
            // no extra required symbols are added to the production wrapper.
            mippp::detail::dynamic_library library(api.library_path());
            auto log_level = library.get_function<void(mippp::clp::v1_17::Clp_Simplex *, int)>("Clp_setLogLevel");
            auto version = library.get_function<const char *()>("Clp_Version");
            auto factory = [&] {
                mippp::clp_lp model(api);
                log_level(model.native_model(), 0);
                return model;
            };
            benchmark(factory, "Clp", version());
        } else {
            const auto & api = mippp::highs_api::load();
            auto factory = [&] {
                mippp::highs_lp model(api);
                if(api.setBoolOptionValue(model.native_model(), "output_flag", 0) != 0 ||
                   api.setIntOptionValue(model.native_model(), "threads", 1) != 0)
                    throw std::runtime_error("Cannot configure benchmark solver");
                return model;
            };
            benchmark(factory, "HiGHS", api.version());
        }
        csv.close(); // report delayed write errors before announcing success
        std::cout << "Local benchmark saved to " << output << '\n';
    } catch(const std::exception & error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
