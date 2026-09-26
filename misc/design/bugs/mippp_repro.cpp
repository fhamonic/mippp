// Reproductions for docs/upstream/mippp.md (mippp 1.0.0).
//
//   g++ -std=c++23 -O2 -I<mippp>/include mippp_repro.cpp -o mippp_repro -ldl
//   MIPPP_NO_VERSION_WARNING=1 ./mippp_repro [issue]
//
// issue: highs-warning, limit-solution, scip-gap-limit, native-time-limit,
// mosek-limit, mosek-double-solve, controls, stdout, gurobi-banner, gap,
// xpress-miptol; all of them when omitted. The backends must be loadable (see
// mippp's library lookup); a missing one is reported as such.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <format>
#include <functional>
#include <memory>
#include <optional>
#include <print>
#include <random>
#include <ranges>
#include <string>
#include <string_view>
#include <vector>

#include <dlfcn.h>
#include <unistd.h>

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

using namespace mippp;
using namespace mippp::operators;

// Multidimensional 0/1 knapsack: `rows` capacity rows over `n` items.
template <class Model>
void knapsack(Model & model, std::size_t n, int rows, int max_value,
              unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> dist(1, max_value);
    auto items = std::views::iota(std::size_t{0}, n);
    auto x = model.add_binary_variables(n);
    std::vector<double> profit(n);
    for(double & p : profit) p = dist(rng);
    model.set_maximization();
    model.set_objective(
        xsum(items, [&](std::size_t k) { return profit[k] * x(k); }));
    for(int row = 0; row < rows; ++row) {
        std::vector<double> w(n);
        for(double & v : w) v = dist(rng);
        model.add_constraint(
            xsum(items, [&](std::size_t k) { return w[k] * x(k); }) <=
            0.25 * max_value * static_cast<double>(n));
    }
}

void highs_quiet(auto & model) {
    model.native_api().setBoolOptionValue(model.native_model(), "output_flag",
                                          0);
}

// Points file descriptor 1 at `fd` while alive. Restoring it on unwinding
// matters: a backend that fails to load throws from inside the redirection,
// and its diagnostic must still reach the terminal.
class stdout_redirect {
    int _saved;

public:
    explicit stdout_redirect(int fd) : _saved(::dup(STDOUT_FILENO)) {
        std::fflush(nullptr);
        ::dup2(fd, STDOUT_FILENO);
    }
    ~stdout_redirect() {
        std::fflush(nullptr);
        ::dup2(_saved, STDOUT_FILENO);
        ::close(_saved);
    }
    stdout_redirect(const stdout_redirect &) = delete;
    stdout_redirect & operator=(const stdout_redirect &) = delete;
};

struct file_closer {
    void operator()(std::FILE * f) const { std::fclose(f); }
};
using file_ptr = std::unique_ptr<std::FILE, file_closer>;

// Runs `f` with solver logs discarded, so that only this program's lines
// remain; the `stdout` issue measures those logs separately.
void quietly(const std::function<void()> & f) {
    file_ptr null(std::fopen("/dev/null", "w"));
    stdout_redirect redirect(::fileno(null.get()));
    f();
}

// Bytes `f` writes to file descriptor 1, and their first line.
std::pair<long, std::string> stdout_of(const std::function<void()> & f) {
    file_ptr capture(std::tmpfile());
    {
        stdout_redirect redirect(::fileno(capture.get()));
        f();
    }
    std::fseek(capture.get(), 0, SEEK_END);
    const long size = std::ftell(capture.get());
    std::rewind(capture.get());
    char line[160] = {};
    if(size > 0 && std::fgets(line, sizeof line, capture.get()) == nullptr)
        line[0] = 0;
    std::string first(line);
    if(!first.empty() && first.back() == '\n') first.pop_back();
    return {size, first};
}

double seconds_of(const std::function<void()> & f) {
    const auto start = std::chrono::steady_clock::now();
    f();
    return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         start)
        .count();
}

// ---------------------------------------------------------------------------
void highs_warning() {
    std::println("== highs-warning: HiGHS reaching its time limit throws");
    highs_milp model;
    highs_quiet(model);
    knapsack(model, 200, 5, 100, 42);
    model.set_time_limit(std::chrono::duration<double>(0.2));
    try {
        model.solve();
        std::println("solve() returned: time_limit={:d} solution_available={:d}",
                     std::holds_alternative<status::time_limit>(model.get_status()),
                     status::solution_available(model.get_status()));
    } catch(const solver_error & e) {
        namespace h = mippp::highs::impl::v1;
        int primal = -1;
        model.native_api().getIntInfoValue(model.native_model(),
                                           "primal_solution_status", &primal);
        std::println("solve() threw solver_error(\"{}\")", e.what());
        std::println("  HiGHS model status           {} (kHighsModelStatusTimeLimit = {})",
                     model.native_api().getModelStatus(model.native_model()),
                     int(h::kHighsModelStatusTimeLimit));
        std::println("  HiGHS primal_solution_status {} (kHighsSolutionStatusFeasible = {})",
                     primal, int(h::kHighsSolutionStatusFeasible));
        std::println("  HiGHS incumbent objective    {}",
                     model.native_api().getObjectiveValue(model.native_model()));
        std::println("  mippp solution_available()   {:d} (status stale after the throw)",
                     status::solution_available(model.get_status()));
    }
}

// ---------------------------------------------------------------------------
template <class Model>
void limit_solution_of(const char * name) {
    try {
        std::optional<Model> model;
        quietly([&] { model.emplace(); });
        knapsack(*model, 400, 10, 1000, 7);
        quietly([&] {
            model->set_time_limit(std::chrono::duration<double>(0.2));
            model->solve();
        });
        double incumbent = std::nan("");
        try {
            incumbent = model->get_solution_value();
        } catch(const std::exception &) {
        }
        std::println("  {:<12} time_limit={:d}  solution_available={:d}  "
                     "get_solution_value()={}",
                     name,
                     std::holds_alternative<status::time_limit>(model->get_status()),
                     status::solution_available(model->get_status()), incumbent);
    } catch(const std::exception & e) {
        std::println("  {:<12} unavailable: {}", name, e.what());
    }
}

void limit_solution() {
    std::println("== limit-solution: is the incumbent reported after a time limit?");
    limit_solution_of<cbc_milp>("cbc_milp");
    limit_solution_of<gurobi_milp>("gurobi_milp");
    limit_solution_of<cplex_milp>("cplex_milp");
    limit_solution_of<xpress_milp>("xpress_milp");
    limit_solution_of<copt_milp>("copt_milp");
}

// ---------------------------------------------------------------------------
template <class S>
constexpr std::string_view name_of() {
    using namespace status;
    if constexpr(std::is_same_v<S, optimal>) return "optimal";
    else if constexpr(std::is_same_v<S, limit_reached>) return "limit_reached";
    else if constexpr(std::is_same_v<S, time_limit>) return "time_limit";
    else if constexpr(std::is_same_v<S, unknown>) return "unknown";
    else return "another status";
}

template <class Model>
void gap_limit_of(const char * name) {
    try {
        std::optional<Model> model;
        quietly([&] { model.emplace(); });
        knapsack(*model, 200, 5, 100, 42);
        quietly([&] {
            model->set_optimality_tolerance(0.01);
            model->solve();
        });
        std::println("  {:<12} {:<14} solution_available={:d}  get_solution_value()={}",
                     name,
                     std::visit([]<class S>(const S &) { return name_of<S>(); },
                                model->get_status()),
                     status::solution_available(model->get_status()),
                     model->get_solution_value());
        if constexpr(std::is_same_v<Model, scip_milp>) {
            namespace s = mippp::scip::impl::v1;
            std::println("  {:<12} (SCIP status {}, SCIP_STATUS_GAPLIMIT = {}; "
                         "SCIPgetBestSol() {})",
                         "", int(model->native_api().getStatus(model->native_model())),
                         int(s::SCIP_STATUS_GAPLIMIT),
                         model->native_api().getBestSol(model->native_model())
                             ? "non-null"
                             : "null");
        }
    } catch(const std::exception & e) {
        std::println("  {:<12} unavailable: {}", name, e.what());
    }
}

void scip_gap_limit() {
    std::println("== scip-gap-limit: set_optimality_tolerance(0.01), then solve()");
    gap_limit_of<scip_milp>("scip_milp");
    gap_limit_of<gurobi_milp>("gurobi_milp");
    gap_limit_of<cplex_milp>("cplex_milp");
    gap_limit_of<copt_milp>("copt_milp");
    gap_limit_of<cbc_milp>("cbc_milp");
}

// ---------------------------------------------------------------------------
void native_time_limit() {
    std::println("== native-time-limit: SCIP's limits/time through the loaded SCIPsetRealParam");
    scip_milp model;
    knapsack(model, 200, 5, 100, 42);
    model.native_api().setRealParam(model.native_model(), "limits/time", 0.5);
    const double elapsed = seconds_of([&] { quietly([&] { model.solve(); }); });
    std::println("  stopped after {:.2f} s: time_limit={:d} solution_available={:d} "
                 "incumbent objective {}",
                 elapsed, std::holds_alternative<status::time_limit>(model.get_status()),
                 status::solution_available(model.get_status()),
                 model.get_solution_value());
}

// ---------------------------------------------------------------------------
void mosek_limit() {
    std::println("== mosek-limit: MSK_DPAR_OPTIMIZER_MAX_TIME = 0.5 s, set natively");
    namespace m = mippp::mosek::impl::v1;
    constexpr int MSK_DPAR_OPTIMIZER_MAX_TIME = 50;  // mosek.h 11.0
    mosek_milp model;
    knapsack(model, 400, 10, 1000, 7);
    const auto & api = model.native_api();
    const auto task = model.native_model().second;
    api.putdouparam(task, MSK_DPAR_OPTIMIZER_MAX_TIME, 0.5);
    auto print_integer_solution = [&] {
        m::MSKbooleant defined = 0;
        api.solutiondef(task, m::MSK_SOL_ITG, &defined);
        double objective = std::nan("");
        if(defined) api.getprimalobj(task, m::MSK_SOL_ITG, &objective);
        std::println("    MOSEK integer solution defined={:d}, objective {}",
                     defined != 0, objective);
    };
    try {
        quietly([&] { model.solve(); });
        std::println("  model.solve() returned: time_limit={:d} solution_available={:d}",
                     std::holds_alternative<status::time_limit>(model.get_status()),
                     status::solution_available(model.get_status()));
    } catch(const solver_error & e) {
        std::println("  model.solve() threw solver_error(\"{}\")", e.what());
    }
    print_integer_solution();
    m::MSKrestrmcode trm = m::MSK_RES_OK;
    int result = -1;
    quietly([&] { result = api.optimizetrm(task, &trm); });
    std::println("  MSK_optimizetrm() on the same task returns {} (MSK_RES_OK = 0), "
                 "trmcode {} (MSK_RES_TRM_MAX_TIME = {})",
                 result, int(trm), int(m::MSK_RES_TRM_MAX_TIME));
    print_integer_solution();
}

// ---------------------------------------------------------------------------
void mosek_double_solve() {
    std::println("== mosek-double-solve: time of model.solve() against one MSK_optimizetrm()");
    namespace m = mippp::mosek::impl::v1;
    for(int round = 0; round < 3; ++round) {
        mosek_milp through_mippp;
        knapsack(through_mippp, 300, 10, 1000, 7);
        const double solve_time =
            seconds_of([&] { quietly([&] { through_mippp.solve(); }); });
        mosek_milp direct;
        knapsack(direct, 300, 10, 1000, 7);
        m::MSKrestrmcode trm = m::MSK_RES_OK;
        const double once = seconds_of([&] {
            quietly([&] {
                direct.native_api().optimizetrm(direct.native_model().second, &trm);
            });
        });
        std::println("  model.solve() {:.3f} s   one MSK_optimizetrm() {:.3f} s   ratio {:.2f}",
                     solve_time, once, solve_time / once);
    }
}

// ---------------------------------------------------------------------------
void controls() {
    std::println("== controls: what each wrapper exposes (mippp concepts)");
    auto row = []<class M>(const char * name) {
        std::println("  {:<12} time_limit={:d} iteration_limit={:d} node_limit={:d} "
                     "solution_limit={:d} memory_limit={:d} optimality_tolerance={:d}",
                     name, has_time_limit<M>, has_iteration_limit<M>,
                     has_node_limit<M>, has_solution_limit<M>, has_memory_limit<M>,
                     has_optimality_tolerance<M>);
    };
    row.template operator()<highs_milp>("highs_milp");
    row.template operator()<cbc_milp>("cbc_milp");
    row.template operator()<scip_milp>("scip_milp");
    row.template operator()<glpk_milp>("glpk_milp");
    row.template operator()<gurobi_milp>("gurobi_milp");
    row.template operator()<cplex_milp>("cplex_milp");
    row.template operator()<xpress_milp>("xpress_milp");
    row.template operator()<copt_milp>("copt_milp");
    row.template operator()<mosek_milp>("mosek_milp");
    row.template operator()<highs_lp>("highs_lp");
    row.template operator()<clp_lp>("clp_lp");
    row.template operator()<glpk_lp>("glpk_lp");
    row.template operator()<soplex_lp>("soplex_lp");
    row.template operator()<mosek_lp>("mosek_lp");
}

// ---------------------------------------------------------------------------
std::string excerpt(const std::pair<long, std::string> & output, std::size_t n) {
    return output.first ? "\"" + output.second.substr(0, n) + "\"" : "";
}

template <class Model>
void stdout_of_backend(const char * name) {
    try {
        std::optional<Model> model;
        const auto created = stdout_of([&] { model.emplace(); });
        std::string limited = "-";
        if constexpr(has_time_limit<Model>) {
            const auto output = stdout_of(
                [&] { model->set_time_limit(std::chrono::seconds(10)); });
            limited = std::format("{} B {}", output.first, excerpt(output, 36));
        }
        const auto solved = stdout_of([&] {
            model->set_maximization();
            auto x = model->add_variable();
            auto y = model->add_variable();
            model->set_objective(x + y);
            model->add_constraint(x + 2 * y <= 4);
            model->add_constraint(3 * x + y <= 6);
            model->solve();
        });
        std::println("  {:<7} construction {:>3} B {:<24}  set_time_limit {:<43}  solve {:>4} B {}",
                     name, created.first, excerpt(created, 22), limited, solved.first,
                     excerpt(solved, 34));
    } catch(const std::exception & e) {
        std::println("  {:<7} unavailable: {}", name, std::string(e.what()).substr(0, 60));
    }
}

void output_on_stdout() {
    std::println("== stdout: bytes written to stdout by a construction, set_time_limit(10 s) "
                 "(\"-\" where absent) and solving a 2-variable LP");
    stdout_of_backend<highs_lp>("highs");
    stdout_of_backend<cbc_milp>("cbc");
    stdout_of_backend<clp_lp>("clp");
    stdout_of_backend<glpk_lp>("glpk");
    stdout_of_backend<scip_milp>("scip");
    stdout_of_backend<soplex_lp>("soplex");
    stdout_of_backend<gurobi_lp>("gurobi");
    stdout_of_backend<cplex_lp>("cplex");
    stdout_of_backend<xpress_lp>("xpress");
    stdout_of_backend<copt_milp>("copt");
    stdout_of_backend<mosek_lp>("mosek");
}

// ---------------------------------------------------------------------------
// Replays gurobi_base's constructor (empty env, start, new model, env of the
// model), then optimizes the empty model to see whether solve logs still show.
void gurobi_banner() {
    std::println("== gurobi-banner: OutputFlag=0 on the empty environment, before GRBstartenv");
    const auto & api = gurobi_api::load();
    enum class variant { as_is, quiet, quiet_until_optimize };
    for(variant v : {variant::as_is, variant::quiet, variant::quiet_until_optimize}) {
        mippp::gurobi::impl::v1::GRBmodel * model = nullptr;
        auto [constructed, line] = stdout_of([&] {
            auto * env = api._empty_env();
            if(v != variant::as_is) api.setintparam(env, "OutputFlag", 0);
            api.startenv(env);
            api.newmodel(env, &model, "GUROBI", 0, nullptr, nullptr, nullptr,
                         nullptr, nullptr);
            api.freeenv(env);
        });
        auto [optimized, optimize_line] = stdout_of([&] {
            if(v == variant::quiet_until_optimize)
                api.setintparam(api.getenv(model), "OutputFlag", 1);
            api.optimize(model);
        });
        api.freemodel(model);
        const char * label = v == variant::as_is   ? "as gurobi_base does"
                             : v == variant::quiet ? "OutputFlag 0 before GRBstartenv"
                                                   : "same, OutputFlag 1 before GRBoptimize";
        std::println("  {:<38} construction {:>4} B {:<26} optimize {:>4} B {}", label,
                     constructed, constructed ? "\"" + line.substr(0, 22) + "\"" : "",
                     optimized,
                     optimized ? "\"" + optimize_line.substr(0, 36) + "\"" : "");
    }
}

// ---------------------------------------------------------------------------
// Profits of 1e5..1e6 make the default relative gap (1e-4) worth hundreds of
// units; seed 9 is one instance where HiGHS stops early (found by search).
void gap() {
    std::println("== gap: HiGHS reports optimal within its default mip_rel_gap");
    for(bool exact : {false, true}) {
        highs_milp model;
        highs_quiet(model);
        if(exact)
            model.native_api().setDoubleOptionValue(model.native_model(),
                                                    "mip_rel_gap", 0.0);
        std::mt19937 rng(9);
        std::uniform_int_distribution<int> profit(100000, 1000000);
        std::uniform_int_distribution<int> weight(1, 100);
        const std::size_t n = 150;
        auto items = std::views::iota(std::size_t{0}, n);
        auto x = model.add_binary_variables(n);
        std::vector<double> p(n);
        for(double & v : p) v = profit(rng);
        model.set_maximization();
        model.set_objective(xsum(items, [&](std::size_t k) { return p[k] * x(k); }));
        for(int row = 0; row < 5; ++row) {
            std::vector<double> w(n);
            for(double & v : w) v = weight(rng);
            model.add_constraint(
                xsum(items, [&](std::size_t k) { return w[k] * x(k); }) <=
                25.0 * static_cast<double>(n));
        }
        model.set_time_limit(std::chrono::duration<double>(40));
        model.solve();
        double mip_rel_gap = -1;
        model.native_api().getDoubleOptionValue(model.native_model(),
                                                "mip_rel_gap", &mip_rel_gap);
        std::println("  mip_rel_gap {:<6} ({:<10}) optimal={:d} objective {:.0f}",
                     mip_rel_gap, exact ? "set to 0" : "default",
                     std::holds_alternative<status::optimal>(model.get_status()),
                     model.get_solution_value());
    }
}

// ---------------------------------------------------------------------------
// With XPRS_MIPTOL as the integrality tolerance, 0.45 lets x = 1.4 count as
// integral; as a gap it could not raise the objective above 1. Presolve is
// switched off because it would round 2x <= 2.8 to x <= 1 first.
void xpress_miptol() {
    std::println("== xpress-miptol: what set_optimality_tolerance changes on Xpress");
    constexpr int XPRS_PRESOLVE = 8011;  // xprs.h
    for(double tolerance : {-1.0, 0.45}) {
        xpress_milp model;
        // XPRSsetintcontrol is not in mippp's API table: take it from the
        // library mippp loaded, found through a function mippp resolved.
        Dl_info info{};
        ::dladdr(reinterpret_cast<void *>(model.native_api().setdblcontrol), &info);
        void * library = ::dlopen(info.dli_fname, RTLD_NOW | RTLD_NOLOAD);
        using setintcontrol_t = int(decltype(model.native_model()), int, int);
        auto * setintcontrol =
            library ? reinterpret_cast<setintcontrol_t *>(::dlsym(library, "XPRSsetintcontrol"))
                    : nullptr;
        if(setintcontrol == nullptr) {
            std::println("  skipped: XPRSsetintcontrol not found");
            return;
        }
        setintcontrol(model.native_model(), XPRS_PRESOLVE, 0);
        if(tolerance >= 0) model.set_optimality_tolerance(tolerance);
        model.set_maximization();
        auto x = model.add_integer_variable({.upper_bound = 10});
        model.set_objective(1 * x);
        model.add_constraint(2 * x <= 2.8);
        quietly([&] { model.solve(); });
        std::println("  max x, x integer, 2x <= 2.8; set_optimality_tolerance {:<7} -> "
                     "objective {}",
                     tolerance < 0 ? "not set" : std::format("{}", tolerance),
                     model.get_solution_value());
        ::dlclose(library);
    }
}

int main(int argc, char ** argv) {
    const std::string_view only = argc > 1 ? argv[1] : "";
    auto run = [&](std::string_view name, auto f) {
        if(only.empty() || only == name) f();
    };
    run("highs-warning", highs_warning);
    run("limit-solution", limit_solution);
    run("scip-gap-limit", scip_gap_limit);
    run("native-time-limit", native_time_limit);
    run("mosek-limit", mosek_limit);
    run("mosek-double-solve", mosek_double_solve);
    run("controls", controls);
    run("stdout", output_on_stdout);
    run("gurobi-banner", gurobi_banner);
    run("gap", gap);
    run("xpress-miptol", xpress_miptol);
}
