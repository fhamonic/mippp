// User-cut survey, Cbc. Dispatch on argv[1], see main() at the end.
//
// The wrapper's solve() below Cbc 3.0 runs on a private fresh copy (mip_copy),
// so a hook registered on native_model() never runs. There is no outside way
// to reach the copy, so this probe opens the class up and re-implements solve()
// with one added line: register the hook on the model that is about to be solved.
#include <cxxabi.h>
#include <dlfcn.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <charconv>
#include <chrono>
#include <cmath>
#include <compare>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <format>
#include <fstream>
#include <functional>
#include <initializer_list>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <print>
#include <random>
#include <ranges>
#include <ratio>
#include <set>
#include <span>
#include <sstream>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <tuple>
#include <type_traits>
#include <typeinfo>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>
#include <version>
#define private public
#include "mippp/solvers/cbc/all.hpp"
#undef private
#include "cut_common.hpp"

namespace v1 = mippp::cbc::impl::v1;
using v1::Cbc_Model;

// ------------------------------------------------------------- native ABI ---
// 2.10.x: void cb(osi, cuts, app);             Cbc_addCutCallback(model, cb, name, app)
// master: void cb(osi, cuts, app, level, pass); Cbc_addCutCallback(model, cb, name, app, howOften, atSolution)
struct native {
    using cb3_t = void (*)(void *, void *, void *);
    using cb5_t = void (*)(void *, void *, void *, int, int);
    void * lib = nullptr;
    bool master_abi = false;
    void (*add3)(Cbc_Model *, cb3_t, const char *, void *) = nullptr;
    void (*add5)(Cbc_Model *, cb5_t, const char *, void *, int, char) = nullptr;
    void (*add_row)(void *, int, const int *, const double *, char, double) = nullptr;
    void (*add_global)(void *, int, const int *, const double *, char, double) = nullptr;
    int (*ncols)(void *) = nullptr;
    int (*nrows)(void *) = nullptr;
    const double * (*sol)(void *) = nullptr;
    const double * (*lb)(void *) = nullptr;
    const double * (*ub)(void *) = nullptr;
    void (*colname)(void *, int, char *, int) = nullptr;
    int (*proven_optimal)(void *) = nullptr;
    double (*objvalue)(void *) = nullptr;
    void (*add_lazy)(Cbc_Model *, int, int *, double *, char, double) = nullptr;
    template <class F> void get(F & f, const char * name) { f = reinterpret_cast<F>(dlsym(lib, name)); }
    explicit native(const std::filesystem::path & l) {
        lib = dlopen(l.c_str(), RTLD_NOW | RTLD_NOLOAD);
        if(!lib) { std::println(stderr, "dlopen NOLOAD failed: {}", dlerror()); std::exit(2); }
        void * cut = dlsym(lib, "Cbc_addCutCallback");
        get(add_global, "OsiCuts_addGlobalRowCut");
        master_abi = add_global != nullptr;
        if(master_abi) add5 = reinterpret_cast<decltype(add5)>(cut);
        else add3 = reinterpret_cast<decltype(add3)>(cut);
        get(add_row, "OsiCuts_addRowCut");
        get(ncols, "Osi_getNumCols");
        get(nrows, "Osi_getNumRows");
        get(sol, "Osi_getColSolution");
        get(lb, "Osi_getColLower");
        get(ub, "Osi_getColUpper");
        get(colname, "Osi_getColName");
        get(proven_optimal, "Osi_isProvenOptimal");
        get(objvalue, "Osi_getObjValue");
        get(add_lazy, "Cbc_addLazyConstraint");
    }
};

struct hook_ctx {
    cp::separator sep;
    std::unique_ptr<native> abi;
    int cols = 0;
    std::string mode = "plain";
    int how_often = 1;     // master only
    char at_solution = 0;  // master only
    // called first, on every call
    std::function<void(void * osi, int level, int pass)> observer;
    std::function<bool(const std::vector<double> &)> candidate;  // mode "mixed": global on candidates
    bool guard_columns = false;  // call the separator only on a model with all the columns
    std::mutex osi_mutex;
    std::set<void *> osis;
    std::atomic<long> calls_total{0}, cols_mismatch{0}, out_of_range{0}, caught{0}, nonoptimal{0};
    std::atomic<int> last_cols{0};
    std::mutex level_mutex;
    std::map<int, long> by_level, by_pass;
};

static void hook_body(void * osi, void * cuts, hook_ctx & c, int level, int pass) {
    // a CoinError from the add entry point must not unwind through Cbc's frames
    try {
        c.calls_total.fetch_add(1);
        { std::lock_guard l(c.osi_mutex); c.osis.insert(osi); }
        if(level >= 0) { std::lock_guard l(c.level_mutex); c.by_level[level]++; c.by_pass[pass]++; }
        if(c.observer) c.observer(osi, level, pass);
        const int n = c.abi->ncols(osi);
        c.last_cols.store(n);
        if(n != c.cols) { c.cols_mismatch.fetch_add(1); if(c.guard_columns) return; }
        if(c.abi->proven_optimal && !c.abi->proven_optimal(osi)) c.nonoptimal.fetch_add(1);
        std::vector<double> x(static_cast<std::size_t>(std::max(n, c.cols)), 0.0);
        std::copy_n(c.abi->sol(osi), n, x.begin());
        const bool global = c.mode == "global" || (c.mode == "mixed" && c.candidate(x));
        c.sep(x, [&](const cp::raw_cut & r) -> int {
            for(int j : r.ind) if(j >= n) { c.out_of_range.fetch_add(1); return -2; }
            try {
                (global ? c.abi->add_global : c.abi->add_row)(cuts, static_cast<int>(r.ind.size()), r.ind.data(), r.val.data(), r.sense, r.rhs);
                return 0;
            } catch(...) {
                c.caught.fetch_add(1);
                const std::type_info * t = abi::__cxa_current_exception_type();
                std::println(stderr, "   [add threw {}]", t ? cp::short_name(t->name()) : "?");
                return -1;
            }
        });
    } catch(...) { c.caught.fetch_add(1); }
}
static void hook3(void * osi, void * cuts, void * app) { hook_body(osi, cuts, *static_cast<hook_ctx *>(app), -1, -1); }
static void hook5(void * osi, void * cuts, void * app, int level, int pass) { hook_body(osi, cuts, *static_cast<hook_ctx *>(app), level, pass); }

struct M : mippp::cbc_milp {
    using mippp::cbc_milp::cbc_milp;
    std::shared_ptr<hook_ctx> hook;
    bool name_columns = false;
    bool register_hook = true;
    void param(const std::string & n, const std::string & v) { _set_parameter(n, v); }
    // cbc_milp::solve(), plus the registration
    void solve() {
        reset_status();
        _searched = false;
        if(mip_copy) { Cbc->deleteModel(mip_copy); mip_copy = nullptr; }
        if(_lazy_num_variables == 0u) { _mark_results_valid(); return; }
        if(copy_mip_solves && Cbc->getNumIntegers(model) > 0) _copy_into_mip_copy();
        if(name_columns)
            for(int j = 0; j < static_cast<int>(_lazy_num_variables); ++j)
                Cbc->setColName(_solved_model(), j, std::format("c{}", j).c_str());
        if(hook && register_hook) {
            hook->cols = static_cast<int>(_lazy_num_variables);
            if(hook->abi->master_abi)
                hook->abi->add5(_solved_model(), hook5, "probe", hook.get(), hook->how_often, hook->at_solution);
            else
                hook->abi->add3(_solved_model(), hook3, "probe", hook.get());
        }
        Cbc->solve(_solved_model());
        if(hook && std::getenv("PROBE_NO_EXTRAS") == nullptr) {
            std::string lev, pas;
            for(auto [k, v] : hook->by_level) lev += std::format(" {}:{}", k, v);
            for(auto [k, v] : hook->by_pass) pas += std::format(" {}:{}", k, v);
            std::println(stderr, "   [hook: total_calls={} cols_mismatch={} last_cols={} model_cols={} out_of_range={} caught={} osi_nonoptimal={}{}{}]",
                hook->calls_total.load(), hook->cols_mismatch.load(), hook->last_cols.load(), hook->cols, hook->out_of_range.load(), hook->caught.load(), hook->nonoptimal.load(),
                lev.empty() ? std::string{} : " levels" + lev.substr(0, 200), pas.empty() ? std::string{} : " passes" + pas.substr(0, 200));
        }
        _status = _get_status(_solved_model());
        _searched = Cbc->getNumIntegers(_solved_model()) > 0 &&
                    (mippp::is_a<mippp::status::optimal>(_status) ||
                     mippp::is_a<mippp::status::interrupted>(_status) ||
                     mippp::is_a<mippp::status::limit_reached>(_status));
        _mark_results_valid();
    }
};

static bool preprocess_off_in_attach = true;
static void env_params(M & m) {
    if(std::getenv("PROBE_VERBOSE")) m.set_verbose(true);
    // PROBE_PARAMS="name=value,name=value": local adaptation of an experiment
    if(const char * e = std::getenv("PROBE_PARAMS")) {
        std::stringstream ss(e);
        std::string item;
        while(std::getline(ss, item, ',')) {
            const auto p = item.find('=');
            if(p != std::string::npos) m.param(item.substr(0, p), item.substr(p + 1));
        }
    }
}
static std::shared_ptr<hook_ctx> make_hook(M & m, cp::separator sep, const std::string & mode) {
    auto h = std::make_shared<hook_ctx>();
    h->sep = std::move(sep);
    h->abi = std::make_unique<native>(m.native_api().library_path());
    h->mode = mode;
    if(const char * e = std::getenv("PROBE_HOW_OFTEN")) h->how_often = std::atoi(e);
    if(const char * e = std::getenv("PROBE_AT_SOLUTION")) h->at_solution = static_cast<char>(std::atoi(e));
    m.hook = h;
    return h;
}
// Column indices are the original ones only without preprocessing.
static void attach(M & m, cp::separator sep, const std::string & mode) {
    if(preprocess_off_in_attach) m.param("preprocess", "off");
    env_params(m);
    make_hook(m, std::move(sep), mode);
}
static void one_thread(M & m) {
    // threads defaults to 0, the serial code path (CbcSolver.cpp:10829)
    env_params(m);
}
static void scope_settings(M & m) {
    m.param("preprocess", "off");
    m.param("presolve", "off");
    m.param("cuts", "off");
    m.param("heuristics", "off");
    env_params(m);
}
static cp::backend<M> make_backend() {
    cp::backend<M> B;
    B.attach = attach; B.scope_settings = scope_settings; B.one_thread = one_thread;
    { M m; native n(m.native_api().library_path());
      B.cut_modes = n.master_abi ? std::vector<std::string>{"plain", "global"} : std::vector<std::string>{"plain"}; }
    if(const char * e = std::getenv("PROBE_ATSP_N")) { B.atsp_n = std::atoi(e); B.atsp_complement_n = B.atsp_n; }
    if(const char * e = std::getenv("PROBE_GADGETS_K")) B.gadgets_k = std::atoi(e);
    return B;
}
static void banner() {
    M m;
    native n(m.native_api().library_path());
    std::println(stderr, "library {} version {} abi {} add_global={} add_lazy={} isProvenOptimal={}", m.native_api().library_path().string(), m.native_api().getVersion(),
        n.master_abi ? "master(5-arg callback)" : "2.10(3-arg callback)", n.add_global != nullptr, n.add_lazy != nullptr, n.proven_optimal != nullptr);
}


// ------------------------------------------------------------- experiments ---
using clock_t_ = clk;
static std::string full(M & m, const double t) { return cp::summary(m, t); }

// One cut at the first hook call, on the gadgets instance, under scope_settings.
static void odd_once(const std::string & name, const std::function<cp::raw_cut(const cp::gadgets<M> &)> & make, bool verbose = false) {
    M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, false); m.set_time_limit(std::chrono::duration<double>(15));
    if(verbose) m.set_verbose(true);
    auto s = std::make_shared<cp::stats>();
    const cp::raw_cut cut = make(g);
    bool once = false;
    attach(m, cp::watched([&](const auto &, const cp::add_fn & add) { if(!once) { once = true; add(cut); } }, s, [&g](const auto & x) { return g.objective(x); }), "plain");
    const double t = cp::timed_solve(m);
    std::println(stderr, "== WX {}: {} (known opt {})\n   {}", name, full(m, t), g.opt, s->line());
}
static void cmd_odd() {
    odd_once("singleton za3 <= 1", [](const auto & g) { return cp::raw_cut{{g.za[3]}, {1.0}, 'L', 1.0}; });
    odd_once("singleton za3 <= 3", [](const auto & g) { return cp::raw_cut{{g.za[3]}, {1.0}, 'L', 3.0}; });
    odd_once("za3 + zb3 <= 4", [](const auto & g) { return cp::raw_cut{{g.za[3], g.zb[3]}, {1.0, 1.0}, 'L', 4.0}; });
    odd_once("za3 + zb3 == 4", [](const auto & g) { return cp::raw_cut{{g.za[3], g.zb[3]}, {1.0, 1.0}, 'E', 4.0}; });
    odd_once("dummy + za3 <= 1", [](const auto & g) { return cp::raw_cut{{g.dummy, g.za[3]}, {1.0, 1.0}, 'L', 1.0}; });
    odd_once("dummy + za3 == 1", [](const auto & g) { return cp::raw_cut{{g.dummy, g.za[3]}, {1.0, 1.0}, 'E', 1.0}; });
    odd_once("dummy + y1 >= 1.5", [](const auto & g) { return cp::raw_cut{{g.dummy, g.y[1]}, {1.0, 1.0}, 'G', 1.5}; });
    odd_once("equality dummy == 1 (verbose)", [](const auto & g) { return cp::raw_cut{{g.dummy}, {1.0}, 'E', 1.0}; }, true);
    odd_once("dummy >= 1", [](const auto & g) { return cp::raw_cut{{g.dummy}, {1.0}, 'G', 1.0}; });
    odd_once("dummy >= 0.5", [](const auto & g) { return cp::raw_cut{{g.dummy}, {1.0}, 'G', 0.5}; });
    odd_once("equality dummy == 0.5", [](const auto & g) { return cp::raw_cut{{g.dummy}, {1.0}, 'E', 0.5}; });
    odd_once("equality dummy == 0 (not violated)", [](const auto & g) { return cp::raw_cut{{g.dummy}, {1.0}, 'E', 0.0}; });
    odd_once("equality dummy + za3 == 1 (violated by a valid-looking row?)", [](const auto & g) { return cp::raw_cut{{g.dummy, g.za[3]}, {1.0, 1.0}, 'E', 1.0}; });
    odd_once("equality as two inequalities: only the >= half, dummy >= 1 then <= 1 not needed", [](const auto & g) { return cp::raw_cut{{g.dummy}, {2.0}, 'G', 2.0}; });
}

// The harness gadgets, adapted to what Cbc does, see REPORT:
//  - gadget 0 has its LP peak on the WORSE side (y = 0.86 from it): Cbc branches
//    first towards the nearest integer whatever nodeStrategy says, so the worse
//    subtree comes first, before any incumbent;
//  - two rewarded binaries v1, v2: a cut on integer columns survives Cbc's re-solve of
//    the continuous part on the cut-free LP (checkSolution), which a cut on the z alone
//    does not. v1 + v2 <= 1 costs 1 if it holds where the optimum lives.
template <class G> struct with_rewards : G { int v1, v2; cp::raw_cut reward_cut() const { return cp::raw_cut{{v1, v2}, {1.0, 1.0}, 'L', 1.0}; } };
static with_rewards<cp::gadgets<M>> build_gadgets_cbc(M & m, const int k, const bool flip) {
    with_rewards<cp::gadgets<M>> g{};
    g.k = k; g.flip = flip;
    auto y = m.add_binary_variables(static_cast<std::size_t>(k));
    auto za = m.add_variables(static_cast<std::size_t>(k));
    auto zb = m.add_variables(static_cast<std::size_t>(k));
    auto d = m.add_variables(1, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 1});
    const auto Is = std::views::iota(std::size_t{0}, static_cast<std::size_t>(k));
    m.set_maximization();
    g.dummy = m.native_id(d(std::size_t{0}));
    for(std::size_t I : Is) {
        const double c = I == 0 ? 5.0 : 1.0 + 0.03 * static_cast<double>(I);
        g.y.push_back(m.native_id(y(I)));
        g.za.push_back(m.native_id(za(I)));
        g.zb.push_back(m.native_id(zb(I)));
        g.z.push_back(g.za.back()); g.z.push_back(g.zb.back());
        g.c.push_back(c);
        if(I == 0) {
            // in u = y (flip=false) or u = 1 - y (flip=true): 6c at u=0, 4c at u=1, LP peak at u = 0.861
            g.total0.push_back(flip ? 4 : 6); g.total1.push_back(flip ? 6 : 4);
            // col + a u <= rhs, with u = y or 1 - y
            const auto row = [&](auto col, const double a, const double rhs) {
                if(flip) g.rows.push_back(m.add_constraint(col - a * y(I) <= rhs - a));
                else g.rows.push_back(m.add_constraint(col + a * y(I) <= rhs));
            };
            row(za(I), -c * 30, c * 5);
            row(za(I), c * 8, c * 9);
            row(zb(I), -c * 12, c * 1);
            row(zb(I), c * 60, c * 63);
            g.opt += c * 6;
        } else {
            g.total0.push_back(4); g.total1.push_back(4);
            g.rows.push_back(m.add_constraint(za(I) - (c * 30) * y(I) <= c * 3));
            g.rows.push_back(m.add_constraint(za(I) + (c * 6) * y(I) <= c * 7));
            g.rows.push_back(m.add_constraint(zb(I) + (c * 30) * y(I) <= c * 33));
            g.rows.push_back(m.add_constraint(zb(I) - (c * 6) * y(I) <= c * 1));
            g.opt += c * 4;
        }
    }
    auto yb = m.add_binary_variables(3);
    auto zab = m.add_variables(3);
    auto zbb = m.add_variables(3);
    for(std::size_t J = 0; J < 3; ++J) {
        const double c = 3;
        g.za.push_back(m.native_id(zab(J)));
        g.zb.push_back(m.native_id(zbb(J)));
        g.z.push_back(g.za.back()); g.z.push_back(g.zb.back());
        g.rows.push_back(m.add_constraint(zab(J) - (c * 30) * yb(J) <= c * 3));
        g.rows.push_back(m.add_constraint(zab(J) + (c * 6) * yb(J) <= c * 7));
        g.rows.push_back(m.add_constraint(zbb(J) + (c * 30) * yb(J) <= c * 33));
        g.rows.push_back(m.add_constraint(zbb(J) - (c * 6) * yb(J) <= c * 1));
        if(flip) m.add_constraint(yb(J) + y(std::size_t{0}) <= 1);
        else m.add_constraint(yb(J) - y(std::size_t{0}) <= 0);
        g.opt += c * 4;
    }
    auto v = m.add_binary_variables(2);
    g.v1 = m.native_id(v(std::size_t{0})); g.v2 = m.native_id(v(std::size_t{1}));
    // a row that can bind, or Cbc fixes v1 = v2 = 1 before the search, preprocessing off or not
    m.add_constraint(v(std::size_t{0}) + v(std::size_t{1}) + y(std::size_t{1}) <= 2.5);
    g.opt += 2;
    m.set_objective(xsum(Is, [&](std::size_t i) { return za(i) + zb(i); }) +
                    xsum(std::views::iota(std::size_t{0}, std::size_t{3}), [&](std::size_t j) { return zab(j) + zbb(j); }) +
                    v(std::size_t{0}) + v(std::size_t{1}));
    return g;
}
static std::vector<double> best_point(M & m) {
    const double * b = m.Cbc->bestSolution(m._solved_model());
    if(!b) return {};
    return std::vector<double>(b, b + m.num_variables());
}
static std::vector<std::string> modes_of_library() {
    M m; native n(m.native_api().library_path());
    return n.master_abi ? std::vector<std::string>{"plain", "global"} : std::vector<std::string>{"plain"};
}
// S0/S1/S2 of the shared harness on the adapted instance. S1 twice: with the harness cut,
// on the z only, then with a cut on integer columns. Then the same cuts at the root.
static void cmd_scope() {
    using std::chrono::duration;
    const int k = std::getenv("PROBE_GADGETS_K") ? std::atoi(std::getenv("PROBE_GADGETS_K")) : 10;
    const auto ch = [](int st) { return st < 0 ? 'f' : st ? '1' : '0'; };
    for(const bool flip : {false, true}) {
        { M m; scope_settings(m); auto g = build_gadgets_cbc(m, k, flip); m.set_time_limit(duration<double>(30));
          const double t = cp::timed_solve(m);
          std::println(stderr, "== S0c k={} flip={} no hook: {} (known opt {})", k, flip, full(m, t), g.opt); }
        { M m; scope_settings(m); auto g = build_gadgets_cbc(m, k, flip); m.set_time_limit(duration<double>(30));
          auto s = std::make_shared<cp::stats>();
          long st[3] = {0, 0, 0};
          std::string order;
          attach(m, cp::watched([&](const auto & x, const auto &) { st[g.state(x) + 1]++; if(order.size() < 40) order += ch(g.state(x)); }, s, [&g](const auto & x) { return g.objective(x); }, [&g](const auto & x) { return g.point_ok(x); }), "plain");
          const double t = cp::timed_solve(m);
          std::println(stderr, "== S0c same, no-op hook: {}\n   {} y0 frac:{} y0=0:{} y0=1:{} visit order of y0 states: {}", full(m, t), s->line(), st[0], st[1], st[2], order); }
    }
    const auto run = [&](const char * tag, const std::string & mode, const bool flip, const std::function<cp::raw_cut(const with_rewards<cp::gadgets<M>> &)> & make, const bool at_root) {
        M m; scope_settings(m); auto g = build_gadgets_cbc(m, k, flip); m.set_time_limit(duration<double>(30));
        auto s = std::make_shared<cp::stats>();
        auto tr = std::make_shared<cp::scope_tracker>();
        tr->cut = make(g); tr->trigger = at_root ? -1 : g.worse_state(); tr->state = [&g](const auto & x) { return g.state(x); };
        std::string order;
        long ck[2] = {0, 0}, vi[2] = {0, 0};
        attach(m, cp::watched([&, tr](const auto & x, const cp::add_fn & add) {
            const int st = g.state(x);
            if(order.size() < 40) order += ch(st);
            if(tr->added && st >= 0) { ck[st]++; vi[st] += tr->cut.violation(x) > 1e-6; }
            if(tr->wants_add(x)) { tr->add_error = add(tr->cut); order += '*'; }
        }, s, [&g](const auto & x) { return g.objective(x); }, [&g](const auto & x) { return g.point_ok(x); }), mode);
        const double t = cp::timed_solve(m);
        double value = NAN; try { value = m.get_solution_value(); } catch(const std::exception &) {}
        const auto b = best_point(m);
        std::println(stderr, "== {} mode {} flip={}: {} (known opt {}) => {}\n   {}\n   after the cut viol/checks y0=0:{}/{} y0=1:{}/{} ; worse state={} ; visit order (* = cut added): {}\n   final point y0={} v1={} v2={} cut violation at final point={:.4g}\n   {}",
            tag, mode, flip, full(m, t), g.opt, std::fabs(value - g.opt) < 1e-4 ? "optimum KEPT" : "optimum LOST", tr->line(), vi[0], ck[0], vi[1], ck[1], g.worse_state(), order,
            b.empty() ? -1.0 : b[static_cast<std::size_t>(g.y[0])], b.empty() ? -1.0 : b[static_cast<std::size_t>(g.v1)], b.empty() ? -1.0 : b[static_cast<std::size_t>(g.v2)], b.empty() ? NAN : tr->cut.violation(b), s->line());
    };
    // g.opt counts the two rewards, which are not in z: the z optimum is g.opt - 2
    const auto z_cut = [](const with_rewards<cp::gadgets<M>> & g) { auto c = g.invalid_global_cut(); c.rhs -= 2; return c; };
    for(const auto & mode : modes_of_library()) for(const bool flip : {false, true}) {
        run("S1z (invalid cut on the z only)", mode, flip, z_cut, false);
        run("S1i (cut on integer columns)", mode, flip, [](const auto & g) { return g.reward_cut(); }, false);
        run("S2c (hull cut of gadget 3)", mode, flip, [](const auto & g) { return g.hull_cut(3); }, false);
        run("S1i-root (cut on integer columns, at the root)", mode, flip, [](const auto & g) { return g.reward_cut(); }, true);
        run("S1z-root (invalid cut on the z only, at the root)", mode, flip, z_cut, true);
    }
}

static void * last_osi = nullptr;
static const double * last_abi_lb = nullptr;
static native * the_abi = nullptr;
static double seen_lb(int j) { return the_abi->lb(last_osi)[j]; }
static double seen_ub(int j) { return the_abi->ub(last_osi)[j]; }

// Why is a cut on a rowless column, or on columns in a redundant row, "infeasible"?
static void cmd_bounds() {
    using std::chrono::duration;
    struct cs { const char * name; int kind; };
    for(const cs c : {cs{"binary rewards in a redundant row v1+v2<=2, cut v1+v2<=1", 0}, cs{"binary rewards in a redundant row v1+v2+y1<=3, cut v1+v2<=1", 1}, cs{"binary rewards in row v1+v2+y1<=2.5, cut v1+v2<=1", 2}, cs{"rowless dummy, cut dummy>=0.5", 3}, cs{"dummy d2 in a row d2+y1<=2, cut d2>=0.5", 4}, cs{"rowless binary reward v1, cut v1<=0.5", 5}, cs{"continuous rewards in a redundant row v1+v2<=2, cut v1+v2<=1", 6}, cs{"binary rewards in row v1+v2<=1.5, cut v1+v2<=1", 7}}) {
        M m; scope_settings(m);
        auto g = cp::build_gadgets(m, 4, false);
        m.set_time_limit(duration<double>(15));
        cp::raw_cut cut;
        std::vector<int> watch;
        std::size_t y1 = 0;
        auto Y = mippp::model_variable<int, double>(g.y[1]);
        if(c.kind == 3) { cut = cp::raw_cut{{g.dummy}, {1.0}, 'G', 0.5}; watch = {g.dummy}; }
        else if(c.kind == 4) { auto d2 = m.add_variables(1, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 1}); m.add_constraint(d2(y1) + Y <= 2); cut = cp::raw_cut{{m.native_id(d2(y1))}, {1.0}, 'G', 0.5}; watch = cut.ind; }
        else if(c.kind == 5) { auto v = m.add_binary_variables(1); m.set_objective_coefficient(v(y1), 1.0); cut = cp::raw_cut{{m.native_id(v(y1))}, {1.0}, 'L', 0.5}; watch = cut.ind; }
        else {
            auto v = c.kind == 6 ? m.add_variables(2, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 1}) : m.add_binary_variables(2);
            auto v1 = v(std::size_t{0}); auto v2 = v(std::size_t{1});
            m.set_objective_coefficient(v1, 1.0); m.set_objective_coefficient(v2, 1.0);
            if(c.kind == 0 || c.kind == 6) m.add_constraint(v1 + v2 <= 2);
            else if(c.kind == 1) m.add_constraint(v1 + v2 + Y <= 3);
            else if(c.kind == 2) m.add_constraint(v1 + v2 + Y <= 2.5);
            else if(c.kind == 7) m.add_constraint(v1 + v2 <= 1.5);
            cut = cp::raw_cut{{m.native_id(v1), m.native_id(v2)}, {1.0, 1.0}, 'L', 1.0}; watch = cut.ind;
        }
        auto s = std::make_shared<cp::stats>();
        bool once = false;
        std::string seen;
        auto h = std::make_shared<hook_ctx>();
        attach(m, cp::watched([&](const auto & x, const cp::add_fn & add) {
            if(once) return;
            once = true;
            seen += std::format("at first call:");
            for(int j : watch) seen += std::format(" col {} x={} [{},{}]", j, x[static_cast<std::size_t>(j)], m.hook->abi ? seen_lb(j) : 0.0, seen_ub(j));
            add(cut);
        }, s), "plain");
        m.hook->observer = [](void * osi, int, int) { last_osi = osi; last_abi_lb = nullptr; };
        native n(m.native_api().library_path());
        the_abi = &n;
        const double t = cp::timed_solve(m);
        std::println(stderr, "== B {}: {}\n   {} ; cuts={}", c.name, full(m, t), seen, s->cuts.load());
    }
}

// --------------------------------------------------------------- Q6: side effects
static void no_preprocess_param_attach(M & m, cp::separator sep, const std::string & mode) {
    env_params(m);
    make_hook(m, std::move(sep), mode);
}
static const cp::separator noop = [](const std::vector<double> &, const cp::add_fn &) {};
static void cmd_side() {
    using std::chrono::duration;
    const int n = std::getenv("PROBE_ATSP_N") ? std::atoi(std::getenv("PROBE_ATSP_N")) : 26;
    { M m; one_thread(m); auto a = cp::build_atsp(m, n); m.set_time_limit(duration<double>(60)); const double t = cp::timed_solve(m); std::println(stderr, "== SE1 atsp n={} default, no hook: {}", n, full(m, t)); }
    { M m; one_thread(m); m.param("preprocess", "off"); auto a = cp::build_atsp(m, n); m.set_time_limit(duration<double>(60)); const double t = cp::timed_solve(m); std::println(stderr, "== SE2 preprocess off, no hook: {}", full(m, t)); }
    { M m; one_thread(m); auto a = cp::build_atsp(m, n); m.set_time_limit(duration<double>(60)); auto s = std::make_shared<cp::stats>(); attach(m, cp::watched(noop, s, [&a](const auto & x) { return a.objective(x); }, [&a](const auto & x) { return a.point_ok(x); }, [&a](const auto & x) { return a.integral(x); }), "plain"); const double t = cp::timed_solve(m); std::println(stderr, "== SE3 preprocess off, no-op hook: {}\n   {}", full(m, t), s->line()); }
    { M m; one_thread(m); auto a = cp::build_atsp(m, n); m.set_time_limit(duration<double>(60)); auto s = std::make_shared<cp::stats>(); no_preprocess_param_attach(m, cp::watched(noop, s, [&a](const auto & x) { return a.objective(x); }, [&a](const auto & x) { return a.point_ok(x); }, [&a](const auto & x) { return a.integral(x); }), "plain"); const double t = cp::timed_solve(m); std::println(stderr, "== SE4 default preprocess, no-op hook: {}\n   {}", full(m, t), s->line()); }
    { M m; one_thread(m); m.set_verbose(true); auto a = cp::build_atsp(m, n, true); m.set_time_limit(duration<double>(60)); const double t = cp::timed_solve(m); std::println(stderr, "== SE5 complements, default, no hook: {}", full(m, t)); }
    { M m; one_thread(m); m.param("preprocess", "off"); m.set_verbose(true); auto a = cp::build_atsp(m, n, true); m.set_time_limit(duration<double>(60)); const double t = cp::timed_solve(m); std::println(stderr, "== SE6 complements, preprocess off, no hook: {}", full(m, t)); }
    { M m; one_thread(m); cp::build_mkp(m, 30, 100); m.set_time_limit(duration<double>(10)); const double t = cp::timed_solve(m); std::println(stderr, "== SE7 knapsack default, no hook: {}", full(m, t)); }
    { M m; one_thread(m); m.param("preprocess", "off"); cp::build_mkp(m, 30, 100); m.set_time_limit(duration<double>(10)); const double t = cp::timed_solve(m); std::println(stderr, "== SE8 knapsack preprocess off, no hook: {}", full(m, t)); }
    { M m; one_thread(m); cp::build_mkp(m, 30, 100); m.set_time_limit(duration<double>(10)); auto s = std::make_shared<cp::stats>(); attach(m, cp::watched(noop, s), "plain"); const double t = cp::timed_solve(m); std::println(stderr, "== SE9 knapsack preprocess off, no-op hook: {}\n   {}", full(m, t), s->line()); }
}

// ---------------------------------------------------------------- Q5: column space
// A no-op hook against no hook, same settings, stopped by a node limit rather than a clock.
static void cmd_path() {
    using std::chrono::duration;
    const auto one = [](const char * label, const auto & build, const std::size_t limit, const bool hooked) {
        M m; one_thread(m); m.param("preprocess", "off"); build(m); m.set_time_limit(duration<double>(60)); m.set_node_limit(limit);
        auto s = std::make_shared<cp::stats>();
        if(hooked) attach(m, cp::watched(noop, s), "plain");
        const double t = cp::timed_solve(m);
        std::println(stderr, "== PATH {} node limit {} {}: {} calls={}", label, limit, hooked ? "no-op hook" : "no hook   ", full(m, t), s->calls.load());
    };
    for(const bool h : {false, false, true, true}) one("knapsack 30x100", [](M & m) { cp::build_mkp(m, 30, 100); }, 5000, h);
    for(const bool h : {false, true}) one("atsp n=26", [](M & m) { cp::build_atsp(m, 26); }, 3000, h);
}
static void cmd_space() {
    using std::chrono::duration;
    const int n = std::getenv("PROBE_ATSP_N") ? std::atoi(std::getenv("PROBE_ATSP_N")) : 26;
    const auto dfj = [](const cp::atsp & a) { return [&a](const std::vector<double> & x, const cp::add_fn & add) { for(const auto & c : a.separate(x)) add(c); }; };
    double base = NAN;
    { M m; one_thread(m); m.param("preprocess", "off"); auto a = cp::build_atsp(m, n); m.set_time_limit(duration<double>(60)); auto s = std::make_shared<cp::stats>(); attach(m, cp::watched(dfj(a), s, [&a](const auto & x) { return a.objective(x); }, [&a](const auto & x) { return a.point_ok(x); }, [&a](const auto & x) { return a.integral(x); }), "plain"); const double t = cp::timed_solve(m); base = m.get_solution_value(); std::println(stderr, "== C0 control: preprocess off, DFJ cuts: {}\n   {}", full(m, t), s->line()); }
    { M m; one_thread(m); auto a = cp::build_atsp(m, n); m.set_time_limit(duration<double>(60)); auto s = std::make_shared<cp::stats>(); no_preprocess_param_attach(m, cp::watched(dfj(a), s, [&a](const auto & x) { return a.objective(x); }, [&a](const auto & x) { return a.point_ok(x); }, [&a](const auto & x) { return a.integral(x); }), "plain"); const double t = cp::timed_solve(m); std::println(stderr, "== C1 NO preprocess param, plain atsp, DFJ cuts by original index: {} (optimum {})\n   {}", full(m, t), base, s->line()); }
    { M m; one_thread(m); auto a = cp::build_atsp(m, n, true); m.set_time_limit(duration<double>(60)); auto s = std::make_shared<cp::stats>(); no_preprocess_param_attach(m, cp::watched(dfj(a), s, [&a](const auto & x) { return a.objective(x); }, [&a](const auto & x) { return a.point_ok(x); }, [&a](const auto & x) { return a.integral(x); }), "plain"); const double t = cp::timed_solve(m); std::println(stderr, "== C2 NO preprocess param, eliminated complements, cuts on the complements: {} (optimum {})\n   {}", full(m, t), base, s->line()); }
    // the indices shift: columns the preprocessing removes come first
    const auto prefixed = [&](M & m) {
        auto f = m.add_variables(30, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 0});
        (void)f;
        return cp::build_atsp(m, n);
    };
    { M m; one_thread(m); m.param("preprocess", "off"); auto a = prefixed(m); m.set_time_limit(duration<double>(60)); auto s = std::make_shared<cp::stats>(); attach(m, cp::watched(dfj(a), s, [&a](const auto & x) { return a.objective(x); }, [&a](const auto & x) { return a.point_ok(x); }, [&a](const auto & x) { return a.integral(x); }), "plain"); const double t = cp::timed_solve(m); std::println(stderr, "== C3 control: 30 fixed columns first, preprocess off: {} (optimum {})\n   {}", full(m, t), base, s->line()); }
    { M m; one_thread(m); auto a = prefixed(m); m.set_time_limit(duration<double>(60)); auto s = std::make_shared<cp::stats>(); no_preprocess_param_attach(m, cp::watched(dfj(a), s, [&a](const auto & x) { return a.objective(x); }, [&a](const auto & x) { return a.point_ok(x); }, [&a](const auto & x) { return a.integral(x); }), "plain"); const double t = cp::timed_solve(m); std::println(stderr, "== C4 NO preprocess param, 30 fixed columns first, cuts by original index: {} (optimum {})\n   {}", full(m, t), base, s->line()); }
    // names survive preprocessing?
    { M m; one_thread(m); m.name_columns = true; auto a = cp::build_atsp(m, n, true); m.set_time_limit(duration<double>(20));
      auto s = std::make_shared<cp::stats>(); no_preprocess_param_attach(m, cp::watched(noop, s), "plain");
      native nat(m.native_api().library_path());
      bool once = false; int nc = 0, moved = 0; std::string sample;
      m.hook->observer = [&](void * osi, int, int) {
          if(once) return;
          once = true;
          nc = nat.ncols(osi);
          for(int i = 0; i < nc; ++i) {
              char buf[64] = {0};
              nat.colname(osi, i, buf, 63);
              if(std::string(buf) != std::format("c{}", i)) ++moved;
              if(i < 3 || i >= nc - 3) sample += std::format(" {}={}", i, buf);
          }
      };
      const double t = cp::timed_solve(m);
      std::println(stderr, "== C5 names under default preprocess on the complements instance: {}\n   hook saw {} columns of {}, {} columns whose name differs from their index; sample:{}", full(m, t), nc, m.num_variables(), moved, sample); }
}

// --------------------------------------------------------------------- Q7: firing
static std::vector<std::pair<std::string, std::string>> parse_params(const std::string & e) {
    std::vector<std::pair<std::string, std::string>> out;
    std::stringstream ss(e);
    std::string item;
    while(std::getline(ss, item, ',')) { const auto p = item.find('='); if(p != std::string::npos) out.emplace_back(item.substr(0, p), item.substr(p + 1)); }
    return out;
}
static void cmd_firing() {
    using std::chrono::duration;
    const bool master = modes_of_library().size() > 1;
    const std::vector<std::string> sets = master
        ? std::vector<std::string>{"", "passTreeCuts=1", "depthMiniBab=-999", "passTreeCuts=1,depthMiniBab=-999"}
        : std::vector<std::string>{"", "depthMiniBab=-999", "strategy=0"};
    for(const auto & ps : sets) {
        { M m; scope_settings(m); for(auto & [a, b] : parse_params(ps)) m.param(a, b); auto g = cp::build_gadgets(m, 10, false); m.set_time_limit(duration<double>(20)); auto s = std::make_shared<cp::stats>(); attach(m, cp::watched(noop, s), "plain"); for(auto & [a, b] : parse_params(ps)) m.param(a, b); const double t = cp::timed_solve(m); std::println(stderr, "== F gadgets, no-op, params [{}]: {}\n   {}", ps, full(m, t), s->line()); }
        { M m; one_thread(m); cp::build_mkp(m, 30, 100); m.set_time_limit(duration<double>(5)); auto s = std::make_shared<cp::stats>(); attach(m, cp::watched(noop, s), "plain"); for(auto & [a, b] : parse_params(ps)) m.param(a, b); const double t = cp::timed_solve(m); std::println(stderr, "== F knapsack 30x100, no-op, params [{}]: {}\n   {}", ps, full(m, t), s->line()); }
        { M m; one_thread(m); auto a = cp::build_atsp(m, 26); m.set_time_limit(duration<double>(15)); auto s = std::make_shared<cp::stats>(); attach(m, cp::watched(noop, s, {}, {}, [&a](const auto & x) { return a.integral(x); }), "plain"); for(auto & [c, b] : parse_params(ps)) m.param(c, b); const double t = cp::timed_solve(m); std::println(stderr, "== F atsp n=26, no-op, params [{}]: {}\n   {}", ps, full(m, t), s->line()); }
    }
    if(master) for(const auto & [hw, atsol] : std::vector<std::pair<int, int>>{{1, 0}, {-1, 0}, {5, 0}, {-99, 0}, {1, 1}}) {
        M m; scope_settings(m); m.param("passTreeCuts", "1"); auto g = cp::build_gadgets(m, 10, false); m.set_time_limit(duration<double>(20));
        auto s = std::make_shared<cp::stats>(); attach(m, cp::watched(noop, s), "plain"); m.hook->how_often = hw; m.hook->at_solution = static_cast<char>(atsol);
        const double t = cp::timed_solve(m);
        std::println(stderr, "== F master howOften={} atSolution={} (passTreeCuts=1), gadgets no-op: {}\n   {}", hw, atsol, full(m, t), s->line());
    }
    // Same node again? Identified by the local bounds of the binaries.
    const auto refire = [&](const std::string & label, const std::string & ps, const bool tree_only = false, const bool verbose = false) {
        M m; scope_settings(m); for(auto & [a, b] : parse_params(ps)) m.param(a, b);
        auto g = cp::build_gadgets(m, 10, false); m.set_time_limit(duration<double>(20));
        if(verbose) m.set_verbose(true);
        auto s = std::make_shared<cp::stats>();
        native nat(m.native_api().library_path());
        std::vector<double> sig, root_sig;
        long same_after_cut = 0, same_after_none = 0, new_after_cut = 0, new_after_none = 0, root_calls = 0, cuts_last = 0;
        bool first = true;
        attach(m, cp::watched([&](const auto & x, const cp::add_fn & add) {
            cuts_last = 0;
            if(tree_only && g.state(x) < 0) return;
            // the most violated hull cut only, so that closing the gap takes many rounds
            int best = -1; double most = 1e-4;
            for(int i = 1; i < g.k; ++i) { const double v = g.hull_cut(i).violation(x); if(v > most) { most = v; best = i; } }
            if(best >= 0 && add(g.hull_cut(best)) == 0) ++cuts_last;
        }, s), "plain");
        long prev_cuts = 0;
        m.hook->observer = [&](void * osi, int, int) {
            std::vector<double> now(nat.lb(osi), nat.lb(osi) + nat.ncols(osi));
            now.insert(now.end(), nat.ub(osi), nat.ub(osi) + nat.ncols(osi));
            if(first) { root_sig = now; first = false; sig = now; ++root_calls; return; }
            prev_cuts = cuts_last;
            const bool same = now == sig;
            if(same) { (prev_cuts ? same_after_cut : same_after_none)++; } else { (prev_cuts ? new_after_cut : new_after_none)++; }
            if(now == root_sig) ++root_calls;
            sig = now;
        };
        const double t = cp::timed_solve(m);
        std::println(stderr, "== FR {} params [{}]: {}\n   root calls={} same node again after a cut={} after no cut={} ; new node after a cut={} after no cut={}\n   {}", label, ps, full(m, t), root_calls, same_after_cut, same_after_none, new_after_cut, new_after_none, s->line());
    };
    refire("one hull cut per call", "");
    refire("one hull cut per call", "passTreeCuts=1");
    refire("one hull cut per call", "passTreeCuts=5");
    refire("one hull cut per call", "passCuts=3");
    refire("one hull cut per call", "passCuts=-3");
    refire("one hull cut per call", "passCuts=3,passTreeCuts=1", false, true);
    refire("one hull cut per call, below the root only", "", true);
    refire("one hull cut per call, below the root only", "passTreeCuts=1", true);
    refire("one hull cut per call, below the root only", "passTreeCuts=5", true);
    refire("one hull cut per call, below the root only", "passTreeCuts=-5", true);
}

// -------------------------------------------------------------------- Q8: threads
static void cmd_threads2() {
    using std::chrono::duration;
    M m; m.param("threads", std::getenv("PROBE_THREADS") ? std::getenv("PROBE_THREADS") : "4");
    auto a = cp::build_atsp(m, 40); m.set_time_limit(duration<double>(20));
    auto s = std::make_shared<cp::stats>();
    const bool t2_noop = std::getenv("PROBE_T2_NOOP") != nullptr;
    attach(m, cp::watched([&a, t2_noop](const auto & x, const cp::add_fn & add) { if(!t2_noop) for(const auto & c : a.separate(x, 1)) add(c); }, s, [&a](const auto & x) { return a.objective(x); }, [&a](const auto & x) { return a.point_ok(x); }, [&a](const auto & x) { return a.integral(x); }, std::chrono::microseconds(100)), "plain");
    if(!std::getenv("PROBE_T2_DEFAULTS")) { m.param("cuts", "off"); m.param("heuristics", "off"); }
    std::mutex mu; std::map<int, long> cols;
    m.hook->observer = [&](void * osi, int, int) { const int n = m.hook->abi->ncols(osi); std::lock_guard l(mu); cols[n]++; };
    const double t = cp::timed_solve(m);
    std::string cs; for(auto [a2, b] : cols) cs += std::format(" {}:{}", a2, b);
    std::println(stderr, "== T2 {}\n   {} osis={} columns seen by the hook:{}", full(m, t), s->line(), m.hook->osis.size(), cs);
}
static void cmd_threads() {
    using std::chrono::duration;
    for(const char * th : {"4", "1", "0"}) {
        { M m; m.param("threads", th); cp::build_mkp(m, 30, 100); m.set_time_limit(duration<double>(8)); auto s = std::make_shared<cp::stats>(); attach(m, cp::watched(noop, s, {}, {}, {}, std::chrono::microseconds(200)), "plain"); const double t = cp::timed_solve(m); std::println(stderr, "== T knapsack threads={} 200 us spin: {}\n   {} ; distinct osi pointers={}", th, full(m, t), s->line(), m.hook->osis.size()); }
        { M m; m.param("threads", th); auto a = cp::build_atsp(m, 40); m.set_time_limit(duration<double>(20)); auto s = std::make_shared<cp::stats>(); attach(m, cp::watched([&a](const auto & x, const cp::add_fn & add) { for(const auto & c : a.separate(x, 1)) add(c); }, s, [&a](const auto & x) { return a.objective(x); }, [&a](const auto & x) { return a.point_ok(x); }, [&a](const auto & x) { return a.integral(x); }, std::chrono::microseconds(100)), "plain"); m.param("cuts", "off"); m.param("heuristics", "off"); const double t = cp::timed_solve(m); std::println(stderr, "== T atsp n=40 threads={} one DFJ cut per call, cuts and heuristics off: {}\n   {} ; distinct osi pointers={}", th, full(m, t), s->line(), m.hook->osis.size()); }
    }
    // a tree that survives the cuts, at 4 threads
    { M m; scope_settings(m); m.param("threads", "4"); auto g = cp::build_gadgets(m, 14, false); m.set_time_limit(duration<double>(20)); auto s = std::make_shared<cp::stats>(); attach(m, cp::watched(noop, s, [&g](const auto & x) { return g.objective(x); }, [&g](const auto & x) { return g.point_ok(x); }, {}, std::chrono::microseconds(200)), "plain"); m.param("passTreeCuts", "1"); const double t = cp::timed_solve(m); std::println(stderr, "== T gadgets k=14 threads=4 200 us spin: {}\n   {} ; distinct osi pointers={}", full(m, t), s->line(), m.hook->osis.size()); }
}

// ---------------------------------------------------------------- Q9: persistence
static void cmd_persist() {
    using std::chrono::duration;
    M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, false); m.set_time_limit(duration<double>(30));
    auto s = std::make_shared<cp::stats>();
    const cp::raw_cut cut = g.hull_cut(3);
    bool once = false;
    attach(m, cp::watched([&](const auto & x, const cp::add_fn & add) { if(!once && cut.violation(x) > 1e-4) { once = true; add(cut); } }, s), "plain");
    double t = cp::timed_solve(m);
    std::println(stderr, "== P first solve with the hull cut of gadget 3 (added at the root, so global): {} (known opt {})\n   {}", full(m, t), g.opt, s->line());
    for(std::size_t r = 12; r < 16; ++r) m.set_constraint_upper_bound(g.rows[r], m.get_constraint_rhs(g.rows[r]) + 50);
    t = cp::timed_solve(m);
    double value = NAN; try { value = m.get_solution_value(); } catch(const std::exception &) {}
    std::println(stderr, "== P re-solve, rhs raised so that the cut is invalid: {} (new opt {}) => {}\n   {}", full(m, t), g.opt + 100, std::fabs(value - (g.opt + 100)) < 1e-4 ? "cut did NOT persist" : "cut PERSISTED (wrong optimum)", s->line());
    // A cut on integer columns, so that it could not hide behind the continuous re-solve
    { M m2; scope_settings(m2); auto g2 = build_gadgets_cbc(m2, 10, false); m2.set_time_limit(duration<double>(30));
      auto s2 = std::make_shared<cp::stats>(); bool once2 = false; const auto c2 = g2.reward_cut();
      attach(m2, cp::watched([&](const auto &, const cp::add_fn & add) { if(!once2) { once2 = true; add(c2); } }, s2), "plain");
      double t2 = cp::timed_solve(m2);
      std::println(stderr, "== P2 first solve, invalid root cut on integer columns: {} (known opt {})", full(m2, t2), g2.opt);
      t2 = cp::timed_solve(m2);
      std::println(stderr, "== P2 re-solve, nothing changed, no new cut: {} (known opt {})", full(m2, t2), g2.opt); }
    // is the registration itself sticky, on a model that is solved in place?
    { M m3; scope_settings(m3); auto g3 = cp::build_gadgets(m3, 8, false); m3.set_time_limit(duration<double>(10));
      auto s3 = std::make_shared<cp::stats>();
      attach(m3, cp::watched(noop, s3), "plain");
      double t3 = cp::timed_solve(m3);
      const long c1 = s3->calls.load();
      std::println(stderr, "== PR first solve: {} calls={}", full(m3, t3), c1);
      m3.register_hook = false;
      m3.set_constraint_upper_bound(g3.rows[0], m3.get_constraint_rhs(g3.rows[0]) + 1);
      t3 = cp::timed_solve(m3);
      const long c2 = s3->calls.load();
      std::println(stderr, "== PR re-solve, hook not registered again: {} calls during it={} => {}", full(m3, t3), c2 - c1, c2 > c1 ? "registration STICKS to the model" : "registration gone");
      if(m3.hook->abi->master_abi) {
          m3.hook->abi->add5(m3._solved_model(), nullptr, "none", nullptr, 1, 0);
          m3.set_constraint_upper_bound(g3.rows[0], m3.get_constraint_rhs(g3.rows[0]) + 1);
          t3 = cp::timed_solve(m3);
          std::println(stderr, "== PR re-solve after Cbc_addCutCallback(model, NULL, ...): {} calls during it={}", full(m3, t3), s3->calls.load() - c2);
      } }
}

// --------------------------------------------------------- Q11: lazy and user cuts
static cp::atsp build_dfj(M & m, const int n, const unsigned seed = 5) {
    cp::atsp a;
    a.n = n;
    const auto N = static_cast<std::size_t>(n);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> P(0, 100), noise(1.0, 1.3);
    std::vector<std::pair<double, double>> pts(N);
    for(auto & p : pts) p = {P(rng), P(rng)};
    a.d.assign(N, std::vector<double>(N, 0));
    for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j)
        a.d[i][j] = std::round(std::hypot(pts[i].first - pts[j].first, pts[i].second - pts[j].second) * noise(rng));
    const auto arcs = N * (N - 1);
    const auto id = [N](std::size_t i, std::size_t j) { return i * (N - 1) + (j < i ? j : j - 1); };
    auto x = m.add_binary_variables(arcs);
    a.col.assign(N, std::vector<int>(N, -1));
    a.comp.assign(N, std::vector<int>(N, -1));
    for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j) a.col[i][j] = m.native_id(x(id(i, j)));
    m.set_minimization();
    const auto nodes = std::views::iota(std::size_t{0}, N);
    m.set_objective(xsum(nodes, [&](std::size_t i) {
        return xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return a.d[i][j] * x(id(i, j)); });
    }));
    for(std::size_t i : nodes) {
        m.add_constraint(xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return 1.0 * x(id(i, j)); }) == 1);
        m.add_constraint(xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return 1.0 * x(id(j, i)); }) == 1);
    }
    return a;
}
static int cycles_of(const cp::atsp & a, const std::vector<double> & x) {
    if(x.empty()) return -1;
    std::vector<int> next(static_cast<std::size_t>(a.n), -1);
    for(int i = 0; i < a.n; ++i) for(int j = 0; j < a.n; ++j) if(i != j && a.X(x, i, j) > 0.5) next[static_cast<std::size_t>(i)] = j;
    std::vector<char> seen(static_cast<std::size_t>(a.n), 0);
    int cycles = 0;
    for(int i = 0; i < a.n; ++i) if(!seen[static_cast<std::size_t>(i)]) {
        ++cycles;
        int v = i;
        while(v >= 0 && !seen[static_cast<std::size_t>(v)]) { seen[static_cast<std::size_t>(v)] = 1; v = next[static_cast<std::size_t>(v)]; }
    }
    return cycles;
}
// A budget row makes the root LP fractional: without it the degree-only LP is integral
// at the root, Cbc never enters branch and bound, and the hook is not called at all.
static void add_budget(M & m, const cp::atsp & a, const double fraction, const unsigned seed = 11) {
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> W(1, 100);
    const auto N = static_cast<std::size_t>(a.n);
    double total = 0;
    std::vector<std::pair<int, double>> terms;
    for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j) { const double w = W(rng); terms.emplace_back(a.col[i][j], w); total += w; }
    const double b = std::floor(fraction * total / static_cast<double>(a.n - 1));
    auto keys = std::views::iota(std::size_t{0}, terms.size());
    m.add_constraint(xsum(keys, [&terms](std::size_t k) { return terms[k].second * mippp::model_variable<int, double>(terms[k].first); }) <= b);
}
static void cmd_lazy() {
    using std::chrono::duration;
    const bool master = modes_of_library().size() > 1;
    std::vector<std::pair<int, double>> cases{{26, 0.0}, {16, 0.8}};
    if(const char * e = std::getenv("PROBE_LAZY")) { cases.clear(); for(auto & [a, b] : parse_params(e)) cases.emplace_back(std::atoi(a.c_str()), std::atof(b.c_str())); }
    for(const auto & [n, budget] : cases) {
        double opt = NAN;
        { M m; one_thread(m); auto a = cp::build_atsp(m, n); if(budget > 0) add_budget(m, a, budget); m.set_time_limit(duration<double>(60)); auto s = std::make_shared<cp::stats>(); attach(m, cp::watched([&a](const auto & x, const cp::add_fn & add) { for(const auto & c : a.separate(x)) add(c); }, s), "plain"); const double t = cp::timed_solve(m); opt = m.get_solution_value(); std::println(stderr, "== L0 n={} budget={} reference: complete MTZ model + DFJ cuts: {}", n, budget, full(m, t)); }
        if(budget > 0) { M m; one_thread(m); auto a = cp::build_atsp(m, n); add_budget(m, a, budget); m.set_time_limit(duration<double>(60)); const double t = cp::timed_solve(m); std::println(stderr, "== L0b same, MTZ only, no hook, default settings: {}", full(m, t)); opt = m.get_solution_value(); }
        struct run { std::string label; std::string mode; int how; int atsol; std::string params; };
        std::vector<run> runs = {
            {"plain cuts at every call, default heuristics", "plain", 1, 0, ""},
            {"plain cuts at every call, heuristics off", "plain", 1, 0, "heuristics=off"},
            {"plain cuts at every call, heuristics off, passTreeCuts=5", "plain", 1, 0, "heuristics=off,passTreeCuts=5"},
        };
        if(master) {
            runs.push_back({"plain cuts, heuristics off, passTreeCuts=1", "plain", 1, 0, "heuristics=off,passTreeCuts=1"});
            runs.push_back({"atSolution=1, global cuts", "global", 1, 1, ""});
            runs.push_back({"atSolution=1, global on candidates, plain on fractional points", "mixed", 1, 1, ""});
            runs.push_back({"atSolution=1, plain cuts everywhere", "plain", 1, 1, ""});
            runs.push_back({"atSolution=1, global cuts, howOften=-1", "global", -1, 1, ""});
            runs.push_back({"atSolution=1, global on candidates, plain on fractional, passTreeCuts=1", "mixed", 1, 1, "passTreeCuts=1"});
        }
        for(const auto & r : runs) {
            M m; one_thread(m); auto a = build_dfj(m, n); if(budget > 0) add_budget(m, a, budget); m.set_time_limit(duration<double>(60));
            auto s = std::make_shared<cp::stats>();
            long frac_calls = 0, cand_calls = 0, frac_cuts = 0, cand_cuts = 0;
            attach(m, cp::watched([&](const auto & x, const cp::add_fn & add) {
                const bool cand = a.integral(x);
                (cand ? cand_calls : frac_calls)++;
                for(const auto & c : a.separate(x)) if(add(c) == 0) (cand ? cand_cuts : frac_cuts)++;
            }, s, [&a](const auto & x) { return a.objective(x); }, [&a](const auto & x) { return a.point_ok(x); }, [&a](const auto & x) { return a.integral(x); }), r.mode);
            for(auto & [c, b] : parse_params(r.params)) m.param(c, b);
            m.hook->how_often = r.how; m.hook->at_solution = static_cast<char>(r.atsol);
            m.hook->candidate = [&a](const std::vector<double> & x) { return a.integral(x); };
            const double t = cp::timed_solve(m);
            const auto b = best_point(m);
            const int cyc = cycles_of(a, b);
            double value = NAN; try { value = m.get_solution_value(); } catch(const std::exception &) {}
            std::println(stderr, "== L n={} budget={} {}: {} (right optimum {}) cycles in the returned point={} => {}\n   calls on candidates={} with {} cuts, on fractional points={} with {} cuts\n   {}", n, budget, r.label, full(m, t), opt, cyc,
                (cyc == 1 && std::fabs(value - opt) < 1e-4) ? "RIGHT" : "WRONG", cand_calls, cand_cuts, frac_calls, frac_cuts, s->line());
        }
    }
}

// Valid DFJ cuts on a complete model (MTZ + budget). The reduced-cost restart hands the hook a model
// with fewer columns: what do cuts by original index do there, with and without a guard?
static void cmd_restart() {
    using std::chrono::duration;
    for(const auto & [n, seed] : std::vector<std::pair<int, unsigned>>{{20, 5}, {24, 5}, {28, 5}, {20, 6}, {24, 6}, {28, 6}, {20, 7}, {24, 7}, {28, 7}, {32, 5}, {32, 6}, {36, 5}}) {
        std::string row = std::format("n={} seed={}:", n, seed);
        for(const bool guard : {true, false}) {
            M m; one_thread(m); auto a = cp::build_atsp(m, n, false, seed); add_budget(m, a, 0.72); m.set_time_limit(duration<double>(15));
            auto s = std::make_shared<cp::stats>();
            attach(m, cp::watched([&a](const auto & x, const cp::add_fn & add) { for(const auto & c : a.separate(x, 1)) add(c); }, s), "plain");
            m.param("passTreeCuts", "1");
            m.hook->guard_columns = guard;
            (void)cp::timed_solve(m);
            double value = NAN; try { value = m.get_solution_value(); } catch(const std::exception &) {}
            row += std::format(" [{}: {} value={} dual={} calls on a smaller model={}]", guard ? "guard" : "no guard", cp::status_name(m.get_status()), value, m.get_dual_bound(), m.hook->cols_mismatch.load());
        }
        std::println(stderr, "== R {}", row);
    }
}

// ----------------------------------------------------------- Q12: node-local reads
static void cmd_local() {
    using std::chrono::duration;
    M m; scope_settings(m); m.param("passTreeCuts", "1"); auto g = build_gadgets_cbc(m, 10, false); m.set_time_limit(duration<double>(20));
    auto s = std::make_shared<cp::stats>();
    native nat(m.native_api().library_path());
    long fixed0 = 0, fixed1 = 0, open = 0, x_int_not_fixed = 0, fixed_x_differs = 0, rows_min = 1 << 30, rows_max = 0;
    std::map<int, long> tight;
    std::string shallow;
    int first_rows = -1;
    attach(m, cp::watched(noop, s), "plain");
    m.hook->observer = [&](void * osi, int level, int pass) {
        const auto j = static_cast<std::size_t>(g.y[0]);
        const double l = nat.lb(osi)[j], u = nat.ub(osi)[j], v = nat.sol(osi)[j];
        if(l == u) { (u > 0.5 ? fixed1 : fixed0)++; if(std::fabs(v - u) > 1e-6) ++fixed_x_differs; }
        else { ++open; if(std::fabs(v - std::round(v)) < 1e-6) ++x_int_not_fixed; }
        int t = 0;
        for(int k : g.y) if(nat.lb(osi)[static_cast<std::size_t>(k)] == nat.ub(osi)[static_cast<std::size_t>(k)]) ++t;
        tight[t]++;
        const int r = nat.nrows(osi);
        if(first_rows < 0) first_rows = r;
        rows_min = std::min<long>(rows_min, r); rows_max = std::max<long>(rows_max, r);
        if(shallow.size() < 300) shallow += std::format(" [level={} pass={} fixed_y={} rows={} y0=[{},{}] x={:.3g}]", level, pass, t, r, l, u, v);
    };
    const double t = cp::timed_solve(m);
    std::string tl;
    for(auto [a, b] : tight) tl += std::format(" {}:{}", a, b);
    std::println(stderr, "== N local reads through the osi argument: {}\n   y0 bounds at calls: open={} fixed at 0={} fixed at 1={} ; x integral but bound open={} ; fixed but x differs={}\n   number of fixed y per call (a depth proxy):{}\n   Osi_getNumRows at calls: first={} min={} max={}\n   first calls:{}\n   {}", full(m, t), open, fixed0, fixed1, x_int_not_fixed, fixed_x_differs, tl, first_rows, rows_min, rows_max, shallow, s->line());
}
// The outer Cbc_Model is not the search's model: asked from inside the hook. Run apart, it may abort.
static void cmd_outer() {
    using std::chrono::duration;
    M m; scope_settings(m); auto g = cp::build_gadgets(m, 8, false); m.set_time_limit(duration<double>(10));
    auto s = std::make_shared<cp::stats>();
    std::set<int> seen;
    long calls = 0;
    attach(m, cp::watched(noop, s), "plain");
    m.hook->observer = [&](void *, int, int) { ++calls; seen.insert(m.Cbc->getNodeCount(m._solved_model())); };
    const double t = cp::timed_solve(m);
    std::string v;
    for(int i : seen) v += std::format(" {}", i);
    std::println(stderr, "== O Cbc_getNodeCount(outer model) from inside the hook: {} ; values seen over {} calls:{}", full(m, t), calls, v);
}

int main(int argc, char ** argv) {
    banner();
    const std::string what = argc > 1 ? argv[1] : "matrix";
    if(what == "matrix") cp::run_matrix(make_backend());
    else if(what == "banner") {}
    else if(what == "odd") cmd_odd();
    else if(what == "scope") cmd_scope();
    else if(what == "bounds") cmd_bounds();
    else if(what == "side") cmd_side();
    else if(what == "space") cmd_space();
    else if(what == "path") cmd_path();
    else if(what == "firing") cmd_firing();
    else if(what == "threads") cmd_threads();
    else if(what == "threads2") cmd_threads2();
    else if(what == "persist") cmd_persist();
    else if(what == "lazy") cmd_lazy();
    else if(what == "restart") cmd_restart();
    else if(what == "local") cmd_local();
    else if(what == "outer") cmd_outer();
    else { std::println(stderr, "unknown command {}", what); return 2; }
    std::fflush(stderr);
}
