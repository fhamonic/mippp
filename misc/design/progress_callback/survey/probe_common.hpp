#pragma once
// Shared harness for the progress-hook survey. A backend probe registers its
// native hook through attach(); everything else goes through the MIP++ API.
// Results go to stderr so that a verbose solver log on stdout stays apart.
#include <cxxabi.h>
#include <dlfcn.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <format>
#include <functional>
#include <limits>
#include <map>
#include <mutex>
#include <print>
#include <random>
#include <set>
#include <string>
#include <thread>
#include <typeinfo>
#include <variant>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"

using namespace mippp;
using namespace mippp::operators;
using clk = std::chrono::steady_clock;

inline void * sym(const std::filesystem::path & lib, const char * name) {
    void * h = dlopen(lib.c_str(), RTLD_NOW | RTLD_NOLOAD);
    if(!h) {
        std::println(stderr, "dlopen NOLOAD failed: {}", dlerror());
        std::exit(2);
    }
    void * f = dlsym(h, name);
    if(!f) {
        std::println(stderr, "missing symbol {}", name);
        std::exit(2);
    }
    return f;
}
template <class F>
F symf(const std::filesystem::path & lib, const char * name) {
    return reinterpret_cast<F>(sym(lib, name));
}

inline std::string short_name(const char * mangled) {
    int st = 0;
    char * d = abi::__cxa_demangle(mangled, nullptr, nullptr, &st);
    std::string s = d ? d : mangled;
    std::free(d);
    auto p = s.rfind("::");
    return p == std::string::npos ? s : s.substr(p + 2);
}
template <class S>
std::string status_name(const S & s) {
    return std::visit(
        [](const auto & t) {
            std::string n = short_name(typeid(t).name());
            if constexpr(requires { t.solution_available; })
                n += std::format("{{sol={}}}", bool(t.solution_available));
            return n;
        },
        s);
}

struct sample {
    double t, primal, dual;
    long long nodes;
};

struct recorder {
    clk::time_point t0 = clk::now();
    std::mutex mtx;
    std::atomic<int> inflight{0};
    int overlaps = 0;
    std::set<std::thread::id> threads;
    std::vector<sample> samples;
    std::map<std::string, long long> events;
    long stop_after = 0;  // 0: never ask for a stop
    bool stop_asked = false;
    clk::time_point stop_time;
    std::string note;
    int quiet_read_errors = 0;

    void start() { t0 = clk::now(); }
    double now() const {
        return std::chrono::duration<double>(clk::now() - t0).count();
    }
    void count(const std::string & ev) {
        std::lock_guard l(mtx);
        ++events[ev];
    }
    // true exactly once, on the call where the hook has to ask for the stop
    bool record(const std::string & ev, double primal, double dual,
                long long nodes) {
        if(inflight.fetch_add(1) != 0) {
            std::lock_guard l(mtx);
            ++overlaps;
        }
        bool stop_now = false;
        {
            std::lock_guard l(mtx);
            ++events[ev];
            threads.insert(std::this_thread::get_id());
            samples.push_back({now(), primal, dual, nodes});
            if(stop_after && !stop_asked &&
               (long)samples.size() >= stop_after) {
                stop_asked = true;
                stop_time = clk::now();
                stop_now = true;
            }
        }
        inflight.fetch_sub(1);
        return stop_now;
    }
};

inline bool same(double a, double b) {
    return std::fabs(a - b) <= 1e-6 * (1 + std::fabs(a));
}

template <class M, class... X>
void final_line(const std::string & label, M & m, double solve_s, X &&...) {
    std::string val = "n/a", dual = "n/a", nodes = "n/a";
    try {
        val = std::format("{:.6g}", m.get_solution_value());
    } catch(const std::exception & e) { val = "throws"; }
    if constexpr(has_dual_bound<M>) {
        try {
            dual = std::format("{:.6g}", m.get_dual_bound());
        } catch(const std::exception & e) { dual = "throws"; }
    }
    if constexpr(has_node_count<M>) {
        try {
            nodes = std::format("{}", m.get_node_count());
        } catch(const std::exception & e) { nodes = "throws"; }
    }
    std::println(stderr, "== {}: status={} solve={:.2f}s value={} dual={} nodes={}",
                 label, status_name(m.get_status()), solve_s, val, dual, nodes);
}

// direction: +1 minimization (primal falls, dual rises), -1 maximization
inline void hook_report(recorder & r, int direction, double solve_s,
                        double final_value = NAN, double final_dual = NAN) {
    const auto & s = r.samples;
    long long total = 0;
    std::string ev;
    for(auto & [k, v] : r.events) {
        total += v;
        ev += std::format(" {}={}", k, v);
    }
    std::println(stderr, "   events:{}", ev.empty() ? " NONE" : ev);
    if(!r.note.empty()) std::println(stderr, "   note: {}", r.note);
    if(s.empty()) {
        std::println(stderr, "   samples: 0");
        return;
    }
    double maxgap = s.front().t, prev = s.front().t;
    int pviol = 0, dviol = 0;
    int psent = 0;
    for(std::size_t i = 1; i < s.size(); ++i) {
        maxgap = std::max(maxgap, s[i].t - prev);
        prev = s[i].t;
        const double eps = 1e-6;
        const bool p_ok_prev = std::fabs(s[i - 1].primal) < 1e19;
        const bool d_ok_prev = std::fabs(s[i - 1].dual) < 1e19;
        if(p_ok_prev && std::fabs(s[i].primal) < 1e19 &&
           direction * (s[i].primal - s[i - 1].primal) >
               eps * (1 + std::fabs(s[i].primal)))
            ++pviol;
        if(d_ok_prev && std::fabs(s[i].dual) < 1e19 &&
           direction * (s[i].dual - s[i - 1].dual) <
               -eps * (1 + std::fabs(s[i].dual)))
            ++dviol;
    }
    for(auto & x : s) psent += std::fabs(x.primal) >= 1e19;
    const auto & a = s.front();
    const auto & z = s.back();
    std::println(stderr,
                 "   samples: {} ({:.1f}/s over {:.2f}s), first@{:.3f}s "
                 "last@{:.3f}s maxgap={:.3f}s threads={} overlaps={}",
                 s.size(), s.size() / std::max(solve_s, 1e-3), solve_s, a.t,
                 z.t, maxgap, r.threads.size(), r.overlaps);
    std::println(stderr,
                 "   primal: first={:.7g} last={:.7g} worse-moves={} "
                 "sentinel-samples={} | dual: first={:.7g} last={:.7g} "
                 "worse-moves={} | nodes: first={} last={}",
                 a.primal, z.primal, pviol, psent, a.dual, z.dual, dviol,
                 a.nodes, z.nodes);
    if(!std::isnan(final_value) || !std::isnan(final_dual))
        std::println(stderr,
                     "   last sample vs final: primal {} dual {} (final {:.7g} "
                     "/ {:.7g})",
                     same(z.primal, final_value) ? "match" : "DIFFER",
                     same(z.dual, final_dual) ? "match" : "DIFFER", final_value,
                     final_dual);
}

// Chu-Beasley style multidimensional knapsack: incumbents and bound both move.
struct mkp {
    int rows, cols;
    std::vector<double> c, b;
    std::vector<std::vector<double>> a;
};
inline mkp make_mkp(int rows, int cols, unsigned seed = 7) {
    mkp I{rows, cols, std::vector<double>(cols), std::vector<double>(rows),
          std::vector<std::vector<double>>(rows, std::vector<double>(cols))};
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> A(1, 1000), R(0, 500);
    for(auto & row : I.a)
        for(auto & v : row) v = A(rng);
    for(int j = 0; j < cols; ++j) {
        double s = 0;
        for(int i = 0; i < rows; ++i) s += I.a[i][j];
        I.c[j] = std::floor(s / rows + R(rng));
    }
    for(int i = 0; i < rows; ++i) {
        double s = 0;
        for(int j = 0; j < cols; ++j) s += I.a[i][j];
        I.b[i] = std::floor(0.5 * s);
    }
    return I;
}
// sense +1: min -c.x + offset; sense -1: max c.x + offset. Same optimum shifted.
template <class M>
void build_mkp(M & m, const mkp & I, int sense, double offset) {
    auto x = m.add_binary_variables(static_cast<std::size_t>(I.cols));
    auto cols = std::views::iota(std::size_t{0},
                                 static_cast<std::size_t>(I.cols));
    if(sense > 0) {
        m.set_minimization();
        m.set_objective(
            xsum(cols, [&](auto j) { return (-I.c[j]) * x(j); }) + offset);
    } else {
        m.set_maximization();
        m.set_objective(xsum(cols, [&](auto j) { return I.c[j] * x(j); }) +
                        offset);
    }
    for(int i = 0; i < I.rows; ++i) {
        const auto & row = I.a[static_cast<std::size_t>(i)];
        m.add_constraint(xsum(cols, [&](auto j) { return row[j] * x(j); }) <=
                         I.b[static_cast<std::size_t>(i)]);
    }
}
// Market split without slacks: no solution for the heuristics to find, so the
// hook has to say "no incumbent yet" for the whole run.
template <class M>
void build_no_incumbent(M & m, std::size_t rows) {
    const std::size_t n = 10 * (rows - 1);
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> coef(0, 99);
    auto x = m.add_binary_variables(n);
    m.set_minimization();
    auto cols = std::views::iota(std::size_t{0}, n);
    m.set_objective(xsum(cols, [&](auto j) { return 1.0 * x(j); }));
    std::vector<int> row(n);
    for(std::size_t i = 0; i < rows; ++i) {
        int total = 0;
        for(int & a : row) total += (a = coef(rng));
        m.add_constraint(xsum(cols, [&](auto j) { return row[j] * x(j); }) ==
                         total / 2);
    }
}
// Integer-free model on a *_milp, or the LP probe's instance.
template <class M>
void build_lp(M & m, int rows, int cols, unsigned seed = 11) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> C(1, 10), A(0.5, 5), B(20, 80);
    std::uniform_int_distribution<int> pick(0, cols - 1);
    auto x = m.add_variables(static_cast<std::size_t>(cols),
                             {.lower_bound = 0, .upper_bound = 10});
    m.set_maximization();
    std::vector<double> obj(static_cast<std::size_t>(cols));
    for(auto & v : obj) v = C(rng);
    auto all = std::views::iota(std::size_t{0},
                                static_cast<std::size_t>(cols));
    m.set_objective(xsum(all, [&](auto j) { return obj[j] * x(j); }));
    for(int i = 0; i < rows; ++i) {
        std::vector<std::pair<std::size_t, double>> nz;
        std::set<std::size_t> seen;
        for(int k = 0; k < 8; ++k) {
            auto j = static_cast<std::size_t>(pick(rng));
            if(seen.insert(j).second) nz.emplace_back(j, A(rng));
        }
        auto idx = std::views::iota(std::size_t{0}, nz.size());
        m.add_constraint(
            xsum(idx, [&](auto k) { return nz[k].second * x(nz[k].first); }) <=
            B(rng));
    }
}

template <class M>
struct backend {
    std::string name;
    std::function<void(M &, recorder &)> attach;
    std::function<void(M &)> one_thread;
    std::function<void(M &)> detach = [](M &) {};
    int large_rows = 30, large_cols = 100;
    int small_rows = 10, small_cols = 60;
    double limit_s = 10;
};

template <class M>
struct run_out {
    double solve_s, value, dual;
};
template <class M>
run_out<M> timed_solve(M & m) {
    auto t = clk::now();
    m.solve();
    double s = std::chrono::duration<double>(clk::now() - t).count();
    double v = NAN, d = NAN;
    try { v = m.get_solution_value(); } catch(...) {}
    if constexpr(has_dual_bound<M>) {
        try { d = m.get_dual_bound(); } catch(...) {}
    }
    return {s, v, d};
}

// The MILP matrix.
template <class M>
void run_milp_matrix(const backend<M> & b) {
    using std::chrono::duration;
    const mkp small = make_mkp(b.small_rows, b.small_cols);
    const mkp large = make_mkp(b.large_rows, b.large_cols);
    const bool quick = std::getenv("PROBE_QUICK") != nullptr;
    const auto limit = duration<double>(quick ? 3.0 : b.limit_s);
    const auto new_model = [] {
        return M();
    };

    long long base_nodes = -1;
    if(!quick) {   // A
        auto m = new_model();
        b.one_thread(m);
        build_mkp(m, small, +1, 100.0);
        m.set_time_limit(duration<double>(60));
        auto o = timed_solve(m);
        final_line("A small min 1 thread, no hook (baseline)", m, o.solve_s);
        if constexpr(has_node_count<M>)
            base_nodes = static_cast<long long>(m.get_node_count());
    }
    if(!quick) {   // B
        auto m = new_model();
        b.one_thread(m);
        build_mkp(m, small, +1, 100.0);
        m.set_time_limit(duration<double>(60));
        recorder r;
        b.attach(m, r);
        r.start();
        auto o = timed_solve(m);
        final_line("B small min 1 thread, hook", m, o.solve_s);
        hook_report(r, +1, o.solve_s, o.value, o.dual);
        if constexpr(has_node_count<M>)
            std::println(stderr, "   search path: nodes {} vs baseline {} {}",
                         m.get_node_count(), base_nodes,
                         (long long)m.get_node_count() == base_nodes
                             ? "(same)" : "(DIFFERENT)");
    }
    if constexpr(has_candidate_solution_callback<M>) if(!quick) {  // C
        auto m = new_model();
        b.one_thread(m);
        build_mkp(m, small, +1, 100.0);
        m.set_time_limit(duration<double>(60));
        long calls = 0;
        m.set_candidate_solution_callback([&](auto & h) {
            ++calls;
            (void)h.get_solution_value();
        });
        auto o = timed_solve(m);
        final_line("C small min 1 thread, no-op candidate callback", m,
                   o.solve_s);
        std::println(stderr,
                     "   candidate calls={}, nodes {} vs baseline {} {}", calls,
                     m.get_node_count(), base_nodes,
                     (long long)m.get_node_count() == base_nodes
                         ? "(same)" : "(DIFFERENT)");
    }
    {   // D
        auto m = new_model();
        build_mkp(m, large, +1, 100.0);
        m.set_time_limit(limit);
        recorder r;
        b.attach(m, r);
        r.start();
        auto o = timed_solve(m);
        final_line("D large min default threads quiet, hook", m, o.solve_s);
        hook_report(r, +1, o.solve_s, o.value, o.dual);
    }
    if(!quick) {   // E
        auto m = new_model();
        build_mkp(m, large, -1, 100.0);
        m.set_time_limit(limit);
        recorder r;
        b.attach(m, r);
        r.start();
        auto o = timed_solve(m);
        final_line("E large max+offset default threads quiet, hook", m,
                   o.solve_s);
        hook_report(r, -1, o.solve_s, o.value, o.dual);
    }
    if(!quick) {   // F
        auto m = new_model();
        build_mkp(m, large, +1, 100.0);
        m.set_time_limit(limit);
        m.set_verbose(true);
        recorder r;
        b.attach(m, r);
        r.start();
        auto o = timed_solve(m);
        final_line("F large min default threads VERBOSE, hook", m, o.solve_s);
        hook_report(r, +1, o.solve_s, o.value, o.dual);
    }
    if(!quick) {   // G
        auto m = new_model();
        build_no_incumbent(m, 7);
        m.set_time_limit(duration<double>(5));
        recorder r;
        b.attach(m, r);
        r.start();
        auto o = timed_solve(m);
        final_line("G no incumbent, 5s, hook", m, o.solve_s);
        hook_report(r, +1, o.solve_s, o.value, o.dual);
    }
    {   // H
        auto m = new_model();
        build_mkp(m, large, +1, 100.0);
        m.set_time_limit(limit);
        recorder r;
        r.stop_after = 3;
        b.attach(m, r);
        r.start();
        auto o = timed_solve(m);
        auto end = clk::now();
        final_line("H large, stop asked at the 3rd sample", m, o.solve_s);
        hook_report(r, +1, o.solve_s);
        if(r.stop_asked)
            std::println(stderr, "   stop asked after {:.3f}s, solve returned {:.3f}s later",
                         duration<double>(r.stop_time - r.t0).count(),
                         duration<double>(end - r.stop_time).count());
        else
            std::println(stderr, "   stop never asked");
        // does the model still work after an interrupted solve?
        r.stop_after = 0;
        m.set_time_limit(duration<double>(2));
        auto o2 = timed_solve(m);
        final_line("H2 resolve after the stop, 2s", m, o2.solve_s);
    }
    {   // I
        auto m = new_model();
        build_lp(m, 400, 600);
        m.set_time_limit(duration<double>(20));
        recorder r;
        b.attach(m, r);
        r.start();
        try {
            auto o = timed_solve(m);
            final_line("I integer-free model on the MILP type, hook", m,
                       o.solve_s);
            hook_report(r, -1, o.solve_s);
        } catch(const std::exception & e) {
            std::println(stderr, "== I integer-free on the MILP type: solve() threw: {}", e.what());
        }
    }
}

// Time slices through the MIP++ API only. Same budget in one piece and cut into
// slices: a solver that resumes ends close to the one-piece run, with a node
// count that keeps growing.
template <class M>
void run_slices(int rows, int cols, void (*one_thread)(M &) = nullptr) {
    using std::chrono::duration;
    const mkp large = make_mkp(rows, cols);
    const int n = 8;
    {
        M m;
        if(one_thread) one_thread(m);
        build_mkp(m, large, +1, 100.0);
        m.set_time_limit(duration<double>(n));
        auto o = timed_solve(m);
        final_line(std::format("S0 one piece {}s", n), m, o.solve_s);
    }
    M m;
    if(one_thread) one_thread(m);
    build_mkp(m, large, +1, 100.0);
    m.set_time_limit(duration<double>(1));
    double total = 0;
    for(int i = 0; i < n; ++i) {
        auto o = timed_solve(m);
        total += o.solve_s;
        final_line(std::format("S{} slice 1s (cumulated {:.2f}s)", i + 1, total),
                   m, o.solve_s);
    }
}

// LP time slices through the MIP++ API, for backends without an LP hook: same
// LP in one piece and cut into slices. A resuming solver adds little time.
template <class L>
void run_lp_slices(int rows, int cols, double slice_s, int max_slices = 200) {
    using std::chrono::duration;
    {
        L m; build_lp(m, rows, cols);
        m.set_time_limit(duration<double>(120));
        auto o = timed_solve(m);
        final_line("LS0 LP one piece", m, o.solve_s);
    }
    L m; build_lp(m, rows, cols);
    m.set_time_limit(duration<double>(slice_s));
    double total = 0; int n = 0; std::string seq;
    for(; n < max_slices; ++n) {
        auto o = timed_solve(m);
        total += o.solve_s;
        const auto s = status_name(m.get_status());
        if(n < 3) seq += s + " ";
        if(s.starts_with("optimal")) { ++n; break; }
    }
    std::println(stderr, "== LS1 LP in {}s slices: {} slices, {:.2f}s, first statuses: {}last status={} value={:.7g}",
                 slice_s, n, total, seq, status_name(m.get_status()), m.get_solution_value());
}
