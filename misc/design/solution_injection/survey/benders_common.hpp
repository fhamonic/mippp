// Benders probe: uncapacitated facility location, one epigraph variable per
// customer, optimality cuts added as lazy constraints from MIP++'s candidate
// callback. Measures what the solver loses when a candidate is rejected although
// its integer part is feasible, and what a native solution injection recovers.
//
// Build with -fno-access-control: the injection reaches the handle's native
// callback data, which the wrapper keeps private.
#pragma once
#include <dlfcn.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <map>
#include <mutex>
#include <print>
#include <random>
#include <set>
#include <string>
#include <vector>

namespace bp {

inline void * sym(const std::filesystem::path & lib, const char * name) {
    void * h = dlopen(lib.c_str(), RTLD_NOW | RTLD_GLOBAL);
    if(!h) {
        std::println(stderr, "dlopen failed: {}", dlerror());
        std::exit(3);
    }
    void * f = dlsym(h, name);
    if(!f) {
        std::println(stderr, "dlsym {} failed", name);
        std::exit(3);
    }
    return f;
}
template <class F>
F symf(const std::filesystem::path & lib, const char * name) {
    return reinterpret_cast<F>(sym(lib, name));
}

struct instance {
    int n, m;
    int variant = 0;      // 0: UFL, one epigraph per customer; 1: budget on
                          // the opening costs; 2: budget, one epigraph in all
    double budget = 0;
    std::vector<double> f;
    std::vector<std::vector<double>> c;  // c[i][j]
};

inline instance make(int n, int m, unsigned seed, double fscale) {
    std::mt19937 gen(seed);
    std::uniform_real_distribution<double> u(0.0, 1.0);
    instance I;
    I.n = n;
    I.m = m;
    I.f.assign(n, 0.0);
    I.c.assign(n, std::vector<double>(m));
    std::vector<std::pair<double, double>> pf(n), pc(m);
    for(auto & p : pf) p = {u(gen), u(gen)};
    for(auto & p : pc) p = {u(gen), u(gen)};
    for(int i = 0; i < n; ++i) I.f[i] = std::round(fscale * (0.5 + u(gen)));
    for(int i = 0; i < n; ++i)
        for(int j = 0; j < m; ++j)
            I.c[i][j] = std::round(100.0 * std::hypot(pf[i].first - pc[j].first,
                                                      pf[i].second - pc[j].second));
    return I;
}

enum class mode { none, full, partial, lie };
inline double last_obj = 0;  // what the native injection call returned as value
inline bool trace = std::getenv("TRACE") != nullptr;

struct event {
    double t;
    double true_obj;
    bool accepted;
};

struct result {
    std::string status;
    double objective = std::numeric_limits<double>::quiet_NaN();
    double seconds = 0;
    long calls = 0, rejected = 0, accepted = 0, cuts = 0;
    long lost_improving = 0;  // rejected, yet truly better than the incumbent
    long refound = 0;         // rejected earlier, accepted later, never injected
    long inj = 0, inj_rc_nonzero = 0, inj_accepted = 0, inj_rejected_again = 0;
    long inj_point_rejected = 0;  // the injected point itself came back cut
    long marked_calls = 0, marked_acc = 0, marked_rej = 0, lies = 0;
    long refound_next = 0;  // accepted in the call right after its rejection
    // incumbents the solver reported that carry the injection mark although
    // the candidate callback never accepted a point with that value
    long unchecked_incumbents = 0;
    std::vector<std::pair<double, double>> incumbents;  // (time, value)
    long nodes = -1;
    double bound = std::numeric_limits<double>::quiet_NaN();
    int first_rc_nonzero = 0;
    double t_first_acc = -1;
    std::vector<event> events;
    std::map<std::string, double> extra;
};

// inject(handle, x, partial) -> native return code. x has one value per column;
// in partial mode the epigraph entries are NaN.
template <class M, class Inject, class Setup>
result run(const instance & I, mode md, Inject && inject, Setup && setup,
           double time_limit) {
    using namespace mippp;
    using namespace mippp::operators;
    using clk = std::chrono::steady_clock;
    result R;
    M model;
    auto Y = model.add_binary_variables(static_cast<std::size_t>(I.n));
    const int ne = I.variant == 2 ? 1 : I.m;
    auto E = model.add_variables(static_cast<std::size_t>(ne));
    model.set_minimization();
    if(I.variant == 0)
        model.set_objective(
            xsum(std::views::iota(0, I.n),
                 [&](int i) { return I.f[i] * Y[i]; }) +
            xsum(std::views::iota(0, ne), [&](int j) { return 1.0 * E[j]; }));
    else {
        model.set_objective(
            xsum(std::views::iota(0, ne), [&](int j) { return 1.0 * E[j]; }));
        model.add_constraint(xsum(std::views::iota(0, I.n), [&](int i) {
                                 return I.f[i] * Y[i];
                             }) <= I.budget);
    }
    model.add_constraint(xsum(std::views::iota(0, I.n), [&](int i) {
                             return 1.0 * Y[i];
                         }) >= 1);
    model.set_time_limit(std::chrono::duration<double>(time_limit));
    setup(model);

    std::mutex mtx;
    std::set<std::vector<char>> rejected_keys, inj_done;
    std::map<std::vector<char>, std::vector<double>> injected_keys;
    double best_acc = std::numeric_limits<double>::infinity();
    clk::time_point t0;
    std::vector<char> prev_rejected;
    std::set<long> accepted_marked;  // values, in 1e-6 units
    std::mutex pmtx;
    constexpr double mark = 0.123456789;
    model.set_progress_callback([&](auto & handle) {
        const std::scoped_lock lock(pmtx);
        const double v = handle.get_solution_value();
        if(!std::isfinite(v)) return;
        if(!R.incumbents.empty() && R.incumbents.back().second == v) return;
        R.incumbents.emplace_back(
            std::chrono::duration<double>(clk::now() - t0).count(), v);
    });

    model.set_candidate_solution_callback([&](auto & handle) {
        const std::scoped_lock lock(mtx);
        const double t =
            std::chrono::duration<double>(clk::now() - t0).count();
        ++R.calls;
        auto sol = handle.get_solution();
        std::vector<char> open(I.n);
        double true_obj = 0;
        for(int i = 0; i < I.n; ++i) {
            open[i] = sol[Y[i]] > 0.5;
            if(open[i]) true_obj += I.f[i];
        }
        std::vector<double> x(I.n + ne);
        // Costs are integers, so only an injected point carries this fraction
        // in its epigraph value (variant 2).
        const bool marked =
            I.variant == 2 &&
            std::fabs(sol[E[0]] - std::floor(sol[E[0]]) - mark) < 1e-7;
        if(marked) ++R.marked_calls;
        for(int i = 0; i < I.n; ++i) x[Y[i].id()] = open[i] ? 1.0 : 0.0;
        if(I.variant != 0) true_obj = 0;
        bool any_cut = false;
        std::vector<double> tjs(I.m);
        for(int j = 0; j < I.m; ++j) {
            double tj = std::numeric_limits<double>::infinity();
            for(int i = 0; i < I.n; ++i)
                if(open[i]) tj = std::min(tj, I.c[i][j]);
            tjs[j] = tj;
            true_obj += tj;
        }
        if(I.variant == 2) {
            double total = 0;
            for(double tj : tjs) total += tj;
            x[E[0].id()] = total + mark;
            if(sol[E[0]] < total - 1e-6 * (1 + std::fabs(total))) {
                any_cut = true;
                ++R.cuts;
                std::vector<double> coef(I.n, 0.0);
                for(int j = 0; j < I.m; ++j)
                    for(int i = 0; i < I.n; ++i)
                        if(I.c[i][j] < tjs[j]) coef[i] += tjs[j] - I.c[i][j];
                handle.add_lazy_constraint(
                    E[0] + xsum(std::views::iota(0, I.n),
                                [&](int i) { return coef[i] * Y[i]; }) >=
                    total);
            }
        } else
            for(int j = 0; j < I.m; ++j) {
                const double tj = tjs[j];
                x[E[j].id()] = tj;
                if(sol[E[j]] >= tj - 1e-6 * (1 + std::fabs(tj))) continue;
                any_cut = true;
                ++R.cuts;
                handle.add_lazy_constraint(
                    E[j] + xsum(std::views::iota(0, I.n) |
                                    std::views::filter([&, tj, j](int i) {
                                        return I.c[i][j] < tj;
                                    }),
                                [&, tj, j](int i) {
                                    return (tj - I.c[i][j]) * Y[i];
                                }) >=
                    tj);
            }
        R.events.push_back({t, true_obj, !any_cut});
        if(trace)
            std::println(stderr, "t={:.3f} {} cand_value={:.4f} true={:.4f} best_acc={:.4f}",
                         t, any_cut ? "REJ" : "ACC", handle.get_solution_value(),
                         true_obj, best_acc);
        if(marked) ++(any_cut ? R.marked_rej : R.marked_acc);
        if(!any_cut) {
            if(open == prev_rejected) ++R.refound_next;
            prev_rejected.clear();
            if(marked) accepted_marked.insert(std::lround(sol[E[0]] * 1e6));
            ++R.accepted;
            if(R.t_first_acc < 0) R.t_first_acc = t;
            best_acc = std::min(best_acc, true_obj);
            if(injected_keys.contains(open)) {
                if(inj_done.insert(open).second) ++R.inj_accepted;
            } else if(rejected_keys.contains(open))
                ++R.refound;
            return;
        }
        ++R.rejected;
        prev_rejected = open;
        if(true_obj < best_acc - 1e-7) ++R.lost_improving;
        if(auto it = injected_keys.find(open); it != injected_keys.end()) {
            ++R.inj_rejected_again;
            bool same = !it->second.empty();
            for(int j = 0; same && j < ne; ++j)
                same = std::fabs(sol[E[j]] - it->second[E[j].id()]) <= 1e-7;
            if(same) ++R.inj_point_rejected;
            return;
        }
        rejected_keys.insert(open);
        if(md == mode::none) return;
        // only what would improve the incumbent the callback knows of
        if(md != mode::lie && true_obj >= best_acc - 1e-7) return;
        if(md == mode::lie) {
            // a neighbour the callback never saw, one unit below its true
            // value: only a cut not separated yet forbids it
            int nopen = 0, drop = -1;
            for(int i = 0; i < I.n; ++i)
                if(open[i]) { ++nopen; drop = i; }
            if(nopen < 2 || I.variant != 2) return;
            std::vector<char> nb = open;
            nb[drop] = 0;
            double total = 0;
            for(int j = 0; j < I.m; ++j) {
                double tj = std::numeric_limits<double>::infinity();
                for(int i = 0; i < I.n; ++i)
                    if(nb[i]) tj = std::min(tj, I.c[i][j]);
                total += tj;
            }
            x[Y[drop].id()] = 0.0;
            // BIG_LIE: half the true value, which rows the solver already
            // holds may forbid; otherwise one unit below the truth
            const double lie = std::getenv("BIG_LIE")
                                   ? std::floor(0.5 * total) + mark
                                   : total - 1.0 + mark;
            x[E[0].id()] = lie;
            ++R.lies;
            const int rc = inject(handle, x, false, lie);
            ++R.inj;
            if(rc != 0) ++R.inj_rc_nonzero;
            return;
        }
        if(md == mode::partial) {
            for(int j = 0; j < ne; ++j)
                x[E[j].id()] = std::numeric_limits<double>::quiet_NaN();
            injected_keys.emplace(open, std::vector<double>{});
        } else
            injected_keys.emplace(open, x);
        const int rc = inject(handle, x, md == mode::partial,
                              true_obj + (I.variant == 2 ? mark : 0.0));
        ++R.inj;
        if(trace)
            std::println(stderr, "    inject rc={} returned_obj={:.6g}", rc, last_obj);
        if(rc != 0) {
            ++R.inj_rc_nonzero;
            if(R.first_rc_nonzero == 0) R.first_rc_nonzero = rc;
        }
    });

    t0 = clk::now();
    model.solve();
    R.seconds = std::chrono::duration<double>(clk::now() - t0).count();
    const auto & st = model.get_status();
    R.status = std::visit(
        [](const auto & s) {
            std::string n = typeid(s).name();
            const auto p = n.find("status");
            return p == std::string::npos ? n : n.substr(p + 7);
        },
        st);
    if(status::solution_available(st)) R.objective = model.get_solution_value();
    if(I.variant == 2) {
        std::set<long> seen;
        for(const auto & [t, v] : R.incumbents) {
            if(std::fabs(v - std::floor(v) - mark) >= 1e-6) continue;
            const long key = std::lround(v * 1e6);
            if(!accepted_marked.contains(key) && seen.insert(key).second)
                ++R.unchecked_incumbents;
        }
    }
    R.nodes = static_cast<long>(model.get_node_count());
    R.bound = model.get_dual_bound();
    return R;
}

inline const char * mode_name(mode m) {
    return m == mode::none   ? "none"
           : m == mode::full ? "full"
           : m == mode::lie  ? "lie"
                             : "partial";
}

inline void report(const char * backend, const instance & I, unsigned seed,
                   int threads, mode md, const result & R, double ref) {
    // time at which a point as good as the final optimum was first seen by
    // the callback, and the time it was first accepted
    double t_seen = -1, t_acc = -1, t_1pct = -1, ti_1pct = -1, ti_opt = -1;
    for(const auto & [t, v] : R.incumbents) {
        const double tgt = std::isnan(ref) ? R.objective : ref;
        if(ti_1pct < 0 && v <= tgt * 1.01) ti_1pct = t;
        if(ti_opt < 0 && v <= tgt + 0.2) ti_opt = t;
    }
    const double target = std::isnan(ref) ? R.objective : ref;
    for(const auto & e : R.events) {
        if(e.true_obj <= target + 0.01) {
            if(t_seen < 0) t_seen = e.t;
            if(e.accepted && t_acc < 0) t_acc = e.t;
        }
        if(e.accepted && t_1pct < 0 && e.true_obj <= target * 1.01) t_1pct = e.t;
    }
    std::println(
        "{} v={} n={} m={} seed={} thr={} mode={} status={} obj={:.4f} time={:.2f} "
        "calls={} rej={} acc={} cuts={} lost_improving={} refound={} inj={} "
        "inj_rc!=0={} rc={} inj_acc={} inj_rej_again={} inj_pt_rej={} "
        "t_first_acc={:.2f} t_1pct_acc={:.2f} t_opt_seen={:.2f} t_opt_acc={:.2f} "
        "nodes={} bound={:.4f} marked_calls={} marked_acc={} marked_rej={} lies={} "
        "refound_next={} unchecked_inc={} n_inc={} ti_1pct={:.2f} ti_opt={:.2f}",
        backend, I.variant, I.n, I.m, seed, threads, mode_name(md), R.status, R.objective,
        R.seconds, R.calls, R.rejected, R.accepted, R.cuts, R.lost_improving,
        R.refound, R.inj, R.inj_rc_nonzero, R.first_rc_nonzero, R.inj_accepted,
        R.inj_rejected_again, R.inj_point_rejected, R.t_first_acc, t_1pct,
        t_seen, t_acc, R.nodes, R.bound, R.marked_calls, R.marked_acc,
        R.marked_rej, R.lies, R.refound_next, R.unchecked_incumbents,
        R.incumbents.size(), ti_1pct, ti_opt);
    std::fflush(stdout);
}

struct args {
    int n = 40, m = 150, threads = 1;
    unsigned seed0 = 1, seeds = 3;
    double fscale = 200, limit = 60, budget = 0.1;
    int variant = 0;
    std::vector<mode> modes{mode::none, mode::full, mode::partial};
};
inline args parse(int argc, char ** argv) {
    args a;
    for(int k = 1; k < argc; ++k) {
        std::string s = argv[k];
        auto val = [&](const char * key) -> const char * {
            const std::string p = std::string(key) + "=";
            return s.rfind(p, 0) == 0 ? s.c_str() + p.size() : nullptr;
        };
        if(auto v = val("n")) a.n = std::atoi(v);
        else if(auto v = val("m")) a.m = std::atoi(v);
        else if(auto v = val("thr")) a.threads = std::atoi(v);
        else if(auto v = val("seed0")) a.seed0 = std::atoi(v);
        else if(auto v = val("seeds")) a.seeds = std::atoi(v);
        else if(auto v = val("fscale")) a.fscale = std::atof(v);
        else if(auto v = val("budget")) a.budget = std::atof(v);
        else if(auto v = val("variant")) a.variant = std::atoi(v);
        else if(auto v = val("limit")) a.limit = std::atof(v);
        else if(auto v = val("modes")) {
            a.modes.clear();
            std::string l = v;
            if(l.find("none") != std::string::npos) a.modes.push_back(mode::none);
            if(l.find("full") != std::string::npos) a.modes.push_back(mode::full);
            if(l.find("partial") != std::string::npos)
                a.modes.push_back(mode::partial);
            if(l.find("lie") != std::string::npos) a.modes.push_back(mode::lie);
        } else {
            std::println(stderr, "unknown argument {}", s);
            std::exit(2);
        }
    }
    return a;
}

template <class M, class Inject, class Setup>
int main_loop(const char * backend, int argc, char ** argv, Inject && inject,
              Setup && setup) {
    const args a = parse(argc, argv);
    for(unsigned seed = a.seed0; seed < a.seed0 + a.seeds; ++seed) {
        instance I = make(a.n, a.m, seed, a.fscale);
        I.variant = a.variant;
        double sumf = 0;
        for(double f : I.f) sumf += f;
        I.budget = a.budget * sumf;
        double ref = std::numeric_limits<double>::quiet_NaN();
        for(mode md : a.modes) {
            result R = run<M>(I, md, inject,
                              [&](M & m) { setup(m, a.threads); }, a.limit);
            if(std::isnan(ref)) ref = R.objective;
            report(backend, I, seed, a.threads, md, R, ref);
        }
    }
    return 0;
}

}  // namespace bp
