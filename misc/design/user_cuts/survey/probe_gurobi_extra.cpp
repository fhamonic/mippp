// Experiments the shared matrix does not run, selected by argv[1]. Each solve has a
// time limit and each run goes under `timeout -k` from the shell. Native error codes
// are printed, nothing throws across the solver's C frames.
#include <sys/resource.h>

#include <numeric>

#include "probe_gurobi_common.hpp"

using std::chrono::duration;
using std::size_t;

static double dattr(M & m, const char * name) {
    double v = NAN;
    auto [env, model] = m.native_model();
    if(m.native_api().getdblattr(model, name, &v)) return NAN;
    return v;
}
static std::string run_facts(M & m) {
    return std::format("iters={} work={:.4f}", dattr(m, "IterCount"), dattr(m, "Work"));
}
static std::string presolved_dims(M & m) {
    auto [env, model] = m.native_model();
    auto & api = m.native_api();
    auto presolve = cp::symf<int (*)(grb::GRBmodel *, grb::GRBmodel **)>(api.library_path(), "GRBpresolvemodel");
    api.updatemodel(model);
    grb::GRBmodel * p = nullptr;
    const int e = presolve(model, &p);
    if(e || !p) return std::format("presolve error {}", e);
    int r = -1, c = -1, nz = -1;
    api.getintattr(p, "NumConstrs", &r);
    api.getintattr(p, "NumVars", &c);
    api.getintattr(p, "NumNZs", &nz);
    api.freemodel(p);
    return std::format("presolved {}r {}c {}nz", r, c, nz);
}
static std::string key_of(const cp::raw_cut & c) {
    std::string k;
    for(std::size_t i = 0; i < c.ind.size(); ++i) k += std::format("{}:{:.6g},", c.ind[i], c.val[i]);
    return k + std::format("{}{:.6g}", c.sense, c.rhs);
}

// ------------------------------------------------------------------ x_tree ---
// cp::atsp::separate keeps the source side in a 64-bit mask: beyond n=64 the shift
// wraps and the set decoded is not the one found (the cut stays valid, it is just
// not the violated one). This copy has no such limit; identical logic otherwise.
static std::vector<cp::raw_cut> separate_any(const cp::atsp & a, const std::vector<double> & x, const std::size_t max_cuts) {
    const auto N = static_cast<std::size_t>(a.n);
    std::map<std::vector<char>, double> found;
    std::vector<std::vector<double>> cap0(N, std::vector<double>(N, 0));
    for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j)
        cap0[i][j] = std::max(0.0, x[static_cast<std::size_t>(a.col[i][j])]);
    const auto flow = [&](std::size_t s, std::size_t t) {
        auto cap = cap0;
        double total = 0;
        std::vector<int> pred(N);
        while(true) {
            std::fill(pred.begin(), pred.end(), -1);
            pred[s] = static_cast<int>(s);
            std::vector<std::size_t> q{s};
            for(std::size_t h = 0; h < q.size() && pred[t] < 0; ++h)
                for(std::size_t v = 0; v < N; ++v)
                    if(pred[v] < 0 && cap[q[h]][v] > 1e-9) { pred[v] = static_cast<int>(q[h]); q.push_back(v); }
            if(pred[t] < 0) break;
            double f = 1e9;
            for(std::size_t v = t; v != s; v = static_cast<std::size_t>(pred[v])) f = std::min(f, cap[static_cast<std::size_t>(pred[v])][v]);
            for(std::size_t v = t; v != s; v = static_cast<std::size_t>(pred[v])) { cap[static_cast<std::size_t>(pred[v])][v] -= f; cap[v][static_cast<std::size_t>(pred[v])] += f; }
            total += f;
            if(total >= 1) return;
        }
        if(total < 1 - 1e-4) {
            std::vector<char> mask(N, 0);
            for(std::size_t v = 0; v < N; ++v) mask[v] = pred[v] >= 0;
            auto & best = found[mask];
            best = std::max(best, 1 - total);
        }
    };
    for(std::size_t t = 1; t < N; ++t) { flow(0, t); flow(t, 0); }
    std::vector<std::pair<double, std::vector<char>>> order;
    for(auto & [mask, v] : found) order.emplace_back(v, mask);
    std::ranges::sort(order, std::greater<>{});
    std::vector<cp::raw_cut> out;
    for(auto & [v, mask] : order) {
        if(out.size() == max_cuts) break;
        cp::raw_cut c;
        c.sense = 'G';
        for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j)
            if(i != j && mask[i] && !mask[j]) { c.ind.push_back(a.col[i][j]); c.val.push_back(1); }
        c.rhs = 1.0;
        out.push_back(std::move(c));
    }
    return out;
}

// DFJ cuts on a larger MTZ ATSP: do cuts run inside a real tree, how many calls per
// node, and is a cut still obeyed later? A cut that a later relaxation violates, or
// that the separator finds again, was filtered, purged or not in that node's LP.
static void x_tree(const int n, const unsigned seed, const size_t max_cuts, const bool skip_root,
                   const std::string & mode, const double limit, const bool log, const int series = 0) {
    M m; one_thread(m);
    if(log) { set_int(m, "OutputFlag", 1); std::println("#### x_tree n={} seed={} max_cuts={} skip_root={} mode={}", n, seed, max_cuts, skip_root, mode); std::fflush(stdout); }
    auto a = cp::build_atsp(m, n, false, seed);
    m.set_time_limit(duration<double>(limit));
    auto s = std::make_shared<cp::stats>();
    auto nodcnt = std::make_shared<double>(0);
    std::vector<cp::raw_cut> added;
    std::vector<char> obeyed_once;
    std::map<std::string, std::size_t> seen;
    std::set<std::size_t> ever;
    long calls_with = 0, total_viol = 0, max_viol = 0, readded = 0, readded_root = 0, readded_tree = 0,
         viol_after_obeyed = 0, viol_never_obeyed = 0, nonroot_viol = 0, root_viol = 0, ncall = 0;
    double max_excess = 0;
    auto sep = cp::watched([&](const auto & x, const cp::add_fn & add) {
        long viol = 0;
        const std::size_t before = added.size();
        for(std::size_t k = 0; k < before; ++k) {
            const double v = added[k].violation(x);
            if(v > 1e-5) {
                ++viol; ever.insert(k); max_excess = std::max(max_excess, v);
                (obeyed_once[k] ? viol_after_obeyed : viol_never_obeyed)++;
                (*nodcnt == 0 ? root_viol : nonroot_viol)++;
            } else obeyed_once[k] = 1;
        }
        if(viol) ++calls_with;
        total_viol += viol;
        max_viol = std::max(max_viol, viol);
        long sep_now = 0, re_now = 0;
        if(!(skip_root && *nodcnt == 0)) {
            for(const auto & c : separate_any(a, x, max_cuts)) {
                ++sep_now;
                if(add(c)) continue;
                const auto k = key_of(c);
                if(seen.contains(k)) { ++re_now; ++readded; (*nodcnt == 0 ? readded_root : readded_tree)++; }
                else { seen[k] = added.size(); added.push_back(c); obeyed_once.push_back(0); }
            }
        }
        if(ncall++ < series) std::println(stderr, "   call {} nodcnt={} obj={:.6g} earlier cuts violated={} of {} separated={} of which seen before={}", ncall - 1, *nodcnt, a.objective(x), viol, before, sep_now, re_now);
    }, s, [&a](const auto & x) { return a.objective(x); }, [&a](const auto & x) { return a.point_ok(x); }, [&a](const auto & x) { return a.integral(x); });
    auto * c = attach_hook(m, sep, mode);
    c->nodcnt_out = nodcnt;
    const double t = cp::timed_solve(m);
    std::println(stderr, "== X_TREE n={} seed={} max_cuts={} skip_root={} mode={}: {} {}\n   {}\n   {}\n   unique_cuts={} re-added(same cut)={} (root {} tree {}) calls_with_an_earlier_cut_violated={} total_violations={} (root {} tree {}) max_at_one_call={} cuts_ever_reviolated={} max_excess={:.3g}; violated after having been obeyed={} violated and never yet obeyed={}",
        n, seed, max_cuts, skip_root, mode, cp::summary(m, t), run_facts(m), s->line(), c->line(), added.size(), readded, readded_root, readded_tree, calls_with, total_viol, root_viol, nonroot_viol, max_viol, ever.size(), max_excess, viol_after_obeyed, viol_never_obeyed);
    c->printed = true;
    if(log) std::fflush(stdout);
}

// ----------------------------------------------------------------- x_cover ---
// A real tree under user cuts: a multidimensional knapsack with violated cover
// inequalities, all valid. Calls per node, cuts per node, and a sampled check of how
// often an earlier cut is violated by a later relaxation (purged, or not in that LP).
static void x_cover(const int rows, const int cols, const double limit, const size_t max_cuts, const bool root_only, const std::string & mode, const int threads) {
    M m;
    m.set_thread_limit(static_cast<size_t>(threads));
    set_int(m, "OutputFlag", 1);
    std::println("#### x_cover {}x{} max_cuts={} root_only={} mode={} threads={}", rows, cols, max_cuts, root_only, mode, threads); std::fflush(stdout);
    std::mt19937 rng(7);
    std::uniform_int_distribution<int> A(1, 1000), R(0, 500);
    const auto C = static_cast<size_t>(cols);
    std::vector<std::vector<double>> a(static_cast<size_t>(rows), std::vector<double>(C));
    std::vector<double> b;
    for(auto & row : a) for(auto & v : row) v = A(rng);
    std::vector<double> cost(C);
    for(size_t j = 0; j < C; ++j) { double sum = 0; for(const auto & row : a) sum += row[j]; cost[j] = std::floor(sum / rows + R(rng)); }
    auto x = m.add_binary_variables(C);
    const auto ids = std::views::iota(size_t{0}, C);
    m.set_maximization();
    m.set_objective(xsum(ids, [&](size_t j) { return cost[j] * x(j); }));
    for(const auto & row : a) {
        double sum = 0;
        for(double v : row) sum += v;
        b.push_back(std::floor(sum / 2));
        m.add_constraint(xsum(ids, [&row, &x](size_t j) { return row[j] * x(j); }) <= b.back());
    }
    std::vector<int> col(C);
    for(size_t j = 0; j < C; ++j) col[j] = m.native_id(x(j));
    m.set_time_limit(duration<double>(limit));
    auto s = std::make_shared<cp::stats>();
    auto nodcnt = std::make_shared<double>(0);
    std::vector<cp::raw_cut> added;
    std::map<std::string, size_t> seen;
    long sep_calls_with_cut = 0, readded = 0, checks = 0, viol = 0, viol_tree = 0, viol_root = 0;
    std::map<long, long> viol_age;
    auto sep = cp::watched([&](const std::vector<double> & xv, const cp::add_fn & add) {
        // sample a window of earlier cuts: the check is quadratic otherwise
        const size_t from = added.size() > 400 ? added.size() - 400 : 0;
        for(size_t k = from; k < added.size(); ++k) {
            ++checks;
            if(added[k].violation(xv) > 1e-5) { ++viol; (*nodcnt == 0 ? viol_root : viol_tree)++; }
        }
        if(root_only && *nodcnt != 0) return;
        size_t n = 0;
        for(const auto & row_idx : std::views::iota(size_t{0}, a.size())) {
            if(n == max_cuts) break;
            const auto & row = a[row_idx];
            std::vector<size_t> order(C);
            std::iota(order.begin(), order.end(), size_t{0});
            std::ranges::sort(order, [&](size_t p, size_t q) { return (1 - xv[static_cast<size_t>(col[p])]) / row[p] < (1 - xv[static_cast<size_t>(col[q])]) / row[q]; });
            std::vector<size_t> cover;
            double w = 0;
            for(size_t j : order) { cover.push_back(j); w += row[j]; if(w > b[row_idx]) break; }
            if(w <= b[row_idx]) continue;
            // minimal: drop items while it stays a cover, the loosest first
            for(size_t k = cover.size(); k-- > 0;) if(w - row[cover[k]] > b[row_idx]) { w -= row[cover[k]]; cover.erase(cover.begin() + static_cast<std::ptrdiff_t>(k)); }
            double lhs = 0;
            for(size_t j : cover) lhs += xv[static_cast<size_t>(col[j])];
            if(lhs <= static_cast<double>(cover.size()) - 1 + 1e-4) continue;
            cp::raw_cut c;
            c.sense = 'L';
            c.rhs = static_cast<double>(cover.size()) - 1;
            std::ranges::sort(cover);
            for(size_t j : cover) { c.ind.push_back(col[j]); c.val.push_back(1.0); }
            if(add(c)) continue;
            ++n;
            const auto k = key_of(c);
            if(seen.contains(k)) ++readded;
            else { seen[k] = added.size(); added.push_back(std::move(c)); }
        }
        sep_calls_with_cut += n > 0;
    }, s);
    auto * c = attach_hook(m, sep, mode);
    c->nodcnt_out = nodcnt;
    const double t = cp::timed_solve(m);
    std::fflush(stdout);
    std::println(stderr, "== X_COVER {}x{} max_cuts={} root_only={} mode={} threads={}: {} {}\n   {}\n   {}\n   unique_cuts={} re-added(same cut)={} calls_with_a_cut={} window checks={} violations={} (root {} tree {})", rows, cols, max_cuts, root_only, mode, threads, cp::summary(m, t), run_facts(m), s->line(), c->line(), added.size(), readded, sep_calls_with_cut, checks, viol, viol_root, viol_tree);
    c->printed = true;
}

// --------------------------------------------------------------- x_rounds ---
// Limits on root rounds: a separator that always has a new cut, against one that
// repeats itself. Does the root ever stop on its own?
static void x_rounds(const std::string & what, const std::string & settings) {
    M m; scope_settings(m);
    if(settings == "cutpasses2") set_int(m, "CutPasses", 2);
    if(settings == "cutpasses0") set_int(m, "CutPasses", 0);
    if(settings == "defaults") { set_int(m, "Cuts", -1); set_dbl(m, "Heuristics", 0.05); set_int(m, "Presolve", -1); }
    if(settings == "cuts_on") set_int(m, "Cuts", -1);
    if(settings == "cuts_on_passes5") { set_int(m, "Cuts", -1); set_int(m, "CutPasses", 5); }
    auto g = cp::build_gadgets(m, 10, false);
    m.set_time_limit(duration<double>(10.0));
    auto s = std::make_shared<cp::stats>();
    auto nodcnt = std::make_shared<double>(0);
    long round = 0;
    double lastobj = NAN;
    auto * c = attach_hook(m, cp::watched([&](const auto & x, const cp::add_fn & add) {
        if(*nodcnt != 0) return;
        ++round;
        lastobj = g.objective(x);
        if(what == "objective_step") add(cp::raw_cut{g.z, std::vector<double>(g.z.size(), 1.0), 'L', g.objective(x) - 1e-3});
        else if(what == "no_progress") add(cp::raw_cut{{g.dummy}, {1.0}, 'L', 0.9 + 1e-6 * static_cast<double>(round)});
        else if(what == "dummy_step") add(cp::raw_cut{{g.dummy}, {1.0}, 'L', 1.0 / (1.0 + static_cast<double>(round))});
    }, s), "cbcut_precrush");
    c->nodcnt_out = nodcnt;
    const double t = cp::timed_solve(m);
    std::println(stderr, "== X_ROUNDS {} settings={}: {} root rounds in the separator={} last root obj={:.6g}\n   {}", what, settings, cp::summary(m, t), round, lastobj, c->line());
    c->printed = true;
}

// ----------------------------------------------------------------- x_trace ---
// Every event on a small tree, in order: re-fire after a cut, after no cut, on
// integral relaxations, MIPNODE against MIPSOL.
struct trace_ctx {
    const grb::gurobi_api * api;
    int cols, left;
    std::function<std::vector<cp::raw_cut>(const std::vector<double> &, double)> sep;
    bool lazy;
    std::function<double(const std::vector<double> &)> objective;
    std::vector<int> intcols;
};
static int trace_hook(grb::GRBmodel *, void * cbdata, int where, void * usr) {
    auto & c = *static_cast<trace_ctx *>(usr);
    if(where == W_MIPSOL) {
        double obj = 0, node = 0;
        c.api->cbget(cbdata, where, SOL_OBJ, &obj);
        c.api->cbget(cbdata, where, SOL_NODCNT, &node);
        if(c.left-- > 0) std::println(stderr, "   MIPSOL  nodcnt={} obj={:.6g}", node, obj);
    } else if(where == W_MIPNODE) {
        int status = 0;
        double node = 0;
        c.api->cbget(cbdata, where, NODE_STATUS, &status);
        c.api->cbget(cbdata, where, NODE_NODCNT, &node);
        std::vector<double> x(static_cast<size_t>(c.cols));
        double obj = NAN;
        bool integral = false;
        int added = 0, rel_err = 0;
        if(status == GRB_OPTIMAL_) {
            rel_err = c.api->cbget(cbdata, where, NODE_REL, x.data());
            obj = c.objective(x);
            integral = true;
            for(int j : c.intcols) if(std::fabs(x[static_cast<size_t>(j)] - std::round(x[static_cast<size_t>(j)])) > 1e-6) integral = false;
            for(const auto & r : c.sep(x, node)) {
                const int n = static_cast<int>(r.ind.size());
                const int e = c.lazy ? c.api->cblazy(cbdata, n, r.ind.data(), r.val.data(), sense_of(r.sense), r.rhs)
                                     : c.api->cbcut(cbdata, n, r.ind.data(), r.val.data(), sense_of(r.sense), r.rhs);
                if(!e) ++added;
            }
        }
        if(c.left-- > 0) std::println(stderr, "   MIPNODE nodcnt={} status={} rel_err={} obj={:.6g} integral={} cuts_added={}", node, status, rel_err, obj, integral, added);
    }
    return 0;
}
static void x_trace(const std::string & mode, const std::string & what, const bool flip, const int k) {
    M m; scope_settings(m);
    auto g = cp::build_gadgets(m, k, flip);
    m.set_time_limit(duration<double>(10.0));
    const auto s = spec_of(mode);
    set_int(m, "PreCrush", s.precrush);
    set_int(m, "LazyConstraints", s.lazyparam);
    trace_ctx c{&m.native_api(), static_cast<int>(m.num_variables()), 60, {}, s.lazy, [&g](const auto & x) { return g.objective(x); }, g.y};
    auto done = std::make_shared<std::set<int>>();
    c.sep = [&, done](const std::vector<double> & x, double) {
        std::vector<cp::raw_cut> out;
        if(what == "hull_all" || what == "hull_one") for(int i = 0; i < k; ++i) {
            auto cut = g.hull_cut(i);
            if(cut.violation(x) > 1e-6) { out.push_back(std::move(cut)); if(what == "hull_one") break; }
        }
        return out;
    };
    auto [env, model] = m.native_model();
    m.native_api()._check(env, m.native_api().setcallbackfunc(model, trace_hook, &c));
    std::println(stderr, "== X_TRACE mode={} what={} flip={} k={}", mode, what, flip, k);
    const double t = cp::timed_solve(m);
    std::println(stderr, "   => {} {}", cp::summary(m, t), run_facts(m));
}

// ------------------------------------------------------- x_reduce: PreCrush ---
// Which presolve reductions make GRBcbcut silently drop a cut? The core is a small
// knapsack so that a search exists, and each reduction gadget adds columns a known
// reduction removes. The cut is deliberately invalid and costs value: value unchanged
// against the no-cut run, and later relaxations violating it, mean it was ignored.
// core[0..3] are items the no-cut optimum of the core selects, so that a cut on them bites.
struct reduction {
    std::string name;
    std::vector<int> ind;
    std::vector<double> val;
    char sense = 'L';
    double rhs = 0;
};
using var_t = model_variable_t<M>;
// parallel_dominance: b1 <= k - 3, where k = b1 + b2 in the no-cut optimum. (k - 3, 3) is
// still optimal, so the cut removes integer feasible points but keeps an optimum.
static double g_dominance_rhs = 0;
static reduction add_gadget(M & m, const std::string & name, const std::vector<var_t> & core) {
    reduction r; r.name = name;
    const auto v = [&](const var_t & x) { return m.native_id(x); };
    const auto row = [&](auto && expr) { return m.add_constraint(std::forward<decltype(expr)>(expr)); };
    if(name == "parallel_continuous_at_bound") {
        // Identical columns worth more than anything: presolve fixes both at their bound.
        auto p1 = m.add_variable({.obj_coef = 200, .lower_bound = 0, .upper_bound = 1});
        auto p2 = m.add_variable({.obj_coef = 200, .lower_bound = 0, .upper_bound = 1});
        row(p1 + p2 + 500.0 * core[0] + 700.0 * core[1] <= 1300.0);
        row(2.0 * p1 + 2.0 * p2 + 300.0 * core[2] + 900.0 * core[3] <= 1300.0);
        r.ind = {v(p1)}; r.val = {1}; r.sense = 'L'; r.rhs = 0.1;
    } else if(name == "parallel_continuous") {
        // Identical columns competing for capacity: presolve can merge them.
        auto p1 = m.add_variable({.obj_coef = 12, .lower_bound = 0, .upper_bound = 100});
        auto p2 = m.add_variable({.obj_coef = 12, .lower_bound = 0, .upper_bound = 100});
        row(10.0 * p1 + 10.0 * p2 + 500.0 * core[0] + 700.0 * core[1] <= 1700.0);
        row(7.0 * p1 + 7.0 * p2 + 300.0 * core[2] + 900.0 * core[3] <= 1700.0);
        r.ind = {v(p1)}; r.val = {1}; r.sense = 'L'; r.rhs = 1;
    } else if(name == "parallel_integer" || name == "parallel_integer_sum") {
        auto b1 = m.add_variable({.obj_coef = 150, .lower_bound = 0, .upper_bound = 3});
        auto b2 = m.add_variable({.obj_coef = 150, .lower_bound = 0, .upper_bound = 3});
        m.set_integer(b1); m.set_integer(b2);
        row(50.0 * b1 + 50.0 * b2 + 500.0 * core[0] + 700.0 * core[1] <= 1500.0);
        row(70.0 * b1 + 70.0 * b2 + 300.0 * core[2] + 900.0 * core[3] <= 1500.0);
        if(name == "parallel_integer") { r.ind = {v(b1)}; r.val = {1}; r.rhs = 1; }
        // the same columns with equal coefficients can be written on the merged one
        else { r.ind = {v(b1), v(b2)}; r.val = {1, 1}; r.rhs = 1; }
        r.sense = 'L';
    } else if(name == "parallel_dominance") {
        // GRB_CAP tightens the rows so that k < 6 and the cut is not trivial
        const double cap = std::getenv("GRB_CAP") ? std::stod(std::getenv("GRB_CAP")) : 1500.0;
        const double bobj = std::getenv("GRB_BOBJ") ? std::stod(std::getenv("GRB_BOBJ")) : 150.0;
        auto b1 = m.add_variable({.obj_coef = bobj, .lower_bound = 0, .upper_bound = 3});
        auto b2 = m.add_variable({.obj_coef = bobj, .lower_bound = 0, .upper_bound = 3});
        m.set_integer(b1); m.set_integer(b2);
        row(50.0 * b1 + 50.0 * b2 + 500.0 * core[0] + 700.0 * core[1] <= cap);
        row(70.0 * b1 + 70.0 * b2 + 300.0 * core[2] + 900.0 * core[3] <= cap);
        r.ind = {v(b1)}; r.val = {1}; r.sense = 'L'; r.rhs = g_dominance_rhs;
    } else if(name == "parallel_symmetry") {
        // b1 <= b2 removes integer feasible points but keeps at least one optimum: the
        // kind of cut the vendor text forbids. value must stay if dominance were safe.
        auto b1 = m.add_variable({.obj_coef = 150, .lower_bound = 0, .upper_bound = 3});
        auto b2 = m.add_variable({.obj_coef = 150, .lower_bound = 0, .upper_bound = 3});
        m.set_integer(b1); m.set_integer(b2);
        row(50.0 * b1 + 50.0 * b2 + 500.0 * core[0] + 700.0 * core[1] <= 1500.0);
        row(70.0 * b1 + 70.0 * b2 + 300.0 * core[2] + 900.0 * core[3] <= 1500.0);
        r.ind = {v(b1), v(b2)}; r.val = {1, -1}; r.sense = 'L'; r.rhs = 0;
    } else if(name == "parallel_binary") {
        auto b1 = m.add_variable({.obj_coef = 150, .lower_bound = 0, .upper_bound = 1});
        auto b2 = m.add_variable({.obj_coef = 150, .lower_bound = 0, .upper_bound = 1});
        m.set_binary(b1); m.set_binary(b2);
        row(50.0 * b1 + 50.0 * b2 + 500.0 * core[0] + 700.0 * core[1] <= 1500.0);
        row(70.0 * b1 + 70.0 * b2 + 300.0 * core[2] + 900.0 * core[3] <= 1500.0);
        r.ind = {v(b1)}; r.val = {1}; r.sense = 'L'; r.rhs = 0;
    } else if(name == "free_singleton") {
        // s only appears here and is implied free: substituted out with its row.
        auto s = m.add_variable({.obj_coef = -40, .lower_bound = -1e4, .upper_bound = 1e4});
        row(s + 3.0 * core[0] + 4.0 * core[1] + 5.0 * core[2] == 20.0);
        r.ind = {v(s)}; r.val = {1}; r.sense = 'G'; r.rhs = 15;  // the three items can no longer all be on
    } else if(name == "dual_fixing") {
        // Max problem, no objective, only in a <= row with a positive coefficient: d = 0
        // loses nothing. d >= 400 can then not be written on a presolved column.
        auto d = m.add_variable({.obj_coef = 0, .lower_bound = 0, .upper_bound = 1000});
        row(d + 500.0 * core[0] + 700.0 * core[1] + 400.0 * core[2] <= 1700.0);
        r.ind = {v(d)}; r.val = {1}; r.sense = 'G'; r.rhs = 400;
    } else if(name == "dominated_column") {
        // c2 costs the same capacity and earns more: c1 stays at its bound.
        auto c1 = m.add_variable({.obj_coef = 5, .lower_bound = 0, .upper_bound = 1e4});
        auto c2 = m.add_variable({.obj_coef = 50, .lower_bound = 0, .upper_bound = 1e4});
        m.set_integer(c1); m.set_integer(c2);
        row(100.0 * c1 + 100.0 * c2 + 500.0 * core[0] + 700.0 * core[1] <= 1500.0);
        r.ind = {v(c1)}; r.val = {1}; r.sense = 'G'; r.rhs = 2;
    } else if(name == "dominated_binary") {
        auto b1 = m.add_variable({.obj_coef = 10, .lower_bound = 0, .upper_bound = 1});
        auto b2 = m.add_variable({.obj_coef = 60, .lower_bound = 0, .upper_bound = 1});
        m.set_binary(b1); m.set_binary(b2);
        row(100.0 * b1 + 100.0 * b2 + 500.0 * core[0] + 700.0 * core[1] <= 1300.0);
        row(40.0 * b1 + 40.0 * b2 + 300.0 * core[2] + 900.0 * core[3] <= 1240.0);
        r.ind = {v(b1)}; r.val = {1}; r.sense = 'G'; r.rhs = 1;
    } else if(name == "doubleton_equality") {
        auto a = m.add_variable({.obj_coef = 30, .lower_bound = 0, .upper_bound = 9});
        auto b = m.add_variable({.obj_coef = 0, .lower_bound = 0, .upper_bound = 4});
        m.set_integer(a); m.set_integer(b);
        row(a - 2.0 * b == 1.0);
        row(50.0 * a + 500.0 * core[0] + 700.0 * core[1] <= 1650.0);
        r.ind = {v(b)}; r.val = {1}; r.sense = 'L'; r.rhs = 0;
    } else if(name == "complement") {
        auto xb = m.add_binary_variable();
        row(core[0] + xb == 1.0);
        r.ind = {v(xb)}; r.val = {1}; r.sense = 'G'; r.rhs = 1;  // forces core[0] = 0
    } else if(name == "fixed_by_bound") {
        auto f = m.add_variable({.obj_coef = 3, .lower_bound = 2, .upper_bound = 2});
        row(f + 500.0 * core[0] + 700.0 * core[1] <= 1300.0);
        r.ind = {v(f)}; r.val = {1}; r.sense = 'L'; r.rhs = 1;  // 2 <= 1: empties the problem
    } else if(name == "slack_singleton") {
        // a slack: the equality becomes an inequality and c disappears.
        auto c = m.add_variable({.obj_coef = 0, .lower_bound = 0, .upper_bound = 1700});
        row(c + 500.0 * core[0] + 700.0 * core[1] + 400.0 * core[2] == 1700.0);
        r.ind = {v(c)}; r.val = {1}; r.sense = 'G'; r.rhs = 400;
    } else {
        std::println(stderr, "unknown gadget {}", name);
        std::exit(2);
    }
    return r;
}
struct core_data {
    std::vector<std::vector<double>> a;
    std::vector<double> c;
    std::vector<size_t> order;  // items selected by the core optimum first
};
static auto build_core(M & m, const core_data & d) {
    const size_t C = d.c.size();
    auto x = m.add_binary_variables(C);
    m.set_maximization();
    const auto ids = std::views::iota(size_t{0}, C);
    m.set_objective(xsum(ids, [&](size_t j) { return d.c[j] * x(j); }));
    for(const auto & row : d.a) { double sum = 0; for(double v : row) sum += v; m.add_constraint(xsum(ids, [&row, &x](size_t j) { return row[j] * x(j); }) <= std::floor(sum / 2)); }
    return x;
}
static core_data make_core(const unsigned seed) {
    core_data d;
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> A(1, 1000), R(0, 500);
    const size_t C = 40;
    d.a.assign(5, std::vector<double>(C));
    for(auto & row : d.a) for(auto & v : row) v = A(rng);
    d.c.resize(C);
    for(size_t j = 0; j < C; ++j) { double sum = 0; for(const auto & row : d.a) sum += row[j]; d.c[j] = std::floor(sum / 5 + R(rng)); }
    M m; one_thread(m);
    auto x = build_core(m, d);
    m.set_time_limit(duration<double>(10.0));
    m.solve();
    auto [env, model] = m.native_model();
    std::vector<double> sol(C);
    m.native_api()._check(env, m.native_api().getdblattrarray(model, "X", 0, static_cast<int>(C), sol.data()));
    for(size_t j = 0; j < C; ++j) if(sol[j] > 0.5) d.order.push_back(j);
    for(size_t j = 0; j < C; ++j) if(sol[j] <= 0.5) d.order.push_back(j);
    return d;
}
struct reduce_run { std::string status; double value; long cuts; long fires; int err; double viol; std::string dims; long later_viol, later_checks; double final_viol; };
static reduce_run run_reduction(const core_data & d, const std::string & name, const std::string & cfg, const bool with_cut, const bool log) {
    M m; one_thread(m);
    set_int(m, "Disconnected", 0);
    if(log) { set_int(m, "OutputFlag", 1); std::println("#### x_reduce {} cfg={} cut={}", name, cfg, with_cut); std::fflush(stdout); }
    auto x = build_core(m, d);
    std::vector<var_t> core;
    for(size_t j : d.order) core.push_back(x(j));
    const auto r = add_gadget(m, name, core);
    m.set_time_limit(duration<double>(10.0));
    if(cfg == "presolve0") set_int(m, "Presolve", 0);
    if(cfg == "dualred0") set_int(m, "DualReductions", 0);
    if(cfg == "heur0" || cfg == "precrush1_heur0") set_dbl(m, "Heuristics", 0.0);
    if(cfg == "precrush1" || cfg == "precrush1_lazy1" || cfg == "precrush1_heur0") set_int(m, "PreCrush", 1);
    if(cfg == "lazy1" || cfg == "precrush1_lazy1" || cfg == "cblazy") set_int(m, "LazyConstraints", 1);
    const std::string dims = presolved_dims(m);
    auto s = std::make_shared<cp::stats>();
    const cp::raw_cut cut{r.ind, r.val, r.sense, r.rhs};
    bool once = false;
    double viol_at_add = NAN;
    long later_viol = 0, later_checks = 0;
    hook_ctx * c = nullptr;
    if(with_cut) {
        const std::string mode = cfg == "cblazy" ? "cblazy_mipnode"
                               : cfg == "precrush1_lazy1" ? "cbcut_precrush_lazyparam"
                               : cfg == "precrush1" || cfg == "precrush1_heur0" ? "cbcut_precrush"
                               : cfg == "lazy1" ? "cbcut_noprecrush_lazyparam" : "cbcut_noprecrush";
        c = attach_hook(m, cp::watched([&](const auto & xv, const cp::add_fn & add) {
            if(once) { ++later_checks; later_viol += cut.violation(xv) > 1e-5; return; }
            if(cut.violation(xv) > 1e-6) { once = true; viol_at_add = cut.violation(xv); add(cut); }
        }, s), mode);
    }
    const double t = cp::timed_solve(m);
    double value = NAN; try { value = m.get_solution_value(); } catch(const std::exception &) {}
    double final_viol = NAN;
    {
        auto [env, model] = m.native_model();
        std::vector<double> sol(m.num_variables());
        if(!m.native_api().getdblattrarray(model, "X", 0, static_cast<int>(sol.size()), sol.data())) {
            final_viol = cut.violation(sol);
            if(name == "parallel_dominance" && !with_cut) {
                const double k = sol[static_cast<size_t>(r.ind[0])] + sol[static_cast<size_t>(r.ind[0]) + 1];
                g_dominance_rhs = std::max(0.0, k - 3);
                std::println(stderr, "   no-cut optimum: b1+b2 = {}, so b1 <= {} keeps an optimum", k, g_dominance_rhs);
            }
        }
    }
    if(c) c->printed = true;
    if(log) std::fflush(stdout);
    return {cp::status_name(m.get_status()), value, s->cuts.load(), c ? c->raw_fires : 0, c ? c->first_error : 0, viol_at_add, std::format("{} t={:.2f}s nodes={:.0f}", dims, t, dattr(m, "NodeCount")), later_viol, later_checks, final_viol};
}
static void x_reduce(const unsigned seed, const std::string & only, const bool log) {
    const auto d = make_core(seed);
    for(const auto & name : {"parallel_integer", "parallel_integer_sum", "parallel_symmetry", "parallel_dominance", "parallel_binary", "parallel_continuous", "parallel_continuous_at_bound", "dominated_column", "dominated_binary", "dual_fixing", "slack_singleton", "free_singleton", "doubleton_equality", "complement", "fixed_by_bound"}) {
        if(!only.empty() && only != name) continue;
        const auto base = run_reduction(d, name, "default", false, log);
        std::println(stderr, "== X_REDUCE seed={} {} no cut, default: {} value={:.6g} {}", seed, name, base.status, base.value, base.dims);
        for(const auto & cfg : {"presolve0", "default", "precrush1", "dualred0", "lazy1", "precrush1_lazy1", "cblazy", "heur0", "precrush1_heur0"}) {
            const auto r = run_reduction(d, name, cfg, true, log);
            const std::string verdict = std::isnan(r.viol) ? "NO POWER: cut never violated, never added"
                                : !r.status.starts_with("optimal") ? "see status"
                                : std::fabs(r.value - base.value) < 1e-4 && r.final_viol > 1e-5 ? "IGNORED: value unchanged, returned point violates the cut"
                                : std::fabs(r.value - base.value) < 1e-4 ? "value unchanged, returned point obeys: no power on the value"
                                : r.final_viol > 1e-5 ? "BIT, but returned point violates it (an incumbent from before the cut)"
                                : "APPLIED: value changed, returned point obeys";
            std::println(stderr, "   cfg={:16} {} value={:.6g} cuts={} fires={} err={} viol_at_add={:.3g} later viol/checks={}/{} final point viol={:.3g} {} -> {}", cfg, r.status, r.value, r.cuts, r.fires, r.err, r.viol, r.later_viol, r.later_checks, r.final_viol, r.dims, verdict);
        }
    }
}

// ------------------------------------------------------------- x_binding ---
// Is a cut binding once added? Each variant is added once, at the first call where
// the hull cut of gadget 3 is violated by the margin the variant needs, and later
// relaxations are checked against the cut.
static void x_binding(const std::string & mode) {
    const auto one = [&](const std::string & name, std::function<cp::raw_cut(const cp::gadgets<M> &, const std::vector<double> &, std::vector<int> &)> make, const int extra_cols = 0, const bool log = false) {
        M m; scope_settings(m);
        if(log) { set_int(m, "OutputFlag", 1); std::println("#### x_binding {} {}", mode, name); std::fflush(stdout); }
        auto g = cp::build_gadgets(m, 10, false);
        std::vector<int> extra;
        if(extra_cols) {
            auto ex = m.add_variables(static_cast<size_t>(extra_cols), {.obj_coef = 0, .lower_bound = 0, .upper_bound = 1});
            for(int j = 0; j < extra_cols; ++j) extra.push_back(m.native_id(ex(static_cast<size_t>(j))));
        }
        m.set_time_limit(duration<double>(15.0));
        auto s = std::make_shared<cp::stats>();
        const auto h = g.hull_cut(3);
        bool done = false;
        cp::raw_cut c;
        long checks = 0, viol = 0, next_checks = 0, next_viol = 0;
        int calls_since = 0, err = 0;
        double worst = 0, add_viol = NAN;
        auto * ctx = attach_hook(m, cp::watched([&](const auto & x, const cp::add_fn & add) {
            if(done) {
                ++calls_since;
                const double v = c.violation(x);
                const bool bad = v > 1e-6;
                ++checks; viol += bad; worst = std::max(worst, v);
                if(calls_since <= 2) { ++next_checks; next_viol += bad; }
                return;
            }
            if(h.violation(x) > 1e-3) {
                c = make(g, x, extra);
                done = true;
                add_viol = c.violation(x);
                err = add(c);
            }
        }, s, [&g](const auto & x) { return g.objective(x); }), mode);
        const double t = cp::timed_solve(m);
        double final_viol = NAN;
        { auto [env, model] = m.native_model(); std::vector<double> sol(m.num_variables()); if(!m.native_api().getdblattrarray(model, "X", 0, static_cast<int>(sol.size()), sol.data())) final_viol = c.violation(sol); }
        std::println(stderr, "== X_BINDING mode={} {}: {} native_error={} viol_at_add={:.3g} next2 viol/checks={}/{} all viol/checks={}/{} worst={:.3g} final point viol={:.3g} -> {}\n   {}", mode, name, cp::summary(m, t), err, add_viol, next_viol, next_checks, viol, checks, worst, final_viol,
            !done ? "NO POWER" : checks == 0 ? "NO POWER: no call after the cut" : viol == 0 ? "OBEYED" : viol == checks ? "NEVER OBEYED: dropped" : "OBEYED IN PART", ctx->line());
        ctx->printed = true;
        if(log) std::fflush(stdout);
    };
    const auto scaled = [](double f) { return [f](const auto & g, const auto &, auto &) { auto c = g.hull_cut(3); for(auto & v : c.val) v *= f; c.rhs *= f; return c; }; };
    one("control: hull cut 3", scaled(1.0), 0, true);
    one("scaled by 1e6", scaled(1e6));
    one("scaled by 1e-6", scaled(1e-6));
    one("scaled by 1e-9", scaled(1e-9));
    one("scaled by 1e9", scaled(1e9));
    for(const double eps : {1e-3, 1e-4, 1e-5, 1e-6, 1e-7, 1e-8}) {
        one(std::format("rhs raised, violation {:.0e} when added", eps), [eps](const auto & g, const auto & x, auto &) {
            auto c = g.hull_cut(3);
            c.rhs = c.activity(x) - eps;
            return c;
        });
    }
    one("coefficient range 1e9: +1e-5 on dummy", [](const auto & g, const auto &, auto &) { auto c = g.hull_cut(3); c.ind.push_back(g.dummy); c.val.push_back(1e-5); return c; });
    one("coefficient range 1e11: +1e7 on dummy", [](const auto & g, const auto &, auto &) { auto c = g.hull_cut(3); c.ind.push_back(g.dummy); c.val.push_back(1e7); return c; });
    one("coefficient range 1e13: 1e-6 and 1e7 on dummy", [](const auto & g, const auto &, auto &) { auto c = g.hull_cut(3); c.ind.push_back(g.dummy); c.val.push_back(1e7); c.ind.push_back(g.y[1]); c.val.push_back(1e-6); return c; });
    one("dense: +1e-4 on 20000 unused columns", [](const auto & g, const auto &, auto & extra) { auto c = g.hull_cut(3); for(int j : extra) { c.ind.push_back(j); c.val.push_back(1e-4); } return c; }, 20000, true);
}

// The same point from the other side: many cuts at one call. How many does the next
// relaxation obey, and how many does the log report as used?
static void x_flood(const std::string & mode, const int count) {
    M m; scope_settings(m);
    set_int(m, "OutputFlag", 1);
    std::println("#### x_flood {} {}", mode, count); std::fflush(stdout);
    auto g = cp::build_gadgets(m, 10, false);
    m.set_time_limit(duration<double>(30.0));
    auto s = std::make_shared<cp::stats>();
    std::vector<cp::raw_cut> cuts;
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> w(0.1, 1.0);
    // Nonnegative combinations of hull cuts: valid, distinct, each violated at the root.
    for(int k = 0; k < count; ++k) {
        std::map<int, double> row;
        double rhs = 0;
        for(int i = 0; i < 10; ++i) {
            if((k + i) % 3 == 0) continue;
            const double f = w(rng);
            const auto c = g.hull_cut(i);
            for(size_t e = 0; e < c.ind.size(); ++e) row[c.ind[e]] += f * c.val[e];
            rhs += f * c.rhs;
        }
        cp::raw_cut c; c.sense = 'L'; c.rhs = rhs;
        for(auto [j, v] : row) { c.ind.push_back(j); c.val.push_back(v); }
        cuts.push_back(std::move(c));
    }
    int phase = 0;
    std::vector<std::string> notes;
    auto * ctx = attach_hook(m, cp::watched([&](const auto & x, const cp::add_fn & add) {
        long violated = 0;
        for(const auto & c : cuts) violated += c.violation(x) > 1e-6;
        if(phase == 0 && violated > 0) {
            for(const auto & c : cuts) add(c);
            notes.push_back(std::format("call 0: {} of {} violated, all added", violated, cuts.size()));
            phase = 1;
        } else if(phase >= 1 && phase < 8) {
            notes.push_back(std::format("call {}: {} of {} still violated, obj={:.6g}", phase, violated, cuts.size(), g.objective(x)));
            ++phase;
        }
    }, s), mode);
    const double t = cp::timed_solve(m);
    std::fflush(stdout);
    std::println(stderr, "== X_FLOOD mode={} {} cuts: {}\n   {}", mode, count, cp::summary(m, t), ctx->line());
    for(const auto & n : notes) std::println(stderr, "   {}", n);
    ctx->printed = true;
}

// ------------------------------------------------------------------- x_q11 ---
// DFJ with degree constraints only, so the model is incomplete: lazy constraints on
// integer candidates are required, user cuts on fractional points are an option.
template <class MM>
static cp::atsp build_dfj(MM & m, const int n, const unsigned seed = 5) {
    cp::atsp a;
    a.n = n;
    const auto N = static_cast<size_t>(n);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> P(0, 100), noise(1.0, 1.3);
    std::vector<std::pair<double, double>> pts(N);
    for(auto & p : pts) p = {P(rng), P(rng)};
    a.d.assign(N, std::vector<double>(N, 0));
    for(size_t i = 0; i < N; ++i) for(size_t j = 0; j < N; ++j) if(i != j)
        a.d[i][j] = std::round(std::hypot(pts[i].first - pts[j].first, pts[i].second - pts[j].second) * noise(rng));
    const auto arcs = N * (N - 1);
    const auto id = [N](size_t i, size_t j) { return i * (N - 1) + (j < i ? j : j - 1); };
    auto x = m.add_binary_variables(arcs);
    a.col.assign(N, std::vector<int>(N, -1));
    a.comp.assign(N, std::vector<int>(N, -1));
    for(size_t i = 0; i < N; ++i) for(size_t j = 0; j < N; ++j) if(i != j)
        a.col[i][j] = m.native_id(x(id(i, j)));
    m.set_minimization();
    const auto nodes = std::views::iota(size_t{0}, N);
    m.set_objective(xsum(nodes, [&](size_t i) {
        return xsum(nodes | std::views::filter([i](size_t j) { return j != i; }),
                    [&, i](size_t j) { return a.d[i][j] * x(id(i, j)); });
    }));
    for(size_t i : nodes) {
        m.add_constraint(xsum(nodes | std::views::filter([i](size_t j) { return j != i; }), [&, i](size_t j) { return 1.0 * x(id(i, j)); }) == 1);
        m.add_constraint(xsum(nodes | std::views::filter([i](size_t j) { return j != i; }), [&, i](size_t j) { return 1.0 * x(id(j, i)); }) == 1);
    }
    return a;
}
struct q11_ctx {
    const grb::gurobi_api * api;
    const cp::atsp * a;
    int cols;
    bool lazy_sol, cut_node, lazy_node;
    long sol_calls = 0, sol_lazy = 0, sol_err = 0, node_calls = 0, node_integral = 0, node_integral_violated = 0, node_cuts = 0, node_lazy = 0, node_err = 0;
    int first_error = 0;
    std::set<std::thread::id> threads;
};
static int q11_hook(grb::GRBmodel *, void * cbdata, int where, void * usr) {
    auto & c = *static_cast<q11_ctx *>(usr);
    c.threads.insert(std::this_thread::get_id());
    const auto err = [&](int e) { if(e && !c.first_error) c.first_error = e; };
    std::vector<double> x(static_cast<size_t>(c.cols));
    if(where == W_MIPSOL && c.lazy_sol) {
        ++c.sol_calls;
        err(c.api->cbget(cbdata, where, SOL_SOL, x.data()));
        for(const auto & r : separate_any(*c.a, x, 100)) {
            const int e = c.api->cblazy(cbdata, static_cast<int>(r.ind.size()), r.ind.data(), r.val.data(), sense_of(r.sense), r.rhs);
            err(e); (e ? c.sol_err : c.sol_lazy)++;
        }
    } else if(where == W_MIPNODE && (c.cut_node || c.lazy_node)) {
        int status = 0;
        c.api->cbget(cbdata, where, NODE_STATUS, &status);
        if(status != GRB_OPTIMAL_) return 0;
        ++c.node_calls;
        err(c.api->cbget(cbdata, where, NODE_REL, x.data()));
        const auto cuts = separate_any(*c.a, x, 5);
        if(c.a->integral(x)) { ++c.node_integral; c.node_integral_violated += !cuts.empty(); }
        for(const auto & r : cuts) {
            const int n = static_cast<int>(r.ind.size());
            const int e = c.lazy_node ? c.api->cblazy(cbdata, n, r.ind.data(), r.val.data(), sense_of(r.sense), r.rhs)
                                      : c.api->cbcut(cbdata, n, r.ind.data(), r.val.data(), sense_of(r.sense), r.rhs);
            err(e);
            if(e) ++c.node_err; else (c.lazy_node ? c.node_lazy : c.node_cuts)++;
        }
    }
    return 0;
}
static void x_q11_one(const int n, const std::string & name, const bool lazy_sol, const bool cut_node, const bool lazy_node, const int lazyparam, const int precrush, const int threads, const bool log = false) {
    M m;
    if(threads) m.set_thread_limit(static_cast<size_t>(threads));
    if(log) { set_int(m, "OutputFlag", 1); std::println("#### x_q11 {} n={} threads={}", name, n, threads); std::fflush(stdout); }
    auto a = build_dfj(m, n);
    m.set_time_limit(duration<double>(40.0));
    set_int(m, "LazyConstraints", lazyparam);
    set_int(m, "PreCrush", precrush);
    q11_ctx c{&m.native_api(), &a, static_cast<int>(m.num_variables()), lazy_sol, cut_node, lazy_node};
    auto [env, model] = m.native_model();
    m.native_api()._check(env, m.native_api().setcallbackfunc(model, q11_hook, &c));
    const double t = cp::timed_solve(m);
    std::string tour = "no solution";
    try {
        auto sol = m.get_solution();
        (void)sol;
        std::vector<double> xv(static_cast<size_t>(c.cols));
        auto & api = m.native_api();
        if(!api.getdblattrarray(model, "X", 0, c.cols, xv.data())) {
            const auto bad = separate_any(a, xv, 100);
            tour = bad.empty() ? "final point is a tour" : std::format("final point has subtours: {} DFJ cuts violated", bad.size());
        }
    } catch(const std::exception & e) { tour = std::string("no solution: ") + e.what(); }
    std::println(stderr, "== X_Q11 n={} {} (LazyConstraints={} PreCrush={} threads={}): {} {}\n   {} | MIPSOL calls={} lazy={} err={} | MIPNODE optimal calls={} cuts={} lazy={} err={} integral={} of which violating={} | threads={} first_native_error={}", n, name, lazyparam, precrush, threads, cp::summary(m, t), run_facts(m), tour, c.sol_calls, c.sol_lazy, c.sol_err, c.node_calls, c.node_cuts, c.node_lazy, c.node_err, c.node_integral, c.node_integral_violated, c.threads.size(), c.first_error);
    if(log) std::fflush(stdout);
}
static void x_q11(const int n) {
    x_q11_one(n, "A lazy on MIPSOL only", true, false, false, 1, 0, 1);
    x_q11_one(n, "B lazy on MIPSOL + cbcut on MIPNODE, PreCrush=1", true, true, false, 1, 1, 1, true);
    x_q11_one(n, "C lazy on MIPSOL + cbcut on MIPNODE, PreCrush=0", true, true, false, 1, 0, 1);
    x_q11_one(n, "D cbcut on MIPNODE only, no lazy at all", false, true, false, 0, 1, 1, true);
    x_q11_one(n, "E cblazy on MIPNODE only", false, false, true, 1, 0, 1);
    x_q11_one(n, "F cblazy on MIPSOL and MIPNODE", true, false, true, 1, 0, 1);
    x_q11_one(n, "G cblazy on MIPSOL without LazyConstraints", true, false, false, 0, 0, 1);
    x_q11_one(n, "H cblazy on MIPNODE without LazyConstraints", false, false, true, 0, 0, 1);
    x_q11_one(n, "B at default threads", true, true, false, 1, 1, 0);
}

// ------------------------------------------------------------------- x_q12 ---
// Node-local reads: what can a hook learn about the node it is on?
struct q12_ctx {
    const grb::gurobi_api * api;
    grb::GRBmodel * user_model;
    const cp::gadgets<M> * g;
    int cols, printed = 0, y0;
    bool any_fixed0 = false, any_fixed1 = false;
};
static int q12_hook(grb::GRBmodel * cbm, void * cbdata, int where, void * usr) {
    auto & c = *static_cast<q12_ctx *>(usr);
    if(where != W_MIPNODE) return 0;
    int status = 0;
    c.api->cbget(cbdata, where, NODE_STATUS, &status);
    std::vector<double> x(static_cast<size_t>(c.cols), NAN);
    const int rel_err = c.api->cbget(cbdata, where, NODE_REL, x.data());
    if(status != GRB_OPTIMAL_) {
        static int seen = 0;
        if(seen++ < 3) std::println(stderr, "   status={}: GRBcbget REL returned {} (x[0]={})", status, rel_err, x[0]);
        return 0;
    }
    const double y = x[static_cast<size_t>(c.y0)];
    const bool fixed0 = y < 1e-9, fixed1 = y > 1 - 1e-9;
    const bool first = !c.printed;
    const bool want = first || (fixed0 && !c.any_fixed0) || (fixed1 && !c.any_fixed1);
    if(!want) return 0;
    c.any_fixed0 |= fixed0; c.any_fixed1 |= fixed1;
    ++c.printed;
    std::println(stderr, "   -- call with y0={} cbmodel==user_model:{}", y, cbm == c.user_model);
    for(const auto [name, what, is_int] : {std::tuple{"MIPNODE_STATUS", 5001, true}, {"MIPNODE_OBJBST", 5003, false}, {"MIPNODE_OBJBND", 5004, false}, {"MIPNODE_NODCNT", 5005, false}, {"MIPNODE_SOLCNT", 5006, true}, {"MIPNODE_BRVAR (undocumented)", 5007, true}, {"MIPNODE_OPENSCENARIOS", 5008, true}, {"MIPNODE_PHASE", 5009, true}, {"5010 (no such code)", 5010, true}, {"5011 (no such code)", 5011, false}, {"5099 (no such code)", 5099, false}, {"MIP_NODCNT from MIPNODE", 3002, false}, {"MIP_NODLFT from MIPNODE", 3005, false}, {"RUNTIME", 6002, false}, {"WORK", 6003, false}, {"MIPSOL_OBJ from MIPNODE", 4002, false}}) {
        int iv = -123456; double dv = NAN;
        const int e = is_int ? c.api->cbget(cbdata, where, what, &iv) : c.api->cbget(cbdata, where, what, &dv);
        if(is_int) std::println(stderr, "   cbget {:32} err={} value={}", name, e, iv);
        else std::println(stderr, "   cbget {:32} err={} value={}", name, e, dv);
    }
    double lb = NAN, ub = NAN;
    int e1 = c.api->getdblattrelement(cbm, "LB", c.y0, &lb);
    int e2 = c.api->getdblattrelement(cbm, "UB", c.y0, &ub);
    int nv = -1;
    int e3 = c.api->getintattr(cbm, "NumVars", &nv);
    double xv = NAN;
    int e4 = c.api->getdblattrelement(cbm, "X", c.y0, &xv);
    double nc = NAN;
    int e5 = c.api->getdblattr(cbm, "NodeCount", &nc);
    std::println(stderr, "   attr LB(y0) err={} {} | UB(y0) err={} {} | NumVars err={} {} | X(y0) err={} {} | NodeCount err={} {}", e1, lb, e2, ub, e3, nv, e4, xv, e5, nc);
    return 0;
}
static void x_q12() {
    M m; scope_settings(m);
    auto g = cp::build_gadgets(m, 10, false);
    m.set_time_limit(duration<double>(10.0));
    auto [env, model] = m.native_model();
    q12_ctx c{&m.native_api(), model, &g, static_cast<int>(m.num_variables()), 0, g.y[0]};
    m.native_api()._check(env, m.native_api().setcallbackfunc(model, q12_hook, &c));
    std::println(stderr, "== X_Q12 scope settings");
    const double t = cp::timed_solve(m);
    std::println(stderr, "   => {}", cp::summary(m, t));
    M d; one_thread(d);
    auto a = cp::build_atsp(d, 26, true);
    d.set_time_limit(duration<double>(10.0));
    auto [env2, model2] = d.native_model();
    // default presolve: does the callback's model handle change, and its NumVars?
    struct dctx { const grb::gurobi_api * api; grb::GRBmodel * user; int cols; int n = 0; } dc{&d.native_api(), model2, static_cast<int>(d.num_variables())};
    d.native_api()._check(env2, d.native_api().setcallbackfunc(model2, [](grb::GRBmodel * cbm, void *, int where, void * usr) -> int {
        auto & c = *static_cast<dctx *>(usr);
        if(where == W_MIPNODE && c.n++ == 0) {
            int nv = -1, e = c.api->getintattr(cbm, "NumVars", &nv);
            std::println(stderr, "   default presolve: cbmodel==user_model:{} NumVars err={} {} (user model has {} columns)", cbm == c.user, e, nv, c.cols);
        }
        return 0;
    }, &dc));
    const double t2 = cp::timed_solve(d);
    std::println(stderr, "   => {}", cp::summary(d, t2));
}

// ---------------------------------------------------------------- x_resume ---
// A solve stopped by a limit resumes on the next GRBoptimize. Do cuts survive it? And
// what does the wrapper's detach-after-solve (GRBsetcallbackfunc NULL) do to that?
static void x_resume(const bool flip, const bool detach, const long first_nodes, const bool reset, const std::string & mode) {
    M m; scope_settings(m);
    auto g = cp::build_gadgets(m, 10, flip);
    m.set_time_limit(duration<double>(15.0));
    auto s = std::make_shared<cp::stats>();
    auto tr = std::make_shared<cp::scope_tracker>();
    tr->cut = g.invalid_global_cut(); tr->trigger = g.worse_state(); tr->state = [&g](const auto & x) { return g.state(x); };
    auto * c = attach_hook(m, cp::watched([tr](const auto & x, const cp::add_fn & add) { if(tr->wants_add(x)) tr->add_error = add(tr->cut); }, s,
        [&g](const auto & x) { return g.objective(x); }), mode);
    m.set_node_limit(static_cast<size_t>(first_nodes));
    const double t = cp::timed_solve(m);
    std::println(stderr, "== X_RESUME flip={} detach={} reset={} first segment NodeLimit={}: {}\n   {}", flip, detach, reset, first_nodes, cp::summary(m, t), tr->line());
    auto [env, model] = m.native_model();
    auto & api = m.native_api();
    if(detach) { api._check(env, api.setcallbackfunc(model, nullptr, nullptr)); api._check(env, api.setcallbackfunc(model, cut_hook, c)); }
    if(reset) api._check(env, api.reset(model, 0));
    m.set_node_limit(2000000000);
    const long fires_before = c->raw_fires;
    tr->same_checks = tr->same_viol = tr->sib_checks = tr->sib_viol = 0;
    const double t2 = cp::timed_solve(m);
    double value = NAN; try { value = m.get_solution_value(); } catch(const std::exception &) {}
    std::println(stderr, "   second segment: {} fires={} (known opt {}) => {}\n   {}", cp::summary(m, t2), c->raw_fires - fires_before, g.opt, std::fabs(value - g.opt) < 1e-4 ? "optimum KEPT" : "optimum LOST: the cut survived", tr->line());
    c->printed = true;
    // and an unchanged, finished model: does a third solve search at all?
    const long f3 = c->raw_fires;
    const double t3 = cp::timed_solve(m);
    std::println(stderr, "   third solve, nothing changed: {} fires={}", cp::summary(m, t3), c->raw_fires - f3);
}

// -------------------------------------------------------------------- x_q3 ---
// Cuts that remove integer feasible points: non-optimal ones, the optimal ones, and
// the incumbent.
static void x_q3() {
    const auto run = [](const std::string & name, const bool flip, std::function<cp::raw_cut(const cp::gadgets<M> &)> make, const bool start_opt, const bool default_settings = false) {
        std::vector<double> start;
        double opt_seen = NAN;
        if(start_opt) {
            M f; scope_settings(f);
            auto g0 = cp::build_gadgets(f, 10, flip);
            f.set_time_limit(duration<double>(15.0));
            f.solve();
            auto [e0, m0] = f.native_model();
            start.resize(f.num_variables());
            f.native_api()._check(e0, f.native_api().getdblattrarray(m0, "X", 0, static_cast<int>(start.size()), start.data()));
            opt_seen = f.get_solution_value();
        }
        M m;
        if(default_settings) one_thread(m); else scope_settings(m);
        auto g = cp::build_gadgets(m, 10, flip);
        m.set_time_limit(duration<double>(15.0));
        auto [env, model] = m.native_model();
        auto & api = m.native_api();
        if(start_opt) api._check(env, api.setdblattrarray(model, "Start", 0, static_cast<int>(start.size()), start.data()));
        auto s = std::make_shared<cp::stats>();
        const auto cut = make(g);
        bool once = false;
        auto * c = attach_hook(m, cp::watched([&](const auto & x, const cp::add_fn & add) { if(!once && cut.violation(x) > 1e-6) { once = true; add(cut); } }, s), "cbcut_precrush");
        const double t = cp::timed_solve(m);
        std::println(stderr, "== X_Q3 {} flip={} start_opt={} default_settings={}: {} (known opt {}) cut_added={}\n   {}", name, flip, start_opt, default_settings, cp::summary(m, t), g.opt, once, c->line());
        c->printed = true;
        std::vector<double> final_x(m.num_variables());
        if(!api.getdblattrarray(model, "X", 0, static_cast<int>(final_x.size()), final_x.data()))
            std::println(stderr, "   returned point: activity {:.6g} rhs {:.6g} violation of the cut {:.6g}; y0={}", cut.activity(final_x), cut.rhs, cut.violation(final_x), final_x[static_cast<size_t>(g.y[0])]);
        (void)opt_seen;
    };
    // flip=false: the optimum has y0=0, so y0 <= 0 removes only non-optimal integer points.
    run("dominance y0 <= 0, keeps every optimum", false, [](const auto & g) { return cp::raw_cut{{g.y[0]}, {1.0}, 'L', 0.0}; }, false);
    run("wrong side y0 >= 1, removes the optimum", false, [](const auto & g) { return cp::raw_cut{{g.y[0]}, {1.0}, 'G', 1.0}; }, false);
    run("objective cutoff Sum z >= opt - 0.5", false, [](const auto & g) { return cp::raw_cut{g.z, std::vector<double>(g.z.size(), 1.0), 'G', g.opt - 0.5}; }, false);
    run("impossible objective Sum z >= opt + 0.5", false, [](const auto & g) { return cp::raw_cut{g.z, std::vector<double>(g.z.size(), 1.0), 'G', g.opt + 0.5}; }, false);
    run("wrong side, optimum given as MIP start", false, [](const auto & g) { return cp::raw_cut{{g.y[0]}, {1.0}, 'G', 1.0}; }, true);
    run("Sum z <= opt-1 with optimum given as MIP start", false, [](const auto & g) { return g.invalid_global_cut(); }, true);
    run("Sum z <= opt-1 at root, default settings", false, [](const auto & g) { return g.invalid_global_cut(); }, false, true);
    run("Sum z <= opt-1 at root, default settings, flip", true, [](const auto & g) { return g.invalid_global_cut(); }, false, true);
}

// ----------------------------------------------------------------- x_wodd ---
// One odd native call per process: some may crash.
struct wodd_ctx {
    const grb::gurobi_api * api;
    int cols;
    std::function<void(void *, int, const std::vector<double> &)> act;
    bool mipsol = false, mip = false;
};
static int wodd_hook(grb::GRBmodel *, void * cbdata, int where, void * usr) {
    auto & c = *static_cast<wodd_ctx *>(usr);
    std::vector<double> x(static_cast<size_t>(c.cols));
    if(where == W_MIPSOL && c.mipsol) { c.api->cbget(cbdata, where, SOL_SOL, x.data()); c.act(cbdata, where, x); }
    if(where == W_MIP && c.mip) c.act(cbdata, where, x);
    if(where == W_MIPNODE && !c.mipsol && !c.mip) {
        int status = 0;
        c.api->cbget(cbdata, where, NODE_STATUS, &status);
        if(status != GRB_OPTIMAL_) return 0;
        c.api->cbget(cbdata, where, NODE_REL, x.data());
        c.act(cbdata, where, x);
    }
    return 0;
}
static void x_wodd(const std::string & item) {
    M m; scope_settings(m);
    auto g = cp::build_gadgets(m, 10, false);
    m.set_time_limit(duration<double>(10.0));
    auto [env, model] = m.native_model();
    auto & api = m.native_api();
    set_int(m, "PreCrush", 1);
    wodd_ctx c{&api, static_cast<int>(m.num_variables()), {}};
    bool once = false;
    int err = 0;
    long after = 0;
    double track_lo = 1e100, track_hi = -1e100, at_add = NAN;
    int track_col = -1;
    const int za3 = g.za[3];
    const auto add_once = [&](std::vector<int> ind, std::vector<double> val, char sense, double rhs, int track = -1) {
        c.act = [&, ind, val, sense, rhs, track](void * cb, int, const std::vector<double> & x) {
            if(once) {
                ++after;
                if(track >= 0) { track_lo = std::min(track_lo, x[static_cast<size_t>(track)]); track_hi = std::max(track_hi, x[static_cast<size_t>(track)]); }
                return;
            }
            once = true;
            if(track >= 0) at_add = x[static_cast<size_t>(track)];
            err = api.cbcut(cb, static_cast<int>(ind.size()), ind.data(), val.data(), sense, rhs);
        };
        track_col = track;
    };
    if(item == "dup_half_half") add_once({za3, za3}, {0.5, 0.5}, '<', 2.0, za3);
    else if(item == "dup_quarter_threequarter") add_once({za3, za3}, {0.25, 0.75}, '<', 2.0, za3);
    else if(item == "dup_threequarter_quarter") add_once({za3, za3}, {0.75, 0.25}, '<', 2.0, za3);
    else if(item == "single_control") add_once({za3}, {1.0}, '<', 2.0, za3);
    else if(item == "equality_dummy_1") add_once({g.dummy}, {1.0}, '=', 1.0, g.dummy);
    else if(item == "equality_za3") add_once({za3}, {1.0}, '=', 2.0, za3);
    else if(item == "empty_infeasible") add_once({}, {}, '<', -1.0);
    else if(item == "bad_index") add_once({c.cols + 5}, {1.0}, '<', 1.0);
    else if(item == "negative_index") add_once({-1}, {1.0}, '<', 1.0);
    else if(item == "bad_sense") add_once({za3}, {1.0}, 'X', 2.0, za3);
    else if(item == "nan_coef") add_once({za3}, {std::numeric_limits<double>::quiet_NaN()}, '<', 2.0, za3);
    else if(item == "inf_coef") add_once({za3}, {1e100}, '<', 2.0, za3);
    else if(item == "huge_coef_1e30") add_once({za3}, {1e30}, '<', 2e30, za3);
    else if(item == "tiny_coef_1e-30") add_once({za3, g.dummy}, {1.0, 1e-30}, '<', 2.0, za3);
    else if(item == "nan_rhs") add_once({za3}, {1.0}, '<', std::numeric_limits<double>::quiet_NaN(), za3);
    else if(item == "inf_rhs") add_once({za3}, {1.0}, '<', 1e100, za3);
    else if(item == "neg_inf_rhs") add_once({za3}, {1.0}, '<', -1e100, za3);
    else if(item == "cbcut_from_mipsol" || item == "cbcut_from_mip") {
        c.mipsol = item == "cbcut_from_mipsol";
        c.mip = item == "cbcut_from_mip";
        const std::vector<int> ind{za3}; const std::vector<double> val{1.0};
        c.act = [&, ind, val](void * cb, int, const std::vector<double> &) { if(!once) { once = true; err = api.cbcut(cb, 1, ind.data(), val.data(), '<', 2.0); } };
    } else if(item == "cblazy_from_mip") {
        c.mip = true;
        set_int(m, "LazyConstraints", 1);
        const std::vector<int> ind{za3}; const std::vector<double> val{1.0};
        c.act = [&, ind, val](void * cb, int, const std::vector<double> &) { if(!once) { once = true; err = api.cblazy(cb, 1, ind.data(), val.data(), '<', 2.0); } };
    } else if(item == "cbget_rel_from_mipsol") {
        c.mipsol = true;
        c.act = [&](void * cb, int w, const std::vector<double> & x) { if(!once) { once = true; std::vector<double> y(x.size()); err = api.cbget(cb, w, NODE_REL, y.data()); } };
    } else if(item == "cbcut_nonoptimal_status") {
        // the only hook variant that does not filter on status
        c.act = {};
    } else { std::println(stderr, "unknown item {}", item); std::exit(2); }
    if(item == "cbcut_nonoptimal_status") {
        struct nctx { const grb::gurobi_api * api; int za3; std::map<int, std::pair<long, int>> by_status; } n{&api, za3};
        api._check(env, api.setcallbackfunc(model, [](grb::GRBmodel *, void * cb, int where, void * usr) -> int {
            auto & c = *static_cast<nctx *>(usr);
            if(where != W_MIPNODE) return 0;
            int status = 0;
            c.api->cbget(cb, where, NODE_STATUS, &status);
            if(status == GRB_OPTIMAL_) return 0;
            const int ind[] = {c.za3};
            const double val[] = {1.0};
            const int e = c.api->cbcut(cb, 1, ind, val, '<', 1e6);
            c.by_status[status].first++;
            c.by_status[status].second = e;
            return 0;
        }, &n));
        const double t = cp::timed_solve(m);
        std::string st;
        for(auto [k, v] : n.by_status) st += std::format(" status {}: {} calls, last native error {};", k, v.first, v.second);
        std::println(stderr, "== X_WODD {}: {}{}", item, cp::summary(m, t), st);
        return;
    }
    api._check(env, api.setcallbackfunc(model, wodd_hook, &c));
    const double t = cp::timed_solve(m);
    std::string msg = err ? std::string(api.geterrormsg(env)) : "";
    std::println(stderr, "== X_WODD {}: {} (known opt {}) called={} native_error={} msg='{}' calls_after={}", item, cp::summary(m, t), g.opt, once, err, msg, after);
    if(track_col >= 0) std::println(stderr, "   column value when added {:.6g}, later relaxations in [{:.6g}, {:.6g}]", at_add, track_lo, track_hi);
}

// --------------------------------------------------------------- x_threads ---
struct thr_ctx {
    const grb::gurobi_api * api;
    int cols;
    long spin_us;
    std::thread::id main_id;
    long calls = 0, off_main = 0, sol = 0, nodes_seen = 0;
    std::atomic<int> inflight{0};
    std::atomic<long> overlaps{0};
    std::set<std::thread::id> threads;
    double cpu_during = 0, wall_during = 0;
    std::mutex mx;
};
static double cpu_now() {
    rusage r{};
    getrusage(RUSAGE_SELF, &r);
    return static_cast<double>(r.ru_utime.tv_sec + r.ru_stime.tv_sec) + 1e-6 * static_cast<double>(r.ru_utime.tv_usec + r.ru_stime.tv_usec);
}
static int thr_hook(grb::GRBmodel *, void * cbdata, int where, void * usr) {
    auto & c = *static_cast<thr_ctx *>(usr);
    if(where != W_MIPNODE && where != W_MIPSOL) return 0;
    if(c.inflight.fetch_add(1) > 0) c.overlaps.fetch_add(1);
    {
        std::lock_guard l(c.mx);
        c.threads.insert(std::this_thread::get_id());
        if(std::this_thread::get_id() != c.main_id) ++c.off_main;
        if(where == W_MIPSOL) ++c.sol;
    }
    if(where == W_MIPNODE) {
        int status = 0;
        c.api->cbget(cbdata, where, NODE_STATUS, &status);
        if(status == GRB_OPTIMAL_) {
            std::vector<double> x(static_cast<size_t>(c.cols));
            c.api->cbget(cbdata, where, NODE_REL, x.data());
            ++c.calls;
            if(c.spin_us) {
                const double cpu0 = cpu_now();
                const auto t0 = clk::now();
                const auto until = t0 + std::chrono::microseconds(c.spin_us);
                while(clk::now() < until) {}
                c.wall_during += duration<double>(clk::now() - t0).count();
                c.cpu_during += cpu_now() - cpu0;
            }
        }
    }
    c.inflight.fetch_sub(1);
    return 0;
}
static void x_threads_one(const std::string & name, const int threads, const int concurrent, const long spin_us, const double limit) {
    M m;
    if(threads) m.set_thread_limit(static_cast<size_t>(threads));
    if(concurrent) set_int(m, "ConcurrentMIP", concurrent);
    cp::build_mkp(m, 30, 100);
    m.set_time_limit(duration<double>(limit));
    thr_ctx c{&m.native_api(), static_cast<int>(m.num_variables()), spin_us, std::this_thread::get_id()};
    auto [env, model] = m.native_model();
    m.native_api()._check(env, m.native_api().setcallbackfunc(model, thr_hook, &c));
    const double t = cp::timed_solve(m);
    std::println(stderr, "== X_THREADS {} threads={} concurrent={} spin={}us: {}\n   optimal calls={} MIPSOL calls={} distinct hook threads={} calls off the optimize() thread={} overlaps={} cpu/wall inside the spin={:.2f} (wall inside {:.2f}s of {:.2f}s)", name, threads, concurrent, spin_us, cp::summary(m, t), c.calls, c.sol, c.threads.size(), c.off_main, c.overlaps.load(), c.wall_during > 0 ? c.cpu_during / c.wall_during : 0.0, c.wall_during, t);
}
static void x_threads() {
    x_threads_one("default threads", 0, 0, 0, 5);
    x_threads_one("8 threads, 200us", 8, 0, 200, 5);
    x_threads_one("1 thread, 2ms", 1, 0, 2000, 5);
    x_threads_one("8 threads, 2ms", 8, 0, 2000, 5);
    x_threads_one("16 threads, 20ms", 16, 0, 20000, 5);
    x_threads_one("ConcurrentMIP=2, 8 threads", 8, 2, 200, 5);
    x_threads_one("ConcurrentMIP=4, 16 threads", 16, 4, 200, 5);
}

// ------------------------------------------------------------------ x_side ---
// Side effects of the hook and of the parameters, under a node limit so that the
// path compares: same nodes, iterations, value and bound means the same search.
static void x_side_one(const std::string & inst, const std::string & cfg, const bool hook_on, const int precrush, const int lazy, const int dual, const int threads) {
    M m;
    if(threads) m.set_thread_limit(static_cast<size_t>(threads));
    if(inst == "atsp26") cp::build_atsp(m, 26);
    else if(inst == "atsp26c") cp::build_atsp(m, 26, true);
    else { cp::build_mkp(m, 30, 100); m.set_node_limit(30000); }
    m.set_time_limit(duration<double>(60.0));
    if(precrush) set_int(m, "PreCrush", 1);
    if(lazy) set_int(m, "LazyConstraints", 1);
    if(dual == 0) set_int(m, "DualReductions", 0);
    const std::string dims = presolved_dims(m);
    auto s = std::make_shared<cp::stats>();
    hook_ctx * c = nullptr;
    if(hook_on) {
        const std::string mode = precrush && lazy ? "cbcut_precrush_lazyparam" : precrush ? "cbcut_precrush" : lazy ? "cbcut_noprecrush_lazyparam" : "cbcut_noprecrush";
        c = attach_hook(m, cp::watched([](const auto &, const auto &) {}, s), mode);
    }
    const double t = cp::timed_solve(m);
    std::println(stderr, "== X_SIDE {} threads={} {:36} {} {} {}", inst, threads, cfg, cp::summary(m, t), run_facts(m), dims);
    if(c) c->printed = true;
}
static void x_side() {
    for(const auto & [inst, thr] : {std::pair{std::string("atsp26"), 1}, {std::string("atsp26c"), 1}, {std::string("mkp"), 1}, {std::string("mkp"), 0}, {std::string("mkp"), 8}}) {
        x_side_one(inst, "no hook", false, 0, 0, 1, thr);
        x_side_one(inst, "no hook (again)", false, 0, 0, 1, thr);
        x_side_one(inst, "no-op hook", true, 0, 0, 1, thr);
        x_side_one(inst, "no hook, PreCrush=1", false, 1, 0, 1, thr);
        x_side_one(inst, "no-op hook, PreCrush=1", true, 1, 0, 1, thr);
        x_side_one(inst, "no hook, LazyConstraints=1", false, 0, 1, 1, thr);
        x_side_one(inst, "no-op hook, LazyConstraints=1", true, 0, 1, 1, thr);
        x_side_one(inst, "no-op hook, both", true, 1, 1, 1, thr);
        x_side_one(inst, "no hook, DualReductions=0", false, 0, 0, 0, thr);
    }
}

// -------------------------------------------------------------- x_settings ---
// Do the cut parameters gate user cuts?
static void x_settings(const int n) {
    for(const auto & [name, setter] : std::vector<std::pair<std::string, std::function<void(M &)>>>{
            {"defaults", [](M &) {}},
            {"Cuts=0", [](M & m) { set_int(m, "Cuts", 0); }},
            {"CutPasses=0", [](M & m) { set_int(m, "CutPasses", 0); }},
            {"Cuts=0 CutPasses=0", [](M & m) { set_int(m, "Cuts", 0); set_int(m, "CutPasses", 0); }},
            {"CutAggPasses=0", [](M & m) { set_int(m, "CutAggPasses", 0); }},
            {"Presolve=0", [](M & m) { set_int(m, "Presolve", 0); }},
            {"Presolve=2", [](M & m) { set_int(m, "Presolve", 2); }},
            {"Heuristics=0", [](M & m) { set_dbl(m, "Heuristics", 0.0); }},
            {"CutPasses=5", [](M & m) { set_int(m, "CutPasses", 5); }},
            {"CutPasses=1000", [](M & m) { set_int(m, "CutPasses", 1000); }},
            {"NodeMethod=2 (barrier)", [](M & m) { set_int(m, "NodeMethod", 2); }}}) {
        M m; one_thread(m);
        auto a = cp::build_atsp(m, n);
        m.set_time_limit(duration<double>(60.0));
        setter(m);
        auto s = std::make_shared<cp::stats>();
        auto * c = attach_hook(m, cp::watched([&a](const auto & x, const cp::add_fn & add) { for(const auto & cut : a.separate(x)) add(cut); }, s, [&a](const auto & x) { return a.objective(x); }), "cbcut_precrush");
        const double t = cp::timed_solve(m);
        std::println(stderr, "== X_SETTINGS n={} E3 {}: {}\n   {}\n   {}", n, name, cp::summary(m, t), s->line(), c->line());
        c->printed = true;
    }
}

// -------------------------------------------------------------- x_s1_default ---
// S1 under default presolve, cuts and heuristics, on a knapsack big enough that the
// tree exists: the objective cut Sum c x <= opt-1 at the first tree node.
static void x_s1_default(const unsigned seed, const std::string & cfg) {
    const auto d = make_core(seed);
    double opt = NAN;
    {
        M m; one_thread(m);
        build_core(m, d);
        m.set_time_limit(duration<double>(10.0));
        if(cfg == "heur0") set_dbl(m, "Heuristics", 0.0);
        m.solve();
        opt = m.get_solution_value();
        std::println(stderr, "== X_S1_DEFAULT seed={} cfg={} no cut: {} {}", seed, cfg, cp::summary(m, 0), presolved_dims(m));
    }
    M m; one_thread(m);
    auto x = build_core(m, d);
    m.set_time_limit(duration<double>(10.0));
    if(cfg == "heur0") set_dbl(m, "Heuristics", 0.0);
    std::vector<int> ind;
    std::vector<double> val;
    for(size_t j = 0; j < d.c.size(); ++j) { ind.push_back(m.native_id(x(j))); val.push_back(d.c[j]); }
    const cp::raw_cut cut{ind, val, 'L', opt - 1};
    auto nodcnt = std::make_shared<double>(0);
    bool once = false;
    double at = NAN;
    long later = 0, later_viol = 0;
    auto s = std::make_shared<cp::stats>();
    auto * c = attach_hook(m, cp::watched([&](const auto & xv, const cp::add_fn & add) {
        if(once) { ++later; later_viol += cut.violation(xv) > 1e-5; return; }
        if(*nodcnt > 0 && cut.violation(xv) > 1e-6) { once = true; at = *nodcnt; add(cut); }
    }, s), "cbcut_precrush");
    c->nodcnt_out = nodcnt;
    const double t = cp::timed_solve(m);
    double value = NAN; try { value = m.get_solution_value(); } catch(const std::exception &) {}
    std::println(stderr, "   with the cut at nodcnt={}: {} (opt {}) => {} later viol/checks={}/{}\n   {}", at, cp::summary(m, t), opt, !once ? "NO POWER" : std::fabs(value - opt) < 1e-4 ? "value kept (incumbent from before the cut?)" : "optimum LOST", later_viol, later, c->line());
    c->printed = true;
}

// ----------------------------------------------------------------- x_phase ---
// Do the NoRel heuristic and the improvement phase fire MIPNODE, and with which status?
static void x_phase() {
    for(const auto & [name, setter] : std::vector<std::pair<std::string, std::function<void(M &)>>>{
            {"default", [](M &) {}},
            {"NoRelHeurTime=2", [](M & m) { set_dbl(m, "NoRelHeurTime", 2.0); }},
            {"ImproveStartTime=2", [](M & m) { set_dbl(m, "ImproveStartTime", 2.0); }}}) {
        M m; one_thread(m);
        cp::build_mkp(m, 30, 100);
        m.set_time_limit(duration<double>(5.0));
        setter(m);
        auto s = std::make_shared<cp::stats>();
        auto * c = attach_hook(m, cp::watched([](const auto &, const auto &) {}, s), "cbcut_precrush");
        const double t = cp::timed_solve(m);
        std::println(stderr, "== X_PHASE {}: {}\n   {}", name, cp::summary(m, t), c->line());
        c->printed = true;
    }
}

// ---------------------------------------------------------------- x_toggle ---
// A setter called between a limit stop and the next solve: does changing the parameter
// lose the resume?
static void x_toggle(const std::string & param, const int value) {
    M m; scope_settings(m);
    auto g = cp::build_gadgets(m, 10, true);
    m.set_time_limit(duration<double>(15.0));
    auto s = std::make_shared<cp::stats>();
    auto tr = std::make_shared<cp::scope_tracker>();
    tr->cut = g.invalid_global_cut(); tr->trigger = g.worse_state(); tr->state = [&g](const auto & x) { return g.state(x); };
    auto * c = attach_hook(m, cp::watched([tr](const auto & x, const cp::add_fn & add) { if(tr->wants_add(x)) tr->add_error = add(tr->cut); }, s), "cbcut_noprecrush");
    m.set_node_limit(200);
    const double t = cp::timed_solve(m);
    std::println(stderr, "== X_TOGGLE {}={} between segments: first {}\n   {}", param, value, cp::summary(m, t), tr->line());
    set_int(m, param.c_str(), value);
    m.set_node_limit(2000000000);
    const long f = c->raw_fires;
    const double t2 = cp::timed_solve(m);
    double v = NAN; try { v = m.get_solution_value(); } catch(const std::exception &) {}
    std::println(stderr, "   second {} fires={} => {}", cp::summary(m, t2), c->raw_fires - f, std::fabs(v - g.opt) < 1e-4 ? "search restarted, the cut is gone (optimum KEPT)" : "resumed, the cut survived (optimum LOST)");
    c->printed = true;
}

// ---------------------------------------------------------------- x_objread ---
// A handle's get_solution_value: no MIPNODE code returns the node's relaxation value,
// so can it be computed from Obj and ObjCon, read on the callback's model?
struct obj_ctx { const grb::gurobi_api * api; int cols; int left; std::vector<double> truth; };
static int obj_hook(grb::GRBmodel * cbm, void * cbdata, int where, void * usr) {
    auto & c = *static_cast<obj_ctx *>(usr);
    if(where != W_MIPNODE || c.left <= 0) return 0;
    int status = 0;
    c.api->cbget(cbdata, where, NODE_STATUS, &status);
    if(status != GRB_OPTIMAL_) return 0;
    --c.left;
    std::vector<double> x(static_cast<size_t>(c.cols)), o(static_cast<size_t>(c.cols));
    c.api->cbget(cbdata, where, NODE_REL, x.data());
    const int e1 = c.api->getdblattrarray(cbm, "Obj", 0, c.cols, o.data());
    double k = NAN, bnd = NAN, nod = NAN;
    int sense = 0;
    const int e2 = c.api->getdblattr(cbm, "ObjCon", &k);
    c.api->getintattr(cbm, "ModelSense", &sense);
    c.api->cbget(cbdata, where, NODE_OBJBND, &bnd);
    c.api->cbget(cbdata, where, NODE_NODCNT, &nod);
    double v = k;
    for(int j = 0; j < c.cols; ++j) v += o[static_cast<size_t>(j)] * x[static_cast<size_t>(j)];
    std::println(stderr, "   nodcnt={} Obj err={} ObjCon err={} ModelSense={} computed c'x+k={:.6g} MIPNODE_OBJBND={:.6g}", nod, e1, e2, sense, v, bnd);
    return 0;
}
static void x_objread() {
    M m; scope_settings(m);
    auto g = cp::build_gadgets(m, 10, false);
    m.set_time_limit(duration<double>(5.0));
    set_int(m, "OutputFlag", 1);
    obj_ctx c{&m.native_api(), static_cast<int>(m.num_variables()), 8, {}};
    auto [env, model] = m.native_model();
    m.native_api()._check(env, m.native_api().setcallbackfunc(model, obj_hook, &c));
    const double t = cp::timed_solve(m);
    std::fflush(stdout);
    std::println(stderr, "== X_OBJREAD {} (objective of the gadgets at x: see computed value)", cp::summary(m, t));
}

// --------------------------------------------------------------- x_default ---
// Scope cut under default settings, independent gadgets: Disconnected at its default
// may solve the components on their own. Does the hook fire, and does it still bite?
static void x_default() {
    for(const bool flip : {false, true}) for(const auto & cfg : {"default", "Disconnected=0", "Heuristics=0 Cuts=0"}) {
        M m; one_thread(m);
        auto g = cp::build_gadgets(m, 10, flip);
        m.set_time_limit(duration<double>(30.0));
        if(std::string(cfg) == "Disconnected=0") set_int(m, "Disconnected", 0);
        if(std::string(cfg) == "Heuristics=0 Cuts=0") { set_dbl(m, "Heuristics", 0.0); set_int(m, "Cuts", 0); }
        auto s = std::make_shared<cp::stats>();
        auto tr = std::make_shared<cp::scope_tracker>();
        tr->cut = g.invalid_global_cut(); tr->trigger = g.worse_state(); tr->state = [&g](const auto & x) { return g.state(x); };
        auto * c = attach_hook(m, cp::watched([tr](const auto & x, const cp::add_fn & add) { if(tr->wants_add(x)) tr->add_error = add(tr->cut); }, s, [&g](const auto & x) { return g.objective(x); }), "cbcut_precrush");
        const double t = cp::timed_solve(m);
        double value = NAN; try { value = m.get_solution_value(); } catch(const std::exception &) {}
        std::println(stderr, "== X_DEFAULT S1 flip={} {}: {} (known opt {}) => {}\n   {}\n   {}", flip, cfg, cp::summary(m, t), g.opt, std::fabs(value - g.opt) < 1e-4 ? "optimum KEPT" : "optimum LOST", tr->line(), c->line());
        c->printed = true;
    }
}

// ------------------------------------------------------------------ x_slot ---
// One native slot: the wrapper's attach replaces a hook set natively, then clears it.
static void x_slot() {
    M m; one_thread(m);
    auto a = cp::build_atsp(m, 10);
    m.set_time_limit(duration<double>(5.0));
    auto s = std::make_shared<cp::stats>();
    auto * c = attach_hook(m, cp::watched([](const auto &, const auto &) {}, s), "cbcut_noprecrush");
    long wrapper_calls = 0;
    m.set_candidate_solution_callback([&](auto &) { ++wrapper_calls; });
    const double t = cp::timed_solve(m);
    auto [env, model] = m.native_model();
    auto & api = m.native_api();
    auto get = cp::symf<int (*)(grb::GRBmodel *, int (**)(grb::GRBmodel *, void *, int, void *))>(api.library_path(), "GRBgetcallbackfunc");
    int (*cb)(grb::GRBmodel *, void *, int, void *) = cut_hook;
    const int e = get(model, &cb);
    int lazyp = -1;
    api.getintparam(env, "LazyConstraints", &lazyp);
    std::println(stderr, "== X_SLOT native hook + wrapper candidate callback: {} native_fires={} wrapper_calls={}; after solve GRBgetcallbackfunc err={} slot_is_null={} slot_is_my_hook={} LazyConstraints={}", cp::summary(m, t), c->raw_fires, wrapper_calls, e, cb == nullptr, cb == cut_hook, lazyp);
    c->printed = true;
}

int main(int argc, char ** argv) {
    if(argc < 2) { std::println(stderr, "which experiment?"); return 2; }
    const std::string w = argv[1];
    {
        M probe;
        int major, minor, tech;
        probe.native_api().version(&major, &minor, &tech);
        std::println(stderr, "== release {}.{}.{}, experiment {}", major, minor, tech, w);
    }
    const auto arg = [&](int i, const char * d) { return std::string(argc > i ? argv[i] : d); };
    if(w == "tree") x_tree(std::stoi(arg(2, "40")), static_cast<unsigned>(std::stoi(arg(3, "5"))), static_cast<size_t>(std::stoi(arg(4, "5"))), arg(5, "0") == "1", arg(6, "cbcut_precrush"), std::stod(arg(7, "60")), arg(8, "0") == "1", std::stoi(arg(9, "0")));
    else if(w == "rounds") x_rounds(arg(2, "objective_step"), arg(3, "scope"));
    else if(w == "cover") x_cover(std::stoi(arg(2, "30")), std::stoi(arg(3, "100")), std::stod(arg(4, "20")), static_cast<size_t>(std::stoi(arg(5, "5"))), arg(6, "0") == "1", arg(7, "cbcut_precrush"), std::stoi(arg(8, "1")));
    else if(w == "trace") x_trace(arg(2, "cbcut_precrush"), arg(3, "none"), arg(4, "0") == "1", std::stoi(arg(5, "4")));
    else if(w == "reduce") x_reduce(static_cast<unsigned>(std::stoi(arg(2, "11"))), arg(3, ""), arg(4, "0") == "1");
    else if(w == "binding") x_binding(arg(2, "cbcut_precrush"));
    else if(w == "flood") x_flood(arg(2, "cbcut_precrush"), std::stoi(arg(3, "2000")));
    else if(w == "q11") x_q11(std::stoi(arg(2, "26")));
    else if(w == "q12") x_q12();
    else if(w == "resume") x_resume(arg(2, "0") == "1", arg(3, "0") == "1", std::stol(arg(4, "20")), arg(5, "0") == "1", arg(6, "cbcut_precrush"));
    else if(w == "q3") x_q3();
    else if(w == "wodd") x_wodd(arg(2, ""));
    else if(w == "threads") x_threads();
    else if(w == "side") x_side();
    else if(w == "settings") x_settings(std::stoi(arg(2, "26")));
    else if(w == "s1default") x_s1_default(static_cast<unsigned>(std::stoi(arg(2, "11"))), arg(3, "default"));
    else if(w == "phase") x_phase();
    else if(w == "objread") x_objread();
    else if(w == "toggle") x_toggle(arg(2, "PreCrush"), std::stoi(arg(3, "1")));
    else if(w == "default") x_default();
    else if(w == "slot") x_slot();
    else { std::println(stderr, "unknown experiment {}", w); return 2; }
    flush_ctx();
}
