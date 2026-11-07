// Sweeps with process isolation: does a recipe overrun the time limit, or answer wrong, over many instances?
// naive: XPRSaddcuts on input columns: is a wrong optimum reachable, not only a stuck root?
#include "verify_xpress_hook.hpp"
#include <sys/wait.h>
#include <unistd.h>
using namespace mippp; using namespace mippp::operators;
using std::chrono::duration;
using cut = cp::raw_cut;
template <class Mo> static auto knap(Mo & m, unsigned seed, std::size_t cols, std::size_t rows) {
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> A(1, 100), R(0, 50);
    std::vector<std::vector<double>> a(rows, std::vector<double>(cols));
    for(auto & r : a) for(auto & v : r) v = A(rng);
    std::vector<double> c(cols), cap(rows);
    for(std::size_t j = 0; j < cols; ++j) { double s = 0; for(auto & r : a) s += r[j]; c[j] = std::floor(s / static_cast<double>(rows) + R(rng)); }
    for(std::size_t i = 0; i < rows; ++i) { double s = 0; for(double v : a[i]) s += v; cap[i] = std::floor(s / 2); }
    auto x = m.add_binary_variables(cols);
    const auto ids = std::views::iota(std::size_t{0}, cols);
    m.set_maximization();
    m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * x(j); }));
    for(std::size_t i = 0; i < rows; ++i) m.add_constraint(xsum(ids, [&, i](std::size_t j) { return a[i][j] * x(j); }) <= cap[i]);
    std::vector<int> col; for(std::size_t j = 0; j < cols; ++j) col.push_back(m.native_id(x(j)));
    return std::tuple{col, a, cap};
}
static cp::separator covers(std::vector<std::vector<double>> a, std::vector<double> cap, std::vector<int> col) {
    return [=](const std::vector<double> & p, const cp::add_fn & add) {
        for(std::size_t i = 0; i < a.size(); ++i) {
            std::vector<std::size_t> order(col.size());
            for(std::size_t j = 0; j < order.size(); ++j) order[j] = j;
            std::ranges::sort(order, [&](std::size_t l, std::size_t r) { return p[static_cast<std::size_t>(col[l])] > p[static_cast<std::size_t>(col[r])]; });
            double w = 0; cut k{{}, {}, 'L', 0};
            for(std::size_t j : order) {
                w += a[i][j]; k.ind.push_back(col[j]); k.val.push_back(1);
                if(w > cap[i]) { k.rhs = static_cast<double>(k.ind.size()) - 1; if(k.violation(p) > 1e-4) add(k); break; }
            }
        }
    };
}
// A child writes one word on a pipe: 0 right, 1 limit stop, 2 wrong value or status. No word and a signal: overran the limit, or crashed.
struct tally { int right = 0, stopped = 0, wrong = 0, overran = 0, crashed = 0, noop = 0; std::string first_bad; };
template <class F>
static void one(tally & t, const std::string & name, double limit, F f) {
    int fd[2]; if(pipe(fd)) std::exit(2);
    std::fflush(stderr);
    const pid_t p = fork();
    if(p == 0) { close(fd[0]); alarm(static_cast<unsigned>(limit) + 10); vx::quiet_flush = true; const int r = f(); const auto n = write(fd[1], &r, sizeof r); (void)n; _exit(0); }
    close(fd[1]);
    int r = -1; const auto got = read(fd[0], &r, sizeof r); close(fd[0]);
    int st = 0; waitpid(p, &st, 0);
    if(got != sizeof r) {
        if(WIFSIGNALED(st) && WTERMSIG(st) == SIGALRM) { ++t.overran; if(t.first_bad.empty()) t.first_bad = name + " overran"; }
        else { ++t.crashed; if(t.first_bad.empty()) t.first_bad = std::format("{} crashed sig {}", name, WIFSIGNALED(st) ? WTERMSIG(st) : -1); }
        return;
    }
    if(r == 0) ++t.right; else if(r == 1) ++t.stopped; else if(r == 3) ++t.noop; else { ++t.wrong; if(t.first_bad.empty()) t.first_bad = name + " wrong"; }
}
static void sweep_covers(const std::vector<std::string> & specs, int seeds, double limit) {
    for(auto [cols, rows] : std::vector<std::pair<std::size_t, std::size_t>>{{40, 5}, {60, 5}, {60, 10}}) for(const auto & spec : specs) {
        tally t;
        for(int seed = 1; seed <= seeds; ++seed) {
            double truth = NAN;
            { M m; m.set_thread_limit(1); m.set_time_limit(duration<double>(limit * 3)); knap(m, static_cast<unsigned>(seed), cols, rows); m.solve(); if(is_a<status::optimal>(m.get_status())) truth = m.get_solution_value(); }
            if(std::isnan(truth)) continue;
            one(t, std::format("covers {}x{} seed={}", cols, rows, seed), limit, [&] {
                M m; m.set_thread_limit(1); m.set_time_limit(duration<double>(limit));
                auto [col, a, cap] = knap(m, static_cast<unsigned>(seed), cols, rows);
                auto & c = vx::attach(m, covers(a, cap, col), vx::parse(spec));
                m.solve();
                if(is_a<status::time_limit>(m.get_status())) return 1;
                if(!is_a<status::optimal>(m.get_status()) || !cf::same(m.get_solution_value(), truth)) return 2;
                return c.added_native.load() == 0 ? 3 : 0;
            });
        }
        std::println(stderr, "covers {}x{} [{}] over {} seeds at limit {}: right={} right_but_no_cut_reached_native={} limit_stop={} wrong={} OVERRAN={} crashed={} {}", cols, rows, spec, seeds, limit, t.right, t.noop, t.stopped, t.wrong, t.overran, t.crashed, t.first_bad);
    }
}
static void naive(const std::string & spec) {
    tally t;
    int n_wrong_detail = 0;
    for(int n : {12, 14, 16, 18, 20, 24}) for(int seed = 1; seed <= 12; ++seed) {
        double truth = NAN;
        { M m; m.set_thread_limit(1); m.set_time_limit(duration<double>(5)); auto a = cp::build_atsp(m, n, false, static_cast<unsigned>(seed)); auto s = std::make_shared<cp::stats>(); vx::attach(m, cp::watched([a](const std::vector<double> & x, const cp::add_fn & add) { for(const auto & k : a.separate(x)) add(k); }, s), vx::parse("R1")); m.solve(); if(is_a<status::optimal>(m.get_status())) truth = m.get_solution_value(); }
        if(std::isnan(truth)) continue;
        for(const int shift : {7, 30}) {
            one(t, std::format("fixed {} first n={} seed={}", shift, n, seed), 5, [&] {
                M m; m.set_thread_limit(1); m.set_time_limit(duration<double>(5));
                auto f = m.add_binary_variables(static_cast<std::size_t>(shift));
                for(std::size_t i = 0; i < static_cast<std::size_t>(shift); ++i) m.add_constraint(1.0 * f(i) == 1.0);
                auto a = cp::build_atsp(m, n, false, static_cast<unsigned>(seed));
                auto & c = vx::attach(m, [a](const std::vector<double> & x, const cp::add_fn & add) { for(const auto & k : a.separate(x)) add(k); }, vx::parse(spec));
                m.solve();
                if(is_a<status::time_limit>(m.get_status())) return 1;
                const double got = is_a<status::optimal>(m.get_status()) ? m.get_solution_value() : NAN;
                const bool ok = is_a<status::optimal>(m.get_status()) && cf::same(got, truth);
                if(!ok) std::println(stderr, "  WRONG n={} seed={} shift={}: {} value {} truth {} native={}", n, seed, shift, cp::status_name(m.get_status()), got, truth, c.added_native.load());
                return ok ? (c.added_native.load() == 0 ? 3 : 0) : 2;
            });
            (void)n_wrong_detail;
        }
    }
    std::println(stderr, "fixed columns first [{}]: right={} no_cut={} limit_stop={} WRONG={} overran={} crashed={} {}", spec, t.right, t.noop, t.stopped, t.wrong, t.overran, t.crashed, t.first_bad);
}
// CG fuzz on models built so that presolve removes many columns: complements, duplicates, fixed columns, aggregated integers.
// All columns are nonnegative integers and rows are <=, so floor(Sum l a) x <= floor(Sum l b) is valid for every integer point,
// whichever columns presolve removes. Reaches native whenever violated.
struct row3 { std::vector<int> ind; std::vector<double> val; double rhs; };
struct cgm { std::vector<row3> rows; int ncols = 0; };
static cgm build_cg(M & m, unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> A(-5, 20), C(1, 40), Pct(0, 99), Y(0, 24);
    cgm g;
    const std::size_t nx = 25, nr = 8;
    auto x = m.add_binary_variables(nx);
    std::vector<int> ix; for(std::size_t j = 0; j < nx; ++j) ix.push_back(m.native_id(x(j)));
    m.set_maximization();
    std::vector<double> cost(nx); for(auto & v : cost) v = C(rng);
    std::vector<std::vector<double>> a(nr, std::vector<double>(nx)); for(auto & r : a) for(auto & v : r) v = A(rng);
    std::vector<double> b(nr); for(std::size_t i = 0; i < nr; ++i) { double t = 0; for(double v : a[i]) t += std::max(v, 0.0); b[i] = std::floor(t * 0.4); }
    std::vector<row3> kn(nr);
    for(std::size_t j = 0; j < nx; ++j) m.set_objective_coefficient(x(j), cost[j]);
    // complements
    std::vector<int> compl_of(nx, -1);
    for(std::size_t j = 0; j < nx; ++j) if(Pct(rng) < 30) {
        auto c = m.add_binary_variable();
        compl_of[j] = m.native_id(c);
        m.add_constraint(x(j) + c == 1);
        g.rows.push_back({{ix[j], compl_of[j]}, {1, 1}, 1}); g.rows.push_back({{ix[j], compl_of[j]}, {-1, -1}, -1});
    }
    // knapsack rows, a term written on the complement half of the time
    for(std::size_t i = 0; i < nr; ++i) {
        row3 r; r.rhs = b[i];
        for(std::size_t j = 0; j < nx; ++j) {
            if(compl_of[j] >= 0 && Pct(rng) < 50) { r.ind.push_back(compl_of[j]); r.val.push_back(-a[i][j]); r.rhs -= a[i][j]; }
            else { r.ind.push_back(ix[j]); r.val.push_back(a[i][j]); }
        }
        kn[i] = r;
    }
    // duplicates: same coefficients and cost as x_j in the knapsack rows, on a new column
    for(std::size_t j = 0; j < nx; ++j) if(Pct(rng) < 20) {
        auto d = m.add_binary_variable();
        const int id = m.native_id(d);
        m.set_objective_coefficient(d, cost[j]);
        for(auto & r : kn) for(std::size_t q = 0; q < r.ind.size(); ++q) if(r.ind[q] == ix[j]) { r.ind.push_back(id); r.val.push_back(r.val[q]); break; }
    }
    // fixed columns in the rows
    for(int k = 0; k < 5; ++k) {
        auto f = m.add_binary_variable();
        const int id = m.native_id(f);
        m.add_constraint(1.0 * f == 1.0);
        g.rows.push_back({{id}, {1}, 1}); g.rows.push_back({{id}, {-1}, -1});
        for(auto & r : kn) { const double v = A(rng); r.ind.push_back(id); r.val.push_back(v); r.rhs += v; }
    }
    // aggregated integers: y == x_a + x_b + x_c, worth a little
    for(int k = 0; k < 4; ++k) {
        auto y = m.add_integer_variable({.lower_bound = 0., .upper_bound = 3.});
        const auto ja = static_cast<std::size_t>(Y(rng)), jb = (ja + 1 + static_cast<std::size_t>(Y(rng)) % 7) % nx, jc = (jb + 1 + static_cast<std::size_t>(Y(rng)) % 7) % nx;
        if(ja == jb || jb == jc || ja == jc) continue;
        m.add_constraint(y - x(ja) - x(jb) - x(jc) == 0.0);
        const int id = m.native_id(y);
        m.set_objective_coefficient(y, static_cast<double>(C(rng)) / 4);
        g.rows.push_back({{id, ix[ja], ix[jb], ix[jc]}, {1, -1, -1, -1}, 0}); g.rows.push_back({{id, ix[ja], ix[jb], ix[jc]}, {-1, 1, 1, 1}, 0});
    }
    for(auto & r : kn) {
        // through the wrapper: terms as (native id) need handles, so write the row with add_constraint over a lambda
        g.rows.push_back(r);
    }
    g.ncols = static_cast<int>(m.num_variables());
    return g;
}
// add the knapsack rows natively so that their terms can stay in native ids
static void add_rows_native(M & m, const cgm & g, std::size_t skip_nonknap) {
    for(std::size_t i = skip_nonknap; i < g.rows.size(); ++i) {
        const auto & r = g.rows[i];
        std::vector<int> st{0, static_cast<int>(r.ind.size())};
        const char sense = 'L';
        const int e = m.native_api().addrows(m.native_model(), 1, static_cast<int>(r.ind.size()), &sense, &r.rhs, nullptr, st.data(), r.ind.data(), r.val.data());
        if(e) { std::println(stderr, "addrows failed {}", e); std::exit(2); }
    }
}
static cp::separator cg_sep(const cgm & g, unsigned seed) {
    auto gen = std::make_shared<std::mt19937>(seed * 977u);
    return [g, gen](const std::vector<double> & p, const cp::add_fn & add) {
        std::vector<std::size_t> tight;
        for(std::size_t i = 0; i < g.rows.size(); ++i) { double act = 0; const auto & r = g.rows[i]; for(std::size_t k = 0; k < r.ind.size(); ++k) act += r.val[k] * p[static_cast<std::size_t>(r.ind[k])]; if(act >= r.rhs - 1e-6) tight.push_back(i); }
        if(tight.empty()) return;
        const int ds[] = {2, 3, 4, 5, 7, 11};
        std::uniform_int_distribution<std::size_t> D(0, 5);
        for(int att = 0, added = 0; att < 400 && added < 3; ++att) {
            const int d = ds[D(*gen)]; std::uniform_int_distribution<int> K(0, d - 1);
            std::vector<double> col(static_cast<std::size_t>(g.ncols), 0.0); double rhs = 0;
            std::uniform_int_distribution<std::size_t> Pick(0, tight.size() - 1);
            for(int q = 0; q < 3; ++q) { const std::size_t i = tight[Pick(*gen)]; const double l = static_cast<double>(K(*gen)) / d; if(l == 0) continue; const auto & r = g.rows[i]; rhs += l * r.rhs; for(std::size_t k = 0; k < r.ind.size(); ++k) col[static_cast<std::size_t>(r.ind[k])] += l * r.val[k]; }
            cut c{{}, {}, 'L', std::floor(rhs + 1e-9)};
            for(int j = 0; j < g.ncols; ++j) { const double v = std::floor(col[static_cast<std::size_t>(j)] + 1e-9); if(v != 0) { c.ind.push_back(j); c.val.push_back(v); } }
            if(c.ind.empty()) continue;
            if(c.violation(p) > 1e-4) { add(c); ++added; }
        }
        // Covers of each row over all its columns, complements, duplicates and fixed ones included. All are binary here but the y:
        // skip rows with a general integer. S with all ones infeasible once the row's best case passes the rhs.
        for(std::size_t i : tight) {
            const auto & r = g.rows[i];
            if(r.ind.size() < 6) continue;
            std::vector<std::size_t> order; double best = 0;
            for(std::size_t k = 0; k < r.ind.size(); ++k) { if(r.val[k] > 0) order.push_back(k); else best += r.val[k]; }
            std::ranges::sort(order, [&](std::size_t l, std::size_t q) { return p[static_cast<std::size_t>(r.ind[l])] > p[static_cast<std::size_t>(r.ind[q])]; });
            cut c{{}, {}, 'L', 0};
            for(std::size_t k : order) {
                best += r.val[k]; c.ind.push_back(r.ind[k]); c.val.push_back(1);
                if(best > r.rhs + 1e-9) { c.rhs = static_cast<double>(c.ind.size()) - 1; if(c.violation(p) > 1e-4) add(c); break; }
            }
        }
    };
}
static void sweep_cg(const std::vector<std::string> & specs, int seeds) {
    for(const auto & spec : specs) {
        tally t; long reached = 0, reduced_runs = 0;
        for(int seed = 1; seed <= seeds; ++seed) {
            double truth = NAN; std::string st;
            { M m; m.set_thread_limit(1); m.set_time_limit(duration<double>(20)); auto g = build_cg(m, static_cast<unsigned>(seed)); const std::size_t nk = g.rows.size() - 8; add_rows_native(m, g, nk); m.solve(); st = cp::status_name(m.get_status()); if(is_a<status::optimal>(m.get_status())) truth = m.get_solution_value(); }
            if(std::isnan(truth)) continue;
            int fd[2]; if(pipe(fd)) std::exit(2);
            std::fflush(stderr);
            const pid_t p = fork();
            if(p == 0) {
                close(fd[0]); alarm(30); vx::quiet_flush = true;
                M m; m.set_thread_limit(1); m.set_time_limit(duration<double>(15));
                auto g = build_cg(m, static_cast<unsigned>(seed)); const std::size_t nk = g.rows.size() - 8; add_rows_native(m, g, nk);
                auto & c = vx::attach(m, cg_sep(g, static_cast<unsigned>(seed)), vx::parse(spec));
                m.solve();
                long msg[3] = {0, c.added_native.load(), (c.cols_seen_lo < c.input_cols) ? 1L : 0L};
                if(seed <= 2) { std::println(stderr, "  seed {}: {} | {}", seed, cp::summary(m, 0), c.line()); std::fflush(stderr); }
                if(is_a<status::time_limit>(m.get_status())) msg[0] = 1;
                else if(!is_a<status::optimal>(m.get_status()) || !cf::same(m.get_solution_value(), truth)) { msg[0] = 2; std::println(stderr, "  WRONG seed={} [{}]: {} truth {} native={}", seed, spec, cp::summary(m, 0), truth, c.added_native.load()); std::fflush(stderr); }
                const auto n = write(fd[1], msg, sizeof msg); (void)n; _exit(0);
            }
            close(fd[1]);
            long msg[3] = {-1, 0, 0}; const auto got = read(fd[0], msg, sizeof msg); close(fd[0]);
            int w = 0; waitpid(p, &w, 0);
            if(got != sizeof msg) { if(WIFSIGNALED(w) && WTERMSIG(w) == SIGALRM) ++t.overran; else { ++t.crashed; if(t.first_bad.empty()) t.first_bad = std::format("seed {} signal {}", seed, WIFSIGNALED(w) ? WTERMSIG(w) : -1); } continue; }
            if(msg[0] == 0) ++t.right; else if(msg[0] == 1) ++t.stopped; else ++t.wrong;
            reached += msg[1]; reduced_runs += msg[2];
        }
        std::println(stderr, "CG fuzz on reduced models [{}] over {} seeds: right={} limit_stop={} WRONG={} OVERRAN={} crashed={} | native cuts in all={} runs with presolved columns={} {}", spec, seeds, t.right, t.stopped, t.wrong, t.overran, t.crashed, reached, reduced_runs, t.first_bad);
    }
}
int main(int argc, char ** argv) {
    const std::string w = argc > 1 ? argv[1] : "";
    { M m; std::println(stderr, "release {}", vx::version(m)); }
    vx::quiet_flush = true;
    if(w == "covers") sweep_covers({"R1", "R1,add2", "R1,first2", "R2", "R2cr", "R2cr,add2"}, 30, 10);
    if(w == "slow") {
        // Is R1's limit stop at 60x10 a stall at one node, or just a search the cuts did not shorten?
        for(int seed = 1; seed <= 30; ++seed) for(const char * sp : {"", "R1", "R1,first2", "R2"}) {
            std::fflush(stderr);
            const pid_t p = fork();
            if(p == 0) {
                alarm(45);
                M m; m.set_thread_limit(1); m.set_time_limit(duration<double>(30));
                auto [col, a, cap] = knap(m, static_cast<unsigned>(seed), 60, 10);
                auto s2 = std::make_shared<cp::stats>();
                vx::ctx * c = nullptr;
                if(sp[0]) c = &vx::attach(m, cp::watched(covers(a, cap, col), s2), vx::parse(sp));
                const double t = cp::timed_solve(m);
                std::println(stderr, "60x10 seed={} [{}]: {} | calls={} native={} seen={} rounds={}", seed, sp[0] ? sp : "no hook", cp::summary(m, t), s2->calls.load(), c ? c->added_native.load() : 0, c ? c->nodes.size() : 0, c ? c->max_rounds : 0);
                std::fflush(stderr);
                _exit(0);
            }
            int st = 0; waitpid(p, &st, 0);
            if(WIFSIGNALED(st)) std::println(stderr, "60x10 seed={} [{}]: killed by signal {}", seed, sp, WTERMSIG(st));
        }
    }
    if(w == "add2") sweep_covers({"R1,add2", "R1"}, 12, 5);
    if(w == "cg") sweep_cg({"R1", "R2", "R2cr", "R1,first2"}, 40);
    if(w == "naive") { naive("R2,no_presolverow"); naive("R2cr,no_presolverow"); naive("R2"); naive("R1"); }
}
