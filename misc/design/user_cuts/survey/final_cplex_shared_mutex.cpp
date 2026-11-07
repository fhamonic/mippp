// One mutex shared by the three CPLEX contexts the wrapper would register:
// RELAXATION (user cuts), CANDIDATE (lazy) and GLOBAL_PROGRESS, which CPLEX
// runs under a lock of its own. Does the pair of locks deadlock? A hang shows
// as the process outliving its 15 s limit and the outer timeout killing it.
#include "verify_cplex_common.hpp"
#include <random>
using namespace cp;
using std::chrono::duration;

struct mkp_data {
    std::vector<std::vector<double>> a;
    std::vector<double> b;
};
static mkp_data build(M & m, const int rows, const int cols, unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> A(1, 1000), R(0, 500);
    const auto C = static_cast<std::size_t>(cols);
    mkp_data d;
    d.a.assign(static_cast<std::size_t>(rows), std::vector<double>(C));
    for(auto & row : d.a) for(auto & v : row) v = A(rng);
    std::vector<double> c(C);
    for(std::size_t j = 0; j < C; ++j) {
        double sum = 0;
        for(const auto & row : d.a) sum += row[j];
        c[j] = std::floor(sum / rows + R(rng));
    }
    auto x = m.add_binary_variables(C);
    const auto ids = std::views::iota(std::size_t{0}, C);
    m.set_maximization();
    m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * x(j); }));
    for(const auto & row : d.a) {
        double sum = 0;
        for(double v : row) sum += v;
        d.b.push_back(std::floor(sum / 2));
        m.add_constraint(xsum(ids, [&row, &x](std::size_t j) { return row[j] * x(j); }) <= d.b.back());
    }
    return d;
}

struct shared {
    const cx::cplex_api * api;
    const vsyms * s;
    const mkp_data * d;
    int cols;
    std::mutex mx;  // the wrapper's one mutex, per solve
    std::atomic<long> rel{0}, cand{0}, prog{0}, cuts{0}, errs{0};
    int spin_prog_us = 0, spin_rel_us = 0;
    bool cplex_call_in_progress = true;
};
static void spin(int us) { if(!us) return; const auto u = clk::now() + std::chrono::microseconds(us); while(clk::now() < u) {} }

static int hook(ctxptr context, cxlong id, void * usr) {
    auto & h = *static_cast<shared *>(usr);
    if(id == CPX_CALLBACKCONTEXT_RELAXATION) {
        std::vector<double> x(static_cast<std::size_t>(h.cols));
        const std::scoped_lock lock(h.mx);  // held across the CPLEX calls, as a handle would be
        if(h.s->point(context, x.data(), 0, h.cols - 1, nullptr)) return 0;
        ++h.rel;
        spin(h.spin_rel_us);
        for(std::size_t r = 0; r < h.d->a.size(); ++r) {
            std::vector<int> order(static_cast<std::size_t>(h.cols));
            std::iota(order.begin(), order.end(), 0);
            std::ranges::sort(order, [&](int i, int j) { return x[i] > x[j]; });
            double w = 0, lhs = 0;
            std::vector<int> cover;
            for(int j : order) {
                cover.push_back(j);
                w += h.d->a[r][static_cast<std::size_t>(j)];
                lhs += x[static_cast<std::size_t>(j)];
                if(w > h.d->b[r]) break;
            }
            if(w <= h.d->b[r]) continue;
            const double rhs = static_cast<double>(cover.size()) - 1;
            if(lhs <= rhs + 1e-6) continue;
            std::vector<double> ones(cover.size(), 1.0);
            const int beg = 0, purge = CPX_USECUT_PURGE, local = 0;
            const char sense = 'L';
            const int e = h.api->callbackaddusercuts(context, 1, static_cast<int>(cover.size()), &rhs, &sense, &beg,
                                                     cover.data(), ones.data(), &purge, &local);
            if(e) ++h.errs; else ++h.cuts;
        }
    } else if(id == CPX_CALLBACKCONTEXT_CANDIDATE) {
        std::vector<double> x(static_cast<std::size_t>(h.cols));
        const std::scoped_lock lock(h.mx);
        h.s->candpoint(context, x.data(), 0, h.cols - 1, nullptr);
        ++h.cand;
    } else if(id == CPX_CALLBACKCONTEXT_GLOBAL_PROGRESS) {
        const std::scoped_lock lock(h.mx);
        ++h.prog;
        if(h.cplex_call_in_progress) {
            double v = 0;
            h.s->infodbl(context, CPXCALLBACKINFO_BEST_BND, &v);
            h.s->infodbl(context, CPXCALLBACKINFO_BEST_SOL, &v);
        }
        spin(h.spin_prog_us);
    }
    return 0;
}

int main(int, char ** argv) {
    { M m; print_version(m); }
    const int spin_prog = argv[1] ? std::atoi(argv[1]) : 0;
    const int spin_rel = (argv[1] && argv[2]) ? std::atoi(argv[2]) : 0;
    for(int rep = 0; rep < 3; ++rep) {
        for(const char * inst : {"mkp10x200", "mkp30x100"}) {
            M m;
            const mkp_data d = inst[3] == '3' ? build(m, 30, 100, 7 + rep) : build(m, 10, 200, 3 + rep);
            m.set_time_limit(duration<double>(15));
            auto [env, lp] = m.native_model();
            shared h;
            h.api = &m.native_api(); h.s = &vsyms::get(m); h.d = &d; h.cols = static_cast<int>(m.num_variables());
            h.spin_prog_us = spin_prog; h.spin_rel_us = spin_rel;
            m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp,
                CPX_CALLBACKCONTEXT_RELAXATION | CPX_CALLBACKCONTEXT_CANDIDATE | CPX_CALLBACKCONTEXT_GLOBAL_PROGRESS, hook, &h));
            const double t = timed_solve(m);
            std::println("rep {} {} spin_prog={}us spin_rel={}us: {} | rel={} cand={} prog={} cuts={} add_errors={}", rep, inst,
                         spin_prog, spin_rel, summary(m, t), h.rel.load(), h.cand.load(), h.prog.load(), h.cuts.load(), h.errs.load());
            std::fflush(stdout);
            m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, 0, nullptr, nullptr));
        }
    }
}
