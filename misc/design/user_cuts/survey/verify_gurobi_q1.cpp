// Q1: PreCrush=1 cost with no hook. Nodes, Work (deterministic), wall, across Gurobi seeds to read noise.
#include "verify_gurobi_common.hpp"
using builder = std::function<void(M &)>;
static rawm dup_mkp(int rows, int cols, unsigned seed) {
    rawm r; r.sense = -1; std::mt19937 rng(seed);
    std::uniform_int_distribution<int> A(1, 1000), R(0, 500);
    std::vector<std::vector<double>> a(static_cast<size_t>(rows), std::vector<double>(static_cast<size_t>(cols)));
    for(auto & row : a) for(auto & v : row) v = A(rng);
    std::vector<int> ca, cb;
    for(int j = 0; j < cols; ++j) { double s = 0; for(auto & row : a) s += row[static_cast<size_t>(j)]; const double c = std::floor(s / rows + R(rng)); ca.push_back(r.bin(c)); cb.push_back(r.bin(c)); }
    for(auto & row : a) { std::vector<int> i; std::vector<double> v; double s = 0; for(int j = 0; j < cols; ++j) { i.push_back(ca[static_cast<size_t>(j)]); v.push_back(row[static_cast<size_t>(j)]); i.push_back(cb[static_cast<size_t>(j)]); v.push_back(row[static_cast<size_t>(j)]); s += 2 * row[static_cast<size_t>(j)]; } r.add(i, v, '<', std::floor(s / 2)); }
    return r;
}
static rawm par_int(unsigned seed, size_t cols, size_t rows, int u) {
    rawm r; r.sense = -1; std::mt19937 rng(seed);
    std::uniform_int_distribution<int> A(1, 60), C(1, 40);
    std::vector<std::vector<double>> a(rows, std::vector<double>(cols));
    for(auto & row : a) for(auto & v : row) v = A(rng);
    std::vector<int> p, q; for(size_t j = 0; j < cols; ++j) { const double c = C(rng); p.push_back(r.var(c, 0, u, 'I')); q.push_back(r.var(c, 0, u, 'I')); }
    for(size_t i = 0; i < rows; ++i) { double t = 0; std::vector<int> ind; std::vector<double> val; for(size_t j = 0; j < cols; ++j) { t += a[i][j]; ind.push_back(p[j]); val.push_back(a[i][j]); ind.push_back(q[j]); val.push_back(a[i][j]); } r.add(ind, val, '<', std::floor(t * u * 0.6)); }
    return r;
}
static rawm fixed_charge(unsigned seed, size_t fac, size_t cli) {
    rawm r; r.sense = 1; std::mt19937 rng(seed);
    std::uniform_int_distribution<int> C(1, 60), F(200, 600), DD(5, 30);
    std::vector<double> d(cli); for(auto & v : d) v = DD(rng);
    double tot = 0; for(double v : d) tot += v;
    std::vector<int> y; for(size_t i = 0; i < fac; ++i) y.push_back(r.bin(F(rng)));
    std::vector<std::vector<int>> f(fac, std::vector<int>(cli));
    for(auto & row : f) for(auto & v : row) v = r.var(C(rng), 0, 1e9, 'C');
    for(size_t j = 0; j < cli; ++j) { std::vector<int> i; for(size_t k = 0; k < fac; ++k) i.push_back(f[k][j]); r.add(i, std::vector<double>(i.size(), 1.0), '=', d[j]); }
    const double cap = std::ceil(tot / 4);
    for(size_t k = 0; k < fac; ++k) { std::vector<int> i; std::vector<double> v; for(size_t j = 0; j < cli; ++j) { i.push_back(f[k][j]); v.push_back(1); } i.push_back(y[k]); v.push_back(-cap); r.add(i, v, '<', 0); }
    return r;
}
int main(int argc, char ** argv) {
    { M p; print_release(p); }
    const std::string only = argc > 1 ? argv[1] : "";
    const double limit = argc > 2 ? std::atof(argv[2]) : 20;
    std::vector<std::pair<std::string, builder>> insts = {
        {"atsp26s1 (no parallel)", [](M & m) { cp::build_atsp(m, 26, false, 1); }},
        {"atsp30s1 (no parallel)", [](M & m) { cp::build_atsp(m, 30, false, 1); }},
        {"mkp5x100 (no parallel)", [](M & m) { cp::build_mkp(m, 5, 100, 3); }},
        {"fixedcharge30x60", [](M & m) { fixed_charge(5, 30, 60).load(m); }},
        {"parint20x8s2 (parallel ints)", [](M & m) { par_int(2, 20, 8, 3).load(m); }},
        {"parint30x6s3 (parallel ints)", [](M & m) { par_int(3, 30, 6, 3).load(m); }},
        {"mkp5x50 doubled (parallel binaries)", [](M & m) { dup_mkp(5, 50, 4).load(m); }},
    };
    for(const auto & [name, build] : insts) {
        if(!only.empty() && name.find(only) == std::string::npos) continue;
        for(int precrush : {0, 1}) for(int seed : {0, 1, 2}) {
            M m; seti(m, "OutputFlag", 1); seti(m, "LogToConsole", 0); seti(m, "Threads", 1); setd(m, "TimeLimit", limit); seti(m, "PreCrush", precrush); seti(m, "Seed", seed); build(m);
            hctx c; std::string dims; c.on_msg = [&](hctx &, const char * s) { std::string t(s); if(t.starts_with("Presolved:")) { dims = t; if(dims.back() == '\n') dims.pop_back(); } };
            auto [env, model] = m.native_model();
            c.api = &m.native_api(); c.cols = num_vars(m);
            m.native_api().setcallbackfunc(model, hook, &c);
            const sol s = solve_native(m, false);
            std::println(stderr, "{} PreCrush={} Seed={}: {} | {}", name, precrush, seed, line(s), dims);
            std::fflush(stderr);
        }
    }
    return 0;
}
