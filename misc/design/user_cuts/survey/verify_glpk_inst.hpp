#pragma once
// Instances with an exact oracle: every feasible point is enumerated, so "valid" and
// "optimal" need no solver. Max c'x, Ax <= b, x binary, plus side columns.
#include "verify_glpk_model.hpp"
namespace vg {
struct oracle {
    int n = 0;
    std::vector<std::vector<double>> a;
    std::vector<double> b, c;
    std::vector<std::uint32_t> feas;
    std::vector<int> col;     // native column of bit k
    double opt = -1e300;
    std::uint32_t arg = 0;
    double value(std::uint32_t mask) const { double s = 0; for(int k = 0; k < n; ++k) if(mask >> k & 1) s += c[static_cast<std::size_t>(k)]; return s; }
    void enumerate() {
        const std::uint32_t total = 1u << n;
        const auto R = a.size();
        std::vector<double> lhs(R, 0.0);
        feas.clear();
        for(std::uint32_t g = 0; g < total; ++g) {
            bool ok = true;
            for(std::size_t i = 0; i < R && ok; ++i) {
                double s = 0;
                for(int k = 0; k < n; ++k) if(g >> k & 1) s += a[i][static_cast<std::size_t>(k)];
                if(s > b[i] + 1e-9) ok = false;
            }
            if(!ok) continue;
            feas.push_back(g);
            const double v = value(g);
            if(v > opt) { opt = v; arg = g; }
        }
    }
    // max of w'x over the feasible points: w'x <= this is valid and tight
    double support(const std::vector<double> & w) const {
        double best = -1e300;
        for(auto g : feas) { double s = 0; for(int k = 0; k < n; ++k) if(g >> k & 1) s += w[static_cast<std::size_t>(k)]; best = std::max(best, s); }
        return best;
    }
    double support_in_box(const std::vector<double> & w, std::uint32_t must1, std::uint32_t must0) const {
        double best = -1e300;
        for(auto g : feas) {
            if((g & must1) != must1 || (g & must0)) continue;
            double s = 0; for(int k = 0; k < n; ++k) if(g >> k & 1) s += w[static_cast<std::size_t>(k)];
            best = std::max(best, s);
        }
        return best;
    }
};
template <class Model>
oracle build_oracle(Model & m, unsigned seed, int n, int rows, bool correlated = true) {
    oracle o;
    o.n = n;
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> A(1, 100), N(0, 30), C(1, 100);
    const auto Ns = static_cast<std::size_t>(n);
    o.a.assign(static_cast<std::size_t>(rows), std::vector<double>(Ns));
    for(auto & r : o.a) for(auto & v : r) v = A(rng);
    o.c.resize(Ns);
    for(std::size_t j = 0; j < Ns; ++j) {
        double s = 0; for(auto & r : o.a) s += r[j];
        o.c[j] = correlated ? std::floor(s / rows) + N(rng) : C(rng);
    }
    for(auto & r : o.a) { double s = 0; for(double v : r) s += v; o.b.push_back(std::floor(s / 2)); }
    auto x = m.add_binary_variables(Ns);
    const auto ids = std::views::iota(std::size_t{0}, Ns);
    m.set_maximization();
    m.set_objective(xsum(ids, [&](std::size_t j) { return o.c[j] * x(j); }));
    for(const auto & r : o.a) m.add_constraint(xsum(ids, [&r, &x](std::size_t j) { return r[j] * x(j); }) <= o.b[static_cast<std::size_t>(&r - o.a.data())]);
    for(std::size_t j = 0; j < Ns; ++j) o.col.push_back(m.native_id(x(j)));
    o.enumerate();
    return o;
}
inline void generators_off(M & m) {
    m.model_params.mir_cuts = m.model_params.gmi_cuts = m.model_params.cov_cuts = m.model_params.clq_cuts = v1::GLP_OFF;
}
inline void heuristics_off(M & m) { m.model_params.fp_heur = m.model_params.sr_heur = m.model_params.ps_heur = v1::GLP_OFF; }
template <class Mm> std::string st(Mm & m) { return cp::status_name(m.get_status()); }
}  // namespace vg
