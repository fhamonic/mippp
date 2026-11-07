// Pin the threshold: absolute v on cuts of different length and different largest coefficient, frozen root.
#include "verify_cplex_common.hpp"
using namespace cp;
using std::chrono::duration;
static bool spin_case(M & m, const double limit, const char * line, const std::function<void(M &)> & attach_fn) {
    (void)line; attach_fn(m);
    m.set_time_limit(duration<double>(limit));
    const double t = timed_solve(m);
    return t > limit * 0.9 && m.get_node_count() == 0;
}
int main() {
    { M m; print_version(m); }
    const double limit = 1.5;
    const std::vector<double> rs = {3e-7, 5e-7, 7e-7, 8e-7, 9e-7, 9.5e-7, 1.05e-6, 1.1e-6, 1.2e-6, 1.5e-6, 2e-6, 3e-6, 5e-6};
    for(const auto & [n, seed, big] : std::initializer_list<std::tuple<int, unsigned, double>>{{22, 1, 1.0}, {26, 5, 1.0}, {31, 5, 1.0}, {26, 5, 1000.0}, {31, 5, 1000.0}, {26, 5, 1e6}}) {
        std::string line = std::format("relaxed DFJ atsp{}s{} a_inf={:g}:", n, seed, big);
        int nz = 0;
        for(const double r : rs) {
            M m; m.set_thread_limit(1); auto a = build_atsp(m, n, false, seed); all_families_off(m); vparam(m, CPX_PARAM_MIRCUTS, 2); vparam(m, CPX_PARAM_EACHCUTLIM, 0);
            auto cutp = std::make_shared<std::optional<raw_cut>>();
            bool sp = spin_case(m, limit, "", [&](M & mm) {
                vattach(mm, [=, &nz](const std::vector<double> & x, const add_fn & add) {
                    if(!*cutp) {
                        auto cs = a.separate(x, 1);
                        if(cs.empty()) return;
                        raw_cut c = cs.front();
                        // a weakening (x >= 0, sense >=): raise the coefficient of a term the point has at zero
                        if(big != 1.0) for(std::size_t t = 0; t < c.ind.size(); ++t) if(x[static_cast<std::size_t>(c.ind[t])] < 1e-12) { c.val[t] = big; break; }
                        c.rhs = c.activity(x) + r * big;
                        nz = static_cast<int>(c.ind.size());
                        *cutp = c;
                    }
                    add(**cutp);
                });
            });
            line += std::format(" {:g}{}", r, sp ? "=SPIN" : "=ok");
        }
        std::println(stderr, "{} nz={} (v/a_inf=result)", line, nz);
    }
    // objective floor, largest coefficient = the largest cost; three instances
    for(const auto & [rows, cols, seed] : std::initializer_list<std::tuple<int, int, unsigned>>{{5, 60, 3}, {5, 100, 3}, {30, 100, 7}}) {
        std::string line = std::format("objective floor mkp{}x{}s{}:", rows, cols, seed);
        double ainf = 0, n2 = 0;
        for(const double r : rs) {
            M m; m.set_thread_limit(1); build_mkp(m, rows, cols, seed); all_families_off(m); vparam(m, CPX_PARAM_MIRCUTS, 2); vparam(m, CPX_PARAM_EACHCUTLIM, 0);
            auto [env, lp] = m.native_model();
            const int N = static_cast<int>(m.num_variables());
            std::vector<double> c(static_cast<std::size_t>(N));
            m.native_api()._check(env, m.native_api().getobj(env, lp, c.data(), 0, N - 1));
            ainf = 0; n2 = 0; for(double w : c) { ainf = std::max(ainf, std::fabs(w)); n2 += w * w; }
            const double v = r * ainf;
            auto init = std::make_shared<std::optional<double>>();
            bool sp = spin_case(m, limit, "", [&](M & mm) {
                vattach(mm, [=](const std::vector<double> & x, const add_fn & add) {
                    raw_cut cut{{}, {}, 'L', 0}; double act = 0;
                    for(std::size_t j = 0; j < c.size(); ++j) if(c[j] != 0) { cut.ind.push_back(static_cast<int>(j)); cut.val.push_back(c[j]); act += c[j] * x[j]; }
                    if(!*init) *init = act - v;
                    cut.rhs = **init;
                    add(cut);
                });
            });
            line += std::format(" {:g}{}", r, sp ? "=SPIN" : "=ok");
        }
        std::println(stderr, "{} a_inf={:g} a_2={:.1f} (v/a_inf=result)", line, ainf, std::sqrt(n2));
    }
    // how long does a slow-but-moving root loop last? fresh cut per point, violation 1e-5 a_inf, 60 s limit.
    for(const double r : {1e-5, 1e-4}) for(const int cutpass : {0, 50}) {
        M m; m.set_thread_limit(1); auto a = build_atsp(m, 26, false, 5); if(cutpass) vparam(m, CPX_PARAM_CUTPASS, cutpass);
        m.set_time_limit(duration<double>(60));
        auto & h = vattach(m, [=](const std::vector<double> & x, const add_fn & add) {
            auto cs = a.separate(x, 1); if(cs.empty()) return;
            raw_cut c = cs.front();
            const double rhs = c.activity(x) + r;
            if(rhs > 1) return;
            c.rhs = rhs; add(c);
        });
        const double t = timed_solve(m);
        std::println(stderr, "slow loop atsp26s5 fresh cut violation {:g} CUTPASS={}: {} added={}", r, cutpass, summary(m, t), h.added.load());
    }
}
