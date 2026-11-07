// E: what presolve off costs. One instance, one configuration, one line.
//   on       wrapper default: presolve on, no user cuts
//   off      recipe path, no user cuts
//   offcuts  recipe path, user cuts (family separator, or generic covers on MPS rows)
//   npp      4.65+ glp_npp route: preprocess1 + build + presolve-off search on Q, no user cuts
//   nppcuts  same, with the family separator mapped by name; a term on a column the route
//            eliminated makes the cut unusable and it is skipped (counted)
#include "verify_glpk_inst.hpp"
using namespace vg;
using cp::raw_cut;

struct npp_api {
    using glp_prep = void;
    glp_prep * (*alloc)();
    void (*load)(glp_prep *, glp_prob *, int, int);
    int (*pre1)(glp_prep *, int);
    void (*build)(glp_prep *, glp_prob *);
    void (*post)(glp_prep *, glp_prob *);
    void (*obtain)(glp_prep *, glp_prob *);
    void (*free_)(glp_prep *);
    static npp_api of(const Base & m) {
        const auto l = m.native_api().library_path();
        npp_api a;
        a.alloc = cp::symf<decltype(a.alloc)>(l, "glp_npp_alloc_wksp");
        a.load = cp::symf<decltype(a.load)>(l, "glp_npp_load_prob");
        a.pre1 = cp::symf<decltype(a.pre1)>(l, "glp_npp_preprocess1");
        a.build = cp::symf<decltype(a.build)>(l, "glp_npp_build_prob");
        a.post = cp::symf<decltype(a.post)>(l, "glp_npp_postprocess");
        a.obtain = cp::symf<decltype(a.obtain)>(l, "glp_npp_obtain_sol");
        a.free_ = cp::symf<decltype(a.free_)>(l, "glp_npp_free_wksp");
        return a;
    }
};

// Generic covers over the rows of a loaded problem. Valid for every integer point: non
// binaries enter at their minimum activity, negative binaries through their complement.
struct cover_rows {
    struct term { int j; double a; };
    struct row { std::vector<term> bin; double b; };
    std::vector<row> rows;
    void load(const v1::glpk_api & api, glp_prob * P) {
        const int m = api.get_num_rows(P), n = api.get_num_cols(P);
        std::vector<int> ind(static_cast<std::size_t>(n) + 1);
        std::vector<double> val(static_cast<std::size_t>(n) + 1);
        for(int i = 1; i <= m; ++i) {
            const int t = api.get_row_type(P, i);
            const int len = api.get_mat_row(P, i, ind.data(), val.data());
            const auto add = [&](double sign, double side) {
                row r; r.b = side * sign;
                for(int k = 1; k <= len; ++k) {
                    const int j = ind[static_cast<std::size_t>(k)];
                    const double a = sign * val[static_cast<std::size_t>(k)];
                    const bool bin = api.get_col_kind(P, j) == v1::GLP_BV;
                    if(bin && api.get_col_lb(P, j) == 0.0 && api.get_col_ub(P, j) == 1.0) { r.bin.push_back({j, a}); continue; }
                    const double lb = api.get_col_lb(P, j), ub = api.get_col_ub(P, j);
                    const int ct = api.get_col_type(P, j);
                    if(a > 0) { if(ct == v1::GLP_FR || ct == v1::GLP_UP) return; r.b -= a * lb; }
                    else { if(ct == v1::GLP_FR || ct == v1::GLP_LO) return; r.b -= a * ub; }
                }
                if(r.bin.size() >= 2) rows.push_back(std::move(r));
            };
            if(t == v1::GLP_UP || t == v1::GLP_DB || t == v1::GLP_FX) add(1.0, api.get_row_ub(P, i));
            if(t == v1::GLP_LO || t == v1::GLP_DB || t == v1::GLP_FX) add(-1.0, api.get_row_lb(P, i));
        }
    }
    void separate(const std::vector<double> & x, const cp::add_fn & add) const {
        struct it { int j; double abs; double cv; bool neg; };
        std::vector<it> v;
        for(const auto & r : rows) {
            v.clear();
            double b = r.b;
            for(const auto & t : r.bin) {
                const double xj = x[static_cast<std::size_t>(t.j)];
                if(t.a >= 0) v.push_back({t.j, t.a, xj, false});
                else { b -= t.a; v.push_back({t.j, -t.a, 1 - xj, true}); }
            }
            std::ranges::sort(v, [](const it & l, const it & u) { return l.cv > u.cv; });
            std::vector<it> c;
            double w = 0;
            for(const auto & e : v) { c.push_back(e); w += e.abs; if(w > b + 1e-7) break; }
            if(w <= b + 1e-7) continue;
            // minimal: drop members the cover does not need, least fractional weight first
            for(std::size_t k = c.size(); k-- > 0;) if(w - c[k].abs > b + 1e-7) { w -= c[k].abs; c.erase(c.begin() + static_cast<std::ptrdiff_t>(k)); }
            double act = 0;
            for(const auto & e : c) act += e.cv;
            const double s = static_cast<double>(c.size());
            if(act <= s - 1 + 1e-3) continue;
            raw_cut cut; cut.sense = 'L'; cut.rhs = s - 1;
            for(const auto & e : c) {
                cut.ind.push_back(e.j); cut.val.push_back(e.neg ? -1.0 : 1.0);
                if(e.neg) cut.rhs -= 1.0;
            }
            add(cut);
        }
    }
};

struct built { cp::separator sep; std::string fam; };

static built build_instance(M & m, const std::string & spec) {
    std::vector<std::string> a;
    { std::stringstream ss(spec); std::string t; while(std::getline(ss, t, ':')) a.push_back(t); }
    const auto I = [&](std::size_t k) { return std::stoi(a.at(k)); };
    const std::string & fam = a.at(0);
    auto & api = m.native_api();
    if(fam == "mps") {
        auto & n = natives::of(m);
        const std::string path = spec.substr(4);
        const int rc = n.read_mps(m.native_model(), 2, nullptr, path.c_str());
        if(rc != 0) { std::println(stderr, "cannot read {}", path); std::exit(4); }
        auto cover = std::make_shared<cover_rows>();
        cover->load(api, m.native_model());
        return {[cover](const std::vector<double> & x, const cp::add_fn & add) { cover->separate(x, add); }, "mps"};
    }
    if(fam == "atsp") {
        auto at = cp::build_atsp(m, I(1), false, static_cast<unsigned>(I(2)));
        return {[at](const std::vector<double> & x, const cp::add_fn & add) { for(const auto & c : at.separate(x)) add(c); }, fam};
    }
    if(fam == "mkp") {
        cp::build_mkp(m, I(1), I(2), static_cast<unsigned>(I(3)));
        auto cover = std::make_shared<cover_rows>();
        cover->load(api, m.native_model());
        return {[cover](const std::vector<double> & x, const cp::add_fn & add) { cover->separate(x, add); }, fam};
    }
    if(fam == "gap") {  // generalized assignment: agents, jobs, seed
        const auto A = static_cast<std::size_t>(I(1)), J = static_cast<std::size_t>(I(2));
        std::mt19937 rng(static_cast<unsigned>(I(3)));
        std::uniform_int_distribution<int> W(5, 25), C(10, 50);
        std::vector<double> w(A * J), c(A * J), cap(A, 0.0);
        for(auto & v : w) v = W(rng);
        for(std::size_t i = 0; i < A * J; ++i) c[i] = 111 - 3 * w[i] + C(rng) * 0.4;
        for(std::size_t i = 0; i < A; ++i) { double s = 0; for(std::size_t j = 0; j < J; ++j) s += w[i * J + j]; cap[i] = std::floor(0.8 * s / static_cast<double>(A)); }
        auto x = m.add_binary_variables(A * J);
        const auto As = std::views::iota(std::size_t{0}, A); const auto Js = std::views::iota(std::size_t{0}, J);
        m.set_minimization();
        m.set_objective(xsum(As, [&](std::size_t i) { return xsum(Js, [&, i](std::size_t j) { return c[i * J + j] * x(i * J + j); }); }));
        for(std::size_t j : Js) m.add_constraint(xsum(As, [&, j](std::size_t i) { return 1.0 * x(i * J + j); }) == 1);
        for(std::size_t i : As) m.add_constraint(xsum(Js, [&, i](std::size_t j) { return w[i * J + j] * x(i * J + j); }) <= cap[i]);
        auto cover = std::make_shared<cover_rows>();
        cover->load(api, m.native_model());
        return {[cover](const std::vector<double> & x_, const cp::add_fn & add) { cover->separate(x_, add); }, fam};
    }
    if(fam == "ufl") {  // uncapacitated facility location, aggregated links with M = factor * |C|
        const auto F = static_cast<std::size_t>(I(1)), C = static_cast<std::size_t>(I(2));
        std::mt19937 rng(static_cast<unsigned>(I(3)));
        const double factor = std::stod(a.at(4));
        std::uniform_real_distribution<double> P(0, 100);
        std::vector<std::pair<double, double>> fp(F), cp_(C);
        for(auto & p : fp) p = {P(rng), P(rng)};
        for(auto & p : cp_) p = {P(rng), P(rng)};
        std::vector<double> open(F), dist(F * C);
        for(auto & v : open) v = 80 + P(rng);
        for(std::size_t i = 0; i < F; ++i) for(std::size_t j = 0; j < C; ++j) dist[i * C + j] = std::round(std::hypot(fp[i].first - cp_[j].first, fp[i].second - cp_[j].second));
        auto y = m.add_binary_variables(F);
        auto x = m.add_variables(F * C, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 1});
        const auto Fs = std::views::iota(std::size_t{0}, F); const auto Cs = std::views::iota(std::size_t{0}, C);
        m.set_minimization();
        m.set_objective(xsum(Fs, [&](std::size_t i) { return open[i] * y(i); }) + xsum(Fs, [&](std::size_t i) { return xsum(Cs, [&, i](std::size_t j) { return dist[i * C + j] * x(i * C + j); }); }));
        for(std::size_t j : Cs) m.add_constraint(xsum(Fs, [&, j](std::size_t i) { return 1.0 * x(i * C + j); }) == 1);
        const double bigm = factor * static_cast<double>(C);
        for(std::size_t i : Fs) m.add_constraint(xsum(Cs, [&, i](std::size_t j) { return 1.0 * x(i * C + j); }) <= bigm * y(i));
        std::vector<int> cy, cx;
        for(std::size_t i : Fs) cy.push_back(m.native_id(y(i)));
        for(std::size_t i = 0; i < F * C; ++i) cx.push_back(m.native_id(x(i)));
        return {[=](const std::vector<double> & p, const cp::add_fn & add) {
            for(std::size_t i = 0; i < F; ++i) for(std::size_t j = 0; j < C; ++j)
                if(p[static_cast<std::size_t>(cx[i * C + j])] > p[static_cast<std::size_t>(cy[i])] + 1e-3) add(raw_cut{{cx[i * C + j], cy[i]}, {1.0, -1.0}, 'L', 0.0});
        }, fam};
    }
    if(fam == "lot") {  // capacitated multi-item lot sizing, q <= M y with M = factor * horizon demand
        const auto Items = static_cast<std::size_t>(I(1)), T = static_cast<std::size_t>(I(2));
        std::mt19937 rng(static_cast<unsigned>(I(3)));
        const double factor = std::stod(a.at(4));
        std::uniform_int_distribution<int> Dm(0, 40), Fx(100, 400), H(1, 4);
        std::vector<double> d(Items * T), fc(Items), h(Items);
        double total = 0;
        for(auto & v : d) { v = Dm(rng); total += v; }
        for(auto & v : fc) v = Fx(rng);
        for(auto & v : h) v = H(rng);
        const double capt = std::ceil(1.3 * total / static_cast<double>(T));
        auto q = m.add_variables(Items * T, {.obj_coef = 0, .lower_bound = 0, .upper_bound = capt});
        auto s = m.add_variables(Items * T);
        auto y = m.add_binary_variables(Items * T);
        const auto Is = std::views::iota(std::size_t{0}, Items); const auto Ts = std::views::iota(std::size_t{0}, T);
        m.set_minimization();
        m.set_objective(xsum(Is, [&](std::size_t i) { return xsum(Ts, [&, i](std::size_t t) { return fc[i] * y(i * T + t) + h[i] * s(i * T + t); }); }));
        for(std::size_t i : Is) for(std::size_t t : Ts) {
            if(t == 0) m.add_constraint(q(i * T) - s(i * T) == d[i * T]);
            else m.add_constraint(s(i * T + t - 1) + q(i * T + t) - s(i * T + t) == d[i * T + t]);
            double rest = 0;
            for(std::size_t u = t; u < T; ++u) rest += d[i * T + u];
            const double bigm = factor * std::max(rest, 1.0);
            m.add_constraint(q(i * T + t) - bigm * y(i * T + t) <= 0);
        }
        for(std::size_t t : Ts) m.add_constraint(xsum(Is, [&, t](std::size_t i) { return 1.0 * q(i * T + t); }) <= capt);
        std::vector<int> cq, cs, cy;
        for(std::size_t k = 0; k < Items * T; ++k) { cq.push_back(m.native_id(q(k))); cs.push_back(m.native_id(s(k))); cy.push_back(m.native_id(y(k))); }
        return {[=](const std::vector<double> & p, const cp::add_fn & add) {
            // (l,S): Sum_{t in S} q_t <= Sum_{t in S} d_{t..l} y_t + s_l
            for(std::size_t i = 0; i < Items; ++i) for(std::size_t l = 0; l < T; ++l) {
                raw_cut c; c.sense = 'L'; c.rhs = 0;
                double act = -p[static_cast<std::size_t>(cs[i * T + l])];
                double dtl = 0;
                std::vector<double> sums(l + 1);
                for(std::size_t u = l + 1; u-- > 0;) { dtl += d[i * T + u]; sums[u] = dtl; }
                for(std::size_t t = 0; t <= l; ++t) {
                    const double qv = p[static_cast<std::size_t>(cq[i * T + t])], yv = p[static_cast<std::size_t>(cy[i * T + t])];
                    if(qv > sums[t] * yv + 1e-6) {
                        c.ind.push_back(cq[i * T + t]); c.val.push_back(1.0);
                        c.ind.push_back(cy[i * T + t]); c.val.push_back(-sums[t]);
                        act += qv - sums[t] * yv;
                    }
                }
                if(act > 1e-3 && !c.ind.empty()) { c.ind.push_back(cs[i * T + l]); c.val.push_back(-1.0); add(c); }
            }
        }, fam};
    }
    if(fam == "fcnd") {  // single-commodity fixed-charge network: f <= M y, M = factor * total demand
        const int V = I(1);
        const auto N = static_cast<std::size_t>(V);
        std::mt19937 rng(static_cast<unsigned>(I(2)));
        const double factor = std::stod(a.at(3));
        std::uniform_int_distribution<int> Dm(1, 10), Fx(20, 80), Vc(1, 6);
        std::uniform_real_distribution<double> P(0, 1);
        std::vector<std::pair<std::size_t, std::size_t>> arcs;
        for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j && (P(rng) < 0.3 || j == (i + 1) % N || i == (j + 1) % N)) arcs.emplace_back(i, j);
        std::vector<double> dem(N, 0.0);
        double tot = 0;
        for(std::size_t i = 1; i < N; ++i) { dem[i] = Dm(rng); tot += dem[i]; }
        const auto Es = std::views::iota(std::size_t{0}, arcs.size());
        std::vector<double> fx(arcs.size()), vc(arcs.size());
        for(auto & v : fx) v = Fx(rng);
        for(auto & v : vc) v = Vc(rng);
        auto f = m.add_variables(arcs.size(), {.obj_coef = 0, .lower_bound = 0, .upper_bound = tot});
        auto y = m.add_binary_variables(arcs.size());
        m.set_minimization();
        m.set_objective(xsum(Es, [&](std::size_t e) { return fx[e] * y(e) + vc[e] * f(e); }));
        for(std::size_t v = 1; v < N; ++v) {
            const auto in = xsum(Es | std::views::filter([&, v](std::size_t e) { return arcs[e].second == v; }), [&](std::size_t e) { return 1.0 * f(e); });
            const auto out = xsum(Es | std::views::filter([&, v](std::size_t e) { return arcs[e].first == v; }), [&](std::size_t e) { return 1.0 * f(e); });
            m.add_constraint(in - out == dem[v]);
        }
        const double bigm = factor * tot;
        for(std::size_t e : Es) m.add_constraint(f(e) - bigm * y(e) <= 0);
        std::vector<int> cf, cy;
        for(std::size_t e : Es) { cf.push_back(m.native_id(f(e))); cy.push_back(m.native_id(y(e))); }
        const auto arcs_ = arcs;
        return {[=](const std::vector<double> & p, const cp::add_fn & add) {
            // f_e <= (demand beyond the arc's head at most the total) y_e: the tight link
            for(std::size_t e = 0; e < cf.size(); ++e) if(p[static_cast<std::size_t>(cf[e])] > tot * p[static_cast<std::size_t>(cy[e])] + 1e-3) add(raw_cut{{cf[e], cy[e]}, {1.0, -tot}, 'L', 0.0});
        }, fam};
    }
    std::println(stderr, "unknown family {}", fam);
    std::exit(4);
}

int main(int argc, char ** argv) {
    const std::string spec = argv[1], config = argv[2];
    const double limit = argc > 3 ? std::atof(argv[3]) : 15;
    M m;
    auto B = build_instance(m, spec);
    auto & api = m.native_api();
    auto & n = natives::of(m);
    double bound = NAN; long fired = 0, cuts = 0, skipped_terms = 0; int tree_rows = -1, tree_cols = -1, tree_ints = -1;
    const int P_rows = api.get_num_rows(m.native_model()), P_cols = api.get_num_cols(m.native_model()), P_ints = n.get_num_int(m.native_model());
    m.set_time_limit(std::chrono::duration<double>(limit));
    std::string name = spec;
    if(auto p = name.rfind('/'); p != std::string::npos) name = name.substr(p + 1);
    if(auto p = name.find(".mps"); p != std::string::npos) name = name.substr(0, p);
    const bool obj_max = api.get_obj_dir(m.native_model()) == v1::GLP_MAX;
    const auto stats_of = [&](glp_tree * t, int why) {
        if(why == R_SELECT) { const int b = api.ios_best_node(t); if(b) bound = api.ios_node_bound(t, b); }
        if(tree_rows < 0 && why == R_PREPRO) { glp_prob * Q = api.ios_get_prob(t); tree_rows = api.get_num_rows(Q); tree_cols = api.get_num_cols(Q); tree_ints = n.get_num_int(Q); }
    };
    std::string st_name;
    double value = NAN;
    const auto t0 = clk::now();
    long closed = -1, iters = 0;
    if(config == "npp" || config == "nppcuts") {
        // The route: names on P so that Q's columns can be mapped back.
        glp_prob * P = m.native_model();
        const std::vector<std::string> names = [&] { std::vector<std::string> v; for(int j = 1; j <= P_cols; ++j) { auto s = std::format("c{}", j); api.set_col_name(P, j, s.c_str()); v.push_back(s); } return v; }();
        auto a = npp_api::of(m);
        const int term_out = api.term_out(v1::GLP_OFF);
        void * prep = a.alloc();
        a.load(prep, P, 3 /* GLP_MIP */, v1::GLP_ON);
        int rc = a.pre1(prep, std::getenv("VG_HARD") ? v1::GLP_ON : v1::GLP_OFF);
        if(rc != 0) { st_name = std::format("preprocess1 returned {:#x}", rc); }
        else {
            glp_prob * Q = api.create_prob();
            a.build(prep, Q);
            tree_rows = api.get_num_rows(Q); tree_cols = api.get_num_cols(Q); tree_ints = n.get_num_int(Q);
            if(tree_cols == 0) { st_name = "route: empty Q"; }
            else {
                std::vector<int> to_q(static_cast<std::size_t>(P_cols) + 1, 0), to_p(static_cast<std::size_t>(tree_cols) + 1, 0);
                for(int j = 1; j <= tree_cols; ++j) {
                    const char * nm = api.get_col_name(Q, j);
                    const int pj = nm ? std::atoi(nm + 1) : 0;
                    to_p[static_cast<std::size_t>(j)] = pj;
                    if(pj > 0) to_q[static_cast<std::size_t>(pj)] = j;
                }
                n.scale_prob(Q, SF_PATH);
                n.adv_basis(Q, 0);
                v1::glp_smcp s; api.init_smcp(&s); s.msg_lev = v1::GLP_MSG_OFF; s.tm_lim = m.model_params.tm_lim;
                rc = api.simplex(Q, &s);
                const int stq = api.get_status(Q);
                if(rc != 0 || stq != v1::GLP_OPT) { st_name = std::format("root simplex rc={:#x} status={}", rc, stq); }
                else {
                    auto parm = m.model_params;
                    parm.presolve = v1::GLP_OFF;
                    struct cb_t { std::function<void(glp_tree *, int)> f; const v1::glpk_api * api; } cb;
                    std::vector<double> xp(static_cast<std::size_t>(P_cols) + 1), lower(static_cast<std::size_t>(P_cols) + 1);
                    cb.api = &api;
                    cb.f = [&](glp_tree * t, int why) {
                        stats_of(t, why);
                        if(config != "nppcuts" || why != R_CUTGEN) return;
                        ++fired;
                        glp_prob * q = api.ios_get_prob(t);
                        // an eliminated column has no value here: the separator sees 0, which is a wrong
                        // point for what preprocess1 fixed at another value, unless bounds pin it
                        for(int j = 1; j <= P_cols; ++j) {
                            const int qj = to_q[static_cast<std::size_t>(j)];
                            xp[static_cast<std::size_t>(j)] = qj ? api.get_col_prim(q, qj) : api.get_col_lb(P, j);
                        }
                        B.sep(xp, [&](const raw_cut & c) {
                            std::vector<int> ind{0}; std::vector<double> val{0};
                            for(std::size_t k = 0; k < c.ind.size(); ++k) {
                                const int qj = to_q[static_cast<std::size_t>(c.ind[k])];
                                if(!qj) { ++skipped_terms; return 0; }
                                ind.push_back(qj); val.push_back(c.val[k]);
                            }
                            n.ios_add_row(t, nullptr, 0, 0, static_cast<int>(ind.size()) - 1, ind.data(), val.data(), c.sense == 'L' ? v1::GLP_UP : v1::GLP_LO, c.rhs);
                            ++cuts;
                            return 0;
                        });
                    };
                    parm.cb_func = [](glp_tree * t, void * u) { auto & c = *static_cast<cb_t *>(u); c.f(t, c.api->ios_reason(t)); };
                    parm.cb_info = &cb;
                    parm.msg_lev = v1::GLP_MSG_ON; parm.out_frq = 100000000;
                    n.term_hook(M::on_term, &m);
                    const int t2 = api.term_out(v1::GLP_ON);
                    const int ret = api.intopt(Q, &parm);
                    api.term_out(t2);
                    n.term_hook(nullptr, nullptr);
                    closed = m.closed;
                    const int ms = api.mip_status(Q);
                    iters = api.get_it_cnt(Q);
                    if(ms == v1::GLP_OPT || ms == v1::GLP_FEAS) {
                        a.post(prep, Q);
                        a.obtain(prep, P);
                        value = api.mip_obj_val(P);
                    }
                    st_name = cp::status_name(Base::_intopt_status(ret, ms));
                }
            }
            api.delete_prob(Q);
        }
        a.free_(prep);
        api.term_out(term_out);
    } else {
        cp::separator sep = [](const std::vector<double> &, const cp::add_fn &) {};
        if(config == "offcuts") sep = B.sep;
        if(config != "on") m.p.presolve_off = true;
        if(std::getenv("VG_NOSCALE")) m.p.scale = false;
        if(std::getenv("VG_NOADV")) m.p.adv_basis = false;
        long * pc = &cuts;
        auto & c = attach(m, [sep, pc](const std::vector<double> & x, const cp::add_fn & add) {
            sep(x, [&](const raw_cut & r) { const int e = add(r); ++*pc; return e; });
        });
        c.also = [&](glp_tree * t, int why) { if(why == R_CUTGEN) ++fired; stats_of(t, why); };
        m.solve();
        closed = m.closed; iters = m.iters;
        st_name = cp::status_name(m.get_status());
        value = m.get_solution_value();
    }
    const double secs = std::chrono::duration<double>(clk::now() - t0).count();
    (void)obj_max;
    std::println("C|{}|{}|{}|{}|{}|{}|{}|{}|{:.3f}|{}|{}|{}|{}|{}|{}|{}|{}|{}", name, config, P_rows, P_cols, P_ints, tree_rows, tree_cols, tree_ints, secs, st_name, value, bound, closed, iters, fired, cuts, skipped_terms, vers(m));
    return 0;
}
