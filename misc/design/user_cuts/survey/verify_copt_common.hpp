// Skeptic harness for COPT. V derives from the shipped copt_milp and hides solve()
// only when a recipe is attached: MIPRELAX is added to the shipped mask, MIPRELAX goes
// to the recipe, every other context goes to the SHIPPED callback_func. No recipe: the
// shipped solve() as is. Needs -fno-access-control for callback_func and the privates.
#pragma once
#include "mippp/solvers/copt/all.hpp"
#include "cut_conformance.hpp"
namespace c = mippp::copt::impl::v1;

struct recipe_opts {
    bool fold = true, strip_zero = true, skip_empty = true, drop_satisfied = true;
    bool mask_node = true;
    bool pass_relax_to_shipped_first = false;  // the dispatch trap: BESTBND path runs on MIPRELAX
    int hook_return = 0;
    double tol = 1e-9;
};
struct recipe {
    cp::separator sep;
    recipe_opts o;
    int cols = 0;
    std::atomic<long> relax{0}, native_adds{0}, native_err{0}, skip_empty{0}, skip_sat{0}, zero_stripped{0}, folded{0}, read_err{0}, exc{0};
    std::atomic<int> last_rc{0};
    std::string line() const {
        return std::format("relax={} native_adds={} native_err={}(last {}) skip_empty={} skip_sat={} zeros_stripped={} folded={} read_err={} exc={}",
            relax.load(), native_adds.load(), native_err.load(), last_rc.load(), skip_empty.load(), skip_sat.load(), zero_stripped.load(), folded.load(), read_err.load(), exc.load());
    }
};

struct V : c::copt_milp {
    std::shared_ptr<recipe> rec;  // for the next solve only
    std::shared_ptr<recipe> last;
    int native_solve_rc = 0;
    bool empty_slot = false;   // no SetCallback at all
    int extra_mask = 0;        // extra contexts, for firing tests, on the shipped path
    std::function<int(c::copt_prob *, void *, int)> raw;  // replaces the recipe; other contexts as given
    int raw_mask = 0;

    void attach(cp::separator sep, recipe_opts o = {}) {
        rec = std::make_shared<recipe>();
        rec->sep = std::move(sep); rec->o = o; rec->cols = static_cast<int>(num_variables());
    }
    static int dispatch(c::copt_prob * prob, void * cbdata, int cbctx, void * usr) {
        auto * m = static_cast<V *>(static_cast<c::copt_milp *>(usr));
        if(m->raw) { try { return m->raw(prob, cbdata, cbctx); } catch(...) { return 0; } }
        auto & r = *m->rec;
        if(cbctx != c::COPT_CBCONTEXT_MIPRELAX || r.o.pass_relax_to_shipped_first) {
            const int rc = c::copt_milp::callback_func(prob, cbdata, cbctx, usr);
            if(cbctx != c::COPT_CBCONTEXT_MIPRELAX) return rc;
        }
        ++r.relax;
        std::vector<double> x(static_cast<std::size_t>(r.cols));
        const int e = m->COPT->GetCallbackInfo(cbdata, c::COPT_CBINFO_RELAXSOLUTION, x.data());
        if(e) { ++r.read_err; return 0; }
        try {
            r.sep(x, [&](const cp::raw_cut & in) -> int {
                cp::raw_cut out = in;
                if(r.o.fold) {
                    std::map<int, double> f;
                    for(std::size_t k = 0; k < in.ind.size(); ++k) f[in.ind[k]] += in.val[k];
                    if(f.size() != in.ind.size()) ++r.folded;
                    out.ind.clear(); out.val.clear();
                    for(auto [j, v] : f) { out.ind.push_back(j); out.val.push_back(v); }
                }
                if(r.o.strip_zero) {
                    cp::raw_cut t{{}, {}, out.sense, out.rhs};
                    for(std::size_t k = 0; k < out.ind.size(); ++k) if(out.val[k] != 0.0) { t.ind.push_back(out.ind[k]); t.val.push_back(out.val[k]); } else ++r.zero_stripped;
                    out = std::move(t);
                }
                if(r.o.skip_empty && out.ind.empty()) { ++r.skip_empty; return 0; }
                if(r.o.drop_satisfied && out.violation(x) <= r.o.tol) { ++r.skip_sat; return 0; }
                const int rc = m->COPT->AddCallbackUserCut(cbdata, static_cast<int>(out.ind.size()), out.ind.data(), out.val.data(), out.sense, out.rhs);
                ++r.native_adds;
                if(rc) { ++r.native_err; r.last_rc = rc; }
                return rc;
            });
        } catch(...) { ++r.exc; }
        return r.o.hook_return;
    }
    void solve() {
        if(empty_slot) {
            reset_status();
            check(COPT->GetIntAttr(prob, c::COPT_INTATTR_ISMIP, &_is_mip));
            check(COPT->Solve(prob));
            _status = _get_status_milp();
            _mark_results_valid();
            return;
        }
        if(!rec && !raw) {
            if(extra_mask == 0) { c::copt_milp::solve(); return; }
        }
        reset_status();
        check(COPT->GetIntAttr(prob, c::COPT_INTATTR_ISMIP, &_is_mip));
        if(!_is_mip) { std::println(stderr, "NOT A MIP"); std::abort(); }
        int sense;
        check(COPT->GetIntAttr(prob, c::COPT_INTATTR_OBJSENSE, &sense));
        _search_bound = sense == c::COPT_MINIMIZE ? -c::COPT_INFINITY : c::COPT_INFINITY;
        int contexts = (solution_callback ? c::COPT_CBCONTEXT_MIPSOL : 0) | (progress_callback ? c::COPT_CBCONTEXT_INCUMBENT : 0);
        if(raw) contexts = raw_mask;
        else contexts |= (rec && rec->o.mask_node ? c::COPT_CBCONTEXT_MIPNODE : 0) | (rec ? c::COPT_CBCONTEXT_MIPRELAX : 0);
        contexts |= extra_mask;
        check(COPT->SetCallback(prob, dispatch, contexts, static_cast<c::copt_milp *>(this)));
        native_solve_rc = COPT->Solve(prob);
        COPT->SetCallback(prob, nullptr, 0, nullptr);
        check(static_cast<c::ret_code>(native_solve_rc));
        _status = _get_status_milp();
        _mark_results_valid();
        if(rec && std::getenv("RECIPE_LINE")) std::println(stderr, "   [{}]", rec->line());
        last = rec; rec = nullptr;
    }
    void iparam(const char * name, int v) { COPT->_check(nullptr, COPT->SetIntParam(prob, name, v)); }
    long simplex_iters() { double d = 0; int i = 0; if(COPT->GetDblAttr(prob, "SimplexIter", &d) == 0) return (long)d; if(COPT->GetIntAttr(prob, "SimplexIter", &i) == 0) return i; return -1; }
};
inline void logging_if_asked(V & m) { if(std::getenv("CUT_LOG")) m.iparam("Logging", 1); }

// ------------------------------------------------------------ my own instances
// Fixed-charge transportation. Sum_j x_ij <= s_i, Sum_i x_ij == d_j, x_ij <= M y_ij with
// M loose. Valid and violated by the LP: x_ij <= min(s_i, d_j) y_ij.
struct fctp {
    std::size_t S, D;
    std::vector<int> x, y;   // native columns, index S*i + j
    std::vector<double> tight;  // min(s_i, d_j)
    double M;
    int X(std::size_t i, std::size_t j) const { return x[i * D + j]; }
    int Y(std::size_t i, std::size_t j) const { return y[i * D + j]; }
    // up to k violated tight links
    std::vector<cp::raw_cut> links(const std::vector<double> & p, std::size_t k = 10, double min_viol = 1e-2) const {
        std::vector<std::pair<double, std::size_t>> v;
        for(std::size_t a = 0; a < x.size(); ++a) {
            const double d = p[(std::size_t)x[a]] - tight[a] * p[(std::size_t)y[a]];
            if(d > min_viol) v.emplace_back(d, a);
        }
        std::ranges::sort(v, std::greater<>{});
        std::vector<cp::raw_cut> out;
        for(auto [d, a] : v) { if(out.size() == k) break; out.push_back(cp::raw_cut{{x[a], y[a]}, {1.0, -tight[a]}, 'L', 0.0}); }
        return out;
    }
};
template <class Mdl>
fctp build_fctp(Mdl & m, const std::size_t S, const std::size_t D, const unsigned seed, const bool hidden_links = false, const double tighten = 1.2) {
    fctp f; f.S = S; f.D = D;
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> dem(5, 30), cost(1, 20), fix(40, 200);
    std::vector<double> d(D), s(S);
    double total = 0;
    for(auto & v : d) { v = dem(rng); total += v; }
    for(auto & v : s) v = std::ceil(total * tighten / static_cast<double>(S));
    f.M = total;
    auto x = m.add_variables(S * D);
    auto y = m.add_binary_variables(S * D);
    const auto idx = [D](std::size_t i, std::size_t j) { return i * D + j; };
    std::vector<double> cx(S * D), cy(S * D);
    for(auto & v : cx) v = cost(rng);
    for(auto & v : cy) v = fix(rng);
    const auto all = std::views::iota(std::size_t{0}, S * D);
    const auto Is = std::views::iota(std::size_t{0}, S);
    const auto Js = std::views::iota(std::size_t{0}, D);
    m.set_minimization();
    m.set_objective(xsum(all, [&](std::size_t a) { return cx[a] * x(a) + cy[a] * y(a); }));
    for(std::size_t i : Is) m.add_constraint(xsum(Js, [&, i](std::size_t j) { return 1.0 * x(idx(i, j)); }) <= s[i]);
    for(std::size_t j : Js) m.add_constraint(xsum(Is, [&, j](std::size_t i) { return 1.0 * x(idx(i, j)); }) == d[j]);
    for(std::size_t a : all) { f.x.push_back(m.native_id(x(a))); f.y.push_back(m.native_id(y(a))); }
    for(std::size_t i : Is) for(std::size_t j : Js) f.tight.push_back(std::min(s[i], d[j]));
    if(!hidden_links) for(std::size_t a : all) m.add_constraint(x(a) - f.M * y(a) <= 0.0);
    return f;
}

// Multicommodity fixed-charge network design on a random digraph. Sum_k x^k_e <= cap y_e
// is the only link in the model (or hidden, for lazy tests); x^k_e <= d_k y_e is valid for
// every integer feasible point when the flow is a path-like conservation with demand d_k
// and arcs carry at most d_k of commodity k. That needs no cycle flow: flows have positive
// cost, and I also bound each x^k_e by d_k explicitly, so the point stays feasible.
struct mcnd {
    std::size_t A = 0, K = 0;
    std::vector<int> y;                      // per arc
    std::vector<std::vector<int>> x;         // [k][arc]
    std::vector<double> d;                   // demand of k
    std::vector<double> cap;
    std::vector<int> obj_ind; std::vector<double> obj_val;   // the objective, as a cut body
    std::vector<int> all_cont() const { std::vector<int> v; for(auto & r : x) v.insert(v.end(), r.begin(), r.end()); return v; }
    std::vector<cp::raw_cut> links(const std::vector<double> & p, std::size_t limit = 10, double min_viol = 1e-2) const {
        std::vector<std::tuple<double, std::size_t, std::size_t>> v;
        for(std::size_t k = 0; k < K; ++k) for(std::size_t e = 0; e < A; ++e) {
            const double g = p[(std::size_t)x[k][e]] - d[k] * p[(std::size_t)y[e]];
            if(g > min_viol) v.emplace_back(g, k, e);
        }
        std::ranges::sort(v, std::greater<>{});
        std::vector<cp::raw_cut> out;
        for(auto [g, k, e] : v) { if(out.size() == limit) break; out.push_back(cp::raw_cut{{x[k][e], y[e]}, {1.0, -d[k]}, 'L', 0.0}); }
        return out;
    }
    // the capacity rows a candidate violates, for the hidden variant
    std::vector<std::size_t> violated_caps(const std::vector<double> & p, double tol = 1e-6) const {
        std::vector<std::size_t> out;
        for(std::size_t e = 0; e < A; ++e) {
            double s = 0; for(std::size_t k = 0; k < K; ++k) s += p[(std::size_t)x[k][e]];
            if(s > cap[e] * p[(std::size_t)y[e]] + tol) out.push_back(e);
        }
        return out;
    }
};
template <class Mdl>
struct mcnd_vars {
    mcnd g;
    decltype(std::declval<Mdl &>().add_binary_variables(std::size_t{1})) yv;
    decltype(std::declval<Mdl &>().add_variables(std::size_t{1})) xv;
    std::size_t idx(std::size_t k, std::size_t e) const { return k * g.A + e; }
};
template <class Mdl>
auto build_mcnd(Mdl & m, const std::size_t N, const std::size_t extra_arcs, const std::size_t K, const unsigned seed, const bool hidden = false) {
    std::mt19937 rng(seed);
    std::uniform_int_distribution<std::size_t> node(0, N - 1);
    std::uniform_int_distribution<int> dem(1, 9), cost(1, 8), fix(20, 90);
    std::vector<std::pair<std::size_t, std::size_t>> arcs;
    for(std::size_t i = 0; i < N; ++i) { arcs.emplace_back(i, (i + 1) % N); arcs.emplace_back((i + 1) % N, i); }
    for(std::size_t a = 0; a < extra_arcs; ++a) { auto u = node(rng), v = node(rng); if(u != v) arcs.emplace_back(u, v); }
    const std::size_t A = arcs.size();
    std::vector<std::tuple<std::size_t, std::size_t, double>> com;
    double total = 0;
    while(com.size() < K) { auto o = node(rng), t = node(rng); if(o == t) continue; const double dk = dem(rng); com.emplace_back(o, t, dk); total += dk; }
    auto yv = m.add_binary_variables(A);
    auto xv = m.add_variables(K * A);
    const auto idx = [A](std::size_t k, std::size_t e) { return k * A + e; };
    const auto Es = std::views::iota(std::size_t{0}, A);
    const auto Ks = std::views::iota(std::size_t{0}, K);
    std::vector<double> ce(A), fe(A), cap(A);
    for(auto & v : ce) v = cost(rng);
    for(auto & v : fe) v = fix(rng);
    { const double capf = std::getenv("CAPF") ? std::atof(std::getenv("CAPF")) : 0.5; for(auto & v : cap) v = std::ceil(total * capf); }
    m.set_minimization();
    m.set_objective(xsum(Es, [&](std::size_t e) { return fe[e] * yv(e); }) +
                    xsum(Ks, [&](std::size_t k) { return xsum(Es, [&, k](std::size_t e) { return ce[e] * xv(idx(k, e)); }); }));
    for(std::size_t k : Ks) {
        auto [o, t, dk] = com[k];
        for(std::size_t v = 0; v < N; ++v) {
            const double b = v == o ? dk : v == t ? -dk : 0.0;
            m.add_constraint(xsum(Es | std::views::filter([&, v](std::size_t e) { return arcs[e].first == v; }), [&, k](std::size_t e) { return 1.0 * xv(idx(k, e)); })
                           - xsum(Es | std::views::filter([&, v](std::size_t e) { return arcs[e].second == v; }), [&, k](std::size_t e) { return 1.0 * xv(idx(k, e)); }) == b);
        }
        for(std::size_t e : Es) m.set_variable_upper_bound(xv(idx(k, e)), dk);
    }
    if(!hidden) for(std::size_t e : Es) m.add_constraint(xsum(Ks, [&, e](std::size_t k) { return 1.0 * xv(idx(k, e)); }) - cap[e] * yv(e) <= 0.0);
    mcnd_vars<Mdl> out{{}, yv, xv};
    out.g.A = A; out.g.K = K; out.g.cap = cap;
    for(std::size_t e : Es) out.g.y.push_back(m.native_id(yv(e)));
    out.g.x.assign(K, {});
    for(std::size_t k : Ks) { out.g.d.push_back(std::get<2>(com[k])); for(std::size_t e : Es) out.g.x[k].push_back(m.native_id(xv(idx(k, e)))); }
    for(std::size_t e : Es) { out.g.obj_ind.push_back(out.g.y[e]); out.g.obj_val.push_back(fe[e]); for(std::size_t k : Ks) { out.g.obj_ind.push_back(out.g.x[k][e]); out.g.obj_val.push_back(ce[e]); } }
    return out;
}
