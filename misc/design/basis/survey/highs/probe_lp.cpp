// LP basis survey probe for the HiGHS backend (highs_lp).
// Builds models with MIP++ and reaches Highs_getBasis / Highs_setBasis
// through model.native_api() / native_model() / native_id().
#include <dlfcn.h>

#include <chrono>
#include <cstdio>
#include <optional>
#include <ranges>
#include <string>
#include <utility>
#include <vector>

#include "mippp/solvers/highs/all.hpp"

using namespace mippp;
using namespace mippp::operators;
using M = highs_lp;
using V = model_variable_t<M>;
using C = model_constraint_t<M>;
using P = model_variable_params_t<M>;

static const char * cn(int c) {
    switch(c) {
        case 0: return "L0";
        case 1: return "B1";
        case 2: return "U2";
        case 3: return "Z3";
        case 4: return "N4";
        case -7: return "--";  // sentinel: never written
        default: return "??";
    }
}

static void flush() { std::fflush(stdout); }

struct basis_t {
    std::vector<int> col, row;
    int rc = 0;
};

static std::pair<int, int> info_int(M & m, const char * name) {
    int v = -99;
    int rc = m.native_api().getIntInfoValue(m.native_model(), name, &v);
    return {rc, v};
}

static void state(M & m, const char * tag) {
    auto & api = m.native_api();
    void * h = m.native_model();
    auto [rcv, bv] = info_int(m, "basis_validity");
    auto [rci, it] = info_int(m, "simplex_iteration_count");
    auto [rcp, ipm] = info_int(m, "ipm_iteration_count");
    auto [rcc, cr] = info_int(m, "crossover_iteration_count");
    auto [rcps, ps] = info_int(m, "primal_solution_status");
    auto [rcds, ds] = info_int(m, "dual_solution_status");
    std::printf(
        "  [%s] model_status=%d basis_validity=%d(rc%d) simplex_it=%d(rc%d) "
        "ipm_it=%d crossover_it=%d psol=%d dsol=%d obj=%.9g\n",
        tag, api.getModelStatus(h), bv, rcv, it, rci, ipm, cr, ps, ds,
        api.getObjectiveValue(h));
    flush();
}

static basis_t read_basis(M & m) {
    auto & api = m.native_api();
    void * h = m.native_model();
    const int nc = api.getNumCol(h), nr = api.getNumRow(h);
    basis_t b{std::vector<int>(static_cast<std::size_t>(nc), -7),
              std::vector<int>(static_cast<std::size_t>(nr), -7), 0};
    b.rc = api.getBasis(h, b.col.data(), b.row.data());
    return b;
}

static int write_basis(M & m, const basis_t & b) {
    return m.native_api().setBasis(m.native_model(), b.col.data(),
                                   b.row.data());
}

static void print_codes(const char * tag, const basis_t & b,
                        const std::vector<std::string> & cnames = {},
                        const std::vector<std::string> & rnames = {}) {
    int cnt[5] = {0, 0, 0, 0, 0}, unwritten = 0, other = 0;
    auto count = [&](int c) {
        if(c >= 0 && c <= 4)
            ++cnt[c];
        else if(c == -7)
            ++unwritten;
        else
            ++other;
    };
    for(int c : b.col) count(c);
    const int col_basic = cnt[1];
    for(int c : b.row) count(c);
    std::printf(
        "  basis[%s] rc=%d ncol=%zu nrow=%zu basic=%d (cols %d) L=%d U=%d Z=%d "
        "N=%d unwritten=%d other=%d  basic==nrow:%s\n",
        tag, b.rc, b.col.size(), b.row.size(), cnt[1], col_basic, cnt[0],
        cnt[2], cnt[3], cnt[4], unwritten, other,
        cnt[1] == static_cast<int>(b.row.size()) ? "yes" : "NO");
    if(!cnames.empty()) {
        std::printf("    cols:");
        for(std::size_t i = 0; i < b.col.size(); ++i)
            std::printf(" %s=%s",
                        i < cnames.size() ? cnames[i].c_str() : "new",
                        cn(b.col[i]));
        std::printf("\n    rows:");
        for(std::size_t i = 0; i < b.row.size(); ++i)
            std::printf(" %s=%s",
                        i < rnames.size() ? rnames[i].c_str() : "new",
                        cn(b.row[i]));
        std::printf("\n");
    }
    flush();
}

static void set_opt(M & m, const char * name, const char * value) {
    int rc = m.native_api().setStringOptionValue(m.native_model(), name,
                                                 value);
    if(rc != 0) std::printf("  !! set %s=%s rc=%d\n", name, value, rc);
}
static void set_int(M & m, const char * name, int value) {
    int rc =
        m.native_api().setIntOptionValue(m.native_model(), name, value);
    if(rc != 0) std::printf("  !! set %s=%d rc=%d\n", name, value, rc);
}

///////////////////////////////////////////////////////////////////////////////
// LP "C": unique nondegenerate optimum (x1..x6 >= 0 except x1 in [0,4],
// f fixed at 2, w free and in no row)
//   min -3x1 - 2x3 + 3x4 + x6
//   rle  : x1 + x2 + x3 <= 10          tight (dual -1)
//   rge  : x2 - x3 + x4 >= 1           tight (dual 2)
//   req  : x3 + x4 + x5 + f == 9       (dual 1)
//   rrng : 2 <= x2 + x5 <= 6           tight at upper (dual -1)
//   rsl  : x2 + x3 + x4 + x5 + x6 <= 100   slack
// optimum x1=4 (at ub), x2=3 x3=3 x4=1 x5=3 basic, x6=0 at lb, f=2, w=0
// objective -15
///////////////////////////////////////////////////////////////////////////////
struct lpc {
    V x1, x2, x3, x4, x5, x6, f, w;
    C rle, rge, req, rrng, rsl;
};
static const std::vector<std::string> c_cols = {"x1[0,4]", "x2", "x3", "x4",
                                                "x5",      "x6", "f=2", "w(free)"};
static const std::vector<std::string> c_rows = {"rle(<=)", "rge(>=)", "req(==)",
                                                "rrng[2,6]", "rsl(slack)"};

static lpc build_c(M & m, bool maximize = false) {
    const double s = maximize ? -1.0 : 1.0;
    lpc h{m.add_variable(P{.obj_coef = s * -3, .lower_bound = 0., .upper_bound = 4.}),
          m.add_variable(P{.obj_coef = 0, .lower_bound = 0., .upper_bound = std::nullopt}),
          m.add_variable(P{.obj_coef = s * -2, .lower_bound = 0., .upper_bound = std::nullopt}),
          m.add_variable(P{.obj_coef = s * 3, .lower_bound = 0., .upper_bound = std::nullopt}),
          m.add_variable(P{.obj_coef = 0, .lower_bound = 0., .upper_bound = std::nullopt}),
          m.add_variable(P{.obj_coef = s * 1, .lower_bound = 0., .upper_bound = std::nullopt}),
          m.add_variable(P{.obj_coef = 0, .lower_bound = 2., .upper_bound = 2.}),
          m.add_variable(P{.obj_coef = 0, .lower_bound = -m.infinity(), .upper_bound = std::nullopt}),
          C{0}, C{0}, C{0}, C{0}, C{0}};
    if(maximize) m.set_maximization();
    h.rle = m.add_constraint(h.x1 + h.x2 + h.x3 <= 10);
    h.rge = m.add_constraint(h.x2 - h.x3 + h.x4 >= 1);
    h.req = m.add_constraint(h.x3 + h.x4 + h.x5 + h.f == 9);
    h.rrng = m.add_constraint(h.x2 + h.x5 <= 6);
    m.set_constraint_lower_bound(h.rrng, 2.);
    h.rsl = m.add_constraint(h.x2 + h.x3 + h.x4 + h.x5 + h.x6 <= 100);
    m.set_time_limit(std::chrono::seconds(10));
    return h;
}

static void print_c_solution(M & m) {
    auto & api = m.native_api();
    void * h = m.native_model();
    const int nc = api.getNumCol(h), nr = api.getNumRow(h);
    std::vector<double> cv(nc), cd(nc), rv(nr), rd(nr);
    api.getSolution(h, cv.data(), cd.data(), rv.data(), rd.data());
    std::printf("    col values:");
    for(int i = 0; i < nc; ++i) std::printf(" %.4g", cv[i]);
    std::printf("\n    col duals :");
    for(int i = 0; i < nc; ++i) std::printf(" %.4g", cd[i]);
    std::printf("\n    row values:");
    for(int i = 0; i < nr; ++i) std::printf(" %.4g", rv[i]);
    std::printf("\n    row duals :");
    for(int i = 0; i < nr; ++i) std::printf(" %.4g", rd[i]);
    std::printf("\n");
    flush();
}

///////////////////////////////////////////////////////////////////////////////
// Random LP "R": max c.x, A x <= b, 0 <= x <= 5 (every third column), x >= 0
///////////////////////////////////////////////////////////////////////////////
struct lcg {
    unsigned long long s;
    double next() {
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<double>((s >> 11) & ((1ULL << 53) - 1)) /
               static_cast<double>(1ULL << 53);
    }
};
static std::vector<V> build_r(M & m, int nrow = 40, int ncol = 60) {
    lcg g{12345};
    std::vector<V> xs;
    for(int j = 0; j < ncol; ++j) {
        const double c = 1.0 + g.next();
        std::optional<double> ub;
        if(j % 3 == 0) ub = 5.0;
        xs.push_back(
            m.add_variable(P{.obj_coef = c, .lower_bound = 0., .upper_bound = ub}));
    }
    m.set_maximization();
    for(int i = 0; i < nrow; ++i) {
        std::vector<double> a(static_cast<std::size_t>(ncol));
        for(auto & v : a) v = (g.next() < 0.5) ? g.next() : 0.0;
        const double b = 10.0 + 10.0 * g.next();
        m.add_constraint(xsum(std::views::iota(0, ncol),
                              [&xs, a](int j) {
                                  return a[static_cast<std::size_t>(j)] *
                                         xs[static_cast<std::size_t>(j)];
                              }) <= b);
    }
    m.set_time_limit(std::chrono::seconds(10));
    return xs;
}

static const char * section_filter = nullptr;
static bool run(const char * s) {
    return section_filter == nullptr || std::string(section_filter) == s;
}

int main(int argc, char ** argv) {
    if(argc > 1) section_filter = argv[1];
    {
        M probe_model;
        auto & api = probe_model.native_api();
        std::printf("library: %s version %s\n",
                    api.library_path().string().c_str(), api.version());
    }

    ///////////////////////////////////////////////////////////////////////
    if(run("C")) {
        std::printf("\n=== C: row side convention (LP C) ===\n");
        for(int variant = 0; variant < 4; ++variant) {
            const bool maximize = (variant & 1);
            const bool presolve_off = (variant & 2);
            M m;
            auto h = build_c(m, maximize);
            if(presolve_off) set_opt(m, "presolve", "off");
            m.solve();
            char tag[64];
            std::snprintf(tag, sizeof tag, "%s presolve=%s",
                          maximize ? "max(-obj)" : "min",
                          presolve_off ? "off" : "choose");
            state(m, tag);
            print_c_solution(m);
            print_codes(tag, read_basis(m), c_cols, c_rows);
            std::printf("    native ids: x1=%d w=%d rle=%d rsl=%d\n",
                        m.native_id(h.x1), m.native_id(h.w),
                        m.native_id(h.rle), m.native_id(h.rsl));
        }
        // the same rows written with the opposite sense: -x1 - x2 - x3 >= -10
        // and -x2 + x3 - x4 <= -1: does the code follow the activity?
        {
            M m;
            auto x1 = m.add_variable(P{.obj_coef = -3, .lower_bound = 0., .upper_bound = 4.});
            auto x2 = m.add_variable(P{.obj_coef = 0, .lower_bound = 0., .upper_bound = std::nullopt});
            auto x3 = m.add_variable(P{.obj_coef = -2, .lower_bound = 0., .upper_bound = std::nullopt});
            auto x4 = m.add_variable(P{.obj_coef = 3, .lower_bound = 0., .upper_bound = std::nullopt});
            auto x5 = m.add_variable(P{.obj_coef = 0, .lower_bound = 0., .upper_bound = std::nullopt});
            auto x6 = m.add_variable(P{.obj_coef = 1, .lower_bound = 0., .upper_bound = std::nullopt});
            auto f = m.add_variable(P{.obj_coef = 0, .lower_bound = 2., .upper_bound = 2.});
            auto w = m.add_variable(P{.obj_coef = 0, .lower_bound = -m.infinity(), .upper_bound = std::nullopt});
            (void)w;
            m.add_constraint(-x1 - x2 - x3 >= -10);
            m.add_constraint(-x2 + x3 - x4 <= -1);
            m.add_constraint(x3 + x4 + x5 + f == 9);
            auto rr = m.add_constraint(-x2 - x5 >= -6);
            m.set_constraint_upper_bound(rr, -2.);
            m.add_constraint(x2 + x3 + x4 + x5 + x6 <= 100);
            set_opt(m, "presolve", "off");
            m.set_time_limit(std::chrono::seconds(10));
            m.solve();
            state(m, "negated rows, presolve off");
            print_c_solution(m);
            print_codes("negated rows", read_basis(m), c_cols,
                        {"-rle(>=-10)", "-rge(<=-1)", "req", "-rrng[-6,-2]",
                         "rsl"});
        }
    }

    ///////////////////////////////////////////////////////////////////////
    if(run("D")) {
        std::printf("\n=== D: availability ===\n");
        {
            M m;
            build_c(m);
            state(m, "before any solve");
            print_codes("before any solve", read_basis(m), c_cols, c_rows);
        }
        {
            M m;
            state(m, "empty model, before solve");
            print_codes("empty model", read_basis(m));
        }
        for(int presolve_off = 0; presolve_off < 2; ++presolve_off) {
            {  // infeasible
                M m;
                auto x = m.add_variable();
                auto y = m.add_variable();
                m.set_objective(x + y);
                m.add_constraint(x + y >= 5);
                m.add_constraint(x + 2 * y <= 3);
                m.add_constraint(x - y <= 100);
                m.set_time_limit(std::chrono::seconds(10));
                if(presolve_off) set_opt(m, "presolve", "off");
                m.solve();
                state(m, presolve_off ? "infeasible presolve=off"
                                      : "infeasible presolve=choose");
                print_codes("infeasible", read_basis(m), {"x", "y"},
                            {"r0", "r1", "r2"});
            }
            {  // unbounded
                M m;
                auto x = m.add_variable();
                auto y = m.add_variable();
                m.set_objective(-x - y);
                m.add_constraint(x - y <= 1);
                m.add_constraint(x + 2 * y >= 1);
                m.set_time_limit(std::chrono::seconds(10));
                if(presolve_off) set_opt(m, "presolve", "off");
                m.solve();
                state(m, presolve_off ? "unbounded presolve=off"
                                      : "unbounded presolve=choose");
                print_codes("unbounded", read_basis(m), {"x", "y"},
                            {"r0", "r1"});
            }
        }
        {  // iteration limit
            for(int limit : {1, 5}) {
                M m;
                build_r(m);
                m.set_iteration_limit(static_cast<std::size_t>(limit));
                m.solve();
                char tag[64];
                std::snprintf(tag, sizeof tag, "R iteration_limit=%d", limit);
                state(m, tag);
                print_codes(tag, read_basis(m));
            }
            M m;
            build_r(m);
            m.set_time_limit(std::chrono::duration<double>(0.0));
            m.solve();
            state(m, "R time_limit=0");
            print_codes("R time_limit=0", read_basis(m));
        }
        {  // algorithms
            struct alg {
                const char * solver;
                const char * crossover;
                const char * presolve;
            };
            for(alg a : {alg{"simplex", "on", "choose"}, alg{"choose", "on", "choose"},
                         alg{"ipm", "off", "choose"}, alg{"ipm", "on", "choose"},
                         alg{"ipm", "off", "off"}, alg{"pdlp", "off", "choose"}}) {
                for(int which = 0; which < 2; ++which) {
                    M m;
                    if(which == 0)
                        build_c(m);
                    else
                        build_r(m);
                    set_opt(m, "solver", a.solver);
                    set_opt(m, "run_crossover", a.crossover);
                    set_opt(m, "presolve", a.presolve);
                    m.solve();
                    char tag[96];
                    std::snprintf(tag, sizeof tag,
                                  "%s solver=%s crossover=%s presolve=%s",
                                  which == 0 ? "C" : "R", a.solver,
                                  a.crossover, a.presolve);
                    state(m, tag);
                    print_codes(tag, read_basis(m));
                }
            }
        }
        {  // modifications after an optimal solve, no re-solve
            auto fresh = [](M & m) {
                auto h = build_c(m);
                set_opt(m, "presolve", "off");
                m.solve();
                return h;
            };
            {
                M m;
                auto h = fresh(m);
                m.set_variable_upper_bound(h.x1, 3.5);
                state(m, "after x1 ub 4->3.5");
                print_codes("after bound change", read_basis(m), c_cols, c_rows);
                m.set_verbose(true);
                set_int(m, "simplex_iteration_limit", 0);
                std::printf("  -- resolve with limit 0, log:\n");
                flush();
                m.solve();
                m.set_verbose(false);
                state(m, "resolve limit 0");
                set_int(m, "simplex_iteration_limit", 1000000);
                m.solve();
                state(m, "resolve unlimited");
                print_codes("resolved", read_basis(m), c_cols, c_rows);
            }
            {
                M m;
                auto h = fresh(m);
                m.set_constraint_upper_bound(h.rle, 11.);
                state(m, "after rle ub 10->11");
                print_codes("after row bound change", read_basis(m), c_cols, c_rows);
            }
            {
                M m;
                auto h = fresh(m);
                m.add_constraint(h.x2 + h.x4 <= 50);
                state(m, "after add row (slack)");
                print_codes("after add row", read_basis(m), c_cols, c_rows);
                set_int(m, "simplex_iteration_limit", 0);
                m.solve();
                state(m, "resolve limit 0 after add row");
            }
            {
                M m;
                auto h = fresh(m);
                m.add_column({{h.rsl, 1.0}},
                             P{.obj_coef = 5, .lower_bound = 0., .upper_bound = std::nullopt});
                m.add_variable(P{.obj_coef = 0, .lower_bound = -1., .upper_bound = 3.});
                m.add_variable(P{.obj_coef = 0, .lower_bound = -5., .upper_bound = 3.});
                m.add_variable(P{.obj_coef = 0, .lower_bound = -m.infinity(), .upper_bound = 3.});
                m.add_variable(P{.obj_coef = 0, .lower_bound = -m.infinity(), .upper_bound = std::nullopt});
                state(m, "after add 5 cols");
                print_codes("after add cols [0,inf) [-1,3] [-5,3] (-inf,3] free",
                            read_basis(m), c_cols, c_rows);
                set_int(m, "simplex_iteration_limit", 0);
                m.solve();
                state(m, "resolve limit 0 after add cols");
            }
            {
                M m;
                auto h = fresh(m);
                m.set_objective_coefficient(h.x6, 2.0);
                state(m, "after cost change");
                print_codes("after cost change", read_basis(m), c_cols, c_rows);
            }
            {
                M m;
                auto h = fresh(m);
                m.remove_variable(h.x6);  // nonbasic at lower
                state(m, "after remove x6 (nonbasic)");
                auto b = read_basis(m);
                print_codes("after remove x6", b);
                std::printf("    by handle: x1=%s x5=%s f=%s w=%s\n",
                            cn(b.col[static_cast<std::size_t>(m.native_id(h.x1))]),
                            cn(b.col[static_cast<std::size_t>(m.native_id(h.x5))]),
                            cn(b.col[static_cast<std::size_t>(m.native_id(h.f))]),
                            cn(b.col[static_cast<std::size_t>(m.native_id(h.w))]));
                m.set_verbose(true);
                set_int(m, "simplex_iteration_limit", 0);
                std::printf("  -- resolve with limit 0, log:\n");
                flush();
                m.solve();
                m.set_verbose(false);
                state(m, "resolve limit 0 after remove nonbasic");
            }
            {
                M m;
                auto h = fresh(m);
                m.remove_variable(h.x4);  // basic
                state(m, "after remove x4 (basic)");
                print_codes("after remove x4", read_basis(m));
                m.set_verbose(true);
                std::printf("  -- resolve, log:\n");
                flush();
                m.solve();
                m.set_verbose(false);
                state(m, "resolve after remove basic");
                print_codes("resolved after remove basic", read_basis(m));
            }
            {
                M m;
                auto h = fresh(m);
                (void)h;
                void * lib = dlopen(m.native_api().library_path().c_str(),
                                    RTLD_NOW | RTLD_NOLOAD);
                using fn = int (*)(void *);
                auto setLogical =
                    lib ? reinterpret_cast<fn>(dlsym(lib, "Highs_setLogicalBasis"))
                        : nullptr;
                std::printf("  Highs_setLogicalBasis found: %s\n",
                            setLogical ? "yes" : "no");
                if(setLogical) {
                    int rc = setLogical(m.native_model());
                    std::printf("  setLogicalBasis rc=%d\n", rc);
                    state(m, "after setLogicalBasis");
                    print_codes("after setLogicalBasis", read_basis(m), c_cols, c_rows);
                    m.set_verbose(true);
                    std::printf("  -- resolve, log:\n");
                    flush();
                    m.solve();
                    m.set_verbose(false);
                    state(m, "resolve after setLogicalBasis");
                }
            }
        }
    }

    ///////////////////////////////////////////////////////////////////////
    if(run("E")) {
        std::printf("\n=== E: warm start ===\n");
        basis_t bA;
        {
            M a;
            build_r(a);
            a.solve();
            state(a, "R model A default");
            bA = read_basis(a);
            print_codes("R A", bA);
        }
        {
            M b;
            build_r(b);
            set_int(b, "simplex_iteration_limit", 0);
            b.set_verbose(true);
            std::printf("  -- control B0 (no basis, limit 0), log:\n");
            flush();
            b.solve();
            b.set_verbose(false);
            state(b, "R B0 control no basis limit 0");
        }
        {
            M b;
            build_r(b);
            int rc = write_basis(b, bA);
            std::printf("  setBasis on fresh B rc=%d\n", rc);
            state(b, "R B1 after setBasis, before solve");
            auto rb = read_basis(b);
            std::printf("  basis read back equals written: %s\n",
                        (rb.col == bA.col && rb.row == bA.row) ? "yes" : "NO");
            set_int(b, "simplex_iteration_limit", 0);
            b.set_verbose(true);
            std::printf("  -- B1 (basis, limit 0), log:\n");
            flush();
            b.solve();
            b.set_verbose(false);
            state(b, "R B1 basis + limit 0");
        }
        {
            M b;
            build_r(b);
            write_basis(b, bA);
            b.solve();
            state(b, "R B2 basis, no limit");
        }
        for(const char * solver : {"simplex", "ipm", "pdlp"}) {
            M b;
            build_r(b);
            set_opt(b, "solver", solver);
            write_basis(b, bA);
            set_int(b, "simplex_iteration_limit", 0);
            b.solve();
            char tag[64];
            std::snprintf(tag, sizeof tag, "R B3 basis + limit 0 solver=%s",
                          solver);
            state(b, tag);
            print_codes(tag, read_basis(b));
        }
        {
            M b;
            build_r(b);
            set_opt(b, "presolve", "on");
            write_basis(b, bA);
            set_int(b, "simplex_iteration_limit", 0);
            b.solve();
            state(b, "R B4 basis + limit 0 presolve=on");
        }
        for(int strategy : {1, 4}) {  // dual, primal
            M b;
            build_r(b);
            set_int(b, "simplex_strategy", strategy);
            write_basis(b, bA);
            set_int(b, "simplex_iteration_limit", 0);
            b.solve();
            char tag[64];
            std::snprintf(tag, sizeof tag,
                          "R B5 basis + limit 0 simplex_strategy=%d", strategy);
            state(b, tag);
        }
        {  // setBasis then modify then solve
            M b;
            auto xs = build_r(b);
            write_basis(b, bA);
            b.set_variable_upper_bound(xs[1], 0.5);
            state(b, "R B6 setBasis then bound change");
            b.set_verbose(true);
            std::printf("  -- B6 solve, log:\n");
            flush();
            b.solve();
            b.set_verbose(false);
            state(b, "R B6 solved");
        }
        // bound change on A, then write A's basis back
        {
            M a;
            auto xs = build_r(a);
            a.solve();
            auto b0 = read_basis(a);
            int moved = -1;
            for(std::size_t j = 0; j < xs.size(); ++j)
                if(b0.col[static_cast<std::size_t>(a.native_id(xs[j]))] == 1) {
                    moved = static_cast<int>(j);
                    break;
                }
            std::printf("  changing ub of basic column %d to 0.1\n", moved);
            a.set_variable_upper_bound(xs[static_cast<std::size_t>(moved)], 0.1);
            state(a, "A after bound change");
            auto after = read_basis(a);
            std::printf("  HiGHS kept its basis through the change: %s\n",
                        (after.col == b0.col && after.row == b0.row) ? "same codes" : "codes differ");
            int rc = write_basis(a, b0);
            std::printf("  setBasis(old) after change rc=%d\n", rc);
            auto rb = read_basis(a);
            std::printf("  read back equals written: %s\n",
                        (rb.col == b0.col && rb.row == b0.row) ? "yes" : "NO");
            state(a, "A after setBasis(old)");
            a.set_verbose(true);
            std::printf("  -- A re-solve, log:\n");
            flush();
            a.solve();
            a.set_verbose(false);
            state(a, "A re-solved");
            M c;
            auto cx = build_r(c);
            c.set_variable_upper_bound(cx[static_cast<std::size_t>(moved)], 0.1);
            c.solve();
            state(c, "cold solve of the changed model (reference)");
        }
        // LP C warm start
        basis_t bc;
        {
            M a;
            build_c(a);
            a.solve();
            bc = read_basis(a);
            print_codes("C A", bc, c_cols, c_rows);
            M b;
            build_c(b);
            write_basis(b, bc);
            set_int(b, "simplex_iteration_limit", 0);
            b.solve();
            state(b, "C B basis + limit 0");
            M b0;
            build_c(b0);
            set_int(b0, "simplex_iteration_limit", 0);
            b0.solve();
            state(b0, "C B0 control no basis limit 0");
            M b1;
            build_c(b1);
            set_opt(b1, "presolve", "off");
            set_int(b1, "simplex_iteration_limit", 0);
            b1.solve();
            state(b1, "C B0' control no basis limit 0 presolve off");
        }
        // invalid bases on LP C
        auto try_basis = [&](const char * tag, basis_t b) {
            M m;
            build_c(m);
            std::printf("  -- invalid case: %s\n", tag);
            int rc = write_basis(m, b);
            std::printf("    setBasis rc=%d\n", rc);
            state(m, "after setBasis");
            print_codes("read back after setBasis", read_basis(m), c_cols, c_rows);
            m.solve();
            state(m, "solved");
            print_codes("after solve", read_basis(m), c_cols, c_rows);
        };
        {
            basis_t b = bc;
            for(auto & c : b.col) c = 1;
            for(auto & c : b.row) c = 1;
            try_basis("all basic (13 > 5 rows)", b);
        }
        {
            basis_t b = bc;
            for(auto & c : b.col) c = 0;
            for(auto & c : b.row) c = 0;
            try_basis("all nonbasic at lower (0 basic)", b);
        }
        {
            basis_t b = bc;
            b.col[5] = 2;  // x6: ub infinite
            try_basis("optimal basis but x6 U2 with infinite ub", b);
        }
        {
            basis_t b = bc;
            b.col[7] = 0;  // w free
            try_basis("optimal basis but w(free) L0", b);
        }
        {
            basis_t b = bc;
            b.col[0] = 3;  // x1 boxed
            try_basis("optimal basis but x1[0,4] Z3", b);
        }
        {
            basis_t b = bc;
            b.col[0] = 4;  // x1 boxed
            try_basis("optimal basis but x1[0,4] N4", b);
        }
        {
            basis_t b = bc;
            b.col[0] = 0;  // x1 boxed, wrong side
            try_basis("optimal basis but x1[0,4] L0 (wrong side)", b);
        }
        {
            basis_t b = bc;
            b.row[0] = 0;  // rle is <=: lower side infinite
            try_basis("optimal basis but rle(<=) L0 (infinite side)", b);
        }
        {
            basis_t b = bc;
            b.row[3] = 0;  // rrng tight at upper; lower side finite
            try_basis("optimal basis but rrng L0 (wrong finite side)", b);
        }
        {
            basis_t b = bc;
            b.col[0] = 5;
            try_basis("code 5 on x1", b);
        }
        {
            basis_t b = bc;
            b.row[0] = -1;
            try_basis("code -1 on rle", b);
        }
        {
            basis_t b = bc;
            // singular: x2,x3,x4,x5,w basic, every row nonbasic (w is empty)
            b.col = {2, 1, 1, 1, 1, 0, 0, 1};
            b.row = {2, 0, 0, 2, 2};
            try_basis("singular (empty column w basic)", b);
        }
    }

    ///////////////////////////////////////////////////////////////////////
    if(run("F")) {
        std::printf("\n=== F: partial / extended bases ===\n");
        basis_t bc;
        {
            M a;
            build_c(a);
            a.solve();
            bc = read_basis(a);
        }
        {  // model B = C + new column + new row, basis extended by hand
            M b;
            auto h = build_c(b);
            b.add_column({{h.rsl, 1.0}},
                         P{.obj_coef = 5, .lower_bound = 0., .upper_bound = std::nullopt});
            b.add_constraint(h.x2 + h.x4 <= 50);
            basis_t e = bc;
            e.col.push_back(0);
            e.row.push_back(1);
            int rc = write_basis(b, e);
            std::printf("  extended basis (new col L0, new row B1) rc=%d\n", rc);
            set_int(b, "simplex_iteration_limit", 0);
            b.solve();
            state(b, "extended basis + limit 0");
        }
        {  // new column made basic, new row nonbasic -> repaired?
            M b;
            auto h = build_c(b);
            b.add_column({{h.rsl, 1.0}},
                         P{.obj_coef = 5, .lower_bound = 0., .upper_bound = std::nullopt});
            b.add_constraint(h.x2 + h.x4 <= 50);
            basis_t e = bc;
            e.col.push_back(1);
            e.row.push_back(2);
            int rc = write_basis(b, e);
            std::printf("  extended basis (new col B1, new row U2) rc=%d\n", rc);
            print_codes("read back", read_basis(b), c_cols, c_rows);
            b.solve();
            state(b, "solved");
        }
        {  // only the basic entries, others N4
            M b;
            build_c(b);
            basis_t e = bc;
            for(auto & c : e.col)
                if(c != 1) c = 4;
            for(auto & c : e.row)
                if(c != 1) c = 4;
            int rc = write_basis(b, e);
            std::printf("  basics only, others N4 rc=%d\n", rc);
            print_codes("read back", read_basis(b), c_cols, c_rows);
            set_int(b, "simplex_iteration_limit", 0);
            b.solve();
            state(b, "basics-only N4 + limit 0");
            print_codes("after", read_basis(b), c_cols, c_rows);
        }
        {  // only the basic entries, others L0
            M b;
            build_c(b);
            basis_t e = bc;
            for(auto & c : e.col)
                if(c != 1) c = 0;
            for(auto & c : e.row)
                if(c != 1) c = 0;
            int rc = write_basis(b, e);
            std::printf("  basics only, others L0 rc=%d\n", rc);
            print_codes("read back", read_basis(b), c_cols, c_rows);
            set_int(b, "simplex_iteration_limit", 0);
            b.solve();
            state(b, "basics-only L0 + limit 0");
            set_int(b, "simplex_iteration_limit", 1000000);
            b.solve();
            state(b, "basics-only L0 unlimited");
        }
        {  // the C LP's own handling: setBasis, then add col + row, then solve
            M b;
            auto h = build_c(b);
            write_basis(b, bc);
            b.add_column({{h.rsl, 1.0}},
                         P{.obj_coef = 5, .lower_bound = 0., .upper_bound = std::nullopt});
            b.add_constraint(h.x2 + h.x4 <= 50);
            print_codes("setBasis then add col+row", read_basis(b), c_cols, c_rows);
            set_int(b, "simplex_iteration_limit", 0);
            b.solve();
            state(b, "setBasis, add col+row, limit 0");
        }
        {  // setBasis then delete a basic column through the wrapper
            M b;
            auto h = build_c(b);
            write_basis(b, bc);
            b.remove_variable(h.x4);
            print_codes("setBasis then remove basic x4", read_basis(b));
            b.set_verbose(true);
            std::printf("  -- solve, log:\n");
            flush();
            b.solve();
            b.set_verbose(false);
            state(b, "setBasis, remove basic, solve");
        }
    }
    std::printf("\ndone\n");
    return 0;
}
