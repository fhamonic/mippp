// QP basis survey probe for the HiGHS backend (highs_qp).
#include <chrono>
#include <cstdio>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "mippp/solvers/highs/all.hpp"

using namespace mippp;
using namespace mippp::operators;
using M = highs_qp;
using V = model_variable_t<M>;
using P = model_variable_params_t<M>;

static const char * cn(int c) {
    switch(c) {
        case 0: return "L0";
        case 1: return "B1";
        case 2: return "U2";
        case 3: return "Z3";
        case 4: return "N4";
        case -7: return "--";
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
    auto [rcq, qit] = info_int(m, "qp_iteration_count");
    auto [rcps, ps] = info_int(m, "primal_solution_status");
    std::printf(
        "  [%s] model_status=%d basis_validity=%d(rc%d) simplex_it=%d "
        "qp_it=%d(rc%d) psol=%d obj=%.9g\n",
        tag, api.getModelStatus(h), bv, rcv, it, qit, rcq, ps,
        api.getObjectiveValue(h));
    std::vector<double> cv(static_cast<std::size_t>(api.getNumCol(h)));
    std::vector<double> rv(static_cast<std::size_t>(api.getNumRow(h)));
    api.getSolution(h, cv.data(), nullptr, rv.data(), nullptr);
    std::printf("    x:");
    for(double v : cv) std::printf(" %.4g", v);
    std::printf("  rows:");
    for(double v : rv) std::printf(" %.4g", v);
    std::printf("\n");
    flush();
}
static basis_t read_basis(M & m) {
    auto & api = m.native_api();
    void * h = m.native_model();
    basis_t b{std::vector<int>(static_cast<std::size_t>(api.getNumCol(h)), -7),
              std::vector<int>(static_cast<std::size_t>(api.getNumRow(h)), -7), 0};
    b.rc = api.getBasis(h, b.col.data(), b.row.data());
    return b;
}
static int write_basis(M & m, const basis_t & b) {
    return m.native_api().setBasis(m.native_model(), b.col.data(),
                                   b.row.data());
}
static void print_codes(const char * tag, const basis_t & b) {
    int basic = 0;
    for(int c : b.col) basic += (c == 1);
    for(int c : b.row) basic += (c == 1);
    std::printf("  basis[%s] rc=%d basic=%d nrow=%zu basic==nrow:%s\n    cols:",
                tag, b.rc, basic, b.row.size(),
                basic == static_cast<int>(b.row.size()) ? "yes" : "NO");
    for(int c : b.col) std::printf(" %s", cn(c));
    std::printf("\n    rows:");
    for(int c : b.row) std::printf(" %s", cn(c));
    std::printf("\n");
    flush();
}

// min (x-1)^2 + (y-2)^2 + z  s.t. x + y <= 2 (active), x - y <= 10 (slack),
// x, y >= 0, z in [0,5] (at lower), u in [0,1] with -u in the objective
// (at upper). Optimum x=0.5 y=1.5 z=0 u=1.
static void build_qp(M & m) {
    auto x = m.add_variable();
    auto y = m.add_variable();
    auto z = m.add_variable(P{.obj_coef = 0, .lower_bound = 0., .upper_bound = 5.});
    auto u = m.add_variable(P{.obj_coef = 0, .lower_bound = 0., .upper_bound = 1.});
    m.set_quadratic_objective(x * x + y * y - 2 * x - 4 * y + z - u + 5);
    m.add_constraint(x + y <= 2);
    m.add_constraint(x - y <= 10);
    m.set_time_limit(std::chrono::seconds(10));
}
// interior optimum: no active constraint at all
static void build_qp_interior(M & m) {
    auto x = m.add_variable();
    auto y = m.add_variable();
    m.set_quadratic_objective(x * x + y * y - 2 * x - 4 * y);
    m.add_constraint(x + y <= 100);
    m.add_constraint(x - y <= 100);
    m.set_time_limit(std::chrono::seconds(10));
}

int main() {
    {
        M m;
        std::printf("library: %s version %s\n",
                    m.native_api().library_path().string().c_str(),
                    m.native_api().version());
    }
    basis_t bA;
    {
        M a;
        build_qp(a);
        state(a, "QP before solve");
        print_codes("QP before solve", read_basis(a));
        a.solve();
        state(a, "QP A solved");
        bA = read_basis(a);
        print_codes("QP A", bA);
    }
    {
        M a;
        build_qp_interior(a);
        a.solve();
        state(a, "QP interior solved");
        print_codes("QP interior", read_basis(a));
    }
    {
        M a;
        build_qp(a);
        a.native_api().setStringOptionValue(a.native_model(), "solver", "ipm");
        a.solve();
        state(a, "QP solver=ipm");
        print_codes("QP solver=ipm", read_basis(a));
    }
    {  // control: fresh, qp_iteration_limit 0
        M b;
        build_qp(b);
        b.set_iteration_limit(0);
        b.solve();
        state(b, "QP B0 control limit 0");
        print_codes("QP B0", read_basis(b));
    }
    {  // setBasis then qp limit 0
        M b;
        build_qp(b);
        int rc = write_basis(b, bA);
        std::printf("  setBasis rc=%d\n", rc);
        print_codes("QP B1 read back after setBasis", read_basis(b));
        b.set_iteration_limit(0);
        b.solve();
        state(b, "QP B1 setBasis + limit 0");
    }
    for(int hot = 0; hot < 2; ++hot) {  // setSolution + setBasis (+ hot start)
        M a;
        build_qp(a);
        a.solve();
        auto & api = a.native_api();
        std::vector<double> cv(4), rv(2), cd(4), rd(2);
        api.getSolution(a.native_model(), cv.data(), cd.data(), rv.data(),
                        rd.data());
        M b;
        build_qp(b);
        if(hot) {
            int rc = api.setBoolOptionValue(b.native_model(),
                                            "qp_allow_hot_start", 1);
            std::printf("  set qp_allow_hot_start rc=%d\n", rc);
        }
        int rcs = api.setSolution(b.native_model(), cv.data(), rv.data(),
                                  cd.data(), rd.data());
        int rcb = write_basis(b, bA);
        std::printf("  setSolution rc=%d setBasis rc=%d\n", rcs, rcb);
        print_codes("read back", read_basis(b));
        b.set_iteration_limit(0);
        b.solve();
        state(b, hot ? "QP B2 sol+basis+hot_start limit 0"
                     : "QP B2 sol+basis limit 0");
    }
    for(int hot = 0; hot < 2; ++hot) {  // re-solve the same QP model in place
        for(int limit0 = 0; limit0 < 2; ++limit0) {
            M a;
            build_qp(a);
            a.solve();
            if(hot)
                a.native_api().setBoolOptionValue(a.native_model(),
                                                  "qp_allow_hot_start", 1);
            if(limit0) a.set_iteration_limit(0);
            a.solve();
            char tag[96];
            std::snprintf(tag, sizeof tag,
                          "QP A re-solved in place hot_start=%d limit0=%d", hot,
                          limit0);
            state(a, tag);
        }
    }
    {  // hot start into a fresh model, no limit
        M a;
        build_qp(a);
        a.solve();
        auto & api = a.native_api();
        std::vector<double> cv(4), rv(2), cd(4), rd(2);
        api.getSolution(a.native_model(), cv.data(), cd.data(), rv.data(),
                        rd.data());
        M b;
        build_qp(b);
        api.setBoolOptionValue(b.native_model(), "qp_allow_hot_start", 1);
        api.setSolution(b.native_model(), cv.data(), rv.data(), cd.data(),
                        rd.data());
        write_basis(b, bA);
        b.solve();
        state(b, "QP B3 sol+basis+hot_start no limit");
    }
    {  // highs_qp with a linear objective only: simplex basis
        M a;
        auto x = a.add_variable(P{.obj_coef = 0, .lower_bound = 0., .upper_bound = 4.});
        auto y = a.add_variable();
        a.set_objective(-x - y);
        a.add_constraint(x + 2 * y <= 6);
        a.add_constraint(x - y >= -10);
        a.set_time_limit(std::chrono::seconds(10));
        a.solve();
        state(a, "highs_qp linear objective");
        print_codes("highs_qp linear", read_basis(a));
    }
    std::printf("done\n");
    return 0;
}
