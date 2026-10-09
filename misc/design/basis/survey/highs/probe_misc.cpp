// Leftover cases: presolve reducing the LP to empty, a model with no row,
// setBasis on an already solved model, the basis after compute_iis().
#include <chrono>
#include <cstdio>
#include <optional>
#include <utility>
#include <vector>

#include "mippp/solvers/highs/all.hpp"

using namespace mippp;
using namespace mippp::operators;
using M = highs_lp;
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
    auto [rcv, bv] = info_int(m, "basis_validity");
    auto [rci, it] = info_int(m, "simplex_iteration_count");
    std::printf("  [%s] model_status=%d basis_validity=%d(rc%d) simplex_it=%d obj=%.9g\n",
                tag, m.native_api().getModelStatus(m.native_model()), bv, rcv,
                it, m.native_api().getObjectiveValue(m.native_model()));
    std::fflush(stdout);
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
    std::printf("  basis[%s] rc=%d basic=%d nrow=%zu  cols:", tag, b.rc, basic,
                b.row.size());
    for(int c : b.col) std::printf(" %s", cn(c));
    std::printf("  rows:");
    for(int c : b.row) std::printf(" %s", cn(c));
    std::printf("\n");
    std::fflush(stdout);
}

int main() {
    {
        M m;
        std::printf("library: %s version %s\n",
                    m.native_api().library_path().string().c_str(),
                    m.native_api().version());
    }
    {  // presolve solves everything: singleton rows and bounds only
        M m;
        auto x = m.add_variable(P{.obj_coef = -1, .lower_bound = 0., .upper_bound = std::nullopt});
        auto y = m.add_variable(P{.obj_coef = -2, .lower_bound = 0., .upper_bound = std::nullopt});
        auto z = m.add_variable(P{.obj_coef = 3, .lower_bound = 1., .upper_bound = 7.});
        m.add_constraint(x <= 4);
        m.add_constraint(2 * y <= 6);
        m.add_constraint(x + y + z <= 100);
        m.set_time_limit(std::chrono::seconds(10));
        m.set_verbose(true);
        m.solve();
        m.set_verbose(false);
        state(m, "presolve to empty");
        print_codes("presolve to empty", read_basis(m));
    }
    {  // no row
        M m;
        auto x = m.add_variable(P{.obj_coef = -1, .lower_bound = 0., .upper_bound = 3.});
        auto y = m.add_variable(P{.obj_coef = 1, .lower_bound = 0., .upper_bound = 3.});
        (void)x;
        (void)y;
        m.set_time_limit(std::chrono::seconds(10));
        m.solve();
        state(m, "no row");
        print_codes("no row", read_basis(m));
        basis_t b{{1, 1}, {}, 0};
        int rc = m.native_api().setBasis(m.native_model(), b.col.data(), nullptr);
        std::printf("  setBasis(all basic, row=nullptr) rc=%d\n", rc);
        print_codes("no row read back", read_basis(m));
    }
    {  // setBasis on an already solved model, no modification
        M m;
        auto x = m.add_variable(P{.obj_coef = -1, .lower_bound = 0., .upper_bound = 4.});
        auto y = m.add_variable(P{.obj_coef = -1, .lower_bound = 0., .upper_bound = std::nullopt});
        m.add_constraint(x + 2 * y <= 6);
        m.add_constraint(x - y >= -10);
        m.add_constraint(3 * x + y <= 20);
        m.set_time_limit(std::chrono::seconds(10));
        m.native_api().setStringOptionValue(m.native_model(), "presolve", "off");
        m.solve();
        state(m, "solved");
        auto opt = read_basis(m);
        print_codes("optimal", opt);
        basis_t slack{{0, 0}, {1, 1, 1}, 0};
        int rc = write_basis(m, slack);
        std::printf("  setBasis(slack basis) on the solved model rc=%d\n", rc);
        state(m, "after setBasis(slack) no re-solve");
        auto sol = m.get_solution();
        std::printf("  get_solution still reads x=%g y=%g\n", sol[x], sol[y]);
        print_codes("read back", read_basis(m));
        m.solve();
        state(m, "re-solved from slack basis");
        rc = write_basis(m, opt);
        std::printf("  setBasis(optimal) rc=%d\n", rc);
        m.set_iteration_limit(0);
        m.solve();
        state(m, "re-solved from optimal basis, limit 0");
    }
    {  // compute_iis on a feasible solved LP, then the basis
        M m;
        auto x = m.add_variable(P{.obj_coef = -1, .lower_bound = 0., .upper_bound = 4.});
        auto y = m.add_variable(P{.obj_coef = -1, .lower_bound = 0., .upper_bound = std::nullopt});
        m.add_constraint(x + 2 * y <= 6);
        m.add_constraint(x - y >= -10);
        m.set_time_limit(std::chrono::seconds(10));
        m.solve();
        state(m, "before compute_iis");
        print_codes("before compute_iis", read_basis(m));
        try {
            auto iis = m.compute_iis();
            (void)iis;
            state(m, "after compute_iis");
            print_codes("after compute_iis", read_basis(m));
        } catch(const std::exception & e) {
            std::printf("  compute_iis threw: %s\n", e.what());
        }
    }
    std::printf("done\n");
    return 0;
}
