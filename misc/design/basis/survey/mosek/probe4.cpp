// Reads the keys through the already declared MSK_getsolution (nullptr for
// every other output) and compares them with MSK_getskc / MSK_getskx; also the
// wrapper status after a 0-iteration primal stop at the optimum.
#include "common.hpp"

bool same_keys(probe_lp & m, const char * label) {
    const auto nc = m.num_constraints(), nv = m.num_variables();
    std::vector<MSKstakeye> c1(nc), x1(nv), c2(nc, (MSKstakeye)99),
        x2(nv, (MSKstakeye)99);
    int a = EXT->getskc(T(m), MSK_SOL_BAS, c1.data());
    int b = EXT->getskx(T(m), MSK_SOL_BAS, x1.data());
    int c = API->getsolution(T(m), MSK_SOL_BAS, nullptr, nullptr, c2.data(),
                             x2.data(), nullptr, nullptr, nullptr, nullptr,
                             nullptr, nullptr, nullptr, nullptr, nullptr);
    bool eq = c1 == c2 && x1 == x2;
    std::printf("  [%s] getskc rc=%s getskx rc=%s getsolution(keys only) rc=%s "
                "-> arrays equal: %d\n",
                label, rcs(a).c_str(), rcs(b).c_str(), rcs(c).c_str(), eq);
    return eq;
}

int main() {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    API = &mosek_api::load();
    ext_api ext(API->library_path().c_str());
    EXT = &ext;
    {
        probe_lp m;
        build_c(m, true);
        m.solve();
        same_keys(m, "C optimal");
    }
    {
        probe_lp m;
        build_r(m);
        m.solve();
        same_keys(m, "R optimal");
    }
    {
        probe_lp m;
        build_r(m);
        simplex_settings(m, MSK_OPTIMIZER_DUAL_SIMPLEX, MSK_PRESOLVE_MODE_OFF, 3);
        m.solve();
        same_keys(m, "R dual iteration limit");
    }
    {
        probe_lp m;
        build_r(m);
        MSKstakeye k;
        int rc = API->getsolution(T(m), MSK_SOL_BAS, nullptr, nullptr, &k,
                                  nullptr, nullptr, nullptr, nullptr, nullptr,
                                  nullptr, nullptr, nullptr, nullptr, nullptr);
        std::printf("  [unsolved] getsolution(keys) rc=%s\n", rcs(rc).c_str());
    }
    basis_arrays bA;
    {
        probe_lp A;
        build_r(A);
        A.solve();
        bA = dump(A, "A", MSK_SOL_BAS, false);
        API->getprimalobj(T(A), MSK_SOL_BAS, &ref_obj);
    }
    for(auto opt : {MSK_OPTIMIZER_PRIMAL_SIMPLEX, MSK_OPTIMIZER_DUAL_SIMPLEX}) {
        probe_lp B;
        build_r(B);
        put_basis(B, bA, "B");
        simplex_settings(B, opt, MSK_PRESOLVE_MODE_OFF, 0);
        B.solve();
        const auto & st = B.get_status();
        std::printf("  wrapper solve() after put, %s maxit=0: status index=%zu",
                    opt == MSK_OPTIMIZER_PRIMAL_SIMPLEX ? "primal" : "dual",
                    st.index());
        if(auto * il = std::get_if<status::iteration_limit>(&st))
            std::printf(" iteration_limit solution_available=%d", (int)il->solution_available);
        std::printf("\n");
        try {
            std::printf("  get_solution_value()=%.9g (ref %.9g)\n",
                        B.get_solution_value(), ref_obj);
        } catch(const std::exception & e) {
            std::printf("  get_solution_value() threw: %s\n", e.what());
        }
    }
    std::printf("DONE\n");
}
