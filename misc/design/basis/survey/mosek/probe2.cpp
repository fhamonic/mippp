// Follow-up measurements: cold iteration counts, dual-simplex 0-iteration
// confirmations, invalid bases under dual simplex with MOSEK's own log and
// warning streams, write-mapping variants on model C, odd statuses.
#include "common.hpp"

#include <cstring>
#include <map>

static void stream_printer(MSKuserhandle_t h, const char * str) {
    std::printf("    %s| %s", static_cast<const char *>(h), str);
    std::size_t n = std::strlen(str);
    if(n == 0 || str[n - 1] != '\n') std::printf("\n");
}
void link_streams(probe_lp & m, bool log) {
    static const char * wrn = "WRN";
    static const char * err = "ERR";
    static const char * lg = "LOG";
    API->linkfunctotaskstream(T(m), MSK_STREAM_WRN, (void *)wrn, stream_printer);
    API->linkfunctotaskstream(T(m), MSK_STREAM_ERR, (void *)err, stream_printer);
    if(log) {
        API->linkfunctotaskstream(T(m), MSK_STREAM_LOG, (void *)lg,
                                  stream_printer);
        API->putintparam(T(m), MSK_IPAR_LOG, 10);
    }
}

void histogram(const basis_arrays & b) {
    std::map<int, int> hc, hx;
    for(auto k : b.skc) hc[k]++;
    for(auto k : b.skx) hx[k]++;
    std::printf("  rows:");
    for(auto [k, n] : hc) std::printf(" %s=%d", skname(k), n);
    std::printf(" | cols:");
    for(auto [k, n] : hx) std::printf(" %s=%d", skname(k), n);
    std::printf("\n");
}

int main() {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    API = &mosek_api::load();
    ext_api ext(API->library_path().c_str());
    EXT = &ext;

    basis_arrays bA;
    {
        probe_lp A;
        build_r(A);
        raw_solve(A, "A default");
        bA = dump(A, "A optimal", MSK_SOL_BAS, false);
        API->getprimalobj(T(A), MSK_SOL_BAS, &ref_obj);
        MSKstakeye k;
        int rc = EXT->getskc(T(A), MSK_SOL_ITR, &k);
        std::printf("  getskc(ITR) after default solve rc=%s\n", rcs(rc).c_str());
    }
    std::printf("\n== (a) cold iteration counts, presolve OFF, no limit ==\n");
    for(auto opt : {MSK_OPTIMIZER_PRIMAL_SIMPLEX, MSK_OPTIMIZER_DUAL_SIMPLEX,
                    MSK_OPTIMIZER_FREE_SIMPLEX}) {
        auto B = fresh_r();
        simplex_settings(B, opt, MSK_PRESOLVE_MODE_OFF, -1);
        char label[64];
        std::snprintf(label, sizeof label, "COLD opt=%d", (int)opt);
        raw_solve(B, label);
        obj_line(B, label);
        MSKstakeye k;
        int rc = EXT->getskc(T(B), MSK_SOL_ITR, &k);
        std::printf("  getskc(ITR) after simplex rc=%s\n", rcs(rc).c_str());
    }
    std::printf("\n== (n) MOSEK log: cold vs warm dual simplex ==\n");
    {
        auto B = fresh_r();
        link_streams(B, true);
        simplex_settings(B, MSK_OPTIMIZER_DUAL_SIMPLEX, MSK_PRESOLVE_MODE_OFF,
                         -1);
        raw_solve(B, "COLD dual with log");
    }
    {
        auto B = fresh_r();
        put_basis(B, bA, "WARM dual with log");
        link_streams(B, true);
        simplex_settings(B, MSK_OPTIMIZER_DUAL_SIMPLEX, MSK_PRESOLVE_MODE_OFF,
                         -1);
        raw_solve(B, "WARM dual with log");
    }
    {
        auto B = fresh_r();
        put_basis(B, bA, "WARM wrapper defaults with log");
        link_streams(B, true);
        raw_solve(B, "WARM wrapper defaults (FREE) with log");
    }

    std::printf("\n== (b) dual simplex maxit=0 confirmations ==\n");
    {
        auto B = fresh_r();
        put_basis(B, bA, "B exact");
        simplex_settings(B, MSK_OPTIMIZER_DUAL_SIMPLEX, MSK_PRESOLVE_MODE_FREE,
                         0);
        raw_solve(B, "B exact, dual presolve FREE maxit=0");
        obj_line(B, "B exact");
    }
    {
        auto B = fresh_r();
        put_basis(B, bA, "B exact");
        simplex_settings(B, MSK_OPTIMIZER_PRIMAL_SIMPLEX, MSK_PRESOLVE_MODE_OFF,
                         1);
        raw_solve(B, "B exact, primal presolve OFF maxit=1");
        obj_line(B, "B exact primal maxit=1");
    }
    {
        probe_lp A;
        auto r = build_r(A);
        raw_solve(A, "A3 default");
        auto b = dump(A, "A3", MSK_SOL_BAS, false);
        A.set_variable_upper_bound(r.x[5], 1000);
        put_basis(A, b, "A3 put after relaxing a bound");
        simplex_settings(A, MSK_OPTIMIZER_DUAL_SIMPLEX, MSK_PRESOLVE_MODE_OFF, 0);
        raw_solve(A, "A3 dual maxit=0");
        obj_line(A, "A3");
    }
    {
        probe_lp A;
        auto r = build_r(A);
        raw_solve(A, "F1 A default");
        auto b = dump(A, "F1 A", MSK_SOL_BAS, false);
        A.add_variable({.obj_coef = -100, .lower_bound = 0});
        A.add_constraint(r.x[2] + r.x[3] <= 100000);
        b.skx.push_back(MSK_SK_LOW);
        b.skc.push_back(MSK_SK_BAS);
        put_basis(A, b, "F1 extended");
        simplex_settings(A, MSK_OPTIMIZER_DUAL_SIMPLEX, MSK_PRESOLVE_MODE_OFF, 0);
        raw_solve(A, "F1 dual maxit=0");
        obj_line(A, "F1");
    }
    {
        probe_lp A;
        auto r = build_r(A);
        raw_solve(A, "F1b A default");
        auto b = dump(A, "F1b A", MSK_SOL_BAS, false);
        A.add_variable({.obj_coef = -100, .lower_bound = 0});
        A.add_constraint(r.x[2] + r.x[3] <= 100000);
        // the new row nonbasic and the new column basic: wrong choice
        b.skx.push_back(MSK_SK_BAS);
        b.skc.push_back(MSK_SK_UPR);
        put_basis(A, b, "F1b extended the wrong way");
        simplex_settings(A, MSK_OPTIMIZER_DUAL_SIMPLEX, MSK_PRESOLVE_MODE_OFF, 0);
        raw_solve(A, "F1b dual maxit=0");
        obj_line(A, "F1b");
    }
    {
        probe_lp A;
        auto r = build_r(A);
        raw_solve(A, "F2 A default");
        A.add_variable({.obj_coef = -100, .lower_bound = 0});
        A.add_constraint(r.x[2] + r.x[3] <= 100000);
        auto g = dump(A, "F2 grown, keys MOSEK kept", MSK_SOL_BAS, false);
        std::printf("  new col=%s new row=%s\n", skname(g.skx.back()),
                    skname(g.skc.back()));
        simplex_settings(A, MSK_OPTIMIZER_DUAL_SIMPLEX, MSK_PRESOLVE_MODE_OFF, 0);
        raw_solve(A, "F2 grown, no put, dual maxit=0");
        obj_line(A, "F2");
        auto after = dump(A, "F2 after re-solve", MSK_SOL_BAS, false);
        std::printf("  new col after re-solve=%s\n", skname(after.skx.back()));
    }
    {
        // a new column with bounds set at creation time on an unsolved model
        probe_lp B;
        build_r(B);
        B.add_variable({.obj_coef = -100, .lower_bound = 0});
        int rc = EXT->putskc(T(B), MSK_SOL_BAS, bA.skc.data());
        std::printf("  putskc only on unsolved grown model rc=%s\n",
                    rcs(rc).c_str());
        basis_arrays b;
        b.skx.assign(B.num_variables(), (MSKstakeye)99);
        EXT->getskx(T(B), MSK_SOL_BAS, b.skx.data());
        std::printf("  column keys after defining the slot by putskc: last=%s\n",
                    skname(b.skx.back()));
    }

    std::printf("\n== (c) invalid bases, dual simplex presolve OFF ==\n");
    auto find_code = [](const std::vector<MSKstakeye> & v, MSKstakeye k) {
        for(int i = 0; i < (int)v.size(); ++i)
            if(v[i] == k) return i;
        return -1;
    };
    struct inval {
        const char * name;
        std::function<void(basis_arrays &)> f;
    };
    std::vector<inval> invals = {
        {"exact (control)", [&](basis_arrays &) {}},
        {"one basic too few", [&](basis_arrays & b) {
             b.skx[find_code(b.skx, MSK_SK_BAS)] = MSK_SK_LOW; }},
        {"one basic too many", [&](basis_arrays & b) {
             b.skc[find_code(b.skc, MSK_SK_UPR)] = MSK_SK_BAS; }},
        {"UPR on +inf upper bound", [&](basis_arrays & b) {
             b.skx[1] = MSK_SK_UPR; }},
        {"LOW on free basic column (count -1)", [&](basis_arrays & b) {
             b.skx[30] = MSK_SK_LOW; }},
        {"SUPBAS on nonbasic bounded column", [&](basis_arrays & b) {
             b.skx[1] = MSK_SK_SUPBAS; }},
        {"FIX on non-fixed nonbasic column", [&](basis_arrays & b) {
             b.skx[1] = MSK_SK_FIX; }},
        {"INF on nonbasic column", [&](basis_arrays & b) {
             b.skx[1] = MSK_SK_INF; }},
        {"UNK on nonbasic column", [&](basis_arrays & b) {
             b.skx[1] = MSK_SK_UNK; }},
        {"LOW on a tight <= row (lower = -inf)", [&](basis_arrays & b) {
             int i = find_code(b.skc, MSK_SK_UPR);
             std::printf("    row %d UPR -> LOW\n", i);
             b.skc[i] = MSK_SK_LOW; }},
        {"UNK everywhere", [&](basis_arrays & b) {
             for(auto & k : b.skx) k = MSK_SK_UNK;
             for(auto & k : b.skc) k = MSK_SK_UNK; }},
    };
    for(auto & iv : invals) {
        for(int maxit : {0, -1}) {
            auto B = fresh_r();
            basis_arrays b = bA;
            std::printf("* %s, dual maxit=%d (col1 was %s)\n", iv.name, maxit,
                        skname(bA.skx[1]));
            iv.f(b);
            put_basis(B, b, iv.name);
            if(maxit == 0) link_streams(B, false);
            simplex_settings(B, MSK_OPTIMIZER_DUAL_SIMPLEX,
                             MSK_PRESOLVE_MODE_OFF, maxit);
            raw_solve(B, "dual");
            obj_line(B, iv.name);
        }
    }

    std::printf("\n== (d) dual simplex iteration limit: SUPBAS entries ==\n");
    {
        probe_lp m;
        build_r(m);
        simplex_settings(m, MSK_OPTIMIZER_DUAL_SIMPLEX, MSK_PRESOLVE_MODE_OFF,
                         3);
        raw_solve(m, "R dual maxit=3");
        dump(m, "R dual iter-limit entries", MSK_SOL_BAS, true);
    }
    {
        probe_lp m;
        build_r(m);
        simplex_settings(m, MSK_OPTIMIZER_PRIMAL_SIMPLEX, MSK_PRESOLVE_MODE_OFF,
                         3);
        raw_solve(m, "R primal maxit=3");
        auto b = dump(m, "R primal iter-limit", MSK_SOL_BAS, false);
        histogram(b);
    }

    std::printf("\n== (e) time limit key distribution ==\n");
    for(int opt : {(int)MSK_OPTIMIZER_FREE, (int)MSK_OPTIMIZER_DUAL_SIMPLEX}) {
        probe_lp m;
        build_r(m, 400, 600, 777);
        API->putdouparam(T(m), MSK_DPAR_OPTIMIZER_MAX_TIME, 1e-4);
        iparam(m, MSK_IPAR_OPTIMIZER, opt);
        raw_solve(m, opt == 2 ? "big R time 1e-4 FREE" : "big R time 1e-4 DUAL");
        auto b = dump(m, "time-limit", MSK_SOL_BAS, false);
        histogram(b);
    }
    {
        // a time limit that lets simplex run a little
        probe_lp m;
        build_r(m, 800, 1200, 778);
        API->putdouparam(T(m), MSK_DPAR_OPTIMIZER_MAX_TIME, 0.02);
        iparam(m, MSK_IPAR_OPTIMIZER, MSK_OPTIMIZER_DUAL_SIMPLEX);
        raw_solve(m, "bigger R time 0.02 DUAL");
        auto b = dump(m, "time-limit 0.02", MSK_SOL_BAS, false);
        histogram(b);
    }

    std::printf("\n== (f) write mapping variants on model C (dual, presolve "
                "OFF, maxit=0) ==\n");
    basis_arrays bC;
    cmodel cn;
    {
        probe_lp m;
        cn = build_c(m, true);
        simplex_settings(m, MSK_OPTIMIZER_DUAL_SIMPLEX, MSK_PRESOLVE_MODE_OFF,
                         -1);
        raw_solve(m, "C cold dual");
        bC = dump(m, "C basis", MSK_SOL_BAS, false);
        API->getprimalobj(T(m), MSK_SOL_BAS, &ref_obj);
    }
    {
        probe_lp m;
        build_c(m, true);
        simplex_settings(m, MSK_OPTIMIZER_DUAL_SIMPLEX, MSK_PRESOLVE_MODE_OFF, 0);
        raw_solve(m, "C cold dual maxit=0 (control)");
        obj_line(m, "C cold control");
    }
    struct cvar {
        const char * name;
        std::function<void(basis_arrays &)> f;
    };
    // indices in model C: rows 0 y1<=3,1 y2>=2,2 y3==5,3 ranged up,4 ranged lo,
    // 5 -y6<=-4, 6 -y7>=-8, 7 slack<=, 8 slack>=; cols 7 x1[0,4], 8 x2 fixed,
    // 9 x4 lo, 10 x5 (-inf,7], 11 x3 free empty
    std::vector<cvar> cvars = {
        {"exact", [](basis_arrays &) {}},
        {"fixed col FIX->LOW", [](basis_arrays & b) { b.skx[8] = MSK_SK_LOW; }},
        {"fixed col FIX->UPR", [](basis_arrays & b) { b.skx[8] = MSK_SK_UPR; }},
        {"== row FIX->LOW", [](basis_arrays & b) { b.skc[2] = MSK_SK_LOW; }},
        {"== row FIX->UPR", [](basis_arrays & b) { b.skc[2] = MSK_SK_UPR; }},
        {"free col SUPBAS->LOW", [](basis_arrays & b) { b.skx[11] = MSK_SK_LOW; }},
        {"free col SUPBAS->UPR", [](basis_arrays & b) { b.skx[11] = MSK_SK_UPR; }},
        {"free col SUPBAS->UNK", [](basis_arrays & b) { b.skx[11] = MSK_SK_UNK; }},
        {"free col SUPBAS->FIX", [](basis_arrays & b) { b.skx[11] = MSK_SK_FIX; }},
        {"ranged row up UPR->LOW (wrong side)",
         [](basis_arrays & b) { b.skc[3] = MSK_SK_LOW; }},
        {"<= row UPR->LOW (side does not exist)",
         [](basis_arrays & b) { b.skc[0] = MSK_SK_LOW; }},
        {">= row LOW->UPR (side does not exist)",
         [](basis_arrays & b) { b.skc[1] = MSK_SK_UPR; }},
        {"col x1[0,4] UPR->LOW (wrong side)",
         [](basis_arrays & b) { b.skx[7] = MSK_SK_LOW; }},
        {"col x5(-inf,7] UPR->LOW (side does not exist)",
         [](basis_arrays & b) { b.skx[10] = MSK_SK_LOW; }},
        {"-y6<=-4 row UPR->FIX", [](basis_arrays & b) { b.skc[5] = MSK_SK_FIX; }},
    };
    for(auto & cv : cvars) {
        probe_lp m;
        build_c(m, true);
        basis_arrays b = bC;
        cv.f(b);
        put_basis(m, b, cv.name);
        simplex_settings(m, MSK_OPTIMIZER_DUAL_SIMPLEX, MSK_PRESOLVE_MODE_OFF, 0);
        raw_solve(m, cv.name);
        obj_line(m, cv.name);
        auto after = dump(m, "after", MSK_SOL_BAS, false);
        std::printf("  after: x2fixed=%s eqrow=%s x3free=%s rangedup=%s "
                    "le_row=%s ge_row=%s x1=%s x5=%s\n",
                    skname(after.skx[8]), skname(after.skc[2]),
                    skname(after.skx[11]), skname(after.skc[3]),
                    skname(after.skc[0]), skname(after.skc[1]),
                    skname(after.skx[7]), skname(after.skx[10]));
    }

    std::printf("\n== (g) INF key: column with lb > ub ==\n");
    for(int opt : {(int)MSK_OPTIMIZER_FREE, (int)MSK_OPTIMIZER_DUAL_SIMPLEX}) {
        probe_lp m;
        m.set_time_limit(10s);
        auto x = m.add_variable({.obj_coef = 1, .lower_bound = 0, .upper_bound = 3});
        auto y = m.add_variable({.obj_coef = 1, .lower_bound = 0});
        m.add_constraint(x + y <= 10);
        auto r = m.add_constraint(x - y >= -10);
        m.set_constraint_lower_bound(r, 5);   // ranged? no: >= row lb
        m.set_constraint_upper_bound(r, -20); // now lb 5 > ub -20
        m.set_variable_lower_bound(x, 5);     // lb 5 > ub 3
        iparam(m, MSK_IPAR_OPTIMIZER, opt);
        m.set_maximization();
        raw_solve(m, "lb>ub");
        dump(m, "lb>ub", MSK_SOL_BAS, true);
    }

    std::printf("\n== (h) bounds made equal through the wrapper setters ==\n");
    {
        probe_lp m;
        m.set_time_limit(10s);
        m.set_maximization();
        auto x = m.add_variable({.obj_coef = 1, .lower_bound = 0, .upper_bound = 4});
        auto y = m.add_variable({.obj_coef = 1, .lower_bound = 0});
        auto r = m.add_constraint(y <= 6);
        m.add_constraint(x + y <= 100);
        m.set_variable_lower_bound(x, 4);
        m.set_constraint_lower_bound(r, 6);
        raw_solve(m, "equal bounds via setters");
        dump(m, "equal bounds via setters", MSK_SOL_BAS, true);
    }
    std::printf("\nDONE\n");
}
