// Write-side observability under primal simplex, MOSEK's own log for
// invalid bases, and presolve during a warm start.
#include "common.hpp"

#include <cstring>

static void stream_printer(MSKuserhandle_t h, const char * str) {
    // keep only the lines that describe the start
    if(std::strstr(str, "hotstart") || std::strstr(str, "Num. basic") ||
       std::strstr(str, "Valid bas") || std::strstr(str, "Presolve") ||
       std::strstr(str, "basis") || std::strstr(str, "Basis") ||
       std::strstr(str, "WRN") || std::strstr(str, "arning") ||
       static_cast<const char *>(h)[0] != 'L')
        std::printf("    %s| %s", static_cast<const char *>(h), str);
}
void link_streams(probe_lp & m) {
    static const char * wrn = "WRN";
    static const char * err = "ERR";
    static const char * lg = "LOG";
    static const char * msg = "MSG";
    API->linkfunctotaskstream(T(m), MSK_STREAM_WRN, (void *)wrn, stream_printer);
    API->linkfunctotaskstream(T(m), MSK_STREAM_ERR, (void *)err, stream_printer);
    API->linkfunctotaskstream(T(m), MSK_STREAM_MSG, (void *)msg, stream_printer);
    API->linkfunctotaskstream(T(m), MSK_STREAM_LOG, (void *)lg, stream_printer);
    API->putintparam(T(m), MSK_IPAR_LOG, 10);
}

int main() {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    API = &mosek_api::load();
    ext_api ext(API->library_path().c_str());
    EXT = &ext;

    std::printf("== write side under primal simplex (model C, presolve OFF) ==\n");
    basis_arrays bC;
    {
        probe_lp m;
        build_c(m, true);
        simplex_settings(m, MSK_OPTIMIZER_DUAL_SIMPLEX, MSK_PRESOLVE_MODE_OFF,
                         -1);
        raw_solve(m, "C cold dual");
        bC = dump(m, "C basis", MSK_SOL_BAS, false);
        API->getprimalobj(T(m), MSK_SOL_BAS, &ref_obj);
    }
    {
        probe_lp m;
        build_c(m, true);
        simplex_settings(m, MSK_OPTIMIZER_PRIMAL_SIMPLEX, MSK_PRESOLVE_MODE_OFF,
                         -1);
        raw_solve(m, "C cold primal (control)");
    }
    struct cvar {
        const char * name;
        std::function<void(basis_arrays &)> f;
    };
    std::vector<cvar> cvars = {
        {"exact", [](basis_arrays &) {}},
        {"ranged row up UPR->LOW", [](basis_arrays & b) { b.skc[3] = MSK_SK_LOW; }},
        {"ranged row lo LOW->UPR", [](basis_arrays & b) { b.skc[4] = MSK_SK_UPR; }},
        {"col x1[0,4] UPR->LOW", [](basis_arrays & b) { b.skx[7] = MSK_SK_LOW; }},
        {"<= row UPR->LOW (no such side)",
         [](basis_arrays & b) { b.skc[0] = MSK_SK_LOW; }},
        {"slack-flip convention: every row LOW<->UPR",
         [](basis_arrays & b) {
             for(auto & k : b.skc)
                 if(k == MSK_SK_LOW) k = MSK_SK_UPR;
                 else if(k == MSK_SK_UPR) k = MSK_SK_LOW;
         }},
    };
    for(auto & cv : cvars) {
        for(int maxit : {0, -1}) {
            probe_lp m;
            build_c(m, true);
            basis_arrays b = bC;
            cv.f(b);
            put_basis(m, b, cv.name);
            simplex_settings(m, MSK_OPTIMIZER_PRIMAL_SIMPLEX,
                             MSK_PRESOLVE_MODE_OFF, maxit);
            raw_solve(m, maxit == 0 ? "primal maxit=0" : "primal no limit");
            obj_line(m, cv.name);
            if(maxit == 0) {
                // the primal point MOSEK built from the keys
                std::vector<double> xx(m.num_variables()), xc(m.num_constraints());
                API->getsolution(T(m), MSK_SOL_BAS, nullptr, nullptr, nullptr,
                                 nullptr, nullptr, xc.data(), xx.data(), nullptr,
                                 nullptr, nullptr, nullptr, nullptr, nullptr);
                std::printf("    xc:");
                for(double v : xc) std::printf(" %g", v);
                std::printf("\n    xx:");
                for(double v : xx) std::printf(" %g", v);
                std::printf("\n");
            }
        }
    }

    std::printf("\n== MOSEK log for invalid bases (model R, dual, presolve OFF) "
                "==\n");
    basis_arrays bA;
    {
        probe_lp A;
        build_r(A);
        raw_solve(A, "A default");
        bA = dump(A, "A optimal", MSK_SOL_BAS, false);
        API->getprimalobj(T(A), MSK_SOL_BAS, &ref_obj);
    }
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
        {"one basic too few", [&](basis_arrays & b) {
             b.skx[find_code(b.skx, MSK_SK_BAS)] = MSK_SK_LOW; }},
        {"one basic too many", [&](basis_arrays & b) {
             b.skc[find_code(b.skc, MSK_SK_UPR)] = MSK_SK_BAS; }},
        {"UPR on +inf upper bound", [&](basis_arrays & b) {
             b.skx[1] = MSK_SK_UPR; }},
        {"all rows BAS, all columns LOW (slack basis)", [&](basis_arrays & b) {
             for(auto & k : b.skc) k = MSK_SK_BAS;
             for(auto & k : b.skx) k = MSK_SK_LOW; }},
    };
    for(auto & iv : invals) {
        probe_lp B;
        build_r(B);
        basis_arrays b = bA;
        iv.f(b);
        std::printf("* %s\n", iv.name);
        put_basis(B, b, iv.name);
        link_streams(B);
        simplex_settings(B, MSK_OPTIMIZER_DUAL_SIMPLEX, MSK_PRESOLVE_MODE_OFF,
                         -1);
        raw_solve(B, "dual");
        obj_line(B, iv.name);
    }

    std::printf("\n== presolve during a warm start (dual, presolve FREE / ON) "
                "==\n");
    for(int pre : {MSK_PRESOLVE_MODE_FREE, MSK_PRESOLVE_MODE_ON}) {
        probe_lp B;
        build_r(B);
        put_basis(B, bA, "exact");
        link_streams(B);
        simplex_settings(B, MSK_OPTIMIZER_DUAL_SIMPLEX, pre, 0);
        raw_solve(B, pre == MSK_PRESOLVE_MODE_ON ? "dual presolve ON maxit=0"
                                                 : "dual presolve FREE maxit=0");
        obj_line(B, "presolve warm");
    }
    std::printf("\nDONE\n");
}
