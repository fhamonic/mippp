#include "mippp/solvers/cbc/all.hpp"
#include "probe_common.hpp"
using M = mippp::cbc_milp;
namespace v1 = mippp::cbc::impl::v1;
using v1::Cbc_Model;
// No user pointer on this callback: only a global reaches the recorder.
static recorder * R;
static const v1::cbc_api * API;
static double last_dual = NAN;
static int dumped = 0;
static bool stop_via_seconds = false;
static void (*setmaxsec)(Cbc_Model *, double);
using msg_cb_t = void (*)(Cbc_Model *, int, int, const double *, int, const int *, int, char **);
static void msg_cb(Cbc_Model * model, int msgno, int nd, const double * dv, int ni, const int * iv, int nc, char ** cv) {
    if(dumped < 14) {
        std::string d, i;
        for(int k = 0; k < nd; ++k) d += std::format("{:.6g} ", dv[k]);
        for(int k = 0; k < ni; ++k) i += std::format("{} ", iv[k]);
        std::println(stderr, "   msg {}: dvec=[{}] ivec=[{}] nchar={}", msgno, d, i, nc);
        ++dumped;
    }
    (void)cv;
    if(msgno == 10 && nd >= 2 && ni >= 1) {
        last_dual = dv[1];
        if(R->record("CBC_STATUS(10)", dv[0], dv[1], iv[0]) && stop_via_seconds && setmaxsec) setmaxsec(model, 0.0);
    } else if(msgno == 4 && nd >= 1 && ni >= 2) {
        if(R->record("INCUMBENT(4)", dv[0], last_dual, ni > 1 ? iv[1] : -1) && stop_via_seconds && setmaxsec) setmaxsec(model, 0.0);
    } else R->count(std::format("msg{}", msgno));
}
using reg_t = void (*)(Cbc_Model *, msg_cb_t);
static void attach(M & m, recorder & r) {
    R = &r; API = &m.native_api(); last_dual = NAN; dumped = 0;
    auto l = m.native_api().library_path();
    setmaxsec = symf<void (*)(Cbc_Model *, double)>(l, "Cbc_setMaximumSeconds");
    symf<reg_t>(l, "Cbc_registerCallBack")(m.native_model(), msg_cb);
}
// the cut generator route: called from inside the search
struct cut_ctx { recorder * r; M * m; };
static void cut_cb(void *, void *, void * app) {
    auto * c = static_cast<cut_ctx *>(app);
    auto mdl = c->m->native_model();
    c->r->record("CUTCB", API->getObjValue(mdl), API->getBestPossibleObjValue(mdl), API->getNodeCount(mdl));
}
int main(int argc, char **) {
    using std::chrono::duration;
    { M m; std::println(stderr, "library {} version {}", m.native_api().library_path().string(), m.native_api().getVersion()); }
    const mkp small = make_mkp(10, 60);
    const mkp large = make_mkp(30, 100);
    long long base = -1;
    {
        M m; build_mkp(m, small, +1, 100.0); m.set_time_limit(duration<double>(60));
        auto o = timed_solve(m);
        final_line("A small, no hook", m, o.solve_s); base = m.get_node_count();
    }
    const auto run = [&](const char * label, int sense, double limit, bool verbose) {
        M m; build_mkp(m, large, sense, 100.0); m.set_time_limit(duration<double>(limit));
        if(verbose) m.set_verbose(true);
        recorder r; attach(m, r); r.start();
        auto o = timed_solve(m);
        final_line(label, m, o.solve_s); hook_report(r, sense, o.solve_s, o.value, o.dual);
    };
    run("D message callback quiet, large min 10s", +1, 10, false);
    run("F message callback VERBOSE, large min 10s", +1, 10, true);
    run("E message callback VERBOSE, large max+offset 10s", -1, 10, true);
    {   // stop attempt: lower the time limit from the callback
        M m; build_mkp(m, large, +1, 100.0); m.set_time_limit(duration<double>(10)); m.set_verbose(true);
        recorder r; r.stop_after = 3; stop_via_seconds = true; attach(m, r); r.start();
        auto o = timed_solve(m); auto end = clk::now();
        final_line("H stop by Cbc_setMaximumSeconds(0) from the hook, VERBOSE", m, o.solve_s); hook_report(r, +1, o.solve_s);
        if(r.stop_asked) std::println(stderr, "   stop asked after {:.3f}s, solve returned {:.3f}s later", duration<double>(r.stop_time - r.t0).count(), duration<double>(end - r.stop_time).count());
        stop_via_seconds = false;
    }
    {   // the cut generator route on the small instance
        M m; build_mkp(m, small, +1, 100.0); m.set_time_limit(duration<double>(60));
        recorder r; API = &m.native_api();
        cut_ctx c{&r, &m};
        using add_t = void (*)(Cbc_Model *, void (*)(void *, void *, void *), const char *, void *);
        symf<add_t>(m.native_api().library_path(), "Cbc_addCutCallback")(m.native_model(), cut_cb, "probe", &c);
        r.start(); auto o = timed_solve(m);
        final_line("C2 small, cut callback reading the outer model", m, o.solve_s); hook_report(r, +1, o.solve_s, o.value, o.dual);
        std::println(stderr, "   nodes {} vs baseline {}", m.get_node_count(), base);
    }
    {
        M m; build_lp(m, 400, 600); m.set_time_limit(duration<double>(20)); m.set_verbose(true);
        recorder r; attach(m, r); r.start();
        try { auto o = timed_solve(m); final_line("I integer-free, VERBOSE", m, o.solve_s); hook_report(r, -1, o.solve_s); }
        catch(const std::exception & e) { std::println(stderr, "== I threw {}", e.what()); }
    }
    {   // the callback the development branch adds, when the library has it
        M m; build_mkp(m, large, +1, 100.0); m.set_time_limit(duration<double>(10));
        auto l = m.native_api().library_path();
        void * h = dlopen(l.c_str(), RTLD_NOW | RTLD_NOLOAD);
        void * f = h ? dlsym(h, "Cbc_addProgrCallback") : nullptr;
        if(!f) std::println(stderr, "== P Cbc_addProgrCallback: not in this library");
        else {
            using pcb_t = int (*)(void *, int, int, const char *, double, double, double, int, int *, void *);
            recorder r;
            pcb_t pcb = +[](void *, int, int step, const char * phase, double, double lb, double ub, int, int *, void * u) -> int {
                static_cast<recorder *>(u)->record(phase, ub, lb, step);
                return 0;
            };
            reinterpret_cast<void (*)(Cbc_Model *, pcb_t, void *)>(f)(m.native_model(), pcb, &r);
            r.start(); auto o = timed_solve(m);
            final_line("P Cbc_addProgrCallback, quiet, large 10s", m, o.solve_s); hook_report(r, +1, o.solve_s, o.value, o.dual);
        }
    }
    run_slices<M>(30, 100);
}
