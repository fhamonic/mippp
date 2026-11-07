#include "mippp/solvers/xpress/all.hpp"
#include "probe_common.hpp"
using M = mippp::xpress_milp;
using L = mippp::xpress_lp;
namespace v1 = mippp::xpress::impl::v1;
using P = v1::XPRSprob;
static const v1::xpress_api * API;
using intcb = int (*)(P, void *);
using voidcb = void (*)(P, void *);
using addint = int (*)(P, intcb, void *, int);
using addvoid = int (*)(P, voidcb, void *, int);
using interrupt_t = int (*)(P, int);
static interrupt_t xinterrupt;
constexpr int X_NODES = 1013, X_MIPOBJVAL = 2003, X_BESTBOUND = 2004, X_MIPLOG = 8028, X_SIMPLEXITER = 1009, X_LPOBJVAL = 2001, X_BARITER = 5001, X_BARPRIMALOBJ = 4001;

static bool read(P p, double & obj, double & bnd, int & nodes) {
    int e1 = API->getdblattrib(p, X_MIPOBJVAL, &obj), e2 = API->getdblattrib(p, X_BESTBOUND, &bnd), e3 = API->getintattrib(p, X_NODES, &nodes);
    return !(e1 || e2 || e3);
}
struct ctx { recorder * r; bool nonzero = false; std::string ev; };
static std::vector<std::unique_ptr<ctx>> keep;
static int cb_log(P p, void * u) {
    auto * c = static_cast<ctx *>(u); double o = NAN, b = NAN; int n = -1;
    if(!read(p, o, b, n)) ++c->r->quiet_read_errors;
    if(c->r->record(c->ev, o, b, n)) {
        if(c->nonzero) return 1;
        xinterrupt(p, 9 /*XPRS_STOP_USER*/);
    }
    return 0;
}
static void cb_void(P p, void * u) { (void)cb_log(p, u); }
static int cb_checktime(P p, void * u) {
    auto * c = static_cast<ctx *>(u); double o = NAN, b = NAN; int n = -1;
    bool ok = read(p, o, b, n);
    c->r->record(ok ? "CHECKTIME_readable" : "CHECKTIME_unreadable", o, b, n);
    return 0;
}
template <class X>
static void bind(X & m) {
    API = &m.native_api();
    xinterrupt = symf<interrupt_t>(m.native_api().library_path(), "XPRSinterrupt");
}
template <class F, class X>
static void add(X & m, const char * sy, F f, recorder & r, const char * ev, bool nz = false) {
    bind(m);
    keep.push_back(std::make_unique<ctx>(ctx{&r, nz, ev}));
    using add_t = int (*)(P, F, void *, int);
    auto fn = symf<add_t>(m.native_api().library_path(), sy);
    if(int e = fn(m.native_model(), f, keep.back().get(), 0)) throw std::runtime_error(std::format("{} -> {}", sy, e));
}
static std::atomic<long> msgs{0};
static void msg_cb(P, void *, const char *, int, int) { ++msgs; }
static int lp_cb(P p, void * u) {
    auto * c = static_cast<ctx *>(u); int it = -1; double o = NAN;
    API->getintattrib(p, X_SIMPLEXITER, &it); API->getdblattrib(p, X_LPOBJVAL, &o);
    c->r->record(c->ev, o, NAN, it); return 0;
}
static int bar_cb(P p, void * u) {
    auto * c = static_cast<ctx *>(u); int it = -1; double o = NAN;
    API->getintattrib(p, X_BARITER, &it); API->getdblattrib(p, X_BARPRIMALOBJ, &o);
    c->r->record(c->ev, o, NAN, it); return 0;
}

int main() {
    using std::chrono::duration;
    backend<M> b{.name = "xpress",
        .attach = [](M & m, recorder & r) { add(m, "XPRSaddcbmiplog", cb_log, r, "MIPLOG"); add(m, "XPRSaddcbintsol", cb_void, r, "INTSOL"); },
        .one_thread = [](M & m) { m.set_thread_limit(1); }};
    run_milp_matrix(b);
    if(std::getenv("PROBE_QUICK")) return 0;
    const auto large = make_mkp(30, 100);
    const auto single = [&](const char * label, auto setup) {
        M m; build_mkp(m, large, +1, 100.0); m.set_time_limit(duration<double>(5));
        recorder r; setup(m, r); r.start(); auto o = timed_solve(m);
        final_line(label, m, o.solve_s); hook_report(r, +1, o.solve_s);
        std::println(stderr, "   read errors {}", r.quiet_read_errors);
    };
    single("X1 miplog only, default MIPLOG", [](M & m, recorder & r) { add(m, "XPRSaddcbmiplog", cb_log, r, "MIPLOG"); });
    single("X2 miplog only, MIPLOG=-1", [](M & m, recorder & r) { m.native_api().setintcontrol(m.native_model(), X_MIPLOG, -1); add(m, "XPRSaddcbmiplog", cb_log, r, "MIPLOG"); });
    single("X3 checktime", [](M & m, recorder & r) { add(m, "XPRSaddcbchecktime", cb_checktime, r, "CHECKTIME"); });
    single("X4 optnode", [](M & m, recorder & r) {
        using node_t = void (*)(P, void *, int *);
        add(m, "XPRSaddcboptnode", (node_t)+[](P p, void * u, int *) { (void)cb_log(p, u); }, r, "OPTNODE"); });
    single("X5 nodelpsolved", [](M & m, recorder & r) { add(m, "XPRSaddcbnodelpsolved", cb_void, r, "NODELPSOLVED"); });
    {   // X6 stop by a non-zero return of miplog
        M m; build_mkp(m, large, +1, 100.0); m.set_time_limit(duration<double>(10));
        recorder r; r.stop_after = 3; add(m, "XPRSaddcbmiplog", cb_log, r, "MIPLOG", true); r.start();
        auto o = timed_solve(m);
        final_line("X6 stop by non-zero miplog return", m, o.solve_s); hook_report(r, +1, o.solve_s);
    }
    for(bool verbose : {false, true}) {  // X7 messages
        M m; build_mkp(m, large, +1, 100.0); m.set_time_limit(duration<double>(3));
        if(verbose) m.set_verbose(true);
        bind(m); msgs = 0;
        int e = m.native_api().addcbmessage(m.native_model(), msg_cb, nullptr, 0);
        auto o = timed_solve(m);
        std::println(stderr, "== X7 addcbmessage verbose={}: add={} messages={} in {:.2f}s", verbose, e, msgs.load(), o.solve_s);
    }
    {   // L LP
        L m; build_lp(m, 2000, 2900); // Community: at most 5000 rows and columns
        m.set_time_limit(duration<double>(40));
        recorder r; bind(m);
        keep.push_back(std::make_unique<ctx>(ctx{&r, false, "LPLOG"}));
        symf<addint>(m.native_api().library_path(), "XPRSaddcblplog")(m.native_model(), lp_cb, keep.back().get(), 0);
        keep.push_back(std::make_unique<ctx>(ctx{&r, false, "BARLOG"}));
        symf<addint>(m.native_api().library_path(), "XPRSaddcbbarlog")(m.native_model(), bar_cb, keep.back().get(), 0);
        r.start(); auto o = timed_solve(m);
        final_line("L LP on xpress_lp", m, o.solve_s); hook_report(r, -1, o.solve_s);
    }
    run_slices<M>(30, 100);
}
