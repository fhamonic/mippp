#include "mippp/solvers/scip/all.hpp"
#include "probe_common.hpp"
using M = mippp::scip_milp;
struct Scip; struct Hdlr; struct Event; struct Msg;
using retcode = int; constexpr retcode OKAY = 1;
using exec_t = retcode (*)(void *, Hdlr *, Event *, void *);
using initsol_t = retcode (*)(void *, Hdlr *);
// renumbered in SCIP 10 (type_event.h of 9.2.1 and 10.1.0)
static unsigned long long NODESOLVED = 0x80000 | 0x100000 | 0x200000, BESTSOL = 0x4000000, DUALIMPROVED = 0;
static decltype(&::dlopen) unused = nullptr;
static retcode (*includeHdlr)(void *, Hdlr **, const char *, const char *, exec_t, void *);
static retcode (*setInitsol)(void *, Hdlr *, initsol_t);
static retcode (*setExitsol)(void *, Hdlr *, initsol_t);
static retcode (*catchEv)(void *, unsigned long long, Hdlr *, void *, int *);
static void * (*hdlrData)(Hdlr *);
static unsigned long long (*evType)(Event *);
static long long (*nnodes)(void *);
static double (*dualb)(void *), (*primalb)(void *);
static retcode (*interruptS)(void *);
static unsigned long long catch_mask = 0;

static retcode (*dropEv)(void *, unsigned long long, Hdlr *, void *, int);
static bool drop_at_exit = true;
static int filter_pos = -1;
static retcode initsol(void * s, Hdlr * h) {
    return catchEv(s, catch_mask, h, hdlrData(h), &filter_pos);
}
// without this the restart's initsol adds a second subscription
static retcode exitsol(void * s, Hdlr * h) {
    return dropEv(s, catch_mask, h, hdlrData(h), filter_pos);
}
static retcode exec(void * s, Hdlr * h, Event * ev, void *) {
    auto & r = **static_cast<recorder **>(hdlrData(h));
    const auto t = evType(ev);
    if(r.record(t == BESTSOL ? "BESTSOLFOUND" : t == DUALIMPROVED ? "DUALBOUNDIMPROVED" : "NODESOLVED", primalb(s), dualb(s), nnodes(s))) interruptS(s);
    return OKAY;
}
template <class F> static void get(const std::filesystem::path & l, F & f, const char * n) { f = reinterpret_cast<F>(sym(l, n)); }
static std::vector<std::unique_ptr<recorder *>> slots;
static void set_bits(const std::filesystem::path & l) {
    if(symf<int (*)()>(l, "SCIPmajorVersion")() >= 10) {
        NODESOLVED = 0x100000 | 0x200000 | 0x400000; BESTSOL = 0x10000000; DUALIMPROVED = 0x1000000;
    }
}
static void attach(M & m, recorder & r, unsigned long long extra = 0) {
    auto l = m.native_api().library_path();
    set_bits(l);
    unsigned long long mask = extra ? BESTSOL : NODESOLVED | BESTSOL;
    get(l, includeHdlr, "SCIPincludeEventhdlrBasic"); get(l, setInitsol, "SCIPsetEventhdlrInitsol"); get(l, setExitsol, "SCIPsetEventhdlrExitsol"); get(l, dropEv, "SCIPdropEvent");
    get(l, catchEv, "SCIPcatchEvent"); get(l, hdlrData, "SCIPeventhdlrGetData"); get(l, evType, "SCIPeventGetType");
    get(l, nnodes, "SCIPgetNTotalNodes"); get(l, dualb, "SCIPgetDualbound"); get(l, primalb, "SCIPgetPrimalbound"); get(l, interruptS, "SCIPinterruptSolve");
    catch_mask = mask;
    slots.push_back(std::make_unique<recorder *>(&r));
    Hdlr * h = nullptr;
    void * s = m.native_model();
    int e1 = includeHdlr(s, &h, "probe_progress", "progress probe", exec, slots.back().get());
    int e2 = setInitsol(s, h, initsol);
    if(drop_at_exit) setExitsol(s, h, exitsol);
    if(e1 != OKAY || e2 != OKAY) std::println(stderr, "include {} initsol {}", e1, e2);
}
// a message handler, for the log path
struct msg_ctx { std::atomic<long> lines{0}; };
static msg_ctx msg;
static void info(void *, FILE *, const char *) { ++msg.lines; }
int main() {
    using std::chrono::duration;
    { M m; std::println(stderr, "library {} SCIP {}", m.native_api().library_path().string(), symf<int (*)()>(m.native_api().library_path(), "SCIPmajorVersion")()); }
    backend<M> b{.name = "scip", .attach = [](M & m, recorder & r) { attach(m, r); }, .one_thread = [](M &) {}};
    run_milp_matrix(b);
    if(std::getenv("PROBE_QUICK")) return 0;
    const auto large = make_mkp(30, 100);
    {   // X1 best solutions only: path, and the sparse end of the cadence
        M m; build_mkp(m, large, +1, 100.0); m.set_time_limit(duration<double>(5));
        recorder r; attach(m, r, 1); r.start(); auto o = timed_solve(m);
        final_line("X1 BESTSOLFOUND only", m, o.solve_s); hook_report(r, +1, o.solve_s);
    }
    for(bool verbose : {false, true}) {  // X2 message handler
        M m; build_mkp(m, large, +1, 100.0); m.set_time_limit(duration<double>(3));
        if(verbose) m.set_verbose(true);
        auto l = m.native_api().library_path();
        using create_t = retcode (*)(void **, unsigned, const char *, unsigned, void (*)(void *, FILE *, const char *), void (*)(void *, FILE *, const char *), void (*)(void *, FILE *, const char *), retcode (*)(void *), void *);
        void * h = nullptr; msg.lines = 0;
        int e1 = symf<create_t>(l, "SCIPmessagehdlrCreate")(&h, 0u, nullptr, 0u, info, info, info, nullptr, nullptr);
        int e2 = symf<retcode (*)(void *, void *)>(l, "SCIPsetMessagehdlr")(m.native_model(), h);
        auto o = timed_solve(m);
        std::println(stderr, "== X2 message handler verbose={}: create={} set={} lines={} in {:.2f}s", verbose, e1, e2, msg.lines.load(), o.solve_s);
    }
    {   // X3 the subscription left in place: the restart duplicates it
        drop_at_exit = false;
        M m; build_mkp(m, large, +1, 100.0); m.set_time_limit(duration<double>(10));
        recorder r; attach(m, r); r.start(); auto o = timed_solve(m);
        final_line("X3 no exitsol drop", m, o.solve_s); hook_report(r, +1, o.solve_s);
        drop_at_exit = true;
    }
    run_slices<M>(30, 100);
    {   // limits/time counts from the start of the transformed run: grow it
        M m; build_mkp(m, large, +1, 100.0);
        double total = 0;
        for(int i = 1; i <= 8; ++i) {
            m.set_time_limit(duration<double>(i));
            auto o = timed_solve(m); total += o.solve_s;
            final_line(std::format("G{} growing limit {}s (cumulated {:.2f}s)", i, i, total), m, o.solve_s);
        }
    }
}
