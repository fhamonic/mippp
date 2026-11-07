#include "mippp/solvers/highs/all.hpp"
#include "probe_common.hpp"
using M = mippp::highs_milp;
using L = mippp::highs_lp;
namespace v1 = mippp::highs::impl::v1;
using Out = v1::HighsCallbackDataOut;
using In = v1::HighsCallbackDataIn;
using item_t = const void * (*)(const Out *, const char *);
static item_t item;
enum { T_LOG = 0, T_SPX = 1, T_IPM = 2, T_SOL = 3, T_IMPROVE = 4, T_MIPLOG = 5, T_MIPINT = 6 };
static const char * tname(int t) {
    static const char * n[] = {"Logging","SimplexInterrupt","IpmInterrupt","MipSolution","MipImprovingSolution","MipLogging","MipInterrupt"};
    return t >= 0 && t <= 6 ? n[t] : "other";
}
static std::atomic<int> struct_shown{0};
static void cb(int type, const char *, const Out * o, In * in, void * u) {
    auto & r = *static_cast<recorder *>(u);
    if(type == T_LOG) { r.count("Logging"); return; }
    if(type == T_SPX || type == T_IPM) {
        const int it = *static_cast<const int *>(item(o, type == T_SPX ? "simplex_iteration_count" : "ipm_iteration_count"));
        const double obj = *static_cast<const double *>(item(o, "objective_function_value"));
        if(r.record(tname(type), obj, NAN, it)) in->user_interrupt = 1;
        return;
    }
    const double p = *static_cast<const double *>(item(o, "mip_primal_bound"));
    const double d = *static_cast<const double *>(item(o, "mip_dual_bound"));
    const long long n = *static_cast<const long long *>(item(o, "mip_node_count"));
    // the layout in highs_api.hpp, against the named item
    if(n > 1000 && struct_shown.exchange(1) == 0)
        r.note = std::format("struct in highs_api.hpp reads dual={:.6g} primal={:.6g} nodes={} where the items say dual={:.6g} primal={:.6g} nodes={}",
                             o->mip_dual_bound, o->mip_primal_bound, o->mip_node_count, d, p, n);
    if(r.record(tname(type), p, d, n)) in->user_interrupt = 1;
}
template <class X>
static void attach_types(X & m, recorder & r, std::vector<int> types) {
    auto h = m.native_model();
    item = symf<item_t>(m.native_api().library_path(), "Highs_getCallbackDataOutItem");
    struct_shown = 0;
    // declared in highs_api.hpp, not loaded by it
    auto set = symf<v1::HighsInt (*)(void *, v1::HighsCCallbackType *, void *)>(m.native_api().library_path(), "Highs_setCallback");
    auto start = symf<v1::HighsInt (*)(void *, int)>(m.native_api().library_path(), "Highs_startCallback");
    set(h, cb, &r);
    for(int t : types) if(start(h, t) != 0) std::println(stderr, "startCallback({}) failed", t);
}
int main(int argc, char ** argv) {
    using std::chrono::duration;
    std::string what = argc > 1 ? argv[1] : "all";
    { M m; std::println(stderr, "library {} ", m.native_api().library_path().string()); }
    backend<M> b{.name = "highs",
        .attach = [](M & m, recorder & r) { attach_types(m, r, {T_MIPINT, T_IMPROVE}); },
        .one_thread = [](M & m) { m.native_api().setIntOptionValue(m.native_model(), "threads", 1); }};
    if(what == "all") run_milp_matrix(b);
    if(std::getenv("PROBE_QUICK")) return 0;
    const auto large = make_mkp(30, 100);
    const auto single = [&](std::string label, std::vector<int> types, std::function<void(M &)> setup) {
        M m; build_mkp(m, large, +1, 100.0); m.set_time_limit(duration<double>(5));
        setup(m);
        recorder r; attach_types(m, r, types); r.start(); auto o = timed_solve(m);
        final_line(label, m, o.solve_s); hook_report(r, +1, o.solve_s);
    };
    auto noop = [](M &) {};
    auto verbose = [](M & m) { m.set_verbose(true); };
    auto silent_log = [](M & m) { m.set_verbose(true); m.native_api().setBoolOptionValue(m.native_model(), "log_to_console", false); };
    single("X1 MipLogging, quiet", {T_MIPLOG}, noop);
    single("X2 MipLogging, verbose", {T_MIPLOG}, verbose);
    single("X3 MipLogging, output_flag on + log_to_console off", {T_MIPLOG}, silent_log);
    single("X4 Logging, quiet", {T_LOG}, noop);
    single("X5 Logging, verbose", {T_LOG}, verbose);
    single("X6 Logging, output_flag on + log_to_console off", {T_LOG}, silent_log);
    single("X7 MipInterrupt, verbose (the struct read in the note)", {T_MIPINT}, verbose);
    if(what == "tail") return 0;
    {
        L m; build_lp(m, 12000, 18000); m.set_time_limit(duration<double>(40));
        recorder r; attach_types(m, r, {T_SPX, T_IPM});
        r.start(); auto o = timed_solve(m);
        final_line("L LP on highs_lp, Simplex+Ipm interrupt types", m, o.solve_s); hook_report(r, -1, o.solve_s);
    }
    run_slices<M>(30, 100);
}
