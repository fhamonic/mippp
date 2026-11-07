// O5: a repeated column in a valid cut, raw path (the recipe minus folding), one process per case.
// usage: verify_copt_dup <inst> <form> <after_nodes> [lim]  inst: nd | gadgets | atsp   form: none plain dup_cont dup_bin dup_far dup_quad
#include "verify_copt_common.hpp"
using namespace std::chrono;
int main(int argc, char ** argv) {
    const std::string inst = argv[1], form = argv[2];
    const int after = std::atoi(argv[3]);
    const double lim = argc > 4 ? std::atof(argv[4]) : 15.0;
    V m; m.iparam("Threads", 1); m.set_time_limit(duration<double>(lim));
    std::function<std::vector<cp::raw_cut>(const std::vector<double> &)> sepf;
    std::vector<int> bins;
    std::optional<cp::gadgets<V>> gad;
    std::optional<cp::atsp> at;
    std::optional<mcnd> nd;
    if(inst == "nd") {
        setenv("CAPF", "0.2", 1);
        auto v = build_mcnd(m, 14, 30, 16, 1); nd = v.g;
        sepf = [&](const std::vector<double> & x) { return nd->links(x, 5); };
    } else if(inst == "gadgets") {
        for(const char * n : {"Presolve", "HeurLevel", "ConflictAnalysis"}) m.iparam(n, 0);
        m.iparam("RootCutLevel", 0);
        gad = cp::build_gadgets(m, 10, false);
        sepf = [&](const std::vector<double> & x) {
            std::vector<cp::raw_cut> out;
            for(int i = 1; i < gad->k; ++i) { auto c = gad->hull_cut(i); if(c.violation(x) > 1e-3) { out.push_back(c); if(out.size() == 3) break; } }
            return out;
        };
    } else {
        at = cp::build_atsp(m, 26, false, 5);
        sepf = [&](const std::vector<double> & x) { return at->separate(x); };
    }
    std::atomic<long> relax{0}, adds{0}, errs{0}, dup_added{0};
    std::atomic<int> last_rc{0};
    if(form != "none") {
        m.raw_mask = c::COPT_CBCONTEXT_MIPRELAX | c::COPT_CBCONTEXT_MIPNODE;
        m.raw = [&](c::copt_prob *, void * cb, int ctx) {
            if(ctx != c::COPT_CBCONTEXT_MIPRELAX) return 0;
            ++relax;
            int nc = 0; m.native_api().GetCallbackInfo(cb, "NodeCnt", &nc);
            if(nc < after) return 0;
            std::vector<double> x(m.num_variables());
            if(m.native_api().GetCallbackInfo(cb, c::COPT_CBINFO_RELAXSOLUTION, x.data())) return 0;
            for(auto cut : sepf(x)) {
                if(form == "dup_cont") { const double v0 = cut.val[0]; cut.val[0] = v0 / 2; cut.ind.insert(cut.ind.begin() + 1, cut.ind[0]); cut.val.insert(cut.val.begin() + 1, v0 / 2); ++dup_added; }
                if(form == "dup_bin") { const double v0 = cut.val[1]; cut.val[1] = v0 / 2; cut.ind.push_back(cut.ind[1]); cut.val.push_back(v0 / 2); ++dup_added; }
                if(form == "dup_far") { const double v0 = cut.val[0]; cut.val[0] = v0 / 2; cut.ind.push_back(cut.ind[0]); cut.val.push_back(v0 / 2); ++dup_added; }
                if(form == "dup_quad") { const double v0 = cut.val[0]; cut.val[0] = v0 / 4; for(int q = 0; q < 3; ++q) { cut.ind.push_back(cut.ind[0]); cut.val.push_back(v0 / 4); } ++dup_added; }
                const int rc = m.native_api().AddCallbackUserCut(cb, (int)cut.ind.size(), cut.ind.data(), cut.val.data(), cut.sense, cut.rhs);
                ++adds; if(rc) { ++errs; last_rc = rc; }
            }
            return 0;
        };
    }
    const double t = cp::timed_solve(m);
    std::println(stderr, "{:8} {:9} after={:4}: {} relax={} adds={} dup={} rc_err={} last_rc={}", inst, form, after, cp::summary(m, t), relax.load(), adds.load(), dup_added.load(), errs.load(), last_rc.load());
}
