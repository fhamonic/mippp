// O2, O7, O10, O8. usage: verify_copt_fire <exp>
#include "verify_copt_common.hpp"
using namespace std::chrono;
namespace {
constexpr int R = c::COPT_CBCONTEXT_MIPRELAX, N = c::COPT_CBCONTEXT_MIPNODE, S = c::COPT_CBCONTEXT_MIPSOL, I = c::COPT_CBCONTEXT_INCUMBENT;
struct setting { const char * name; std::vector<std::pair<const char *, int>> p; };
using builder = std::function<void(V &)>;
struct counts { long r = 0, n = 0, s = 0, i = 0; };
void one(const std::string & inst, builder b, const setting & st, const std::string & label, int mask, bool shipped, double lim, long node_limit = -1) {
    V m; m.iparam("Threads", 1); b(m); m.set_time_limit(duration<double>(lim));
    if(node_limit > 0) m.iparam("NodeLimit", (int)node_limit);
    for(auto [k, v] : st.p) m.iparam(k, v);
    auto ct = std::make_shared<counts>();
    if(!shipped) {
        if(mask == 0) m.empty_slot = true;
        else {
            m.raw_mask = mask;
            m.raw = [ct](c::copt_prob *, void *, int ctx) { if(ctx == R) ++ct->r; else if(ctx == N) ++ct->n; else if(ctx == S) ++ct->s; else ++ct->i; return 0; };
        }
    }
    const double t = cp::timed_solve(m);
    std::println(stderr, "{:10} {:22} {:14} {} simplex={} relax={} node={} sol={} inc={}", inst, st.name, label, cp::summary(m, t), m.simplex_iters(), ct->r, ct->n, ct->s, ct->i);
}
}
int main(int argc, char ** argv) {
    const std::string exp = argv[1];
    const builder atsp26 = [](V & m) { cp::build_atsp(m, 26, false, 5); };
    const builder atsp20 = [](V & m) { cp::build_atsp(m, 20, false, 2); };
    const builder mkp = [](V & m) { cp::build_mkp(m, 5, 60, 7); };
    const builder nd = [](V & m) { build_mcnd(m, 14, 30, 16, 3); };
    const setting def{"default", {}};
    if(exp == "o2") {
        for(auto & [nm, b] : std::vector<std::pair<std::string, builder>>{{"atsp26", atsp26}, {"nd", nd}, {"mkp", mkp}}) {
            for(const setting & st : std::vector<setting>{def, {"CutLevel=0", {{"CutLevel", 0}}}, {"CutLevel=0,Presolve=0", {{"CutLevel", 0}, {"Presolve", 0}}},
                 {"Root=0,Tree=0", {{"RootCutLevel", 0}, {"TreeCutLevel", 0}}}, {"RootRounds=0,NodeRounds=0", {{"RootCutRounds", 0}, {"NodeCutRounds", 0}}},
                 {"TreeCutLevel=0", {{"TreeCutLevel", 0}}}, {"RootCutLevel=0", {{"RootCutLevel", 0}}}})
                one(nm, b, st, "RN noop", R | N, false, 30, 30000);
        }
    }
    if(exp == "o7") {
        for(auto & [nm, b] : std::vector<std::pair<std::string, builder>>{{"atsp26", atsp26}, {"atsp20s2", atsp20}, {"nd", nd}, {"mkp", mkp}}) {
            for(const setting & st : std::vector<setting>{def, {"RootCutLevel=0", {{"RootCutLevel", 0}}}, {"RootCutRounds=0", {{"RootCutRounds", 0}}}, {"CutLevel=2", {{"CutLevel", 2}}}, {"Presolve=0", {{"Presolve", 0}}}}) {
                one(nm, b, st, "empty slot", 0, false, 40, 8000);
                one(nm, b, st, "shipped(N)", 0, true, 40, 8000);
                one(nm, b, st, "N noop", N, false, 40, 8000);
                one(nm, b, st, "R noop", R, false, 40, 8000);
                one(nm, b, st, "RN noop", R | N, false, 40, 8000);
            }
        }
    }
    if(exp == "o10") {
        for(int threads : {1, 0}) {
            V m; if(threads) m.iparam("Threads", threads);
            cp::build_mkp(m, 5, 60, 7);
            m.set_time_limit(duration<double>(15.0));
            m.iparam("NodeLimit", 20000);
            std::atomic<long> nodes{0}, rc_bad{0}, attr_nonzero{0}, attr_calls{0}, relax_rc_bad{0}, wrote_more{0}, exact{0}, off{0}, relax_calls{0};
            std::atomic<long> lastv{0}, maxdiff{0}, maxneg{0};
            std::atomic<long> dbl_rc{0};
            std::vector<std::pair<long, long>> samples;
            m.raw_mask = R | N;
            m.raw = [&](c::copt_prob * p, void * cb, int ctx) {
                long long buf = 0x7777777777777777LL;
                const int rc = m.native_api().GetCallbackInfo(cb, "NodeCnt", &buf);
                const int low = static_cast<int>(buf & 0xffffffffLL);
                const long long high = (buf >> 32) & 0xffffffffLL;
                if(high != 0x77777777LL) ++wrote_more;
                int a = -5; m.native_api().GetIntAttr(p, "NodeCnt", &a); ++attr_calls; if(a != 0) ++attr_nonzero;
                if(ctx == N) {
                    const long mine = ++nodes;
                    if(rc) ++rc_bad; else {
                        if(low == mine) ++exact; else ++off;
                        const long dd = low - mine;
                        if(dd > maxdiff) maxdiff = dd;
                        if(dd < maxneg) maxneg = dd;
                        if(mine % 2000 == 0 && threads == 1) samples.emplace_back(mine, low);
                        lastv = low;
                    }
                } else { ++relax_calls; if(rc) ++relax_rc_bad; }
                return 0;
            };
            const double t = cp::timed_solve(m);
            std::println(stderr, "o10 threads={} {} | MIPNODE events={} rc!=0:{} exact={} off={} maxdiff={} maxneg={} last={} | MIPRELAX calls={} rc!=0:{} | wrote beyond int:{} | GetIntAttr NodeCnt nonzero {}/{}", threads, cp::summary(m, t), nodes.load(), rc_bad.load(), exact.load(), off.load(), maxdiff.load(), maxneg.load(), lastv.load(), relax_calls.load(), relax_rc_bad.load(), wrote_more.load(), attr_nonzero.load(), attr_calls.load());
            std::string sm; for(auto [a, b] : samples) sm += std::format(" {}:{}", a, b);
            if(!sm.empty()) std::println(stderr, "   mine:theirs{}", sm);
        }
    }
    if(exp == "o10b") {
        for(const auto & [label, n, seed, threads] : std::vector<std::tuple<std::string, int, unsigned, int>>{{"atsp26 s5 thr1", 26, 5, 1}, {"atsp30 s5 thr1", 30, 5, 1}, {"atsp40 s5 thr1", 40, 5, 1}, {"atsp26 s5 default", 26, 5, 0}, {"atsp40 s5 default", 40, 5, 0}}) {
            V m; if(threads) m.iparam("Threads", threads);
            cp::build_atsp(m, n, false, seed);
            m.set_time_limit(duration<double>(15.0));
            long rc_bad[4] = {0, 0, 0, 0}, reads[4] = {0, 0, 0, 0}, diff_max[4] = {0, 0, 0, 0}, diff_min[4] = {0, 0, 0, 0}, back[4] = {0, 0, 0, 0};
            long mine = 0, prev = -1, lastv = 0;
            std::mutex mu;
            m.raw_mask = R | N | S | I;
            m.raw = [&](c::copt_prob *, void * cb, int ctx) {
                std::lock_guard l(mu);
                const int ix = ctx == R ? 0 : ctx == N ? 1 : ctx == S ? 2 : 3;
                int v = -7; const int rc = m.native_api().GetCallbackInfo(cb, "NodeCnt", &v);
                ++reads[ix]; if(rc) { ++rc_bad[ix]; return 0; }
                if(ctx == N) ++mine;
                const long d = v - mine;
                if(d > diff_max[ix]) diff_max[ix] = d;
                if(d < diff_min[ix]) diff_min[ix] = d;
                if(v < prev) ++back[ix];
                prev = v; lastv = v;
                return 0;
            };
            const double t = cp::timed_solve(m);
            std::println(stderr, "o10b {:18} {} | last NodeCnt={} my MIPNODE count={} | R reads={} rc!=0:{} diff[{},{}] back={} | N reads={} rc!=0:{} diff[{},{}] back={} | S reads={} rc!=0:{} diff[{},{}] back={} | I reads={} rc!=0:{} diff[{},{}] back={}", label, cp::summary(m, t), lastv, mine, reads[0], rc_bad[0], diff_min[0], diff_max[0], back[0], reads[1], rc_bad[1], diff_min[1], diff_max[1], back[1], reads[2], rc_bad[2], diff_min[2], diff_max[2], back[2], reads[3], rc_bad[3], diff_min[3], diff_max[3], back[3]);
        }
    }
    if(exp == "o8b") {
        V m; m.iparam("Threads", 16); m.iparam("MipTasks", 8);
        cp::build_mkp(m, 30, 100);
        m.set_time_limit(duration<double>(10.0));
        std::atomic<int> inflight{0}, peak{0}; std::atomic<long> calls{0}, overlaps{0};
        std::mutex mu; std::set<std::thread::id> tids;
        m.raw_mask = R | N | S | I;
        m.raw = [&](c::copt_prob *, void *, int) {
            const int now = inflight.fetch_add(1) + 1;
            if(now > 1) ++overlaps;
            int s = peak.load(); while(now > s && !peak.compare_exchange_weak(s, now)) {}
            ++calls; { std::lock_guard l(mu); tids.insert(std::this_thread::get_id()); }
            std::this_thread::sleep_for(milliseconds(1));   // gives the CPU away, unlike a spin
            inflight.fetch_sub(1);
            return 0;
        };
        const double t = cp::timed_solve(m);
        std::println(stderr, "o8b sleep 1 ms, 16 threads, 8 tasks, all contexts: {} | calls={} overlaps={} max_inflight={} os_threads={}", cp::summary(m, t), calls.load(), overlaps.load(), peak.load(), tids.size());
    }
    if(exp == "o8") {
        for(const auto & [label, mask, threads, tasks, spin_us] : std::vector<std::tuple<std::string, int, int, int, int>>{
                {"RN default thr 200us", R | N, 0, 0, 200}, {"RNSI default thr 200us", R | N | S | I, 0, 0, 200}, {"RN thr16 tasks8 2ms", R | N, 16, 8, 2000}, {"RNSI thr16 tasks8 2ms", R | N | S | I, 16, 8, 2000},
                {"RN default thr 0us", R | N, 0, 0, 0}}) {
            V m; if(threads) m.iparam("Threads", threads); if(tasks) m.iparam("MipTasks", tasks);
            cp::build_mkp(m, 30, 100);
            m.set_time_limit(duration<double>(10.0));
            std::atomic<int> inflight{0}, peak{0};
            std::atomic<long> calls{0}, overlaps{0}, nc{0}, rc{0}, sc{0}, ic{0};
            std::mutex mu; std::set<std::thread::id> tids; std::set<void *> cbs, probs;
            m.raw_mask = mask;
            m.raw = [&, spin_us = spin_us](c::copt_prob * p, void * cb, int ctx) {
                const int now = inflight.fetch_add(1) + 1;
                if(now > 1) ++overlaps;
                int s = peak.load(); while(now > s && !peak.compare_exchange_weak(s, now)) {}
                ++calls; (ctx == R ? rc : ctx == N ? nc : ctx == S ? sc : ic)++;
                { std::lock_guard l(mu); tids.insert(std::this_thread::get_id()); cbs.insert(cb); probs.insert(p); }
                const auto until = clk::now() + microseconds(spin_us);
                while(clk::now() < until) {}
                inflight.fetch_sub(1);
                return 0;
            };
            const double t = cp::timed_solve(m);
            std::println(stderr, "o8 {:26} {} | calls={} R={} N={} S={} I={} overlaps={} max_inflight={} os_threads={} cbdata={} probs={}", label, cp::summary(m, t), calls.load(), rc.load(), nc.load(), sc.load(), ic.load(), overlaps.load(), peak.load(), tids.size(), cbs.size(), probs.size());
        }
    }
}
