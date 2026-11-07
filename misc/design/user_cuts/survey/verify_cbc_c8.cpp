// C8 alone, with its parameters from argv: stop_size, first limit, attach twice or not.
#include "verify_cbc_recipe.hpp"
using vc::M;
using std::chrono::duration;
int main(int argc, char ** argv) {
    vc::banner();
    const int n = std::atoi(argv[1]); const double first = std::atof(argv[2]); const int rep = argc > 3 ? std::atoi(argv[3]) : 1; const bool hook = argc > 4 ? std::atoi(argv[4]) : 1;
    for(int r = 0; r < rep; ++r) {
        M m; auto a = cp::build_atsp(m, n, false, 9);
        const auto sep = cf::dfj<M>(a);
        auto s = std::make_shared<cp::stats>();
        if(hook) m.attach(cp::watched(sep, s));
        m.set_time_limit(duration<double>(first));
        const double t1 = cp::timed_solve(m);
        std::println(stderr, "first {} | {}", cp::summary(m, t1), hook ? s->line() : "");
        if(hook) m.attach(cp::watched(sep, s));
        m.set_time_limit(duration<double>(60));
        const double t2 = cp::timed_solve(m);
        std::println(stderr, "then {} | {}", cp::summary(m, t2), hook ? s->line() : "");
    }
    return 0;
}
