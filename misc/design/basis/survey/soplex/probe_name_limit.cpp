// Which entities can a SoPlex_readBasisFile line name at all? The reader
// names column j "x0x1...xj" and reads lines into a 256-char buffer.
#include <dlfcn.h>

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "mippp/solvers/soplex/all.hpp"

using namespace mippp;
using namespace mippp::operators;
using var_t = mippp::model_variable<int, double>;

static std::string cname(int j) {
    std::string s;
    for(int k = 0; k <= j; ++k) s += "x" + std::to_string(k);
    return s;
}
static std::string rname(int i) {
    std::string s;
    for(int k = 0; k <= i; ++k) s += "C" + std::to_string(k);
    return s;
}

int main() {
    const soplex_api & api = soplex_api::load();
    void * h = dlopen(api.library_path().c_str(), RTLD_NOW | RTLD_NOLOAD);
    auto readBasisFile =
        (int (*)(void *, const char *))dlsym(h, "SoPlex_readBasisFile");
    auto colStatus = (int (*)(void *, int))dlsym(h, "SoPlex_basisColStatus");
    const std::string path =
        "/tmp/claude-1000/-home-plaiseek-Projects-mippp/"
        "66409ca5-709c-4c59-b3c1-e7b746a2b680/scratchpad/survey/soplex/"
        "limit.bas";
    const int n = 150;
    soplex_lp m(api);
    std::vector<var_t> x;
    for(int j = 0; j < n; ++j)
        x.push_back(m.add_variable({.lower_bound = 0.0, .upper_bound = 10.0}));
    for(int i = 0; i < n; ++i) m.add_constraint(x[(std::size_t)i] <= 5.0);
    auto try_file = [&](const std::string & line) {
        {
            std::ofstream f(path);
            f << "NAME  soplex.bas\n" << line << "ENDATA\n";
        }
        return readBasisFile(m.native_model(), path.c_str());
    };
    int last_ok = -1;
    for(int j = 60; j < 100; ++j) {
        const std::string line = " UL " + cname(j) + "\n";
        const int r = try_file(line);
        const int code = colStatus(m.native_model(), j);
        std::printf("UL col %d: line length %zu -> ret %d, code read back %d\n",
                    j, line.size() - 1, r, code);
        if(r == 1 && code == 0) last_ok = j;
    }
    std::printf("last column nameable on an UL line: %d\n", last_ok);
    int last_row = -1;
    for(int i = 30; i < 120; ++i) {
        const std::string line =
            " XU x0             " + rname(i) + "\n";  // 8-wide col field
        const int r = try_file(line);
        const int rc = api.basisRowStatus(m.native_model(), i);
        if(r == 1 && rc == 0) last_row = i;
        if(i % 5 == 0 || r != 1)
            std::printf("XU x0 with row %d: line length %zu -> ret %d, row "
                        "code %d\n",
                        i, line.size() - 1, r, rc);
        if(r != 1) break;
    }
    std::printf("last row nameable next to column 0: %d\n", last_row);
    std::remove(path.c_str());
    return 0;
}
