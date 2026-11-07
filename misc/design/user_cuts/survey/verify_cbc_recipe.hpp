#pragma once
// Independent verification of the Cbc user-cut recipe. Every piece of the recipe sits
// behind an env toggle, so the negative controls reuse the same binary:
//   RECIPE_GUARD=0 RECIPE_PREPROCESS_OFF=0 RECIPE_SANITIZE=0 RECIPE_DROP_SATISFIED=0
//   RECIPE_PASSTREE=<n, 0 keeps the default> RECIPE_HOWOFTEN=<n> RECIPE_ATSOL=<0|1>
//   RECIPE_RESTORE=0 RECIPE_REGISTER_NATIVE=1 RECIPE_THREADS=<n> RECIPE_PARAMS=a=b,c=d
// The catch around the add entry point stays on in every arm: an exception must never
// cross Cbc's frames.
#include <cxxabi.h>
#include <dlfcn.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <charconv>
#include <chrono>
#include <cmath>
#include <compare>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <format>
#include <fstream>
#include <functional>
#include <initializer_list>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <print>
#include <random>
#include <ranges>
#include <ratio>
#include <set>
#include <span>
#include <sstream>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <tuple>
#include <type_traits>
#include <typeinfo>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>
#include <version>
#define private public
#include "mippp/solvers/cbc/all.hpp"
#undef private
#include "cut_conformance.hpp"

namespace vc {
namespace v1 = mippp::cbc::impl::v1;
using v1::Cbc_Model;

inline bool env_flag(const char * name, bool dflt) {
    const char * e = std::getenv(name);
    return e ? std::atoi(e) != 0 : dflt;
}
inline int env_int(const char * name, int dflt) {
    const char * e = std::getenv(name);
    return e ? std::atoi(e) : dflt;
}
struct toggles {
    bool guard = env_flag("RECIPE_GUARD", true);
    bool preprocess_off = env_flag("RECIPE_PREPROCESS_OFF", true);
    bool sanitize = env_flag("RECIPE_SANITIZE", true);
    bool drop_satisfied = env_flag("RECIPE_DROP_SATISFIED", true);
    int pass_tree = env_int("RECIPE_PASSTREE", 1);
    int how_often = env_int("RECIPE_HOWOFTEN", 1);
    char at_solution = static_cast<char>(env_int("RECIPE_ATSOL", 0));
    bool restore = env_flag("RECIPE_RESTORE", true);
    bool register_native = env_flag("RECIPE_REGISTER_NATIVE", false);
    int threads = env_int("RECIPE_THREADS", 0);
    bool quiet = env_flag("RECIPE_QUIET", false);
    bool range_check = env_flag("RECIPE_RANGECHECK", true);
    int normalize = env_int("RECIPE_NORMALIZE", 0);  // a candidate change. 1: largest |coefficient| = 1; 2: only when it is outside [1e-6, 1e6]; 3: geometric middle = 1, when a coefficient leaves that window
    int force_abi = env_int("RECIPE_FORCE_ABI", 0);  // 3 or 5: the wrong function type, UB on purpose
};
inline const toggles & tg() { static toggles t; return t; }

struct native {
    using cb3_t = void (*)(void *, void *, void *);
    using cb5_t = void (*)(void *, void *, void *, int, int);
    void * lib = nullptr;
    bool master_abi = false;
    void (*add3)(Cbc_Model *, cb3_t, const char *, void *) = nullptr;
    void (*add5)(Cbc_Model *, cb5_t, const char *, void *, int, char) = nullptr;
    void (*add_row)(void *, int, const int *, const double *, char, double) = nullptr;
    void (*add_global)(void *, int, const int *, const double *, char, double) = nullptr;
    int (*ncols)(void *) = nullptr;
    int (*nrows)(void *) = nullptr;
    const double * (*sol)(void *) = nullptr;
    const double * (*lb)(void *) = nullptr;
    const double * (*ub)(void *) = nullptr;
    template <class F> void get(F & f, const char * name) { f = reinterpret_cast<F>(dlsym(lib, name)); }
    explicit native(const std::filesystem::path & l) {
        lib = dlopen(l.c_str(), RTLD_NOW | RTLD_NOLOAD);
        if(!lib) { std::println(stderr, "dlopen NOLOAD failed: {}", dlerror()); std::exit(2); }
        void * cut = dlsym(lib, "Cbc_addCutCallback");
        get(add_global, "OsiCuts_addGlobalRowCut");
        master_abi = add_global != nullptr;   // the ABI switch of the recipe
        if(master_abi) add5 = reinterpret_cast<decltype(add5)>(cut);
        else add3 = reinterpret_cast<decltype(add3)>(cut);
        get(add_row, "OsiCuts_addRowCut");
        get(ncols, "Osi_getNumCols");
        get(nrows, "Osi_getNumRows");
        get(sol, "Osi_getColSolution");
        get(lb, "Osi_getColLower");
        get(ub, "Osi_getColUpper");
    }
};

// What the hook saw, per solve.
struct counters {
    std::atomic<long> calls{0}, mismatch{0}, added{0}, skipped{0}, caught{0}, below_root{0}, out_of_range{0};
    std::atomic<int> min_cols{1 << 30}, max_rows{0}, rows_in_sub_max{0}, max_fixed{0};
    std::map<int, long> by_fixed;
    std::mutex m;
    std::string line() const {
        return std::format("calls={} below_root_proxy={} mismatch={} min_cols={} added={} skipped={} caught={} out_of_range={} max_rows={} rows_in_sub_max={} max_fixed={}",
            calls.load(), below_root.load(), mismatch.load(), mismatch.load() ? min_cols.load() : -1, added.load(), skipped.load(), caught.load(), out_of_range.load(), max_rows.load(), rows_in_sub_max.load(), max_fixed.load());
    }
};
struct hook_ctx {
    cp::separator sep;
    std::shared_ptr<native> abi;
    int cols = 0;
    std::vector<char> is_int;
    std::vector<double> lb0, ub0;
    counters c;
    // Sees every call, on the model the hook is handed, before the guard.
    std::function<void(void * osi, void * cuts, int level, int pass)> observer;
    std::mutex lock;
};
inline void hook_body(void * osi, void * cuts, hook_ctx & h, int level, int pass) noexcept {
    try {
        auto & c = h.c;
        c.calls.fetch_add(1);
        const auto & A = *h.abi;
        const int n = A.ncols(osi);
        const int rows = A.nrows(osi);
        int mc = c.min_cols.load();
        while(n < mc && !c.min_cols.compare_exchange_weak(mc, n)) {}
        if(h.observer) h.observer(osi, cuts, level, pass);
        if(n != h.cols) {
            c.mismatch.fetch_add(1);
            int r = c.rows_in_sub_max.load();
            while(rows > r && !c.rows_in_sub_max.compare_exchange_weak(r, rows)) {}
            if(tg().guard) return;
        } else {
            int r = c.max_rows.load();
            while(rows > r && !c.max_rows.compare_exchange_weak(r, rows)) {}
            const double * lo = A.lb(osi); const double * up = A.ub(osi);
            int fixed = 0;
            for(int j = 0; j < n; ++j) if(h.is_int[static_cast<std::size_t>(j)] && lo[j] == up[j] && h.lb0[static_cast<std::size_t>(j)] != h.ub0[static_cast<std::size_t>(j)]) ++fixed;
            if(fixed) c.below_root.fetch_add(1);
            int f = c.max_fixed.load();
            while(fixed > f && !c.max_fixed.compare_exchange_weak(f, fixed)) {}
            { std::lock_guard l(c.m); c.by_fixed[fixed]++; }
        }
        std::vector<double> x(static_cast<std::size_t>(std::max(n, h.cols)), 0.0);
        std::copy_n(A.sol(osi), n, x.begin());
        h.sep(x, [&](const cp::raw_cut & raw) -> int {
            std::optional<cp::raw_cut> r;
            if(tg().sanitize) r = cf::sanitize(raw, x, tg().drop_satisfied);
            else r = raw;
            if(!r) { c.skipped.fetch_add(1); return 0; }
            if(tg().normalize) {
                double big = 0; for(double v : r->val) big = std::max(big, std::fabs(v));
                double small = big; for(double v : r->val) if(v != 0) small = std::min(small, std::fabs(v));
                if(tg().normalize == 3) {
                    // the geometric middle of the range goes to 1, when a coefficient leaves [1e-6, 1e6]
                    if(big > 1e6 || small < 1e-6) { const double f = std::sqrt(big * small); for(double & v : r->val) v /= f; r->rhs /= f; }
                } else if(big > 0 && (tg().normalize == 1 || big > 1e6 || big < 1e-6)) { for(double & v : r->val) v /= big; r->rhs /= big; }
            }
            if(tg().range_check) for(int j : r->ind) if(j >= n) { c.out_of_range.fetch_add(1); return -2; }
            try {
                A.add_row(cuts, static_cast<int>(r->ind.size()), r->ind.data(), r->val.data(), r->sense, r->rhs);
                c.added.fetch_add(1);
                return 0;
            } catch(...) {
                c.caught.fetch_add(1);
                const std::type_info * t = abi::__cxa_current_exception_type();
                static std::atomic<int> printed{0};
                if(printed.fetch_add(1) < 3) std::println(stderr, "   [OsiCuts_addRowCut threw {}]", t ? cp::short_name(t->name()) : "?");
                return -1;
            }
        });
    } catch(...) { h.c.caught.fetch_add(1); }
}
inline void hook3(void * osi, void * cuts, void * app) { hook_body(osi, cuts, *static_cast<hook_ctx *>(app), -1, -1); }
inline void hook5(void * osi, void * cuts, void * app, int level, int pass) { hook_body(osi, cuts, *static_cast<hook_ctx *>(app), level, pass); }

inline std::vector<std::pair<std::string, std::string>> env_params() {
    std::vector<std::pair<std::string, std::string>> out;
    if(const char * e = std::getenv("RECIPE_PARAMS")) {
        std::stringstream ss(e);
        std::string item;
        while(std::getline(ss, item, ',')) {
            const auto p = item.find('=');
            if(p != std::string::npos) out.emplace_back(item.substr(0, p), item.substr(p + 1));
        }
    }
    return out;
}

// cbc_milp::solve(), plus the recipe between the copy and the native solve. The real
// wrapper owns both, so the `private` opening above is an artefact of this probe.
struct M : mippp::cbc_milp {
    using mippp::cbc_milp::cbc_milp;
    std::shared_ptr<hook_ctx> hook;
    std::vector<std::pair<std::string, std::string>> extra;  // on the solved model, every solve
    std::function<void(M &)> before_native_solve;
    void param(const std::string & n, const std::string & v) { extra.emplace_back(n, v); }
    // Harness shim for master: Cbc_bestSolution and Cbc_getColSolution both return null after a
    // stop with no incumbent, and cbc_milp::get_solution_value() reads through them.
    double get_solution_value() {
        if(_results_valid && _lazy_num_variables != 0u && !Cbc->bestSolution(_solved_model()) && !Cbc->getColSolution(_solved_model()))
            throw std::runtime_error("no solution to read");
        return mippp::cbc_milp::get_solution_value();
    }
    void attach(cp::separator sep) {
        hook = std::make_shared<hook_ctx>();
        hook->sep = std::move(sep);
        hook->abi = std::make_shared<native>(native_api().library_path());
    }
    void solve() {
        reset_status();
        _searched = false;
        if(mip_copy) { Cbc->deleteModel(mip_copy); mip_copy = nullptr; }
        if(_lazy_num_variables == 0u) { _mark_results_valid(); return; }
        if(copy_mip_solves && Cbc->getNumIntegers(model) > 0) _copy_into_mip_copy();
        Cbc_Model * solved = _solved_model();
        for(const auto & [n, v] : env_params()) Cbc->setParameter(solved, n.c_str(), v.c_str());
        for(const auto & [n, v] : extra) Cbc->setParameter(solved, n.c_str(), v.c_str());
        if(tg().threads) Cbc->setParameter(solved, "threads", std::to_string(tg().threads).c_str());
        bool master_in_place = false;
        if(hook) {
            const auto & A = *hook->abi;
            hook->cols = static_cast<int>(_lazy_num_variables);
            hook->is_int.resize(_lazy_num_variables);
            hook->lb0.assign(Cbc->getColLower(model), Cbc->getColLower(model) + _lazy_num_variables);
            hook->ub0.assign(Cbc->getColUpper(model), Cbc->getColUpper(model) + _lazy_num_variables);
            for(std::size_t j = 0; j < _lazy_num_variables; ++j) hook->is_int[j] = static_cast<char>(Cbc->isInteger(model, static_cast<int>(j)));
            if(tg().preprocess_off) Cbc->setParameter(solved, "preprocess", "off");
            Cbc_Model * where = tg().register_native ? model : solved;
            if(tg().force_abi == 3) {
                reinterpret_cast<void (*)(Cbc_Model *, native::cb3_t, const char *, void *)>(dlsym(A.lib, "Cbc_addCutCallback"))(where, hook3, "user", hook.get());
            } else if(tg().force_abi == 5) {
                reinterpret_cast<void (*)(Cbc_Model *, native::cb5_t, const char *, void *, int, char)>(dlsym(A.lib, "Cbc_addCutCallback"))(where, hook5, "user", hook.get(), tg().how_often, tg().at_solution);
            } else if(A.master_abi) {
                if(tg().pass_tree != 0) Cbc->setParameter(solved, "passTreeCuts", std::to_string(tg().pass_tree).c_str());
                A.add5(where, hook5, "user", hook.get(), tg().how_often, tg().at_solution);
                master_in_place = solved == model;
            } else {
                A.add3(where, hook3, "user", hook.get());
            }
        }
        if(before_native_solve) before_native_solve(*this);
        Cbc->solve(solved);
        if(hook && !tg().quiet) std::println(stderr, "   [recipe: {}]", hook->c.line());
        _status = _get_status(solved);
        _searched = Cbc->getNumIntegers(solved) > 0 &&
                    (mippp::is_a<mippp::status::optimal>(_status) ||
                     mippp::is_a<mippp::status::interrupted>(_status) ||
                     mippp::is_a<mippp::status::limit_reached>(_status));
        _mark_results_valid();
        if(master_in_place && tg().restore) {
            // the solve was in place: nothing of the recipe may reach a later solve
            hook->abi->add5(model, nullptr, "user", nullptr, 1, 0);
            Cbc->setParameter(model, "preprocess", "sos");
            Cbc->setParameter(model, "passTreeCuts", "10");
        }
    }
};
// budgeted ATSP, complete through MTZ: a knapsack over random weights on the arcs
inline void add_budget(M & m, const cp::atsp & a, double frac, unsigned seed) {
    std::mt19937 rng(seed * 31u + 17u);
    std::uniform_int_distribution<int> W(1, 100);
    const auto N = static_cast<std::size_t>(a.n);
    std::vector<std::pair<int, double>> t; double total = 0;
    for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j) { const double w = W(rng); t.emplace_back(a.col[i][j], w); total += w; }
    const double b = std::floor(frac * total / static_cast<double>(a.n - 1));
    auto keys = std::views::iota(std::size_t{0}, t.size());
    m.add_constraint(xsum(keys, [&t](std::size_t k) { return t[k].second * mippp::model_variable<int, double>(t[k].first); }) <= b);
}
inline void banner() {
    M m;
    const auto & api = m.native_api();
    native n(api.library_path());
    std::println(stderr, "library {} version {} library_version_parsed={} copy_mip_solves={} abi={}", api.library_path().string(), api.getVersion(),
        api.library_version().has_value(), m.copy_mip_solves, n.master_abi ? "master(5-arg)" : "2.10(3-arg)");
}
}  // namespace vc
