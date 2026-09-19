#pragma once

#include <algorithm>
#include <array>
#include <charconv>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "mippp/detail/dynamic_library.hpp"
#include "mippp/utility/solver_version.hpp"

// MSVC deprecates std::getenv (C4996) in favour of its own _dupenv_s; the
// portable call is kept and the warning silenced for this header only.
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif

namespace mippp::detail {

#if defined(_WIN32)
inline constexpr char path_list_separator = ';';
#else
inline constexpr char path_list_separator = ':';
#endif

// Appends to `dirs` the entries of the path-list env var `env_name`.
inline void append_env_dirs(std::vector<std::filesystem::path> & dirs,
                            const char * env_name) {
    const char * value = std::getenv(env_name);
    if(value == nullptr) return;
    const std::string_view paths(value);
    std::size_t start = 0;
    while(start <= paths.size()) {
        const std::size_t pos = paths.find(path_list_separator, start);
        const std::size_t end =
            (pos == std::string_view::npos) ? paths.size() : pos;
        if(end > start) dirs.emplace_back(paths.substr(start, end - start));
        if(pos == std::string_view::npos) break;
        start = pos + 1;
    }
}

// Appends to `dirs` the directories listed in an ld.so.conf-style file.
// 'include' directives are not followed.
inline void append_conf_dirs(std::vector<std::filesystem::path> & dirs,
                             const std::filesystem::path & file) {
    std::ifstream in(file);
    if(!in) return;
    std::string line;
    while(std::getline(in, line)) {
        const std::size_t b = line.find_first_not_of(" \t");
        if(b == std::string::npos || line[b] == '#') continue;
        const std::size_t e = line.find_last_not_of(" \t\r");
        const std::string_view entry(line.data() + b, e - b + 1);
        if(entry.starts_with("include")) continue;
        dirs.emplace_back(entry);
    }
}

// Reproduces the directories the dynamic loader would search, since
// dynamic_library opens exact paths only.
inline std::vector<std::filesystem::path> system_library_dirs() {
    std::vector<std::filesystem::path> dirs;
#if defined(_WIN32)
    append_env_dirs(dirs, "PATH");
    if(const char * system_root = std::getenv("SystemRoot");
       system_root != nullptr) {
        dirs.emplace_back(std::filesystem::path(system_root) / "System32");
        dirs.emplace_back(system_root);
    }
#elif defined(__APPLE__)
    append_env_dirs(dirs, "DYLD_LIBRARY_PATH");
    append_env_dirs(dirs, "DYLD_FALLBACK_LIBRARY_PATH");
    if(const char * home = std::getenv("HOME"); home != nullptr)
        dirs.emplace_back(std::filesystem::path(home) / "lib");
    dirs.emplace_back("/usr/local/lib");
    dirs.emplace_back("/opt/homebrew/lib");  // Homebrew (Apple Silicon)
    dirs.emplace_back("/opt/local/lib");     // MacPorts
    dirs.emplace_back("/usr/lib");
#else  // Linux and other glibc/ELF systems
    append_env_dirs(dirs, "LD_LIBRARY_PATH");
    std::error_code ec;
    append_conf_dirs(dirs, "/etc/ld.so.conf");
    // scan /etc/ld.so.conf.d, ld.so.conf's standard include target, directly
    if(std::filesystem::is_directory("/etc/ld.so.conf.d", ec)) {
        for(const auto & entry :
            std::filesystem::directory_iterator("/etc/ld.so.conf.d", ec)) {
            if(entry.path().extension() == ".conf")
                append_conf_dirs(dirs, entry.path());
        }
    }
    dirs.emplace_back("/usr/local/lib");
    dirs.emplace_back("/usr/lib");
    dirs.emplace_back("/lib");
#endif
    // drop duplicates (common in PATH and ld.so.conf), keeping the first
    // occurrence to preserve the loader's precedence
    std::vector<std::filesystem::path> unique_dirs;
    unique_dirs.reserve(dirs.size());
    for(auto & dir : dirs)
        if(std::ranges::find(unique_dirs, dir) == unique_dirs.end())
            unique_dirs.push_back(std::move(dir));
    return unique_dirs;
}

template <typename... Ts>
std::string concat_str(Ts &&... strs) {
    std::string result;
    result.reserve((std::string_view(strs).size() + ... + 0));
    (result.append(std::string_view(strs)), ...);
    return result;
}

// entry.path().filename() without materializing the intermediate path.
#if defined(_WIN32)
// the native encoding is wide: narrowing to std::string cannot be avoided
inline std::string entry_filename(
    const std::filesystem::directory_entry & entry) {
    return entry.path().filename().string();
}
#else
// zero-copy view into the entry's storage, valid as long as the entry
inline std::string_view entry_filename(
    const std::filesystem::directory_entry & entry) {
    const std::string & native = entry.path().native();
    return std::string_view(native).substr(native.rfind('/') + 1);
}
#endif

// Looks in `dir` for `decorated` ("libfoo.so", i.e. `base` + `suffix`) or,
// failing that, a version-suffixed variant — runtime-only installs often lack
// the unversioned symlink. The lexicographically greatest match wins,
// approximating the highest version.
inline std::optional<std::filesystem::path> find_library_in_dir(
    const std::filesystem::path & dir, const std::string_view base,
    const std::string_view decorated, const std::string_view suffix) {
    std::error_code ec;
    std::filesystem::path exact = dir / decorated;
    if(std::filesystem::exists(exact, ec)) return exact;

    if(!std::filesystem::is_directory(dir, ec)) return std::nullopt;
    std::string best;
    for(const auto & entry : std::filesystem::directory_iterator(dir, ec)) {
        // `filename` must outlive `f`: on Windows it owns the string
        const auto filename = entry_filename(entry);
        const std::string_view f(filename);
        if(f.size() <= decorated.size()) continue;
        // ELF versioned soname, e.g. libfoo.so.1.2.3
        const bool elf_versioned =
            f.starts_with(decorated) && f[decorated.size()] == '.';
        // Mach-O versioned dylib, e.g. libfoo.1.2.3.dylib
        const bool macho_versioned =
            f.starts_with(base) && f[base.size()] == '.' && f.ends_with(suffix);
        if((elf_versioned || macho_versioned) && f > best) best = f;
    }
    if(!best.empty()) return dir / best;
    return std::nullopt;
}

// Loads a solver's shared library as a `dynamic_library`, resolving it with
// the precedence shared by every `<solver>_api` backend (first match wins):
//
//   1. `path`, if non-null: the exact library file, used verbatim;
//   2. the `MIPPP_<key>_LIBRARY` env var (e.g. MIPPP_HIGHS_LIBRARY): idem,
//      letting versioned sonames like libhighs.so.1.10.0 be pinned;
//   3. the undecorated names of `names` ("highs" -> libhighs.so) searched
//      across the loader's directories (see system_library_dirs): the first
//      directory holding any of them wins, as it would for the loader, and
//      `names` order breaks ties within a directory. Several names cover
//      solvers renamed across releases (Cbc: libCbc / libCbcSolver) or
//      several releases one wrapper drives (Gurobi: libgurobi130 /
//      libgurobi120, newest first).
//
// Only step 3 is memoized: the directory walk costs milliseconds where
// reopening a known file costs microseconds, so default-constructing many api
// objects stays cheap. Successes only, keyed by the exact query, so a failed
// search is retried and two queries never alias; an entry that no longer
// loads is dropped and searched afresh. Steps 1 and 2 bypass it, which is what
// lets two versions of one solver be loaded side by side.
inline dynamic_library load_solver_library(
    const char * path, const char * key, std::span<const char * const> names,
    std::span<const char * const> probe_symbols = {}) {
    // a candidate must export `probe_symbols`: some distributions ship a
    // matching name without the C API (Ubuntu's libCbc.so vs libCbcSolver.so)
    const auto try_load =
        [probe_symbols](const std::filesystem::path & p,
                        std::string & err) -> std::optional<dynamic_library> {
        try {
            dynamic_library lib{p};
            for(auto && probe_symbol : probe_symbols)
                lib.get_symbol(probe_symbol);
            return lib;
        } catch(const std::runtime_error & e) {
            // both dynamic_library errors already name the file
            if(!err.empty()) err += "\n  ";
            err += e.what();
            return std::nullopt;
        }
    };

    std::string errors;
    if(path != nullptr) {
        if(auto lib = try_load(std::filesystem::path(path), errors))
            return std::move(*lib);
        throw std::runtime_error("mippp: failed to load the " +
                                 std::string(key) + " solver library:\n  " +
                                 errors);
    }

    const std::string env_var = detail::concat_str("MIPPP_", key, "_LIBRARY");
    if(const char * full_path = std::getenv(env_var.c_str());
       full_path != nullptr && *full_path != '\0') {
        if(auto lib = try_load(std::filesystem::path(full_path), errors))
            return std::move(*lib);
        throw std::runtime_error("mippp: failed to load the " +
                                 std::string(key) + " solver library from " +
                                 env_var + ":\n  " + errors);
    }

    // a handful of entries at most, one per backend actually constructed
    static std::mutex cache_mutex;
    static std::vector<std::pair<std::string, std::filesystem::path>> cache;
    const auto cache_find = [](const std::string & k) {
        return std::ranges::find_if(
            cache, [&](const auto & e) { return e.first == k; });
    };
    std::string cache_key(key);
    for(const char * n : names) {
        cache_key += ';';
        cache_key += n;
    }
    for(const char * probe_symbol : probe_symbols) {
        cache_key += '|';
        cache_key += probe_symbol;
    }
    std::optional<std::filesystem::path> cached;
    {
        const std::lock_guard<std::mutex> lock(cache_mutex);
        if(auto it = cache_find(cache_key); it != cache.end())
            cached = it->second;
    }
    if(cached) {
        if(auto lib = try_load(*cached, errors)) return std::move(*lib);
        const std::lock_guard<std::mutex> lock(cache_mutex);
        if(auto it = cache_find(cache_key); it != cache.end()) cache.erase(it);
    }

    std::vector<std::pair<std::string, std::string>> decorated_names;
    decorated_names.reserve(names.size());
    for(const char * n : names) {  // libfoo, libfoo.so
        std::string base = detail::concat_str(dynamic_library::prefix, n);
        std::string decorated =
            detail::concat_str(base, dynamic_library::suffix);
        decorated_names.emplace_back(std::move(base), std::move(decorated));
    }
    for(const auto & directory : detail::system_library_dirs()) {
        for(const auto & [base, decorated] : decorated_names)
            if(auto found = detail::find_library_in_dir(
                   directory, base, decorated, dynamic_library::suffix))
                if(auto lib = try_load(*found, errors)) {
                    const std::lock_guard<std::mutex> lock(cache_mutex);
                    if(auto it = cache_find(cache_key); it != cache.end())
                        it->second = *found;
                    else
                        cache.emplace_back(cache_key, *found);
                    return std::move(*lib);
                }
    }

    std::string tried;
    for(const char * n : names) {
        if(!tried.empty()) tried += "', '";
        tried += detail::concat_str(dynamic_library::prefix, n,
                                    dynamic_library::suffix);
    }
    throw std::runtime_error(
        "mippp: could not locate a usable " + std::string(key) +
        " solver library (tried '" + tried + "')." +
        (errors.empty() ? std::string{}
                        : "\nCandidates rejected:\n  " + errors) +
        "\nSet the environment variable " + env_var +
        " to its full path, or add its directory to LD_LIBRARY_PATH.");
}

// "2.10.12" -> {2,10,12}, "5.0" -> {5}, "45.01.02" -> {45,1,2}; text after
// the third component or the last number is ignored ("1.15.1-dev",
// "22.1.2.0"). nullopt when no number leads ("devel", "") or a component
// does not fit an int.
constexpr std::optional<solver_version> parse_solver_version(
    std::string_view text) {
    int components[3] = {0, 0, 0};
    const char * it = text.data();
    const char * const end = it + text.size();
    for(int & component : components) {
        const auto [next, ec] = std::from_chars(it, end, component);
        if(ec != std::errc{}) {
            if(&component == components) return std::nullopt;
            break;
        }
        it = next;
        if(it == end || *it != '.') break;
        ++it;
    }
    return solver_version{components[0], components[1], components[2]};
}

// Releases the wrapper has driven through the full suite, as a half-open
// interval: {{10}, {14}} is every 10.x.y up to 13.x.y. Both bounds come
// from a recorded pass -- a row of docs/solvers/compatibility.md or, for the
// solvers the matrix cannot obtain or license, a maintainer's run noted on
// that page; `before` is the first release not covered, never "anything
// newer".
struct solver_version_range {
    solver_version from, before;

    constexpr bool contains(const solver_version & v) const noexcept {
        return from <= v && v < before;
    }
};

template <std::size_t N>
constexpr bool is_validated(const std::array<solver_version_range, N> & ranges,
                            const solver_version & v) noexcept {
    static_assert(N > 0, "a wrapper validated for nothing is not a claim");
    for(const auto & r : ranges)
        if(r.contains(v)) return true;
    return false;
}

template <std::size_t N>
std::string to_string(const std::array<solver_version_range, N> & ranges) {
    std::string s;
    for(const auto & r : ranges) {
        if(!s.empty()) s += " or ";
        s += ">= " + to_string(r.from) + " and < " + to_string(r.before);
    }
    return s;
}

// Warns on stderr when the loaded library is a release the wrapper is not
// validated for -- usually harmless (the C APIs are stable) but worth knowing
// when behavior differs. `reported` is what the library said, verbatim, for
// the message; `loaded` its parse, nullopt when not a number ("devel").
// Set MIPPP_NO_VERSION_WARNING to silence.
template <std::size_t N>
void warn_on_unsupported_version(
    const char * key, const std::array<solver_version_range, N> & validated,
    std::optional<solver_version> loaded, std::string_view reported) {
    if((loaded && is_validated(validated, *loaded)) ||
       std::getenv("MIPPP_NO_VERSION_WARNING") != nullptr)
        return;
    std::fprintf(stderr,
                 "mippp: warning: the %s wrapper is validated for versions %s "
                 "but the loaded library reports %.*s; behavior may differ. "
                 "Set MIPPP_%s_LIBRARY to a validated library, or set "
                 "MIPPP_NO_VERSION_WARNING to silence this warning.\n",
                 key, to_string(validated).c_str(), int(reported.size()),
                 reported.data(), key);
}

// for libraries reporting components (Gurobi, MOSEK, SCIP)
template <std::size_t N>
void warn_on_unsupported_version(
    const char * key, const std::array<solver_version_range, N> & validated,
    const solver_version & loaded) {
    warn_on_unsupported_version(key, validated, loaded, to_string(loaded));
}

// for libraries reporting a string (HiGHS, Cbc, GLPK, Xpress), never null
template <std::size_t N>
void warn_on_unsupported_version(
    const char * key, const std::array<solver_version_range, N> & validated,
    std::string_view reported) {
    warn_on_unsupported_version(key, validated, parse_solver_version(reported),
                                reported);
}

// Base of every `<solver>_api`: an immortal, interned wrapper over one loaded
// library file. `Derived::load(path)` is the only way to obtain one, and it
// hands out the same instance for the same file, whether that file was
// reached through an explicit path, the env var or the directory search.
// So a model stores a pointer that can never dangle, a thousand subproblems
// share one function table, and whatever the api constructor does once per
// library (version check, licence init) really runs once.
//
// Instances are never destroyed: a solver's teardown must not run during
// static destruction (see dynamic_library), and the mapping is permanent.
//
// Derived declares `friend solver_api;`, a private constructor taking the
// library by rvalue, and the data `load()` and `check_library_version()`
// read:
//   static constexpr const char * key = "HIGHS";   // MIPPP_<key>_LIBRARY
//   static constexpr std::array library_names = {"highs"};
//   static constexpr std::array validated_versions = {...};
//   static constexpr std::array probe_symbols = {...};   // optional
template <typename Derived>
class solver_api {
protected:
    dynamic_library lib;

    explicit solver_api(dynamic_library && library) noexcept
        : lib(std::move(library)) {}

    // Returns the instance wrapping the file `library` refers to, creating it
    // on first sight. Keyed by the loader's handle, which is the identity of
    // a loaded file: two paths naming one file (symlinks) share an instance,
    // two files never do. The lock is held while Derived is constructed so
    // its one-time work cannot race with another thread's first load.
    static const Derived & intern(dynamic_library library) {
        using handle_type = dynamic_library::native_handle_type;
        static std::mutex mutex;
        // leaked on purpose, see above
        static auto & instances = *new std::vector<
            std::pair<handle_type, std::unique_ptr<const Derived>>>();
        const handle_type handle = library.native_handle();
        const std::lock_guard<std::mutex> lock(mutex);
        for(const auto & [h, instance] : instances)
            if(h == handle) return *instance;
        instances.emplace_back(handle, std::unique_ptr<const Derived>(
                                           new Derived(std::move(library))));
        return *instances.back().second;
    }

    // Records what the library reports as its release and warns when it is
    // outside Derived::validated_versions; the constructor calls it once. A
    // library whose C API reports no version (SoPlex) never does.
    void check_library_version(const solver_version & loaded) {
        _library_version = loaded;
        warn_on_unsupported_version(Derived::key, Derived::validated_versions,
                                    loaded);
    }
    void check_library_version(std::string_view reported) {
        _library_version = parse_solver_version(reported);
        warn_on_unsupported_version(Derived::key, Derived::validated_versions,
                                    _library_version, reported);
    }

private:
    std::optional<solver_version> _library_version;

public:
    solver_api(const solver_api &) = delete;
    solver_api & operator=(const solver_api &) = delete;

    // The newest of Derived::library_names found in the first directory
    // holding any (see load_solver_library), or the given file.
    static const Derived & load(const char * lib_path = nullptr) {
        if constexpr(requires { Derived::probe_symbols; })
            return intern(load_solver_library(lib_path, Derived::key,
                                              Derived::library_names,
                                              Derived::probe_symbols));
        else
            return intern(load_solver_library(lib_path, Derived::key,
                                              Derived::library_names));
    }

    // the file this api loaded: tells versions apart when several coexist
    const std::filesystem::path & library_path() const noexcept {
        return lib.path();
    }
    // the release that file reports; empty when it reports none (SoPlex's C
    // API has no version call) or not a number (a Cbc "devel" build)
    std::optional<solver_version> library_version() const noexcept {
        return _library_version;
    }
};

}  // namespace mippp::detail

#if defined(_MSC_VER)
#pragma warning(pop)
#endif
