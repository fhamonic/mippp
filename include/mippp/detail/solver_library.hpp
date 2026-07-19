#pragma once

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "dylib.hpp"

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

// Reproduces the directories the dynamic loader would search, since dylib 3.0
// no longer resolves libraries by name.
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

// Loads a solver's shared library as a `dylib::library`, resolving it with
// the precedence shared by every `<solver>_api` backend (first match wins):
//
//   1. `path`, if non-null: the exact library file, used verbatim;
//   2. the `MIPPP_<key>_LIBRARY` env var (e.g. MIPPP_HIGHS_LIBRARY): idem,
//      letting versioned sonames like libhighs.so.1.10.0 be pinned;
//   3. each undecorated name of `names` ("highs" -> libhighs.so), in order,
//      searched across the loader's directories — dylib 3.0 resolves paths
//      only, so MIP++ searches itself. Several names cover solvers renamed
//      across releases (Cbc: libCbc / libCbcSolver).
inline dylib::library load_solver_library(
    const char * path, const char * key,
    std::initializer_list<const char *> names,
    std::initializer_list<const char *> probe_symbols = {}) {
    // a candidate must export `probe_symbols`: some distributions ship a
    // matching name without the C API (Ubuntu's libCbc.so vs libCbcSolver.so)
    const auto try_load =
        [probe_symbols](const std::filesystem::path & p,
                        std::string & err) -> std::optional<dylib::library> {
        try {
            dylib::library lib{p};
            for(auto && probe_symbol : probe_symbols)
                lib.get_symbol(probe_symbol);
            return lib;
        } catch(const std::exception & e) {
            if(!err.empty()) err += "\n  ";
            err += p.string();
            err += ": ";
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

    const dylib::decorations decorations = dylib::decorations::os_default();
    const auto directories = detail::system_library_dirs();
    for(const char * n : names) {
        const std::string base =
            detail::concat_str(decorations.prefix, n);  // libfoo
        const std::string decorated =
            detail::concat_str(base, decorations.suffix);  // libfoo.so
        for(const auto & directory : directories)
            if(auto found = detail::find_library_in_dir(
                   directory, base, decorated, decorations.suffix))
                if(auto lib = try_load(*found, errors)) return std::move(*lib);
    }

    std::string tried;
    for(const char * n : names) {
        if(!tried.empty()) tried += "', '";
        tried += std::string(decorations.prefix) + n + decorations.suffix;
    }
    throw std::runtime_error(
        "mippp: could not locate a usable " + std::string(key) +
        " solver library (tried '" + tried + "')." +
        (errors.empty() ? std::string{}
                        : "\nCandidates rejected:\n  " + errors) +
        "\nSet the environment variable " + env_var +
        " to its full path, or add its directory to LD_LIBRARY_PATH.");
}

// Warns on stderr when the loaded library's version differs from the one the
// wrapper was written against — usually harmless (the C APIs are stable) but
// worth knowing when behavior differs. `wrapped` must be a dotted-component
// prefix of `loaded`. Set MIPPP_NO_VERSION_WARNING to silence.
inline void warn_on_version_mismatch(const char * key, const char * wrapped,
                                     const char * loaded) {
    if(loaded == nullptr || std::getenv("MIPPP_NO_VERSION_WARNING") != nullptr)
        return;
    const auto prefix_match = [](const char * a, const char * b) {
        while(*a != '\0' && *a == *b) ++a, ++b;
        return *a == '\0' && (*b == '\0' || *b == '.');
    };
    if(prefix_match(wrapped, loaded)) return;
    std::fprintf(stderr,
                 "mippp: warning: the %s wrapper targets version %s but the "
                 "loaded library reports %s; behavior may differ. Set "
                 "MIPPP_%s_LIBRARY to a matching library, or set "
                 "MIPPP_NO_VERSION_WARNING to silence this warning.\n",
                 key, wrapped, loaded, key);
}

// Idem, comparing major versions only, for wrappers that work across minor
// releases (e.g. Gurobi, whose version is reported numerically).
inline void warn_on_version_mismatch(const char * key, int wrapped_major,
                                     int loaded_major) {
    if(wrapped_major == loaded_major ||
       std::getenv("MIPPP_NO_VERSION_WARNING") != nullptr)
        return;
    std::fprintf(stderr,
                 "mippp: warning: the %s wrapper targets major version %d but "
                 "the loaded library reports %d; behavior may differ. Set "
                 "MIPPP_%s_LIBRARY to a matching library, or set "
                 "MIPPP_NO_VERSION_WARNING to silence this warning.\n",
                 key, wrapped_major, loaded_major, key);
}

}  // namespace mippp::detail
