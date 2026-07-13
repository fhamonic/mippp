#ifndef MIPPP_SOLVER_LIBRARY_HPP
#define MIPPP_SOLVER_LIBRARY_HPP

#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include "dylib.hpp"

namespace fhamonic::mippp {

namespace detail {

// Separator between entries of a path-list environment variable: ';' on Windows
// (where ':' also follows drive letters), ':' on POSIX systems.
#if defined(_WIN32)
inline constexpr char path_list_separator = ';';
#else
inline constexpr char path_list_separator = ':';
#endif

// Get the environment variable `env_name` (e.g. LD_LIBRARY_PATH), if it exists
// splits its value by `separator` and append the paths to `dirs`.
inline void append_env_dirs(std::vector<std::filesystem::path> & dirs,
                            const char * env_name,
                            char separator = path_list_separator) {
    const char * value = std::getenv(env_name);
    if(value == nullptr) return;
    const std::string paths(value);
    std::size_t start = 0;
    while(start <= paths.size()) {
        const std::size_t pos = paths.find(separator, start);
        const std::size_t end = (pos == std::string::npos) ? paths.size() : pos;
        if(end > start) dirs.emplace_back(paths.substr(start, end - start));
        if(pos == std::string::npos) break;
        start = pos + 1;
    }
}

// Appends the directory entries listed in an ld.so.conf-style file to `dirs`.
// Blank lines, '#' comments and 'include' directives are ignored (the standard
// include target /etc/ld.so.conf.d is scanned directly below).
inline void append_conf_dirs(std::vector<std::filesystem::path> & dirs,
                             const std::filesystem::path & file) {
    std::ifstream in(file);
    if(!in) return;
    std::string line;
    while(std::getline(in, line)) {
        const std::size_t b = line.find_first_not_of(" \t");
        if(b == std::string::npos || line[b] == '#') continue;
        const std::size_t e = line.find_last_not_of(" \t\r");
        const std::string entry = line.substr(b, e - b + 1);
        if(entry.rfind("include", 0) == 0) continue;
        dirs.emplace_back(entry);
    }
}

// Reproduces the directories the dynamic loader would search, since dylib 3.0
// no longer resolves libraries by name. The exact set is platform-specific:
// the relevant search environment variables plus each platform's conventional
// library directories (and, on Linux, the /etc/ld.so.conf[.d] configuration).
inline std::vector<std::filesystem::path> system_library_dirs() {
    std::vector<std::filesystem::path> dirs;
#if defined(_WIN32)
    // The Windows loader resolves DLLs through PATH (plus the system directory).
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
    return dirs;
}

}  // namespace detail

// Loads a solver's shared library as a `dylib::library`, applying the
// resolution precedence shared by every `<solver>_api` backend (first match
// wins):
//
//   1. the `MIPPP_<key>_LIBRARY` environment variable: used verbatim as the
//      full path to the exact library file (no decoration), so version-suffixed
//      sonames such as `libhighs.so.1.10.0` can be named explicitly;
//   2. an explicit directory passed by the caller (`dir` non-empty): the
//      OS-decorated default name is looked up in that directory;
//   3. otherwise the OS-decorated default name is searched across the dynamic
//      loader's directories (see `detail::system_library_dirs`).
//
// dylib 3.0 requires a path and no longer searches system directories by name,
// so MIP++ performs that search itself in case 3. Set `MIPPP_<key>_LIBRARY`
// to load a specific file deterministically.
//
//   key          uppercase solver identifier used in the env var, e.g. "HIGHS"
//   default_name undecorated library name, e.g. "highs" -> libhighs.so
//   dir          optional directory explicitly requested by the caller
//   name         optional library name overriding `default_name`
inline dylib::library load_solver_library(const char * key,
                                          const char * default_name,
                                          const char * dir = "",
                                          const char * name = nullptr) {
    const char * lib_name =
        (name != nullptr && *name != '\0') ? name : default_name;

    const std::string env_var = std::string("MIPPP_") + key + "_LIBRARY";
    if(const char * full_path = std::getenv(env_var.c_str());
       full_path != nullptr && *full_path != '\0')
        return dylib::library(std::filesystem::path(full_path));

    const dylib::decorations decorations = dylib::decorations::os_default();
    const std::string filename =
        std::string(decorations.prefix) + lib_name + decorations.suffix;

    if(dir != nullptr && *dir != '\0')
        return dylib::library(std::filesystem::path(dir) / filename);

    std::error_code ec;
    for(const auto & directory : detail::system_library_dirs()) {
        const std::filesystem::path candidate = directory / filename;
        if(std::filesystem::exists(candidate, ec))
            return dylib::library(candidate);
    }

    throw std::runtime_error(
        "mippp: could not locate the " + std::string(key) +
        " solver library '" + filename + "'. Set the environment variable " +
        env_var +
        " to its full path, or add its directory to LD_LIBRARY_PATH.");
}

}  // namespace fhamonic::mippp

#endif  // MIPPP_SOLVER_LIBRARY_HPP
