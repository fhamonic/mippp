#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#define MIPPP_UNDEF_WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#define MIPPP_UNDEF_NOMINMAX
#endif
#include <windows.h>
#ifdef MIPPP_UNDEF_WIN32_LEAN_AND_MEAN
#undef WIN32_LEAN_AND_MEAN
#undef MIPPP_UNDEF_WIN32_LEAN_AND_MEAN
#endif
#ifdef MIPPP_UNDEF_NOMINMAX
#undef NOMINMAX
#undef MIPPP_UNDEF_NOMINMAX
#endif
#else
#include <dlfcn.h>
#endif

namespace mippp::detail {

// Thrown when a shared library file cannot be opened.
class library_load_error : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// Thrown when a loaded library does not export a requested symbol.
class symbol_not_found : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// Owning handle over a shared library opened at runtime (dlopen / LoadLibrary).
// Opens exact file paths only: locating a library by name is the job of
// load_solver_library, which reproduces the loader's search itself.
//
// The library stays mapped for the life of the process even after the handle
// is destroyed. Unmapping a solver is never safe: HiGHS detaches its worker
// threads when a model is destroyed, so an unload racing their exit crashed
// roughly a third of the test processes under load, and the other solvers
// keep thread pools and thread-local state of their own.
class dynamic_library {
public:
#if defined(_WIN32)
    using native_handle_type = HMODULE;
    using native_symbol_type = FARPROC;
    static constexpr std::string_view prefix = "";
    static constexpr std::string_view suffix = ".dll";
#else
    using native_handle_type = void *;
    using native_symbol_type = void *;
    static constexpr std::string_view prefix = "lib";
#if defined(__APPLE__)
    static constexpr std::string_view suffix = ".dylib";
#else
    static constexpr std::string_view suffix = ".so";
#endif
#endif

private:
    native_handle_type _handle = nullptr;
    std::filesystem::path _path;

    static native_handle_type open(
        const std::filesystem::path & path) noexcept {
#if defined(_WIN32)
        // with an absolute path, the library's own directory is searched
        // first for its dependencies, which solver SDKs ship next to it
        native_handle_type handle = LoadLibraryExW(
            path.c_str(), nullptr,
            path.is_absolute() ? LOAD_WITH_ALTERED_SEARCH_PATH : 0);
        if(handle != nullptr) {
            // pinned: FreeLibrary keeps the reference count but never unmaps
            HMODULE pinned;
            GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_PIN |
                                   GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS,
                               reinterpret_cast<LPCWSTR>(handle), &pinned);
        }
        return handle;
#else
        int flags = RTLD_NOW | RTLD_LOCAL;
#ifdef RTLD_NODELETE
        flags |= RTLD_NODELETE;
#endif
        return dlopen(path.c_str(), flags);
#endif
    }
    static void close(native_handle_type handle) noexcept {
#if defined(_WIN32)
        FreeLibrary(handle);
#else
        dlclose(handle);
#endif
    }
    static std::string last_error() {
#if defined(_WIN32)
        const DWORD code = GetLastError();
        if(code == 0) return "unknown error";
        char buffer[512];
        const DWORD length = FormatMessageA(
            FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, nullptr,
            code, MAKELANGID(LANG_ENGLISH, SUBLANG_ENGLISH_US), buffer,
            sizeof(buffer), nullptr);
        std::string message(buffer, length);
        while(!message.empty() &&
              (message.back() == '\n' || message.back() == '\r' ||
               message.back() == ' '))
            message.pop_back();
        return message.empty() ? "error " + std::to_string(code) : message;
#else
        const char * message = dlerror();
        return message == nullptr ? "unknown error" : message;
#endif
    }

public:
    // Throws library_load_error if `path` cannot be opened.
    explicit dynamic_library(const std::filesystem::path & path)
        : _handle(open(path)), _path(path) {
        if(_handle == nullptr)
            throw library_load_error("cannot load '" + _path.string() +
                                     "': " + last_error());
    }
    ~dynamic_library() {
        if(_handle != nullptr) close(_handle);
    }

    dynamic_library(const dynamic_library &) = delete;
    dynamic_library & operator=(const dynamic_library &) = delete;
    dynamic_library(dynamic_library && other) noexcept
        : _handle(std::exchange(other._handle, nullptr))
        , _path(std::move(other._path)) {}
    dynamic_library & operator=(dynamic_library && other) noexcept {
        if(this != &other) {
            if(_handle != nullptr) close(_handle);
            _handle = std::exchange(other._handle, nullptr);
            _path = std::move(other._path);
        }
        return *this;
    }

    const std::filesystem::path & path() const noexcept { return _path; }
    native_handle_type native_handle() const noexcept { return _handle; }

    // nullptr when the library does not export `name`.
    native_symbol_type find_symbol(const char * name) const noexcept {
#if defined(_WIN32)
        return GetProcAddress(_handle, name);
#else
        return dlsym(_handle, name);
#endif
    }
    // Throws symbol_not_found when the library does not export `name`.
    native_symbol_type get_symbol(const char * name) const {
        native_symbol_type symbol = find_symbol(name);
        if(symbol == nullptr)
            throw symbol_not_found("symbol '" + std::string(name) +
                                   "' not found in '" + _path.string() +
                                   "': " + last_error());
        return symbol;
    }

    // `F` is the function type (not pointer) of the symbol, as in
    // `find_function<decltype(Cbc_solve)>("Cbc_solve")`.
    template <typename F>
    F * find_function(const char * name) const noexcept {
        return cast_function<F>(find_symbol(name));
    }
    template <typename F>
    F * get_function(const char * name) const {
        return cast_function<F>(get_symbol(name));
    }

private:
    template <typename F>
    static F * cast_function(native_symbol_type symbol) noexcept {
        // object pointer <-> function pointer conversion is conditionally
        // supported; every platform with dlsym/GetProcAddress supports it
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-function-type"
#elif defined(__clang__)
#pragma clang diagnostic push
#if __has_warning("-Wcast-function-type-mismatch")
#pragma clang diagnostic ignored "-Wcast-function-type-mismatch"
#endif
#if __has_warning("-Wcast-function-type")
#pragma clang diagnostic ignored "-Wcast-function-type"
#endif
#endif
        return reinterpret_cast<F *>(symbol);
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#elif defined(__clang__)
#pragma clang diagnostic pop
#endif
    }
};

}  // namespace mippp::detail
