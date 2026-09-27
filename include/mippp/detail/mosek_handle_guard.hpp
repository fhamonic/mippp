#pragma once

namespace mippp::detail {

// Binds to the owner's handles before they are allocated: a constructor body
// that throws runs no destructor, and would otherwise leak what it allocated.
// The Api parameter lets a test drive it with a fake api.
template <typename Api, typename Env, typename Task>
class mosek_handle_guard {
    const Api & _api;
    Env & _env;
    Task & _task;

public:
    mosek_handle_guard(const Api & api, Env & env, Task & task) noexcept
        : _api(api), _env(env), _task(task) {}
    mosek_handle_guard(const mosek_handle_guard &) = delete;
    mosek_handle_guard & operator=(const mosek_handle_guard &) = delete;

    // A failed release must neither throw nor skip the environment.
    ~mosek_handle_guard() {
        if(_task) static_cast<void>(_api.deletetask(&_task));
        if(_env) static_cast<void>(_api.deleteenv(&_env));
        _task = nullptr;
        _env = nullptr;
    }
};

}  // namespace mippp::detail
