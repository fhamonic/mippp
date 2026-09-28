#pragma once

namespace mippp::detail {

// Binds to the owner's handles before they are allocated: a constructor body
// that throws runs no destructor, and would otherwise leak what it allocated.
// Release provides static free_problem(api, env, prob) and free_env(api, env),
// generic over the api so that a test can drive the guard with a fake one.
template <typename Api, typename Env, typename Prob, typename Release>
class handle_guard {
    const Api & _api;
    Env & _env;
    Prob & _prob;

public:
    handle_guard(const Api & api, Env & env, Prob & prob) noexcept
        : _api(api), _env(env), _prob(prob) {}
    handle_guard(const handle_guard &) = delete;
    handle_guard & operator=(const handle_guard &) = delete;

    // A failed release must neither throw nor skip the environment, which
    // outlives the problem because CPXfreeprob still needs it.
    ~handle_guard() {
        if(_prob) static_cast<void>(Release::free_problem(_api, _env, _prob));
        if(_env) static_cast<void>(Release::free_env(_api, _env));
        _prob = nullptr;
        _env = nullptr;
    }
};

}  // namespace mippp::detail
