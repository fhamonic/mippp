#pragma once

#include "mippp/solvers/mosek/impl/v1/mosek_api.hpp"

namespace mippp::mosek::impl::v1::resource_detail {

// Watch the owner's handles before allocation starts, so a failed constructor
// still releases any resources already acquired. The Api parameter also lets
// tests supply failing allocation/cleanup calls without loading MOSEK.
template <typename Api>
class mosek_handle_guard {
    const Api & api_;
    MSKenv_t & env_;
    MSKtask_t & task_;

public:
    mosek_handle_guard(const Api & api, MSKenv_t & env,
                       MSKtask_t & task) noexcept
        : api_(api), env_(env), task_(task) {}
    mosek_handle_guard(const mosek_handle_guard &) = delete;
    mosek_handle_guard & operator=(const mosek_handle_guard &) = delete;
    ~mosek_handle_guard() noexcept {
        // C cleanup error codes cannot be thrown from a destructor. Attempt
        // both releases, in task-before-environment order, even if one fails.
        if(task_) (void)api_.deletetask(&task_);
        if(env_) (void)api_.deleteenv(&env_);
        task_ = nullptr;
        env_ = nullptr;
    }
};
}  // namespace mippp::mosek::impl::v1::resource_detail
