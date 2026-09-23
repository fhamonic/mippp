#pragma once

#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <optional>
#include <stdexcept>
#include <string>

#if defined(_WIN32)
#include <io.h>
#else
#include <unistd.h>
#endif

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"

namespace mippp {

struct process_output {
    std::string out;
    std::string err;
};

// Redirects a file descriptor rather than std::cout: solver libraries write
// through their own stdio calls, which a stream redirection would miss.
class fd_capture {
#if defined(_WIN32)
    static int duplicate(int fd) { return ::_dup(fd); }
    static int redirect(int from, int to) { return ::_dup2(from, to); }
    static int close_fd(int fd) { return ::_close(fd); }
    static int fd_of(std::FILE * file) { return ::_fileno(file); }
#else
    static int duplicate(int fd) { return ::dup(fd); }
    static int redirect(int from, int to) { return ::dup2(from, to); }
    static int close_fd(int fd) { return ::close(fd); }
    static int fd_of(std::FILE * file) { return ::fileno(file); }
#endif
    int _fd;
    int _saved;
    std::FILE * _file;

public:
    explicit fd_capture(int fd)
        : _fd(fd), _saved(duplicate(fd)), _file(nullptr) {
        if(_saved < 0) throw std::runtime_error("fd_capture: dup failed");
        _file = std::tmpfile();
        if(_file == nullptr || redirect(fd_of(_file), fd) < 0) {
            if(_file != nullptr) std::fclose(_file);
            close_fd(_saved);
            throw std::runtime_error("fd_capture: cannot redirect");
        }
    }
    ~fd_capture() {
        std::fflush(nullptr);
        redirect(_saved, _fd);
        close_fd(_saved);
        std::fclose(_file);
    }
    fd_capture(const fd_capture &) = delete;
    fd_capture & operator=(const fd_capture &) = delete;

    std::string contents() {
        std::fflush(nullptr);
        std::string text;
        std::rewind(_file);
        char buffer[4096];
        for(std::size_t n; (n = std::fread(buffer, 1, sizeof buffer, _file));)
            text.append(buffer, n);
        return text;
    }
};

template <typename F>
process_output output_of(F && f) {
    std::fflush(nullptr);  // or what gtest buffered lands in the capture
    fd_capture out(1);
    fd_capture err(2);
    f();
    return {out.contents(), err.contents()};
}

inline std::string describe(const process_output & output) {
    return "stdout:\n" + output.out + "\nstderr:\n" + output.err;
}

template <typename T>
struct VerbosityTest : public T {
    using typename T::model_type;
    static_assert(has_verbosity<model_type>);

    // Binaries under a knapsack row, so that MILP solvers run their cut
    // setup: GLPK prints that part whatever its message level. The explicit
    // zero coefficient makes MOSEK warn.
    static void build(model_type & model) {
        using namespace operators;
        auto x = [&model] {
            if constexpr(milp_model<model_type>)
                return model.add_binary_variables(3);
            else
                return model.add_variables(3, {.upper_bound = 1});
        }();
        model.set_maximization();
        model.set_objective(5 * x[0] + 4 * x[1] + 3 * x[2]);
        model.add_constraint(2 * x[0] + 3 * x[1] + x[2] <= 4);
        model.add_constraint(0.0 * x[0] + x[1] <= 1);
    }
    // Gurobi and COPT echo parameter changes when they log.
    static void set_parameters(model_type & model) {
        if constexpr(has_time_limit<model_type>)
            model.set_time_limit(std::chrono::seconds(60));
        if constexpr(has_optimality_tolerance<model_type>)
            model.set_optimality_tolerance(1e-3);
        if constexpr(has_feasibility_tolerance<model_type>)
            model.set_feasibility_tolerance(1e-7);
    }
};
TYPED_TEST_SUITE_P(VerbosityTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(VerbosityTest);

TYPED_TEST_P(VerbosityTest, set_get_verbose) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        ASSERT_FALSE(model.is_verbose());
        model.set_verbose(false);  // already quiet: a no-op, not an error
        ASSERT_FALSE(model.is_verbose());
        output_of([&] { model.set_verbose(true); });
        ASSERT_TRUE(model.is_verbose());
        model.set_verbose(false);
        ASSERT_FALSE(model.is_verbose());
    });
}

TYPED_TEST_P(VerbosityTest, quiet_by_default) {
    using model_type = typename TestFixture::model_type;
    this->SkipOnLicenseError([this]() {
        std::optional<model_type> model;
        const auto output = output_of([&] {
            model.emplace(this->new_model());
            TestFixture::set_parameters(*model);
            TestFixture::build(*model);
            model->solve();
        });
        ASSERT_TRUE(output.out.empty() && output.err.empty())
            << describe(output);
        ASSERT_TRUE(status::solution_available(model->get_status()));
    });
}

// Also proves that the capture sees what this solver prints, without which
// quiet_by_default would pass vacuously.
TYPED_TEST_P(VerbosityTest, verbose_prints_the_solve_log) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        output_of([&] {
            model.set_verbose(true);
            TestFixture::build(model);
        });
        const auto output = output_of([&] { model.solve(); });
        ASSERT_FALSE(output.out.empty()) << describe(output);
    });
}

TYPED_TEST_P(VerbosityTest, quiet_again_after_verbose) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        output_of([&] { model.set_verbose(true); });
        const auto output = output_of([&] {
            model.set_verbose(false);
            TestFixture::set_parameters(model);
            TestFixture::build(model);
            model.solve();
        });
        ASSERT_TRUE(output.out.empty() && output.err.empty())
            << describe(output);
    });
}

REGISTER_TYPED_TEST_SUITE_P(VerbosityTest, set_get_verbose, quiet_by_default,
                            verbose_prints_the_solve_log,
                            quiet_again_after_verbose);

}  // namespace mippp
