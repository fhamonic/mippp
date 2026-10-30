#pragma once

#include <stdexcept>

namespace mippp {

class solver_error : public std::runtime_error {
public:
    solver_error(char const * const message) : std::runtime_error(message) {}
};

class license_error : public solver_error {
public:
    license_error(char const * const message) : solver_error(message) {}
};

// A member the loaded release lacks, as releases a backend supports only
// partially may (see `<solver>_api::supported_versions`); the model itself
// stays usable.
class feature_unavailable_error : public solver_error {
public:
    feature_unavailable_error(char const * const message)
        : solver_error(message) {}
};

// A result getter called with no solve() since the model was created or last
// changed: a misuse, so not a solver_error a caller may catch as a failure.
class solve_required_error : public std::logic_error {
public:
    solve_required_error(char const * const message)
        : std::logic_error(message) {}
};

}  // namespace mippp
