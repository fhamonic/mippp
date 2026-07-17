#ifndef MIPPP_SOLVER_EXCEPTIONS_HPP
#define MIPPP_SOLVER_EXCEPTIONS_HPP

#include <stdexcept>

class solver_error : public std::runtime_error {
public:
    solver_error(char const * const message) : std::runtime_error(message) {}
};

class license_error : public solver_error {
public:
    license_error(char const * const message) : solver_error(message) {}
};

#endif  // MIPPP_SOLVER_EXCEPTIONS_HPP