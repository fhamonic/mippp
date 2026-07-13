#ifndef MIPPP_LICENSE_ERROR_HPP
#define MIPPP_LICENSE_ERROR_HPP

#include <stdexcept>

class license_error : public std::runtime_error {
public:
    license_error(char const * const message) throw()
        : std::runtime_error(message) {}
    virtual char const * what() const throw() { return exception::what(); }
};

#endif  // MIPPP_LICENSE_ERROR_HPP