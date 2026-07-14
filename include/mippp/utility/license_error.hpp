#ifndef MIPPP_LICENSE_ERROR_HPP
#define MIPPP_LICENSE_ERROR_HPP

#include <stdexcept>

class license_error : public std::runtime_error {
public:
    license_error(char const * const message) : std::runtime_error(message) {}
};

#endif  // MIPPP_LICENSE_ERROR_HPP