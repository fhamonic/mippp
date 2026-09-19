#pragma once
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <string_view>

namespace mippp::iis::detail {
// Only called when reporting a result or error, never in a successful trial.
template <typename T>
std::string setting_value(const T & value) {
    std::ostringstream out;
    out << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
    return out.str();
}
inline std::string setting_help(std::string_view name, std::string_view current,
                                std::string_view available) {
    return std::string(name) + ": current=" + std::string(current) +
           "; available=" + std::string(available) + ".";
}
} // namespace mippp::iis::detail
