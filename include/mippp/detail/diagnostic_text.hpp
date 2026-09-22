#pragma once
#include <array>
#include <cstdlib>
#include <string>
#include <string_view>

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
namespace mippp::detail {
// Quote values as data, not executable shell commands. Only pass specifically
// allowlisted path/boolean variables here; never dump the process environment
// or license contents. Escape control characters to keep messages readable.
inline std::string diagnostic_value(const char * value) {
    if(!value) return "<not set>";
    std::string out = "\"";
    for(const char ch : std::string_view(value)) {
        const auto c = static_cast<unsigned char>(ch);
        if(c == '\\' || c == '"') {
            out += '\\';
            out += static_cast<char>(c);
        } else if(c == '\n')
            out += "\\n";
        else if(c == '\r')
            out += "\\r";
        else if(c == '\t')
            out += "\\t";
        else if(c < 32 || c == 127) {
            constexpr char hex[] = "0123456789abcdef";
            out += "\\x";
            out += hex[c / 16];
            out += hex[c % 16];
        } else
            out += static_cast<char>(c);
    }
    return out + '"';
}
inline std::string environment_help(const char * name,
                                    std::string_view available) {
    return std::string(name) +
           ": current=" + diagnostic_value(std::getenv(name)) +
           "; available=" + std::string(available) + ".";
}

inline std::string private_environment_help(const char * name,
                                            std::string_view available) {
    const auto * value = std::getenv(name);
    const auto current = !value   ? "<not set>"
                         : *value ? "<set; value hidden>"
                                  : "<empty>";
    return std::string(name) + ": current=" + current +
           "; available=" + std::string(available) + ".";
}

// Error-only, vendor-specific guidance stays outside the generic algorithms.
// Sources/semantics are linked in docs/iis.md. MOSEK's variable can contain
// the ENTIRE license, so even a path-looking value is conservatively hidden.
inline std::string license_diagnostic(std::string_view solver,
                                      const char * details) {
    std::string message =
        std::string(solver) + " could not use a license for this solve.\n";
    const char * secret_variable = nullptr;
    if(solver == "Gurobi") {
        message += environment_help(
            "GRB_LICENSE_FILE",
            "the full path to a readable valid license file (not its "
            "directory), or unset for default discovery");
    } else if(solver == "MOSEK") {
        secret_variable = "MOSEKLM_LICENSE_FILE";
        message += private_environment_help(
            secret_variable,
            "a license-file path, port@server, a platform-separated list of "
            "locations, inline license text, or unset for default discovery");
        message +=
            " This value is hidden because it can contain license secrets.";
    } else if(solver == "CPLEX") {
        secret_variable = "CPLEX_STUDIO_KEY";
        message +=
            private_environment_help(secret_variable,
                                     "an IBM-issued subscription key, or unset "
                                     "when that licensing method is not used");
        message +=
            "\n" + private_environment_help(
                       "CPLEX_STUDIO_KEY_SERVER",
                       "the subscription endpoint supplied by IBM (documented: "
                       "https://scx-cos.docloud.ibm.com/cos/query/v1/apikeys), "
                       "or unset when not used");
        message +=
            " The endpoint value is hidden because URLs can include "
            "credentials.";
        message +=
            " Subscription settings require the corresponding entitlement and "
            "server access; "
            "they do not grant a license or remove limits by themselves.";
    }
    message +=
        "\nSet these before starting the application; a loaded license system "
        "may cache earlier settings. "
        "Check the license's validity and any size restrictions. No license "
        "file has been read by this diagnostic.";
    const std::string_view original_details =
        details ? details : "no additional details";
    const std::array names{secret_variable, solver == "CPLEX"
                                                ? "CPLEX_STUDIO_KEY_SERVER"
                                                : nullptr};
    std::array<std::string_view, 2> secrets{};
    for(std::size_t i = 0; i < names.size(); ++i) {
        if(names[i]) {
            if(const auto * value = std::getenv(names[i])) secrets[i] = value;
        }
    }
    // Match against the original text, longest match first. Replacing a key
    // inside an endpoint must not prevent that entire endpoint being hidden.
    std::string safe_details;
    for(std::size_t pos = 0; pos < original_details.size();) {
        std::size_t matched = 0;
        for(auto secret : secrets)
            if(secret.size() > matched &&
               original_details.substr(pos).starts_with(secret))
                matched = secret.size();
        if(matched) {
            safe_details += "<redacted>";
            pos += matched;
        } else
            safe_details += original_details[pos++];
    }
    return message + "\nSolver details: " + safe_details;
}
}  // namespace mippp::detail
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
