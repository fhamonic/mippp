#include <gtest/gtest.h>

#include <charconv>
#include <optional>
#include <string_view>
#include <system_error>

namespace {

// ctest runs one test suite per process and reads the exit code as the
// verdict of the whole suite. A suite whose every test skipped, as when its
// solver cannot be loaded or licensed, must read as skipped rather than
// passed, and a failure next to skips must still read as a failure: a
// dedicated exit code does both, where a regular expression over the output
// would also match a suite that failed.
constexpr std::string_view skip_exit_code_flag = "--mippp_skip_exit_code=";

std::optional<int> skip_exit_code(int argc, char ** argv) {
    for(int i = 1; i < argc; ++i) {
        const std::string_view arg(argv[i]);
        if(!arg.starts_with(skip_exit_code_flag)) continue;
        const std::string_view value = arg.substr(skip_exit_code_flag.size());
        int code = 0;
        const auto [end, error] =
            std::from_chars(value.data(), value.data() + value.size(), code);
        if(error == std::errc{} && end == value.data() + value.size())
            return code;
    }
    return std::nullopt;
}

}  // namespace

int main(int argc, char ** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    const std::optional<int> all_skipped_code = skip_exit_code(argc, argv);
    const int status = RUN_ALL_TESTS();
    const ::testing::UnitTest & unit = *::testing::UnitTest::GetInstance();
    if(status == 0 && all_skipped_code && unit.successful_test_count() == 0 &&
       unit.skipped_test_count() > 0)
        return *all_skipped_code;
    return status;
}
