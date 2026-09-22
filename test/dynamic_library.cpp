#include <gtest/gtest.h>

#include <array>
#include <cstdlib>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "mippp/detail/dynamic_library.hpp"
#include "mippp/detail/solver_library.hpp"

using namespace mippp::detail;
using mippp::is_validated;
using mippp::solver_version;
using mippp::solver_version_range;
using mippp::to_string;

// built by test/CMakeLists.txt from dynamic_library_fixture.cpp; the second
// is the same file name in another directory, answering 43 instead of 42
static const std::filesystem::path fixture_path = MIPPP_TEST_LIBRARY;
static const std::filesystem::path fixture_path_b = MIPPP_TEST_LIBRARY_B;

#if defined(_WIN32)
static const char * loader_path_var = "PATH";
#elif defined(__APPLE__)
static const char * loader_path_var = "DYLD_LIBRARY_PATH";
#else
static const char * loader_path_var = "LD_LIBRARY_PATH";
#endif

// `dirs` prepended to the loader's search path, as backends see it
static std::string loader_path_with(
    std::initializer_list<std::filesystem::path> dirs) {
    std::string value;
    for(const auto & dir : dirs) {
        if(!value.empty()) value += path_list_separator;
        value += dir.string();
    }
    if(const char * previous = std::getenv(loader_path_var))
        value += std::string(1, path_list_separator) + previous;
    return value;
}

// the undecorated name, as backends pass it ("highs" -> libhighs.so)
static std::string fixture_name() {
    return fixture_path.stem().string().substr(dynamic_library::prefix.size());
}

static bool same_file(const std::filesystem::path & a,
                      const std::filesystem::path & b) {
    return std::filesystem::equivalent(a, b);
}

// Sets `name` for the test's duration and restores the previous value after.
class scoped_env {
private:
    std::string _name;
    std::optional<std::string> _previous;

    static void set(const char * name, const char * value) {
#if defined(_WIN32)
        _putenv_s(name, value == nullptr ? "" : value);
#else
        if(value == nullptr)
            unsetenv(name);
        else
            setenv(name, value, 1);
#endif
    }

public:
    scoped_env(const char * name, const std::string & value) : _name(name) {
        if(const char * p = std::getenv(name)) _previous = p;
        set(name, value.c_str());
    }
    // unset for the scope
    explicit scoped_env(const char * name) : _name(name) {
        if(const char * p = std::getenv(name)) _previous = p;
        set(name, nullptr);
    }
    ~scoped_env() {
        set(_name.c_str(), _previous ? _previous->c_str() : nullptr);
    }
};

TEST(dynamic_library, missing_file_throws) {
    const std::filesystem::path missing =
        fixture_path.parent_path() / "no_such_library_mippp.so";
    EXPECT_THROW(dynamic_library{missing}, library_load_error);
    try {
        dynamic_library lib{missing};
        FAIL();
    } catch(const library_load_error & e) {
        EXPECT_NE(std::string(e.what()).find(missing.string()),
                  std::string::npos);
    }
}

TEST(dynamic_library, resolves_functions) {
    dynamic_library lib(fixture_path);
    EXPECT_NE(lib.native_handle(), nullptr);
    EXPECT_EQ(lib.path(), fixture_path);

    auto answer = lib.get_function<int()>("mippp_test_answer");
    EXPECT_EQ(answer(), 42);
    auto add = lib.find_function<int(int, int)>("mippp_test_add");
    ASSERT_NE(add, nullptr);
    EXPECT_EQ(add(2, 3), 5);

    EXPECT_EQ(lib.find_symbol("mippp_no_such_symbol"), nullptr);
    EXPECT_EQ(lib.find_function<int()>("mippp_no_such_symbol"), nullptr);
    try {
        lib.get_symbol("mippp_no_such_symbol");
        FAIL();
    } catch(const symbol_not_found & e) {
        const std::string what = e.what();
        EXPECT_NE(what.find("mippp_no_such_symbol"), std::string::npos);
        EXPECT_NE(what.find(fixture_path.string()), std::string::npos);
    }
}

TEST(dynamic_library, move_transfers_ownership) {
    dynamic_library a(fixture_path);
    dynamic_library b(std::move(a));
    EXPECT_EQ(a.native_handle(), nullptr);
    EXPECT_EQ(b.get_function<int()>("mippp_test_answer")(), 42);

    dynamic_library c(fixture_path);
    c = std::move(b);
    EXPECT_EQ(b.native_handle(), nullptr);
    EXPECT_EQ(c.get_function<int()>("mippp_test_answer")(), 42);
}

TEST(dynamic_library, stays_mapped_after_destruction) {
    void * before;
    {
        dynamic_library lib(fixture_path);
        before = reinterpret_cast<void *>(
            lib.get_function<int()>("mippp_test_answer"));
    }
    // solver worker threads may still run after the handle is gone: the code
    // must stay mapped (RTLD_NODELETE / pinned module). Calling it is the
    // check that holds everywhere; the loader-side probes below are extra
    // where the loader documents them. Not on Apple: dyld keeps a
    // RTLD_NODELETE image mapped but stops reporting it to
    // dlopen(RTLD_NOLOAD) once every handle is closed.
#if defined(_WIN32)
    HMODULE still_loaded = GetModuleHandleW(fixture_path.c_str());
    ASSERT_NE(still_loaded, nullptr);
    EXPECT_EQ(reinterpret_cast<void *>(
                  GetProcAddress(still_loaded, "mippp_test_answer")),
              before);
#elif !defined(__APPLE__)
    void * still_loaded =
        dlopen(fixture_path.c_str(), RTLD_NOW | RTLD_LOCAL | RTLD_NOLOAD);
    ASSERT_NE(still_loaded, nullptr);
    EXPECT_EQ(dlsym(still_loaded, "mippp_test_answer"), before);
    dlclose(still_loaded);
#endif
    EXPECT_EQ(reinterpret_cast<int (*)()>(before)(), 42);
}

TEST(load_solver_library, explicit_path_wins) {
    dynamic_library lib = load_solver_library(fixture_path, "TESTLIB",
                                              std::array{"mippp_test_answer"});
    EXPECT_EQ(lib.get_function<int()>("mippp_test_answer")(), 42);
}

TEST(load_solver_library, explicit_path_rejected_without_probe_symbol) {
    try {
        load_solver_library(fixture_path, "TESTLIB",
                            std::array{"mippp_no_such_symbol"});
        FAIL();
    } catch(const std::runtime_error & e) {
        const std::string what = e.what();
        EXPECT_NE(what.find("TESTLIB"), std::string::npos);
        EXPECT_NE(what.find("mippp_no_such_symbol"), std::string::npos);
    }
}

TEST(load_solver_library, environment_variable_pins_the_file) {
    scoped_env env("MIPPP_TESTLIB_LIBRARY", fixture_path.string());
    dynamic_library lib =
        find_solver_library("TESTLIB", std::array{"mippp_no_such_name"},
                            std::array{"mippp_test_answer"});
    EXPECT_EQ(lib.path(), fixture_path);
}

TEST(load_solver_library, environment_variable_failure_names_it) {
    scoped_env env("MIPPP_TESTLIB_LIBRARY",
                   (fixture_path.parent_path() / "missing.so").string());
    try {
        find_solver_library("TESTLIB", std::array{"mippp_no_such_name"});
        FAIL();
    } catch(const std::runtime_error & e) {
        EXPECT_NE(std::string(e.what()).find("MIPPP_TESTLIB_LIBRARY"),
                  std::string::npos);
    }
}

TEST(load_solver_library,
     errors_show_current_values_available_values_and_precedence) {
    const std::string missing =
        (fixture_path.parent_path() / "missing diagnostic library.so").string();
    scoped_env env("MIPPP_TESTDIAGNOSTIC_LIBRARY", missing);
    scoped_env search(loader_path_var, "first diagnostic directory");
    try {
        find_solver_library("TESTDIAGNOSTIC", std::array{"mippp_no_such_name"});
        FAIL();
    } catch(const std::runtime_error & e) {
        const std::string text = e.what();
        EXPECT_NE(text.find("MIPPP_TESTDIAGNOSTIC_LIBRARY: current=" +
                            diagnostic_value(missing.c_str())),
                  std::string::npos);
        EXPECT_NE(text.find(std::string(loader_path_var) +
                            ": current=\"first diagnostic directory\""),
                  std::string::npos);
        EXPECT_NE(text.find("available=a full path"), std::string::npos);
        EXPECT_NE(text.find("takes priority"), std::string::npos);
        EXPECT_NE(text.find("restart"), std::string::npos);
#if defined(_WIN32)
        EXPECT_EQ(text.find("LD_LIBRARY_PATH"), std::string::npos);
        EXPECT_NE(text.find("separated by ';'"), std::string::npos);
#else
        EXPECT_NE(text.find("separated by ':'"), std::string::npos);
#endif
    }
    try {
        load_solver_library("explicit missing file", "TESTDIAGNOSTIC", {});
        FAIL();
    } catch(const std::runtime_error & e) {
        EXPECT_NE(std::string(e.what()).find(
                      "library path argument: current=\"explicit missing "
                      "file\"; available="),
                  std::string::npos);
    }
}

TEST(load_solver_library,
     diagnostic_values_are_quoted_and_warnings_explain_boolean_semantics) {
    EXPECT_EQ(diagnostic_value(nullptr), "<not set>");
    EXPECT_EQ(diagnostic_value(""), "\"\"");
    EXPECT_EQ(diagnostic_value("a\n\"b"), "\"a\\n\\\"b\"");
    scoped_env env("MIPPP_TESTWARNING_LIBRARY", "chosen library");
    scoped_env suppression("MIPPP_NO_VERSION_WARNING", "0");
    const auto text = version_warning_help("TESTWARNING");
    EXPECT_NE(text.find("MIPPP_TESTWARNING_LIBRARY: current=\"chosen "
                        "library\"; available="),
              std::string::npos);
    EXPECT_NE(text.find("MIPPP_NO_VERSION_WARNING: current=\"0\"; available="),
              std::string::npos);
    EXPECT_NE(text.find("including '0' or empty"), std::string::npos);
    EXPECT_NE(text.find("does not fix a version mismatch"), std::string::npos);
}

TEST(load_solver_library,
     license_help_lists_settings_without_exposing_keys_or_embedded_licenses) {
    scoped_env gurobi("GRB_LICENSE_FILE", "/test/licenses/gurobi.lic");
    scoped_env cplex("CPLEX_STUDIO_KEY", "private-subscription-key");
    scoped_env mosek("MOSEKLM_LICENSE_FILE", "private-inline-license");
    const auto g = license_diagnostic("Gurobi", "license unavailable");
    EXPECT_NE(g.find("GRB_LICENSE_FILE: current=\"/test/licenses/gurobi.lic\"; "
                     "available="),
              std::string::npos);
    const auto c =
        license_diagnostic("CPLEX", "rejected private-subscription-key");
    EXPECT_NE(
        c.find("CPLEX_STUDIO_KEY: current=<set; value hidden>; available="),
        std::string::npos);
    EXPECT_EQ(c.find("private-subscription-key"), std::string::npos);
    const auto m =
        license_diagnostic("MOSEK", "rejected private-inline-license");
    EXPECT_NE(
        m.find("MOSEKLM_LICENSE_FILE: current=<set; value hidden>; available="),
        std::string::npos);
    EXPECT_EQ(m.find("private-inline-license"), std::string::npos);
}

TEST(load_solver_library, license_help_redacts_endpoints_containing_the_key) {
    const std::string key = "private-subscription-key";
    const std::string endpoint =
        "https://license.invalid/" + key + "/private-account";
    scoped_env subscription("CPLEX_STUDIO_KEY", key);
    scoped_env server("CPLEX_STUDIO_KEY_SERVER", endpoint);
    const auto text = license_diagnostic(
        "CPLEX", ("rejected " + endpoint + " and " + key).c_str());
    EXPECT_EQ(text.find(key), std::string::npos);
    EXPECT_EQ(text.find("license.invalid"), std::string::npos);
    EXPECT_EQ(text.find("private-account"), std::string::npos);
    EXPECT_NE(text.find("Solver details: rejected <redacted> and <redacted>"),
              std::string::npos);
}

TEST(load_solver_library, name_search_over_loader_directories) {
    scoped_env env(loader_path_var,
                   loader_path_with({fixture_path.parent_path()}));
    const std::string name = fixture_name();
    dynamic_library lib = find_solver_library(
        "TESTSEARCH", std::array{"mippp_no_such_name", name.c_str()},
        std::array{"mippp_test_answer"});
    EXPECT_TRUE(same_file(lib.path(), fixture_path));

    try {
        find_solver_library("TESTSEARCH", std::array{"mippp_no_such_name"});
        FAIL();
    } catch(const std::runtime_error & e) {
        const std::string what = e.what();
        EXPECT_NE(what.find("MIPPP_TESTSEARCH_LIBRARY"), std::string::npos);
        EXPECT_NE(what.find("mippp_no_such_name"), std::string::npos);
    }
}

// A copy of fixture B under `name`, in a fresh directory, standing in for a
// newer release of the fixture shipped under its own library name.
struct renamed_fixture {
    std::filesystem::path dir, file;
    explicit renamed_fixture(const std::string & name) {
        dir = std::filesystem::temp_directory_path() / ("mippp_" + name);
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
        file = dir / (std::string(dynamic_library::prefix) + name +
                      std::string(dynamic_library::suffix));
        std::filesystem::copy_file(
            fixture_path_b, file,
            std::filesystem::copy_options::overwrite_existing);
    }
    ~renamed_fixture() {
        std::error_code ec;
        std::filesystem::remove_all(dir, ec);
    }
};

// The loader would pick the first directory holding the library, so the
// search does too: several names (several releases, newest first) only rank
// candidates inside one directory.
TEST(load_solver_library, directory_order_beats_name_order) {
    const renamed_fixture newer("test_newer_release");
    const std::string name = fixture_name();
    const char * names[] = {"test_newer_release", name.c_str()};
    {
        scoped_env env(
            loader_path_var,
            loader_path_with({fixture_path.parent_path(), newer.dir}));
        dynamic_library lib = find_solver_library("TESTORDER", names);
        EXPECT_TRUE(same_file(lib.path(), fixture_path));
        EXPECT_EQ(lib.get_function<int()>("mippp_test_answer")(), 42);
    }
    {
        scoped_env env(
            loader_path_var,
            loader_path_with({newer.dir, fixture_path.parent_path()}));
        dynamic_library lib = find_solver_library("TESTORDER2", names);
        EXPECT_TRUE(same_file(lib.path(), newer.file));
        EXPECT_EQ(lib.get_function<int()>("mippp_test_answer")(), 43);
    }
    {
        // both in one directory: the first name wins
        const std::filesystem::path older = newer.dir / fixture_path.filename();
        std::filesystem::copy_file(fixture_path, older);
        scoped_env env(loader_path_var, loader_path_with({newer.dir}));
        dynamic_library lib = find_solver_library("TESTORDER3", names);
        EXPECT_TRUE(same_file(lib.path(), newer.file));
    }
}

TEST(solver_version, numeric_component_order) {
    using v = solver_version;
    static_assert(v{13} == v{13, 0, 0});
    static_assert(v{2, 10, 9} < v{2, 10, 10});
    static_assert(v{1, 8} < v{1, 10});  // not lexicographic
    static_assert(v{9, 5} < v{10});
    EXPECT_EQ(to_string(v{2, 10, 9}), "2.10.9");
    EXPECT_EQ(to_string(v{1, 8}), "1.8");
    EXPECT_EQ(to_string(v{13}), "13");
    EXPECT_EQ(to_string(v{2, 0, 1}), "2.0.1");
}

TEST(solver_version, parse) {
    using v = solver_version;
    static_assert(parse_solver_version("2.10.12") == v{2, 10, 12});
    static_assert(parse_solver_version("5.0") == v{5});
    static_assert(parse_solver_version("13") == v{13});
    static_assert(parse_solver_version("1.15.1-dev") == v{1, 15, 1});
    static_assert(parse_solver_version("22.1.2.0") == v{22, 1, 2});
    static_assert(parse_solver_version("2.") == v{2});
    static_assert(parse_solver_version("45.01.02") == v{45, 1, 2});
    static_assert(parse_solver_version("2147483647") == v{2147483647});
    static_assert(!parse_solver_version("devel"));
    static_assert(!parse_solver_version(""));
    static_assert(!parse_solver_version("v5.0"));
    static_assert(!parse_solver_version("99999999999"));  // does not fit
}

TEST(solver_version_range, half_open) {
    using v = solver_version;
    constexpr solver_version_range majors{{10}, {14}};
    static_assert(majors.contains(v{10}));
    static_assert(majors.contains(v{13, 0, 2}));
    static_assert(!majors.contains(v{14}));
    static_assert(!majors.contains(v{9, 5, 2}));

    constexpr solver_version_range minors{{1, 8}, {1, 16}};
    static_assert(minors.contains(v{1, 8}));
    static_assert(minors.contains(v{1, 10}));
    static_assert(minors.contains(v{1, 15, 1}));
    static_assert(!minors.contains(v{1, 16}));
    static_assert(!minors.contains(v{1, 7, 2}));

    constexpr solver_version_range patches{{2, 10, 9}, {2, 10, 14}};
    static_assert(patches.contains(v{2, 10, 9}));
    static_assert(patches.contains(v{2, 10, 13}));
    static_assert(!patches.contains(v{2, 10, 14}));
    static_assert(!patches.contains(v{2, 10}));  // 2.10.0

    // two ranges: a hole between them is a hole
    constexpr std::array validated = {solver_version_range{{8}, {9}},
                                      solver_version_range{{10}, {11}}};
    static_assert(is_validated(validated, v{8, 1}));
    static_assert(!is_validated(validated, v{9, 2}));
    static_assert(is_validated(validated, v{10, 0, 3}));
    EXPECT_EQ(to_string(validated), ">= 8 and < 9 or >= 10 and < 11");
}

TEST(load_solver_library, search_result_is_memoized) {
    const std::string name = fixture_name();
    {
        scoped_env env(loader_path_var,
                       loader_path_with({fixture_path.parent_path()}));
        dynamic_library lib =
            find_solver_library("TESTCACHE", std::array{name.c_str()});
        EXPECT_TRUE(same_file(lib.path(), fixture_path));
    }
    // the directory is no longer on the loader path: only the cache finds it
    dynamic_library again =
        find_solver_library("TESTCACHE", std::array{name.c_str()});
    EXPECT_TRUE(same_file(again.path(), fixture_path));
}

TEST(load_solver_library, explicit_selection_bypasses_the_cache) {
    const std::string name = fixture_name();
    scoped_env path(loader_path_var,
                    loader_path_with({fixture_path.parent_path()}));
    dynamic_library a =
        find_solver_library("TESTBYPASS", std::array{name.c_str()});
    EXPECT_EQ(a.get_function<int()>("mippp_test_answer")(), 42);

    scoped_env pin("MIPPP_TESTBYPASS_LIBRARY", fixture_path_b.string());
    dynamic_library b =
        find_solver_library("TESTBYPASS", std::array{name.c_str()});
    EXPECT_TRUE(same_file(b.path(), fixture_path_b));
    EXPECT_EQ(b.get_function<int()>("mippp_test_answer")(), 43);

    dynamic_library c = load_solver_library(fixture_path_b, "TESTBYPASS");
    EXPECT_EQ(c.get_function<int()>("mippp_test_answer")(), 43);
}

TEST(load_solver_library, two_versions_of_one_library_coexist) {
    dynamic_library a = load_solver_library(fixture_path, "TESTVERSIONS",
                                            std::array{"mippp_test_bump"});
    dynamic_library b = load_solver_library(fixture_path_b, "TESTVERSIONS",
                                            std::array{"mippp_test_bump"});
    EXPECT_NE(a.native_handle(), b.native_handle());
    EXPECT_EQ(a.get_function<int()>("mippp_test_answer")(), 42);
    EXPECT_EQ(b.get_function<int()>("mippp_test_answer")(), 43);

    // each copy owns its own globals: RTLD_GLOBAL would make them alias
    auto bump_a = a.get_function<int()>("mippp_test_bump");
    auto bump_b = b.get_function<int()>("mippp_test_bump");
    bump_a();
    bump_a();
    EXPECT_EQ(bump_a(), 3);
    EXPECT_EQ(bump_b(), 1);
}

// a backend-shaped api over the fixture library, as every <solver>_api is
class test_api : public solver_api<test_api> {
    friend solver_api<test_api>;
    explicit test_api(dynamic_library && library)
        : solver_api(std::move(library))
        , answer(lib.get_function<int()>("mippp_test_answer")) {
        ++constructions;
    }

public:
    inline static int constructions = 0;
    int (*const answer)();
    static const test_api & load(const std::filesystem::path & lib_path) {
        return intern(load_solver_library(lib_path, "TESTINTERN"));
    }
};

TEST(solver_api, one_instance_per_library_file) {
    test_api::constructions = 0;
    const test_api & a = test_api::load(fixture_path);
    const test_api & again = test_api::load(fixture_path);
    EXPECT_EQ(&a, &again);
    EXPECT_EQ(test_api::constructions, 1);
    EXPECT_EQ(a.answer(), 42);
    EXPECT_TRUE(same_file(a.library_path(), fixture_path));

    const test_api & b = test_api::load(fixture_path_b);
    EXPECT_NE(&a, &b);
    EXPECT_EQ(test_api::constructions, 2);
    EXPECT_EQ(b.answer(), 43);
}

TEST(solver_api, symlinked_paths_share_the_instance) {
    const std::filesystem::path link =
        std::filesystem::temp_directory_path() /
        ("mippp_test_link" + fixture_path.extension().string());
    std::error_code ec;
    std::filesystem::remove(link, ec);
    std::filesystem::create_symlink(fixture_path, link, ec);
    if(ec) GTEST_SKIP() << "cannot create a symlink here: " << ec.message();
    const test_api & direct = test_api::load(fixture_path);
    const test_api & linked = test_api::load(link);
    std::filesystem::remove(link, ec);
    EXPECT_EQ(&direct, &linked);
}

// ---- verifier additions ----
#include <concepts>
#include <type_traits>
template <typename Api, typename Arg>
concept loadable_with = requires(Arg a) { Api::load(a); };
static_assert(std::is_aggregate_v<solver_version>);
static_assert(std::three_way_comparable<solver_version, std::strong_ordering>);
static_assert(loadable_with<test_api, std::filesystem::path>);
static_assert(loadable_with<test_api, std::string>);
static_assert(loadable_with<test_api, const char (&)[4]>);
static_assert(!loadable_with<test_api, std::nullptr_t>);

// a backend with a claim the fixture cannot satisfy: the warning must name
// the file that was loaded
class claiming_api : public solver_api<claiming_api> {
    friend solver_api<claiming_api>;
    explicit claiming_api(dynamic_library && library)
        : solver_api(std::move(library)) {
        check_library_version("0.1");
    }

public:
    static constexpr const char * key = "TESTCLAIM";
    static constexpr std::array library_names = {"mippp_no_such_name"};
    static constexpr std::array validated_versions = {
        solver_version_range{{1}, {2}}};
};

TEST(solver_api, unvalidated_version_warning_names_the_file) {
    const scoped_env unsilenced("MIPPP_NO_VERSION_WARNING");  // CI may set it
    testing::internal::CaptureStderr();
    const claiming_api & api = claiming_api::load(fixture_path);
    const std::string output = testing::internal::GetCapturedStderr();
    EXPECT_EQ(api.library_version(), (solver_version{0, 1}));
    EXPECT_NE(output.find("TESTCLAIM"), std::string::npos) << output;
    EXPECT_NE(output.find(">= 1 and < 2"), std::string::npos) << output;
    EXPECT_NE(output.find(fixture_path.string()), std::string::npos) << output;
    EXPECT_NE(output.find("reports 0.1"), std::string::npos) << output;
}
