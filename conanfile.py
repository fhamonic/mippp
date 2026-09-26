import os
import re
from conan import ConanFile
from conan.errors import ConanException
from conan.tools.files import copy, load
from conan.tools.cmake import CMake
from conan.tools.build import check_min_cppstd

required_conan_version = ">=2.0"


class MipppConan(ConanFile):
    name = "mippp"

    license = "BSL-1.0"
    # Same sentence as the CMake project() DESCRIPTION, so the two cannot
    # drift; this string is what Conan Center displays.
    description = (
        "A modern interface for linear programming solvers using C++23 features."
    )
    topics = ("optimization", "linear-programming", "mip", "header-only", "cpp23")
    homepage = "https://github.com/fhamonic/mippp"
    url = "https://github.com/fhamonic/mippp"

    settings = "os", "arch", "compiler", "build_type"
    package_type = "header-library"
    exports_sources = (
        "include/*",
        "cmake/*",
        "CMakeLists.txt",
        "test/*",
        "LICENSE.md",
    )
    no_copy_source = True
    generators = "CMakeToolchain", "CMakeDeps"

    def set_version(self):
        # include/mippp/version.hpp is the single source of truth for the
        # version number (CMakeLists.txt parses it too).
        version_hpp = load(
            self,
            os.path.join(self.recipe_folder, "include", "mippp", "version.hpp"),
        )
        components = {}
        for level in ("MAJOR", "MINOR", "PATCH"):
            match = re.search(
                rf"#define MIPPP_VERSION_{level} (\d+)", version_hpp
            )
            if match is None:
                # Mirrors the FATAL_ERROR in CMakeLists.txt; a bare
                # AttributeError here would not name the real problem.
                raise ConanException(
                    f"Failed to parse MIPPP_VERSION_{level} from version.hpp"
                )
            components[level] = match.group(1)
        self.version = "{MAJOR}.{MINOR}.{PATCH}".format(**components)

    @property
    def _skip_test(self):
        return self.conf.get("tools.build:skip_test", default=False)

    @property
    def _build_examples(self):
        return os.environ.get("ENABLE_EXAMPLES", "").upper() in ("1", "ON", "TRUE")

    def build_requirements(self):
        # Ungated, the test_requires make skip_test builds fail on missing
        # binaries that build() would never use.
        if not self._skip_test:
            self.test_requires("gtest/[>=1.10.0 <cci]")
        # MELON also drives the travelling_salesman_dfj example.
        if not self._skip_test or self._build_examples:
            self.test_requires("melon/1.0.0")

    def validate(self):
        check_min_cppstd(self, 23)

    def build(self):
        if self._skip_test and not self._build_examples:
            return
        test_filter = os.environ.get("TEST_FILTER")
        test_source = os.environ.get("TEST_SOURCE")
        test_sanitize = os.environ.get("TEST_SANITIZE")

        variables = {
            "ENABLE_TESTING": "OFF" if self._skip_test else "ON",
            "ENABLE_EXAMPLES": "ON" if self._build_examples else "OFF",
        }
        # TEST_SOURCE restricts the build to some backends' tests ("highs",
        # "CLP;CBC"); TEST_SANITIZE builds them with -fsanitize=<list>. Both
        # are documented in test/CMakeLists.txt and CONTRIBUTING.md.
        if test_source:
            variables["TEST_SOURCE"] = test_source
        if test_sanitize:
            variables["MIPPP_SANITIZE"] = test_sanitize
        # Explicit opt-in; installed solver libraries remain the default.
        variables["MIPPP_TEST_FETCH_HIGHS"] = self.conf.get(
            "user.mippp:test_fetch_highs", default=False, check_type=bool
        )

        cmake = CMake(self)
        cmake.configure(variables=variables)
        cmake.build()

        if self._skip_test:
            return
        # Not cmake.test(cli_args=["CTEST_OUTPUT_ON_FAILURE=1"]): that token
        # reaches the native tool, and while make exports it to ctest as an
        # environment variable, MSBuild rejects a bare NAME=VALUE argument
        # and the MSVC job fails before testing.
        cli_args = ["--output-on-failure"]
        if test_filter:
            cli_args += ["-R", test_filter]
        cmake.ctest(cli_args=cli_args)

    def package(self):
        copy(
            self,
            "LICENSE.md",
            self.source_folder,
            os.path.join(self.package_folder, "licenses"),
        )
        # from include/ only: the test suites and examples are headers too
        copy(
            self,
            "*.hpp",
            os.path.join(self.source_folder, "include"),
            os.path.join(self.package_folder, "include"),
        )

    def package_info(self):
        # The names mipppConfig.cmake exports; CMakeDeps' defaults happen
        # to coincide today, but only the explicit properties keep a rename
        # on either side from silently changing what consumers link.
        self.cpp_info.set_property("cmake_file_name", "mippp")
        self.cpp_info.set_property("cmake_target_name", "mippp::mippp")
        self.cpp_info.bindirs = []
        self.cpp_info.libdirs = []
        # dlopen lives in libdl on glibc < 2.34, as CMAKE_DL_LIBS records for
        # the CMake package
        if self.settings.os == "Linux":
            self.cpp_info.system_libs = ["dl"]

    def package_id(self):
        self.info.clear()
