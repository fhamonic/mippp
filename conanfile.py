import os

from conan import ConanFile
from conan.tools.cmake import CMake
from conan.tools.files import copy
from conan.tools.build import check_min_cppstd


class CompressorRecipe(ConanFile):
    name = "mippp"
    version = "1.0.0"
    license = "BSL-1.0"
    description = "A modern interface for linear programming solvers using C++20 ranges and concepts."
    homepage = "https://github.com/fhamonic/mippp.git"
    # url = ""
    settings = "os", "compiler", "arch", "build_type"
    package_type = "header-library"
    exports_sources = "include/*", "cmake/*", "CMakeLists.txt", "test/*"
    no_copy_source = True
    generators = "CMakeToolchain", "CMakeDeps"
    build_policy = "missing"

    def requirements(self):
        self.requires("dylib/2.2.1")

    def build_requirements(self):
        self.test_requires("gtest/[>=1.10.0 <cci]")
        self.test_requires("melon/1.0.0-alpha.1")

    def generate(self):
        print(
            'conanfile.py: Include directories:\n\t"{}"'.format(
                '",\n\t"'.join(
                    [
                        dir
                        for lib, dep in self.dependencies.items()
                        if lib.headers
                        for dir in dep.cpp_info.includedirs
                    ]
                )
            )
        )

    def validate(self):
        check_min_cppstd(self, 26)

    def build(self):
        if self.conf.get("tools.build:skip_test", default=False):
            return
        test_filter = os.environ.get("TEST_FILTER")

        cmake = CMake(self)

        variables = {"ENABLE_TESTING": "ON"}
        if test_filter:
            variables["TEST_FILTER"] = test_filter
        cmake.configure(variables=variables)
        cmake.build()

        cli_args = ["CTEST_OUTPUT_ON_FAILURE=1"]
        if test_filter:
            cli_args.append("ARGS=-R {}".format(test_filter))
        cmake.test(cli_args=cli_args)

    def package(self):
        copy(self, "*.hpp", self.source_folder, self.package_folder)

    def package_info(self):
        self.cpp_info.bindirs = []
        self.cpp_info.libdirs = []

    def package_id(self):
        self.info.clear()
