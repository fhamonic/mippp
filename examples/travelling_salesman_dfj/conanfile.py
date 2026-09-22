from conan import ConanFile
from conan.tools.cmake import CMake


class TravellingSalesmanDfjExample(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeToolchain", "CMakeDeps"

    def requirements(self):
        # MELON (https://github.com/fhamonic/melon) comes from Conan Center;
        # until MIP++ is published there too, export it into your Conan cache
        # first:
        #   conan create <path/to/mippp> -pr=<profile> -b=missing -c tools.build:skip_test=true
        # then build with:
        #   conan build . -of=build -pr=<profile> -b=missing
        self.requires("mippp/1.0.0")
        self.requires("melon/1.0.0")

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
