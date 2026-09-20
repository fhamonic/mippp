from conan import ConanFile
from conan.tools.cmake import CMake


class NQueensExample(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeToolchain", "CMakeDeps"

    def requirements(self):
        self.requires("mippp/1.0.0")

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
