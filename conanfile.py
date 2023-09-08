from conan import ConanFile
from conan.tools.cmake import CMake
from conan.tools.files import copy
from conan.tools.build import check_min_cppstd

class CompressorRecipe(ConanFile):
    name="mippp"
    version="0.1"
    license = "BSL-1.0"
    description="A modern interface for linear programming solvers using C++20 ranges and concepts."
    homepage = "https://github.com/fhamonic/mippp.git"
    #url = ""
    settings = "os", "compiler", "arch", "build_type"
    exports_sources = "include/*", "cmake/*", "CMakeLists.txt", "test/*"
    no_copy_source = True
    generators = "CMakeToolchain", "CMakeDeps"
    build_policy = "missing"

    def requirements(self):
        self.requires("range-v3/0.12.0")
        
    def build_requirements(self):
        self.tool_requires("cmake/3.27.1")
        self.test_requires("gtest/1.14.0")

    def validate(self):
        check_min_cppstd(self, 20)

    def build(self):
        if self.conf.get("tools.build:skip_test", default=False): return
        cmake = CMake(self)
        cmake.configure(variables={"ENABLE_TESTING":"ON"})
        cmake.build()
        cmake.test()
        
    def package(self):
        copy(self, "*.hpp", self.source_folder, self.package_folder)
        
    def package_info(self):
        self.cpp_info.bindirs = []
        self.cpp_info.libdirs = []

    def package_id(self):
        self.info.clear()
