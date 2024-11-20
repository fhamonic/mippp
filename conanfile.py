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
    package_type = "header-library"
    exports_sources = "include/*", "cmake/*", "CMakeLists.txt", "test/*"
    no_copy_source = True
    generators = "CMakeToolchain", "CMakeDeps"
    build_policy = "missing"

    def requirements(self):
        self.requires("range-v3/0.12.0")
        
    def build_requirements(self):
        self.test_requires("gtest/[>=1.10.0 <cci]")

    def generate(self):
        print("conanfile.py: IDE include dirs:")
        for lib, dep in self.dependencies.items():
            if not lib.headers:
                continue
            for inc in dep.cpp_info.includedirs:
                print("\t" + inc)
                
    def validate(self):
        check_min_cppstd(self, 20)

    def build(self):
        if self.conf.get("tools.build:skip_test", default=False): return
        cmake = CMake(self)
        cmake.configure(variables={"ENABLE_TESTING":"ON"})
        cmake.build()
        # cmake.test(cli_args=["CTEST_OUTPUT_ON_FAILURE=1"])
        
    def package(self):
        copy(self, "*.hpp", self.source_folder, self.package_folder)
        
    def package_info(self):
        self.cpp_info.bindirs = []
        self.cpp_info.libdirs = []

    def package_id(self):
        self.info.clear()
