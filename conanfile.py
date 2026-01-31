from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps
from conan.tools.files import copy
import os

class MicroLAConan(ConanFile):
    name = "microla"
    version = "1.0.0"
    license = "MIT"
    author = "James Baldwin"
    url = "https://github.com/yourusername/microla"
    description = "High-performance C++11 linear algebra library for embedded systems"
    topics = ("linear-algebra", "matrix", "vector", "embedded", "simd", "quaternion")
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "enable_tests": [True, False],
        "enable_benchmarks": [True, False],
        "enable_examples": [True, False],
        "enable_neon": [True, False],
        "enable_cmsis": [True, False],
    }
    default_options = {
        "enable_tests": False,
        "enable_benchmarks": False,
        "enable_examples": False,
        "enable_neon": False,
        "enable_cmsis": False,
    }
    exports_sources = "CMakeLists.txt", "include/*", "cmake/*", "tests/*", "benchmarks/*", "examples/*"
    no_copy_source = True

    def requirements(self):
        if self.options.enable_tests:
            self.requires("gtest/1.14.0")

    def layout(self):
        cmake_layout(self)

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()
        tc = CMakeToolchain(self)
        tc.variables["MICROLA_LINEAR_BUILD_TESTS"] = self.options.enable_tests
        tc.variables["MICROLA_LINEAR_BUILD_BENCHMARKS"] = self.options.enable_benchmarks
        tc.variables["MICROLA_LINEAR_BUILD_EXAMPLES"] = self.options.enable_examples
        tc.variables["MICROLA_ENABLE_NEON"] = self.options.enable_neon
        tc.variables["MICROLA_ENABLE_CMSIS"] = self.options.enable_cmsis
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        copy(self, "*.hpp",
             src=os.path.join(self.source_folder, "include"),
             dst=os.path.join(self.package_folder, "include"))
        copy(self, "LICENSE",
             src=self.source_folder,
             dst=os.path.join(self.package_folder, "licenses"))
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.set_property("cmake_file_name", "microla")
        self.cpp_info.set_property("cmake_target_name", "microla::microla")
        self.cpp_info.bindirs = []
        self.cpp_info.libdirs = []

        if self.options.enable_neon:
            self.cpp_info.defines.append("CONFIG_MICROLA_NEON")
        if self.options.enable_cmsis:
            self.cpp_info.defines.append("CONFIG_MICROLA_CMSIS")

    def package_id(self):
        self.info.clear()
