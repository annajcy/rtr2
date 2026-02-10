# conanfile.py
from conan import ConanFile
from conan.tools.cmake import cmake_layout, CMake, CMakeToolchain

required_conan_version = ">=2.0"

class RTRDeps(ConanFile):
    name = "rtr-deps"
    version = "0.1"
    settings = "os", "arch", "compiler", "build_type"

    options = {
        "build_tests": [True, False],
        "build_examples": [True, False],
        "compile_shaders": [True, False],
        "slang_version": ["ANY"]
    }

    default_options = {
        "build_tests": True,
        "build_examples": True,
        "compile_shaders": True,
        "slang_version": "2025.10.4"
    }

    generators = "CMakeDeps", "VirtualBuildEnv", "VirtualRunEnv"

    def layout(self):
        cmake_layout(self, generator="Ninja")

    def requirements(self):
        self.requires("glfw/3.4")
        self.requires("assimp/5.4.3")
        self.requires("imgui/1.92.2b-docking")
        self.requires("stb/cci.20230920")
        self.requires("vulkan-loader/[>=1.3]")
        self.requires("glm/cci.20230113")
        self.requires("tinyobjloader/2.0.0-rc10")

        # 本地 recipe：conan create . --name=slang --version=2025.10.4
        ver = str(self.options.slang_version)
        self.requires(f"slang/{ver}")  

        if self.options.build_tests:
            self.requires("gtest/[>=1.14 <2]")

    def build_requirements(self):
        self.tool_requires("cmake/3.27.9")
        self.tool_requires("ninja/1.12.1")

    def generate(self):
        tc = CMakeToolchain(self)
        tc.generator = "Ninja"
        tc.cache_variables["CMAKE_EXPORT_COMPILE_COMMANDS"] = True
        tc.generate()

    # conan build .
    def build(self):
        cmake = CMake(self)

        variables = {
            "RTR_BUILD_TESTS": "ON" if self.options.build_tests else "OFF",
            "RTR_BUILD_EXAMPLES": "ON" if self.options.build_examples else "OFF",
            "RTR_COMPILE_SHADERS": "ON" if self.options.compile_shaders else "OFF",
            "CMAKE_BUILD_TYPE": str(self.settings.build_type),
        }

        cmake.configure(variables=variables)
        cmake.build()
