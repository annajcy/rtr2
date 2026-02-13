from conan import ConanFile
from conan.errors import ConanInvalidConfiguration
from conan.tools.cmake import CMake, CMakeToolchain, cmake_layout
from conan.tools.files import copy

required_conan_version = ">=2.0"


class RTRConan(ConanFile):
    name = "rtr"
    package_type = "static-library"

    settings = "os", "arch", "compiler", "build_type"

    options = {
        "build_tests": [True, False],
        "build_examples": [True, False],
        "compile_shaders": [True, False],
        "with_pbpt": [True, False],
        "slang_version": ["ANY"],
        "pbpt_version": ["ANY"],
    }

    default_options = {
        "build_tests": True,
        "build_examples": True,
        "compile_shaders": True,
        "with_pbpt": True,
        "slang_version": "2025.10.4",
        "pbpt_version": "0.1.0-dev.transitivefix",
        "embree/*:shared": True,
    }

    generators = "CMakeDeps", "VirtualBuildEnv", "VirtualRunEnv"

    def layout(self):
        cmake_layout(self, generator="Ninja")

    def validate(self):
        cppstd = str(self.settings.get_safe("compiler.cppstd") or "")
        if cppstd not in ("23", "gnu23"):
            raise ConanInvalidConfiguration(
                f"rtr requires compiler.cppstd=23 (or gnu23), got '{cppstd or 'unset'}'"
            )

    def export_sources(self):
        copy(
            self,
            "*",
            src=self.recipe_folder,
            dst=self.export_sources_folder,
            excludes=[
                ".git/*",
                ".github/*",
                ".vscode/*",
                "build/*",
                "**/build/*",
                "output/*",
                "**/output/*",
                "external/pbpt/*",
                "docs/*",
                "**/__pycache__/*",
                "*.pyc",
                "CMakeUserPresets.json",
                "compile_commands.json",
            ],
        )

    def requirements(self):
        self.requires("glfw/3.4")
        self.requires("assimp/5.4.3")
        self.requires("imgui/1.92.2b-docking")
        self.requires("stb/cci.20230920")
        self.requires("vulkan-loader/[>=1.3]")
        self.requires("glm/cci.20230113")
        self.requires("tinyobjloader/2.0.0-rc10")
        self.requires("pugixml/1.14")

        if self.options.with_pbpt:
            pbpt_ver = str(self.options.pbpt_version)
            self.requires(f"pbpt/{pbpt_ver}")

        slang_ver = str(self.options.slang_version)
        self.requires(f"slang/{slang_ver}")

        if self.options.build_tests:
            self.test_requires("gtest/[>=1.14 <2]")

    def build_requirements(self):
        self.tool_requires("cmake/3.27.9")
        self.tool_requires("ninja/1.12.1")

    def generate(self):
        tc = CMakeToolchain(self)
        tc.generator = "Ninja"
        tc.cache_variables["CMAKE_EXPORT_COMPILE_COMMANDS"] = True
        tc.cache_variables["RTR_BUILD_TESTS"] = "ON" if self.options.build_tests else "OFF"
        tc.cache_variables["RTR_BUILD_EXAMPLES"] = "ON" if self.options.build_examples else "OFF"
        tc.cache_variables["RTR_COMPILE_SHADERS"] = "ON" if self.options.compile_shaders else "OFF"
        tc.cache_variables["RTR_ENABLE_PBPT_RUNTIME"] = "ON" if self.options.with_pbpt else "OFF"
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.set_property("cmake_file_name", "rtr")

        imgui_vk_component = self.cpp_info.components["imgui_vk"]
        imgui_vk_component.libs = ["imgui_vk"]
        imgui_vk_component.requires = [
            "imgui::imgui",
            "glfw::glfw",
            "vulkan-loader::vulkan-loader",
        ]

        stb_impl_component = self.cpp_info.components["stb_impl"]
        stb_impl_component.libs = ["stb_impl"]
        stb_impl_component.requires = ["stb::stb"]

        core_component = self.cpp_info.components["rtr"]
        core_component.set_property("cmake_target_name", "rtr::rtr")
        core_component.requires = [
            "imgui_vk",
            "stb_impl",
            "assimp::assimp",
            "slang::slang",
            "glm::glm",
            "tinyobjloader::tinyobjloader",
            "pugixml::pugixml",
        ]

        framework_component = self.cpp_info.components["framework_integration"]
        framework_component.set_property("cmake_target_name", "rtr::framework_integration")
        framework_component.libs = ["rtr_framework_integration"]
        framework_component.requires = ["rtr"]
        if self.options.with_pbpt:
            framework_component.requires.append("pbpt::pbpt")
