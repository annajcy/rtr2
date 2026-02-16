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
        "with_tests": [True, False],
        "with_examples": [True, False],
        "with_editor": [True, False],
        "compile_shaders": [True, False],
        "slang_version": ["ANY"],
    }

    default_options = {
        "with_tests": True,
        "with_examples": True,
        "with_editor": True,
        "compile_shaders": True,
        "slang_version": "2025.10.4",
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
                "docs/*",
                "**/__pycache__/*",
                "*.pyc",
                "CMakeUserPresets.json",
                "compile_commands.json",
            ],
        )

    def requirements(self):
        self.requires("glfw/3.4", transitive_headers=True)
        self.requires("tinygltf/[>=2.8 <3]")
        if self.options.with_editor:
            self.requires("imgui/1.92.2b-docking", transitive_headers=True)
        self.requires("stb/cci.20240531", transitive_headers=True)
        self.requires("spdlog/[>=1.13 <2]", transitive_headers=True)
        self.requires("vulkan-loader/[>=1.3]", transitive_headers=True)
        self.requires("tinyobjloader/2.0.0-rc10", transitive_headers=True)
        self.requires("pugixml/1.14", transitive_headers=True)

        self.requires("openexr/3.2.4", transitive_headers=True)
        self.requires("embree/4.4.0", transitive_headers=True)
        self.requires("onetbb/2021.12.0", transitive_headers=True)

        slang_ver = str(self.options.slang_version)
        self.requires(f"slang/{slang_ver}")

        if self.options.with_tests:
            self.test_requires("gtest/[>=1.14 <2]")

    def build_requirements(self):
        self.tool_requires("cmake/3.27.9")
        self.tool_requires("ninja/1.12.1")

    def generate(self):
        tc = CMakeToolchain(self)
        tc.generator = "Ninja"
        tc.cache_variables["CMAKE_EXPORT_COMPILE_COMMANDS"] = True
        tc.cache_variables["RTR_BUILD_TESTS"] = "ON" if self.options.with_tests else "OFF"
        tc.cache_variables["RTR_BUILD_EXAMPLES"] = "ON" if self.options.with_examples else "OFF"
        tc.cache_variables["RTR_BUILD_EDITOR"] = "ON" if self.options.with_editor else "OFF"
        tc.cache_variables["RTR_COMPILE_SHADERS"] = "ON" if self.options.compile_shaders else "OFF"
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

        if self.options.with_editor:
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

        runtime_component = self.cpp_info.components["runtime"]
        runtime_component.set_property("cmake_target_name", "rtr::runtime")
        runtime_component.requires = [
            "stb_impl",
            "glfw::glfw",
            "vulkan-loader::vulkan-loader",
            "spdlog::spdlog",
            "tinygltf::tinygltf",
            "slang::slang",
            "tinyobjloader::tinyobjloader",
            "pugixml::pugixml",
            "openexr::openexr",
            "embree::embree",
            "onetbb::onetbb",
            "stb::stb",
        ]
        runtime_component.libs.extend([
            "pbpt_rgb_spectrum_lut",
            "pbpt_stb_impl",
        ])

        if self.options.with_editor:
            editor_component = self.cpp_info.components["editor"]
            editor_component.set_property("cmake_target_name", "rtr::editor")
            editor_component.requires = [
                "runtime",
                "imgui_vk",
            ]
