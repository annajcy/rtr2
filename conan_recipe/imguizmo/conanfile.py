from conan import ConanFile
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.files import copy, get
import os


required_conan_version = ">=2.0"


class ImGuizmoRecipe(ConanFile):
    name = "imguizmo"
    version = "0.20231114.1"
    package_type = "static-library"

    settings = "os", "arch", "compiler", "build_type"
    options = {
        "fPIC": [True, False],
    }
    default_options = {
        "fPIC": True,
    }

    def export_sources(self):
        copy(self, "CMakeLists.txt", src=self.recipe_folder, dst=self.export_sources_folder)

    def config_options(self):
        if self.settings.os == "Windows":
            self.options.rm_safe("fPIC")

    def requirements(self):
        self.requires("imgui/1.92.2b-docking", transitive_headers=True)

    def layout(self):
        cmake_layout(self)

    def source(self):
        get(
            self,
            url="https://github.com/CedricGuillemet/ImGuizmo/archive/ba662b119d64f9ab700bb2cd7b2781f9044f5565.zip",
            sha256="5a63baebb5bce96d83e5e3d6daa4598844ba8d5d0e3cb1ee385fcc54cf996115",
            destination=self.source_folder,
            strip_root=True,
        )

    def generate(self):
        tc = CMakeToolchain(self)
        tc.preprocessor_definitions["IMGUI_DEFINE_MATH_OPERATORS"] = ""
        tc.generate()

        deps = CMakeDeps(self)
        deps.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        copy(
            self,
            "LICENSE",
            src=self.source_folder,
            dst=os.path.join(self.package_folder, "licenses"),
        )
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.libs = ["imguizmo"]
        self.cpp_info.set_property("cmake_file_name", "imguizmo")
        self.cpp_info.set_property("cmake_target_name", "imguizmo::imguizmo")
