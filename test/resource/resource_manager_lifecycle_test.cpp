#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"

#include "rtr/resource/resource_manager.hpp"

namespace rtr::resource::test {

namespace {

struct TempDir {
    std::filesystem::path path{};

    explicit TempDir(const std::string& name)
        : path(std::filesystem::temp_directory_path() / name) {
        std::filesystem::remove_all(path);
        std::filesystem::create_directories(path);
    }

    ~TempDir() {
        std::error_code ec;
        std::filesystem::remove_all(path, ec);
    }
};

void write_text_file(const std::filesystem::path& path, const std::string& content) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::binary);
    if (!out.is_open()) {
        throw std::runtime_error("Failed to write file: " + path.string());
    }
    out << content;
}

void write_binary_ppm_1x1_white(const std::filesystem::path& path) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::binary);
    if (!out.is_open()) {
        throw std::runtime_error("Failed to write file: " + path.string());
    }
    out << "P6\n1 1\n255\n";
    const unsigned char white[3]{255, 255, 255};
    out.write(reinterpret_cast<const char*>(white), sizeof(white));
}

utils::ObjMeshData make_triangle_mesh() {
    utils::ObjMeshData mesh{};
    mesh.vertices = {
        {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}},
        {{1.0f, 0.0f, 0.0f}, {1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}},
        {{0.0f, 1.0f, 0.0f}, {0.0f, 1.0f}, {0.0f, 0.0f, 1.0f}},
    };
    mesh.indices = {0, 1, 2};
    return mesh;
}

utils::ImageData make_white_texture() {
    utils::ImageData tex{};
    tex.width = 1;
    tex.height = 1;
    tex.channels = 4;
    tex.pixels = {255, 255, 255, 255};
    return tex;
}

} // namespace

TEST(ResourceManagerLifecycleTest, CreateMeshReturnsValidUniqueHandle) {
    ResourceManager manager{};

    const auto a = manager.create<rtr::resource::MeshResourceKind>(make_triangle_mesh());
    const auto b = manager.create<rtr::resource::MeshResourceKind>(make_triangle_mesh());

    EXPECT_TRUE(a.is_valid());
    EXPECT_TRUE(b.is_valid());
    EXPECT_NE(a, b);
}

TEST(ResourceManagerLifecycleTest, UnloadThenCreateReturnsNewHandle) {
    ResourceManager manager{};

    const auto old_handle = manager.create<rtr::resource::MeshResourceKind>(make_triangle_mesh());
    manager.unload<rtr::resource::MeshResourceKind>(old_handle);

    const auto new_handle = manager.create<rtr::resource::MeshResourceKind>(make_triangle_mesh());
    EXPECT_TRUE(new_handle.is_valid());
    EXPECT_NE(old_handle, new_handle);

    const auto& cpu = manager.cpu<rtr::resource::MeshResourceKind>(new_handle);
    EXPECT_FALSE(cpu.vertices.empty());
    EXPECT_FALSE(cpu.indices.empty());
}

TEST(ResourceManagerLifecycleTest, UnloadMeshInvalidatesCpuAccess) {
    ResourceManager manager{};
    const auto handle = manager.create<rtr::resource::MeshResourceKind>(make_triangle_mesh());

    EXPECT_TRUE(manager.alive<rtr::resource::MeshResourceKind>(handle));
    manager.unload<rtr::resource::MeshResourceKind>(handle);
    EXPECT_FALSE(manager.alive<rtr::resource::MeshResourceKind>(handle));

    EXPECT_THROW((void)manager.cpu<rtr::resource::MeshResourceKind>(handle), std::runtime_error);
    EXPECT_NO_THROW(manager.unload<rtr::resource::MeshResourceKind>(handle));
}

TEST(ResourceManagerLifecycleTest, UnloadedHandleCannotAccessCpuOrGpu) {
    ResourceManager manager{};
    const auto mesh_handle = manager.create<rtr::resource::MeshResourceKind>(make_triangle_mesh());
    manager.unload<rtr::resource::MeshResourceKind>(mesh_handle);

    EXPECT_THROW((void)manager.cpu<rtr::resource::MeshResourceKind>(mesh_handle), std::runtime_error);
    EXPECT_THROW(
        (void)manager.require_gpu<rtr::resource::MeshResourceKind>(
            mesh_handle,
            *reinterpret_cast<rhi::Device*>(0x1)
        ),
        std::runtime_error
    );
}

TEST(ResourceManagerLifecycleTest, CreateTextureThenUnloadIsIdempotent) {
    ResourceManager manager{};
    const auto handle = manager.create<rtr::resource::TextureResourceKind>(
        make_white_texture(),
        TextureCreateOptions{.use_srgb = true}
    );

    EXPECT_TRUE(handle.is_valid());
    EXPECT_TRUE(manager.alive<rtr::resource::TextureResourceKind>(handle));

    manager.unload<rtr::resource::TextureResourceKind>(handle);
    EXPECT_FALSE(manager.alive<rtr::resource::TextureResourceKind>(handle));
    EXPECT_NO_THROW(manager.unload<rtr::resource::TextureResourceKind>(handle));
}

TEST(ResourceManagerLifecycleTest, CreateMeshAndTextureFromRelativePathUsesResourceRoot) {
    TempDir temp_dir("rtr_resource_manager_relative_path_test");
    write_text_file(
        temp_dir.path / "meshes" / "tri.obj",
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"
    );
    write_binary_ppm_1x1_white(temp_dir.path / "textures" / "white.ppm");

    ResourceManager manager(temp_dir.path);
    const auto mesh_handle = manager.create_from_relative_path<rtr::resource::MeshResourceKind>("meshes/tri.obj");
    const auto tex_handle = manager.create_from_relative_path<rtr::resource::TextureResourceKind>(
        "textures/white.ppm",
        TextureCreateOptions{.use_srgb = true}
    );

    EXPECT_TRUE(mesh_handle.is_valid());
    EXPECT_TRUE(tex_handle.is_valid());
    EXPECT_TRUE(manager.alive<rtr::resource::MeshResourceKind>(mesh_handle));
    EXPECT_TRUE(manager.alive<rtr::resource::TextureResourceKind>(tex_handle));
}

TEST(ResourceManagerLifecycleTest, RelativePathApiRejectsAbsolutePath) {
    TempDir temp_dir("rtr_resource_manager_relative_reject_abs_test");
    ResourceManager manager(temp_dir.path);
    const auto abs_path = (temp_dir.path / "meshes" / "tri.obj").string();

    write_text_file(temp_dir.path / "meshes" / "tri.obj", "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");
    write_binary_ppm_1x1_white(temp_dir.path / "meshes" / "tri.ppm");

    EXPECT_THROW((void)manager.create_from_relative_path<rtr::resource::MeshResourceKind>(abs_path), std::invalid_argument);
    EXPECT_THROW(
        (void)manager.create_from_relative_path<rtr::resource::TextureResourceKind>(
            (temp_dir.path / "meshes" / "tri.ppm").string(),
            TextureCreateOptions{.use_srgb = true}
        ),
        std::invalid_argument
    );
}

TEST(ResourceManagerLifecycleTest, RelativePathApiAllowsEscapeFromResourceRoot) {
    TempDir temp_dir("rtr_resource_manager_relative_escape_test");
    ResourceManager manager(temp_dir.path / "assets");
    write_text_file(
        temp_dir.path / "outside" / "tri.obj",
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"
    );
    write_binary_ppm_1x1_white(temp_dir.path / "outside" / "tex.ppm");

    EXPECT_NO_THROW((void)manager.create_from_relative_path<rtr::resource::MeshResourceKind>("../outside/tri.obj"));
    EXPECT_NO_THROW((void)manager.create_from_relative_path<rtr::resource::TextureResourceKind>(
        "../outside/tex.ppm",
        TextureCreateOptions{.use_srgb = true}
    ));
}

} // namespace rtr::resource::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
