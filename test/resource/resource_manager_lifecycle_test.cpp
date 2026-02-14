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

} // namespace

TEST(ResourceManagerLifecycleTest, LoadMeshDeduplicatesNormalizedPath) {
    TempDir temp_dir("rtr_resource_manager_mesh_dedup");
    const auto mesh_path = temp_dir.path / "mesh.obj";
    write_text_file(mesh_path, "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");

    ResourceManager manager{};
    const auto a = manager.load_mesh(mesh_path.string());
    const auto b = manager.load_mesh((temp_dir.path / "." / "mesh.obj").string());

    EXPECT_TRUE(a.is_valid());
    EXPECT_EQ(a, b);
}

TEST(ResourceManagerLifecycleTest, UnloadThenLoadSamePathReturnsNewHandle) {
    TempDir temp_dir("rtr_resource_manager_mesh_reopen");
    const auto mesh_path = temp_dir.path / "mesh.obj";
    write_text_file(mesh_path, "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");

    ResourceManager manager{};
    const auto old_handle = manager.load_mesh(mesh_path.string());
    manager.unload_mesh(old_handle);

    const auto new_handle = manager.load_mesh(mesh_path.string());
    EXPECT_TRUE(new_handle.is_valid());
    EXPECT_NE(old_handle, new_handle);

    const auto& cpu = manager.mesh_cpu(new_handle);
    EXPECT_FALSE(cpu.vertices.empty());
    EXPECT_FALSE(cpu.indices.empty());
}

TEST(ResourceManagerLifecycleTest, UnloadMeshInvalidatesCpuAccess) {
    TempDir temp_dir("rtr_resource_manager_mesh_unload");
    const auto mesh_path = temp_dir.path / "mesh.obj";
    write_text_file(mesh_path, "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");

    ResourceManager manager{};
    const auto handle = manager.load_mesh(mesh_path.string());

    EXPECT_TRUE(manager.mesh_alive(handle));
    manager.unload_mesh(handle);
    EXPECT_FALSE(manager.mesh_alive(handle));

    EXPECT_THROW((void)manager.mesh_cpu(handle), std::runtime_error);
    EXPECT_NO_THROW(manager.unload_mesh(handle));
}

TEST(ResourceManagerLifecycleTest, UnloadedHandleCannotAccessCpuOrGpu) {
    TempDir temp_dir("rtr_resource_manager_unloaded_handle_invalidate");
    const auto mesh_path = temp_dir.path / "mesh.obj";
    write_text_file(mesh_path, "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");

    ResourceManager manager{};
    const auto mesh_handle = manager.load_mesh(mesh_path.string());
    manager.unload_mesh(mesh_handle);

    EXPECT_THROW((void)manager.mesh_cpu(mesh_handle), std::runtime_error);
    EXPECT_THROW(
        (void)manager.require_mesh_rhi(
            mesh_handle,
            reinterpret_cast<rhi::Device*>(0x1)
        ),
        std::runtime_error
    );
}

TEST(ResourceManagerLifecycleTest, LoadTextureDeduplicatesNormalizedPathAndUnloadIsIdempotent) {
    TempDir temp_dir("rtr_resource_manager_texture_dedup");
    const auto tex_path = temp_dir.path / "dummy.ppm";
    write_text_file(tex_path, "P3\n1 1\n255\n255 255 255\n");

    ResourceManager manager{};
    const auto a = manager.load_texture(tex_path.string());
    const auto b = manager.load_texture((temp_dir.path / "." / "dummy.ppm").string());

    EXPECT_TRUE(a.is_valid());
    EXPECT_EQ(a, b);

    manager.unload_texture(a);
    EXPECT_FALSE(manager.texture_alive(a));
    EXPECT_NO_THROW(manager.unload_texture(a));
}

} // namespace rtr::resource::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
