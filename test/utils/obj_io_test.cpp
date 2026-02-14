#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"

#include <glm/geometric.hpp>

#include "rtr/utils/obj_io.hpp"

namespace rtr::utils::test {

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
    std::ofstream out(path);
    if (!out.is_open()) {
        throw std::runtime_error("Failed to write file: " + path.string());
    }
    out << content;
}

} // namespace

TEST(ObjIoTest, LoadsTriangleWithUvAndNormal) {
    TempDir temp_dir("rtr_obj_loader_triangle_test");
    const auto obj_path = temp_dir.path / "triangle.obj";

    write_text_file(
        obj_path,
        "v 0 0 0\n"
        "v 1 0 0\n"
        "v 0 1 0\n"
        "vt 0 0\n"
        "vt 1 0\n"
        "vt 0 1\n"
        "vn 0 0 1\n"
        "f 1/1/1 2/2/1 3/3/1\n"
    );

    const auto data = load_obj_from_path(obj_path.string());
    ASSERT_EQ(data.vertices.size(), 3u);
    ASSERT_EQ(data.indices.size(), 3u);

    EXPECT_EQ(data.indices[0], 0u);
    EXPECT_EQ(data.indices[1], 1u);
    EXPECT_EQ(data.indices[2], 2u);

    EXPECT_FLOAT_EQ(data.vertices[1].uv.x, 1.0f);
    EXPECT_FLOAT_EQ(data.vertices[1].uv.y, 0.0f);
    EXPECT_FLOAT_EQ(data.vertices[2].normal.z, 1.0f);
}

TEST(ObjIoTest, ReusesVertexIndicesForSharedVertices) {
    TempDir temp_dir("rtr_obj_loader_reuse_test");
    const auto obj_path = temp_dir.path / "quad.obj";

    write_text_file(
        obj_path,
        "v 0 0 0\n"
        "v 1 0 0\n"
        "v 1 1 0\n"
        "v 0 1 0\n"
        "f 1 2 3\n"
        "f 1 3 4\n"
    );

    const auto data = load_obj_from_path(obj_path.string());
    ASSERT_EQ(data.vertices.size(), 4u);
    ASSERT_EQ(data.indices.size(), 6u);

    EXPECT_GE(std::count(data.indices.begin(), data.indices.end(), 0u), 2);
    EXPECT_GE(std::count(data.indices.begin(), data.indices.end(), 2u), 2);
}

TEST(ObjIoTest, GeneratesNormalsWhenInputNormalsMissing) {
    TempDir temp_dir("rtr_obj_loader_generate_normal_test");
    const auto obj_path = temp_dir.path / "triangle_no_normals.obj";

    write_text_file(
        obj_path,
        "v 0 0 0\n"
        "v 1 0 0\n"
        "v 0 1 0\n"
        "f 1 2 3\n"
    );

    const auto data = load_obj_from_path(obj_path.string());
    ASSERT_EQ(data.vertices.size(), 3u);
    ASSERT_EQ(data.indices.size(), 3u);

    for (const auto& vertex : data.vertices) {
        const auto len = glm::length(vertex.normal);
        EXPECT_TRUE(std::isfinite(len));
        EXPECT_GT(len, 0.0f);
    }
}

TEST(ObjIoTest, ThrowsWhenFaceReferencesOutOfRangeVertexIndex) {
    TempDir temp_dir("rtr_obj_loader_invalid_index_test");
    const auto obj_path = temp_dir.path / "invalid.obj";

    write_text_file(
        obj_path,
        "v 0 0 0\n"
        "v 1 0 0\n"
        "f 1 2 3\n"
    );

    EXPECT_THROW(
        (void)load_obj_from_path(obj_path.string()),
        std::runtime_error
    );
}

TEST(ObjIoTest, WritesObjThatCanBeReadBack) {
    TempDir temp_dir("rtr_obj_io_write_roundtrip_test");
    const auto obj_path = temp_dir.path / "roundtrip.obj";

    ObjMeshData mesh{};
    mesh.vertices = {
        {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}},
        {{1.0f, 0.0f, 0.0f}, {1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}},
        {{0.0f, 1.0f, 0.0f}, {0.0f, 1.0f}, {0.0f, 0.0f, 1.0f}},
    };
    mesh.indices = {0u, 1u, 2u};

    write_obj_to_path(mesh, obj_path.string());
    const auto loaded = load_obj_from_path(obj_path.string());
    EXPECT_EQ(loaded.indices.size(), 3u);
    EXPECT_EQ(loaded.vertices.size(), 3u);
}

} // namespace rtr::utils::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
