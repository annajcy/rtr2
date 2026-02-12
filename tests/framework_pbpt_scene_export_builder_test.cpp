#include <algorithm>
#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include <glm/mat4x4.hpp>

#include "framework/component/mesh_renderer.hpp"
#include "framework/component/pbpt_mesh.hpp"
#include "framework/core/scene.hpp"
#include "framework/integration/pbpt_scene_export_builder.hpp"

namespace rtr::framework::integration::test {

static void expect_mat4_near(const glm::mat4& lhs, const glm::mat4& rhs, float eps = 1e-5f) {
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
            EXPECT_NEAR(lhs[c][r], rhs[c][r], eps);
        }
    }
}

static std::size_t count_occurrences(const std::string& text, const std::string& needle) {
    std::size_t count = 0;
    std::size_t pos = 0;
    while ((pos = text.find(needle, pos)) != std::string::npos) {
        ++count;
        pos += needle.size();
    }
    return count;
}

static std::string extract_matrix_value(const std::string& xml) {
    const std::string marker = "<matrix value=\"";
    const std::size_t begin = xml.find(marker);
    if (begin == std::string::npos) {
        return {};
    }
    const std::size_t value_begin = begin + marker.size();
    const std::size_t value_end = xml.find('"', value_begin);
    if (value_end == std::string::npos) {
        return {};
    }
    return xml.substr(value_begin, value_end - value_begin);
}

static std::vector<float> parse_csv_floats(const std::string& csv) {
    std::vector<float> values{};
    std::stringstream ss(csv);
    std::string item;
    while (std::getline(ss, item, ',')) {
        std::stringstream item_stream(item);
        float value = 0.0f;
        item_stream >> value;
        values.emplace_back(value);
    }
    return values;
}

TEST(FrameworkPbptSceneExportBuilderTest, BuildsRecordsFromActiveNodesWithMeshAndPbptMesh) {
    core::Scene scene(1, "scene");

    auto& go_ok = scene.create_game_object("");
    (void)go_ok.add_component<component::MeshRenderer>("assets/models/spot.obj", "");
    auto& go_ok_pbpt = go_ok.add_component<component::PbptMesh>();
    go_ok_pbpt.set_reflectance_rgb(0.2f, 0.3f, 0.4f);
    go_ok.node().set_local_position({1.0f, 2.0f, 3.0f});

    auto& go_without_pbpt = scene.create_game_object("mesh_only");
    (void)go_without_pbpt.add_component<component::MeshRenderer>("assets/models/stanford_bunny.obj", "");

    auto& go_with_disabled_component = scene.create_game_object("disabled_component");
    (void)go_with_disabled_component.add_component<component::MeshRenderer>("assets/models/colored_quad.obj", "");
    auto& disabled_pbpt = go_with_disabled_component.add_component<component::PbptMesh>();
    disabled_pbpt.set_enabled(false);

    auto& go_disabled = scene.create_game_object("disabled_go");
    (void)go_disabled.add_component<component::MeshRenderer>("assets/models/spot.obj", "");
    (void)go_disabled.add_component<component::PbptMesh>();
    go_disabled.set_enabled(false);

    scene.scene_graph().update_world_transforms();

    const auto record = build_pbpt_scene_record(scene);
    ASSERT_EQ(record.shapes.size(), 1u);

    const auto& shape = record.shapes.front();
    EXPECT_EQ(shape.object_name, "go_" + std::to_string(static_cast<std::uint64_t>(go_ok.id())));
    EXPECT_EQ(shape.mesh_path, "assets/models/spot.obj");
    EXPECT_EQ(shape.material_id, "mat_0");
    EXPECT_FLOAT_EQ(shape.reflectance_rgb.x, 0.2f);
    EXPECT_FLOAT_EQ(shape.reflectance_rgb.y, 0.3f);
    EXPECT_FLOAT_EQ(shape.reflectance_rgb.z, 0.4f);
    expect_mat4_near(shape.model, scene.scene_graph().node(go_ok.id()).world_matrix());
}

TEST(FrameworkPbptSceneExportBuilderTest, SerializerDeduplicatesDiffuseMaterials) {
    PbptSceneRecord record{};
    record.shapes.emplace_back(PbptShapeRecord{
        .object_name = "a",
        .mesh_path = "assets/models/spot.obj",
        .model = glm::mat4{1.0f},
        .reflectance_rgb = glm::vec3{0.2f, 0.3f, 0.4f},
        .material_id = ""
    });
    record.shapes.emplace_back(PbptShapeRecord{
        .object_name = "b",
        .mesh_path = "assets/models/stanford_bunny.obj",
        .model = glm::mat4{1.0f},
        .reflectance_rgb = glm::vec3{0.2f, 0.3f, 0.4f},
        .material_id = ""
    });

    const std::string xml = serialize_pbpt_scene_xml(record);

    EXPECT_EQ(count_occurrences(xml, "<bsdf type=\"diffuse\""), 1u);
    EXPECT_EQ(count_occurrences(xml, "<ref id=\"mat_0\"/>"), 2u);
    EXPECT_NE(xml.find("<string name=\"filename\" value=\"assets/models/spot.obj\"/>"), std::string::npos);
    EXPECT_NE(xml.find("<string name=\"filename\" value=\"assets/models/stanford_bunny.obj\"/>"), std::string::npos);
}

TEST(FrameworkPbptSceneExportBuilderTest, SerializerUsesStableRowMajorMatrixOrder) {
    glm::mat4 matrix{1.0f};
    float value = 1.0f;
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            matrix[col][row] = value++;
        }
    }

    PbptSceneRecord record{};
    record.shapes.emplace_back(PbptShapeRecord{
        .object_name = "mesh",
        .mesh_path = "assets/models/spot.obj",
        .model = matrix,
        .reflectance_rgb = glm::vec3{0.5f, 0.5f, 0.5f},
        .material_id = ""
    });

    const std::string xml = serialize_pbpt_scene_xml(record);
    const std::string matrix_value = extract_matrix_value(xml);
    ASSERT_FALSE(matrix_value.empty());

    const auto numbers = parse_csv_floats(matrix_value);
    ASSERT_EQ(numbers.size(), 16u);
    for (std::size_t i = 0; i < numbers.size(); ++i) {
        EXPECT_NEAR(numbers[i], static_cast<float>(i + 1), 1e-5f);
    }
}

TEST(FrameworkPbptSceneExportBuilderTest, SerializerThrowsWhenShapeMeshPathIsEmpty) {
    PbptSceneRecord record{};
    record.shapes.emplace_back(PbptShapeRecord{
        .object_name = "mesh",
        .mesh_path = "",
        .model = glm::mat4{1.0f},
        .reflectance_rgb = glm::vec3{0.5f, 0.5f, 0.5f},
        .material_id = ""
    });

    EXPECT_THROW(
        (void)serialize_pbpt_scene_xml(record),
        std::runtime_error
    );
}

} // namespace rtr::framework::integration::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
