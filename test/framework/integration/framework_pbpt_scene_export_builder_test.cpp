#include <pbpt/math/math.h>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"


#include "rtr/framework/component/material/mesh_renderer.hpp"
#include "rtr/framework/component/pbpt/pbpt_light.hpp"
#include "rtr/framework/component/pbpt/pbpt_mesh.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/integration/pbpt/pbpt_scene_export_builder.hpp"
#include "rtr/resource/resource_manager.hpp"

namespace rtr::framework::integration::test {

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

resource::MeshHandle create_test_mesh(resource::ResourceManager& resources) {
    utils::ObjMeshData mesh{};
    mesh.vertices = {
        {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}},
        {{1.0f, 0.0f, 0.0f}, {1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}},
        {{0.0f, 1.0f, 0.0f}, {0.0f, 1.0f}, {0.0f, 0.0f, 1.0f}},
    };
    mesh.indices = {0, 1, 2};
    return resources.create<rtr::resource::MeshResourceKind>(std::move(mesh));
}

component::PbptRgb make_test_rgb(float base) {
    return component::PbptRgb{
        .r = base,
        .g = base + 0.1f,
        .b = base + 0.2f
    };
}

void expect_mat4_near(const pbpt::math::mat4& lhs, const pbpt::math::mat4& rhs, float eps = 1e-5f) {
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
            EXPECT_NEAR(lhs[c][r], rhs[c][r], eps);
        }
    }
}

std::size_t count_occurrences(const std::string& text, const std::string& needle) {
    std::size_t count = 0;
    std::size_t pos = 0;
    while ((pos = text.find(needle, pos)) != std::string::npos) {
        ++count;
        pos += needle.size();
    }
    return count;
}

std::string extract_matrix_value(const std::string& xml) {
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

std::vector<float> parse_csv_floats(const std::string& csv) {
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

} // namespace

TEST(FrameworkPbptSceneExportBuilderTest, BuildsRecordsFromActiveNodesWithMeshAndPbptMesh) {
    core::Scene scene(1, "scene");
    resource::ResourceManager resources{};

    auto& go_ok = scene.create_game_object("");
    const auto expected_handle = create_test_mesh(resources);
    auto& renderer = go_ok.add_component<component::MeshRenderer>(expected_handle);
    const component::PbptRgb reflectance = make_test_rgb(0.2f);
    renderer.set_base_color(pbpt::math::vec4(reflectance.r, reflectance.g, reflectance.b, 1.0f));
    (void)go_ok.add_component<component::PbptMesh>();
    go_ok.node().set_local_position({1.0f, 2.0f, 3.0f});

    auto& go_without_pbpt = scene.create_game_object("mesh_only");
    (void)go_without_pbpt.add_component<component::MeshRenderer>(create_test_mesh(resources));

    auto& go_with_disabled_component = scene.create_game_object("disabled_component");
    (void)go_with_disabled_component.add_component<component::MeshRenderer>(create_test_mesh(resources));
    auto& disabled_pbpt = go_with_disabled_component.add_component<component::PbptMesh>();
    disabled_pbpt.set_enabled(false);

    auto& go_disabled = scene.create_game_object("disabled_go");
    (void)go_disabled.add_component<component::MeshRenderer>(create_test_mesh(resources));
    (void)go_disabled.add_component<component::PbptMesh>();
    go_disabled.set_enabled(false);

    scene.scene_graph().update_world_transforms();

    const auto record = build_pbpt_scene_record(scene, resources);
    ASSERT_EQ(record.shapes.size(), 1u);

    const auto& shape = record.shapes.front();
    EXPECT_EQ(shape.object_name, "go_" + std::to_string(static_cast<std::uint64_t>(go_ok.id())));
    EXPECT_EQ(shape.mesh_handle, expected_handle);
    EXPECT_EQ(shape.material_id, "mat_0");
    EXPECT_FLOAT_EQ(shape.reflectance.r, reflectance.r);
    EXPECT_FLOAT_EQ(shape.reflectance.g, reflectance.g);
    EXPECT_FLOAT_EQ(shape.reflectance.b, reflectance.b);
    EXPECT_FALSE(shape.has_area_emitter);
    expect_mat4_near(shape.model, scene.scene_graph().node(go_ok.id()).world_matrix());
}

TEST(FrameworkPbptSceneExportBuilderTest, ThrowsWhenPbptLightExistsWithoutPbptMesh) {
    core::Scene scene(1, "scene");
    resource::ResourceManager resources{};
    auto& go = scene.create_game_object("light_only");
    (void)go.add_component<component::MeshRenderer>(create_test_mesh(resources));
    (void)go.add_component<component::PbptLight>();

    EXPECT_THROW(
        (void)build_pbpt_scene_record(scene, resources),
        std::runtime_error
    );
}

TEST(FrameworkPbptSceneExportBuilderTest, SerializerDeduplicatesMaterialsAndMeshFilesByHandle) {
    TempDir temp_dir("rtr_pbpt_scene_export_builder_dedup");
    const std::string out_xml = (temp_dir.path / "scene.xml").string();

    resource::ResourceManager resources{};
    const auto shared_mesh = create_test_mesh(resources);

    PbptSceneRecord record{};
    record.shapes.emplace_back(PbptShapeRecord{
        .object_name = "a",
        .mesh_handle = shared_mesh,
        .model = pbpt::math::mat4{1.0f},
        .reflectance = make_test_rgb(0.2f),
        .has_area_emitter = false,
        .radiance_spectrum = {},
        .material_id = ""
    });
    record.shapes.emplace_back(PbptShapeRecord{
        .object_name = "b",
        .mesh_handle = shared_mesh,
        .model = pbpt::math::mat4{1.0f},
        .reflectance = make_test_rgb(0.2f),
        .has_area_emitter = false,
        .radiance_spectrum = {},
        .material_id = ""
    });

    const std::string xml = serialize_pbpt_scene_xml(record, resources, out_xml);

    EXPECT_EQ(count_occurrences(xml, "<bsdf type=\"diffuse\""), 1u);
    EXPECT_EQ(count_occurrences(xml, "<ref id=\"mat_0\"/>"), 2u);
    EXPECT_EQ(count_occurrences(xml, "<string name=\"filename\" value=\"meshes/mesh_"), 2u);

    const auto meshes_dir = temp_dir.path / "meshes";
    ASSERT_TRUE(std::filesystem::exists(meshes_dir));
    const auto expected_mesh_file =
        meshes_dir / ("mesh_" + std::to_string(shared_mesh.value) + ".obj");
    EXPECT_TRUE(std::filesystem::exists(expected_mesh_file));
    std::size_t file_count = 0;
    for (const auto& entry : std::filesystem::directory_iterator(meshes_dir)) {
        if (entry.is_regular_file()) {
            ++file_count;
        }
    }
    EXPECT_EQ(file_count, 1u);
}

TEST(FrameworkPbptSceneExportBuilderTest, SerializerWritesRgbReflectance) {
    TempDir temp_dir("rtr_pbpt_scene_export_builder_rgb_reflectance");
    const std::string out_xml = (temp_dir.path / "scene.xml").string();

    resource::ResourceManager resources{};
    PbptSceneRecord record{};
    record.shapes.emplace_back(PbptShapeRecord{
        .object_name = "rgb_mesh",
        .mesh_handle = create_test_mesh(resources),
        .model = pbpt::math::mat4{1.0f},
        .reflectance = component::PbptRgb{.r = 0.25f, .g = 0.5f, .b = 0.75f},
        .has_area_emitter = false,
        .radiance_spectrum = {},
        .material_id = ""
    });

    const std::string xml = serialize_pbpt_scene_xml(record, resources, out_xml);
    EXPECT_NE(xml.find("<rgb name=\"reflectance\" value=\"0.25 0.5 0.75\"/>"), std::string::npos);
    EXPECT_EQ(count_occurrences(xml, "<spectrum name=\"reflectance\""), 0u);
}

TEST(FrameworkPbptSceneExportBuilderTest, SerializerEmitsAreaEmitterWhenPresent) {
    TempDir temp_dir("rtr_pbpt_scene_export_builder_emitter");
    const std::string out_xml = (temp_dir.path / "scene.xml").string();

    resource::ResourceManager resources{};
    PbptSceneRecord record{};
    record.shapes.emplace_back(PbptShapeRecord{
        .object_name = "light_mesh",
        .mesh_handle = create_test_mesh(resources),
        .model = pbpt::math::mat4{1.0f},
        .reflectance = make_test_rgb(0.2f),
        .has_area_emitter = true,
        .radiance_spectrum = {
            component::PbptSpectrumPoint{400.0f, 0.0f},
            component::PbptSpectrumPoint{500.0f, 8.0f},
            component::PbptSpectrumPoint{600.0f, 15.6f},
            component::PbptSpectrumPoint{700.0f, 18.4f},
        },
        .material_id = ""
    });

    const std::string xml = serialize_pbpt_scene_xml(record, resources, out_xml);
    EXPECT_NE(xml.find("<emitter type=\"area\">"), std::string::npos);
    EXPECT_NE(xml.find("<spectrum name=\"radiance\""), std::string::npos);
}

TEST(FrameworkPbptSceneExportBuilderTest, SerializerUsesStableRowMajorMatrixOrder) {
    TempDir temp_dir("rtr_pbpt_scene_export_builder_matrix");
    const std::string out_xml = (temp_dir.path / "scene.xml").string();

    pbpt::math::mat4 matrix{1.0f};
    float value = 1.0f;
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            matrix[row][col] = value++;
        }
    }

    resource::ResourceManager resources{};
    PbptSceneRecord record{};
    record.shapes.emplace_back(PbptShapeRecord{
        .object_name = "mesh",
        .mesh_handle = create_test_mesh(resources),
        .model = matrix,
        .reflectance = make_test_rgb(0.2f),
        .has_area_emitter = false,
        .radiance_spectrum = {},
        .material_id = ""
    });

    const std::string xml = serialize_pbpt_scene_xml(record, resources, out_xml);
    const std::string matrix_value = extract_matrix_value(xml);
    ASSERT_FALSE(matrix_value.empty());

    const auto numbers = parse_csv_floats(matrix_value);
    ASSERT_EQ(numbers.size(), 16u);
    for (std::size_t i = 0; i < numbers.size(); ++i) {
        EXPECT_NEAR(numbers[i], static_cast<float>(i + 1), 1e-5f);
    }
    EXPECT_NEAR(numbers[3], matrix[0][3], 1e-5f);
    EXPECT_NEAR(numbers[7], matrix[1][3], 1e-5f);
    EXPECT_NEAR(numbers[11], matrix[2][3], 1e-5f);
}

TEST(FrameworkPbptSceneExportBuilderTest, SerializerEmitsSensorAndIntegratorWithMatrix) {
    TempDir temp_dir("rtr_pbpt_scene_export_builder_sensor");
    const std::string out_xml = (temp_dir.path / "scene.xml").string();

    core::Scene scene(1, "scene");
    resource::ResourceManager resources{};

    auto& camera_go = scene.create_game_object("camera");
    auto& camera = scene.camera_manager().create_perspective_camera(camera_go.id());
    camera.fov_degrees() = 39.3077f;
    camera.near_bound() = 10.0f;
    camera.far_bound() = 2800.0f;
    camera_go.node().set_local_position({278.0f, 273.0f, -800.0f});
    camera_go.node().look_at_direction({0.0f, 0.0f, 1.0f});
    ASSERT_TRUE(scene.set_active_camera(camera_go.id()));

    auto& mesh_go = scene.create_game_object("mesh");
    auto& mesh_renderer = mesh_go.add_component<component::MeshRenderer>(create_test_mesh(resources));
    mesh_renderer.set_base_color(pbpt::math::vec4(0.2f, 0.3f, 0.4f, 1.0f));
    (void)mesh_go.add_component<component::PbptMesh>();

    scene.scene_graph().update_world_transforms();

    const auto record = build_pbpt_scene_record(scene, resources);
    const std::string xml = serialize_pbpt_scene_xml(record, resources, out_xml);

    EXPECT_NE(xml.find("<integrator type=\"path\">"), std::string::npos);
    EXPECT_NE(xml.find("<integer name=\"maxDepth\" value=\"-1\"/>"), std::string::npos);
    EXPECT_NE(xml.find("<sensor type=\"perspective\">"), std::string::npos);
    EXPECT_NE(xml.find("<transform name=\"toWorld\">"), std::string::npos);
    EXPECT_NE(xml.find("<matrix value=\""), std::string::npos);
    EXPECT_NE(xml.find("<rgb name=\"reflectance\""), std::string::npos);
    EXPECT_EQ(count_occurrences(xml, "<spectrum name=\"reflectance\""), 0u);
}

TEST(FrameworkPbptSceneExportBuilderTest, SerializerThrowsWhenShapeMeshHandleIsInvalid) {
    TempDir temp_dir("rtr_pbpt_scene_export_builder_invalid_handle");
    const std::string out_xml = (temp_dir.path / "scene.xml").string();

    resource::ResourceManager resources{};
    PbptSceneRecord record{};
    record.shapes.emplace_back(PbptShapeRecord{
        .object_name = "mesh",
        .mesh_handle = resource::MeshHandle{},
        .model = pbpt::math::mat4{1.0f},
        .reflectance = make_test_rgb(0.2f),
        .has_area_emitter = false,
        .radiance_spectrum = {},
        .material_id = ""
    });

    EXPECT_THROW(
        (void)serialize_pbpt_scene_xml(record, resources, out_xml),
        std::runtime_error
    );
}

} // namespace rtr::framework::integration::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
