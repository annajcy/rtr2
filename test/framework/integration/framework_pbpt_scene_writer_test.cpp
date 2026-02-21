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
#include "rtr/framework/integration/pbpt/serde/scene_writer.hpp"
#include "rtr/resource/resource_manager.hpp"

namespace rtr::framework::integration::test {

namespace pbpt_bridge = rtr::framework::integration;
using pbpt_bridge::build_scene_result;
using pbpt_bridge::CompatibleInfo;
using pbpt_bridge::MappedShapeInfo;

namespace {

struct TempDir {
    std::filesystem::path path{};

    explicit TempDir(const std::string& name) : path(std::filesystem::temp_directory_path() / name) {
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
    return component::PbptRgb{.r = base, .g = base + 0.1f, .b = base + 0.2f};
}

void expect_mat4_near(const ::pbpt::math::mat4& lhs, const ::pbpt::math::mat4& rhs, float eps = 1e-5f) {
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
            EXPECT_NEAR(lhs[c][r], rhs[c][r], eps);
        }
    }
}

std::size_t count_occurrences(const std::string& text, const std::string& needle) {
    std::size_t count = 0;
    std::size_t pos   = 0;
    while ((pos = text.find(needle, pos)) != std::string::npos) {
        ++count;
        pos += needle.size();
    }
    return count;
}

std::string extract_matrix_value(const std::string& xml) {
    const std::string marker = "<matrix value=\"";
    const std::size_t begin  = xml.find(marker);
    if (begin == std::string::npos) {
        return {};
    }
    const std::size_t value_begin = begin + marker.size();
    const std::size_t value_end   = xml.find('"', value_begin);
    if (value_end == std::string::npos) {
        return {};
    }
    return xml.substr(value_begin, value_end - value_begin);
}

std::vector<float> parse_csv_floats(const std::string& csv) {
    std::vector<float> values{};
    std::stringstream  ss(csv);
    std::string        item;
    while (std::getline(ss, item, ',')) {
        std::stringstream item_stream(item);
        float             value = 0.0f;
        item_stream >> value;
        values.emplace_back(value);
    }
    return values;
}

}  // namespace

TEST(FrameworkPbptSceneWriterTest, BuildsXmlResultFromActiveNodesWithMeshAndPbptMesh) {
    core::Scene               scene(1, "scene");
    resource::ResourceManager resources{};

    auto& camera_go = scene.create_game_object("camera");
    (void)scene.camera_manager().create_perspective_camera(camera_go.id());
    ASSERT_TRUE(scene.set_active_camera(camera_go.id()));

    auto&                    go_ok           = scene.create_game_object("");
    const auto               expected_handle = create_test_mesh(resources);
    auto&                    renderer        = go_ok.add_component<component::MeshRenderer>(expected_handle);
    const component::PbptRgb reflectance     = make_test_rgb(0.2f);
    renderer.set_base_color(::pbpt::math::vec4(reflectance.r, reflectance.g, reflectance.b, 1.0f));
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

    const auto result = build_scene_result(scene, resources);
    ASSERT_EQ(result.scene.resources.shape_instances.size(), 1u);

    const auto& shape = result.scene.resources.shape_instances.front();
    EXPECT_EQ(shape.shape_id, "go_" + std::to_string(static_cast<std::uint64_t>(go_ok.id())));
    EXPECT_FALSE(shape.emission_spectrum_name.has_value());
    expect_mat4_near(pbpt_bridge::compat_export_detail::to_mat4(shape.object_to_world),
                     scene.scene_graph().node(go_ok.id()).world_matrix());
}

TEST(FrameworkPbptSceneWriterTest, ThrowsWhenPbptLightExistsWithoutPbptMesh) {
    core::Scene               scene(1, "scene");
    resource::ResourceManager resources{};

    auto& camera_go = scene.create_game_object("camera");
    (void)scene.camera_manager().create_perspective_camera(camera_go.id());
    ASSERT_TRUE(scene.set_active_camera(camera_go.id()));

    auto& go = scene.create_game_object("light_only");
    (void)go.add_component<component::MeshRenderer>(create_test_mesh(resources));
    (void)go.add_component<component::PbptLight>();

    EXPECT_THROW((void)build_scene_result(scene, resources), std::runtime_error);
}

TEST(FrameworkPbptSceneWriterTest, BuildXmlResultUsesPbptLightSpectrumAndKeepsPassthroughShapes) {
    core::Scene               scene(1, "scene");
    resource::ResourceManager resources{};

    auto& camera_go = scene.create_game_object("camera");
    (void)scene.camera_manager().create_perspective_camera(camera_go.id());
    ASSERT_TRUE(scene.set_active_camera(camera_go.id()));

    auto& mapped_go = scene.create_game_object("mapped_light_go");
    (void)mapped_go.add_component<component::MeshRenderer>(create_test_mesh(resources));
    (void)mapped_go.add_component<component::PbptMesh>();
    auto& light = mapped_go.add_component<component::PbptLight>();
    light.set_radiance_spectrum({
        component::PbptSpectrumPoint{400.0f, 0.0f},
        component::PbptSpectrumPoint{500.0f, 8.0f},
        component::PbptSpectrumPoint{600.0f, 15.6f},
        component::PbptSpectrumPoint{700.0f, 18.4f},
    });

    CompatibleInfo compatible{};
    compatible.passthrough_shape_ids.insert("legacy_shape");
    compatible.passthrough_spp = 5;

    const auto identity = ::pbpt::geometry::Transform<float>::identity();
    const auto render_transform =
        ::pbpt::camera::RenderTransform<float>::from_camera_to_world(identity, ::pbpt::camera::RenderSpace::World);
    std::vector<int>                         indices{0, 1, 2};
    std::vector<::pbpt::math::Point<float, 3>> positions{
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
    };
    std::vector<::pbpt::math::Normal<float, 3>> normals{
        {0.0f, 0.0f, 1.0f},
        {0.0f, 0.0f, 1.0f},
        {0.0f, 0.0f, 1.0f},
    };
    std::vector<::pbpt::math::Point<float, 2>> uvs{
        {0.0f, 0.0f},
        {1.0f, 0.0f},
        {0.0f, 1.0f},
    };
    (void)compatible.passthrough_resources.mesh_library.add_item(
        "legacy_mesh",
        ::pbpt::shape::TriangleMesh<float>(render_transform, indices, positions, normals, uvs, false, identity));
    const int legacy_material_id = compatible.passthrough_resources.any_material_library.add_item(
        "legacy_mat",
        ::pbpt::material::LambertianMaterial<float>(::pbpt::radiometry::PiecewiseLinearSpectrumDistribution<float>({
            {400.0f, 0.5f},
            {500.0f, 0.5f},
            {600.0f, 0.5f},
            {700.0f, 0.5f},
        })));
    compatible.passthrough_resources.mesh_material_map["legacy_mesh"] = legacy_material_id;
    compatible.passthrough_resources.reflectance_spectrum_library.add_item(
        "legacy_emission", ::pbpt::radiometry::PiecewiseLinearSpectrumDistribution<float>({
                               {400.0f, 1.0f},
                               {500.0f, 2.0f},
                               {600.0f, 3.0f},
                               {700.0f, 4.0f},
                           }));
    compatible.passthrough_resources.shape_instances.emplace_back(::pbpt::scene::ShapeInstanceRecord<float>{
        .shape_id               = "legacy_shape",
        .shape_type             = "obj",
        .mesh_name              = "legacy_mesh",
        .material_ref_name      = "legacy_mat",
        .object_to_world        = identity,
        .emission_spectrum_name = std::optional<std::string>{"legacy_emission"},
    });

    const auto pbpt_result = build_scene_result(scene, resources, &compatible, 320, 200, 12);

    EXPECT_EQ(pbpt_result.spp, 12);
    EXPECT_GE(pbpt_result.scene.resources.shape_instances.size(), 2u);

    const auto legacy_it = std::find_if(pbpt_result.scene.resources.shape_instances.begin(),
                                        pbpt_result.scene.resources.shape_instances.end(),
                                        [](const auto& shape) { return shape.shape_id == "legacy_shape"; });
    ASSERT_NE(legacy_it, pbpt_result.scene.resources.shape_instances.end());
    ASSERT_TRUE(legacy_it->emission_spectrum_name.has_value());
    const auto& legacy_emission =
        pbpt_result.scene.resources.reflectance_spectrum_library.get(legacy_it->emission_spectrum_name.value());
    EXPECT_NEAR(legacy_emission.at(700.0f), 4.0f, 1e-5f);

    const auto mapped_it = std::find_if(pbpt_result.scene.resources.shape_instances.begin(),
                                        pbpt_result.scene.resources.shape_instances.end(),
                                        [](const auto& shape) { return shape.shape_id == "mapped_light_go"; });
    ASSERT_NE(mapped_it, pbpt_result.scene.resources.shape_instances.end());
    ASSERT_TRUE(mapped_it->emission_spectrum_name.has_value());
    const auto& mapped_emission =
        pbpt_result.scene.resources.reflectance_spectrum_library.get(mapped_it->emission_spectrum_name.value());
    EXPECT_NEAR(mapped_emission.at(500.0f), 8.0f, 1e-5f);
    EXPECT_NEAR(mapped_emission.at(700.0f), 18.4f, 1e-5f);
}

TEST(FrameworkPbptSceneWriterTest, BuildXmlResultThrowsWhenMappedShapeIdAlsoInPassthroughSet) {
    core::Scene               scene(1, "scene");
    resource::ResourceManager resources{};

    CompatibleInfo compatible{};
    compatible.mapped_shape_info_by_game_object.emplace(1, MappedShapeInfo{
                                                               .source_shape_id          = "dup_shape",
                                                               .source_mesh_name         = "m",
                                                               .source_material_ref_name = "mat",
                                                           });
    compatible.passthrough_shape_ids.insert("dup_shape");

    const auto identity = ::pbpt::geometry::Transform<float>::identity();
    const auto render_transform =
        ::pbpt::camera::RenderTransform<float>::from_camera_to_world(identity, ::pbpt::camera::RenderSpace::World);
    std::vector<int>                         indices{0, 1, 2};
    std::vector<::pbpt::math::Point<float, 3>> positions{
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
    };
    std::vector<::pbpt::math::Normal<float, 3>> normals{
        {0.0f, 0.0f, 1.0f},
        {0.0f, 0.0f, 1.0f},
        {0.0f, 0.0f, 1.0f},
    };
    std::vector<::pbpt::math::Point<float, 2>> uvs{
        {0.0f, 0.0f},
        {1.0f, 0.0f},
        {0.0f, 1.0f},
    };
    (void)compatible.passthrough_resources.mesh_library.add_item(
        "legacy_mesh",
        ::pbpt::shape::TriangleMesh<float>(render_transform, indices, positions, normals, uvs, false, identity));
    const int legacy_material_id = compatible.passthrough_resources.any_material_library.add_item(
        "legacy_mat",
        ::pbpt::material::LambertianMaterial<float>(::pbpt::radiometry::PiecewiseLinearSpectrumDistribution<float>({
            {400.0f, 0.5f},
            {500.0f, 0.5f},
            {600.0f, 0.5f},
            {700.0f, 0.5f},
        })));
    compatible.passthrough_resources.mesh_material_map["legacy_mesh"] = legacy_material_id;
    compatible.passthrough_resources.shape_instances.emplace_back(::pbpt::scene::ShapeInstanceRecord<float>{
        .shape_id               = "dup_shape",
        .shape_type             = "obj",
        .mesh_name              = "legacy_mesh",
        .material_ref_name      = "legacy_mat",
        .object_to_world        = identity,
        .emission_spectrum_name = std::nullopt,
    });

    EXPECT_THROW((void)build_scene_result(scene, resources, &compatible), std::runtime_error);
}

TEST(FrameworkPbptSceneWriterTest, BuildXmlResultSuffixesMappedMaterialWhenNameCollidesWithPassthrough) {
    core::Scene               scene(1, "scene");
    resource::ResourceManager resources{};

    auto& camera_go = scene.create_game_object("camera");
    (void)scene.camera_manager().create_perspective_camera(camera_go.id());
    ASSERT_TRUE(scene.set_active_camera(camera_go.id()));

    auto&      mapped_go       = scene.create_game_object("mapped_go");
    const auto expected_handle = create_test_mesh(resources);
    auto&      renderer        = mapped_go.add_component<component::MeshRenderer>(expected_handle);
    renderer.set_base_color(::pbpt::math::vec4(0.5f, 0.5f, 0.5f, 1.0f));
    (void)mapped_go.add_component<component::PbptMesh>();

    CompatibleInfo compatible{};
    (void)compatible.passthrough_resources.any_material_library.add_item(
        "rtr_mat_0",
        ::pbpt::material::LambertianMaterial<float>(
            ::pbpt::radiometry::PiecewiseLinearSpectrumDistribution<float>({{400.0f, 0.1f}, {700.0f, 0.1f}})));

    const auto pbpt_result = build_scene_result(scene, resources, &compatible);

    const auto mapped_it = std::find_if(pbpt_result.scene.resources.shape_instances.begin(),
                                        pbpt_result.scene.resources.shape_instances.end(),
                                        [](const auto& s) { return s.shape_id == "mapped_go"; });
    ASSERT_NE(mapped_it, pbpt_result.scene.resources.shape_instances.end());
    EXPECT_EQ(mapped_it->material_ref_name, "rtr_mat_0_1");
}

}  // namespace rtr::framework::integration::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
