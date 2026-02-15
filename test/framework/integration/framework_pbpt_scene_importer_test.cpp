#include <pbpt/math/math.h>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"


#include "rtr/framework/component/material/mesh_renderer.hpp"
#include "rtr/framework/component/camera_control/free_look_camera_controller.hpp"
#include "rtr/framework/component/pbpt/pbpt_light.hpp"
#include "rtr/framework/component/pbpt/pbpt_mesh.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/integration/pbpt/pbpt_reflectance_convert.hpp"
#include "rtr/framework/integration/pbpt/pbpt_scene_importer.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/system/input/input_state.hpp"

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

void write_text_file(const std::filesystem::path& path, const std::string& content) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("Failed to write file: " + path.string());
    }
    out << content;
}

const core::GameObject* find_mesh_object(const core::Scene& scene) {
    for (const auto& go : scene.game_objects()) {
        if (go && go->get_component<component::PbptMesh>() != nullptr) {
            return go.get();
        }
    }
    return nullptr;
}

PbptSceneLocation make_location(
    const std::filesystem::path& resource_root,
    const std::filesystem::path& xml_path
) {
    const std::filesystem::path rel = xml_path.lexically_relative(resource_root);
    std::filesystem::path scene_root = rel.parent_path();
    if (scene_root.empty()) {
        scene_root = ".";
    }

    return make_pbpt_scene_location(
        scene_root.generic_string(),
        rel.filename().string()
    );
}

} // namespace

TEST(FrameworkPbptSceneImporterTest, ImportsCboxSubsetAndAttachesComponents) {
    TempDir temp_dir("rtr_pbpt_scene_importer_test");

    const auto mesh_path = temp_dir.path / "meshes" / "tri.obj";
    write_text_file(mesh_path, "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");

    const auto xml_path = temp_dir.path / "scene.xml";
    write_text_file(
        xml_path,
        R"XML(<?xml version="1.0" encoding="utf-8"?>
<scene version="0.4.0">
  <integrator type="path">
    <integer name="maxDepth" value="-1"/>
  </integrator>
  <sensor type="perspective">
    <string name="fovAxis" value="smaller"/>
    <float name="nearClip" value="0.1"/>
    <float name="farClip" value="1000"/>
    <float name="focusDistance" value="12"/>
    <transform name="toWorld">
      <matrix value="1,0,0,0, 0,1,0,0, 0,0,1,4, 0,0,0,1"/>
    </transform>
    <float name="fov" value="45"/>
    <sampler type="ldsampler">
      <integer name="sampleCount" value="8"/>
    </sampler>
    <film type="hdrfilm">
      <integer name="width" value="320"/>
      <integer name="height" value="200"/>
      <rfilter type="gaussian"/>
    </film>
  </sensor>
  <bsdf type="diffuse" id="mat_white">
    <spectrum name="reflectance" value="400:0.7, 500:0.7, 600:0.7, 700:0.7"/>
  </bsdf>
  <shape type="obj" id="mesh_a">
    <string name="filename" value="meshes/tri.obj"/>
    <transform name="toWorld">
      <matrix value="1,0,0,1, 0,1,0,2, 0,0,1,3, 0,0,0,1"/>
    </transform>
    <ref id="mat_white"/>
    <emitter type="area">
      <spectrum name="radiance" value="400:0, 500:8, 600:15.6, 700:18.4"/>
    </emitter>
  </shape>
</scene>)XML"
    );

    core::Scene scene(1, "scene");
    resource::ResourceManager resources(2, temp_dir.path);
    const auto result = import_pbpt_scene_xml_to_scene(
        make_location(temp_dir.path, xml_path),
        scene,
        resources
    );

    EXPECT_EQ(result.imported_shape_count, 1u);
    EXPECT_EQ(result.imported_light_shape_count, 1u);
    ASSERT_TRUE(result.integrator.has_value());
    EXPECT_EQ(result.integrator->type, "path");
    EXPECT_EQ(result.integrator->max_depth, -1);
    ASSERT_TRUE(result.sensor.has_value());
    EXPECT_EQ(result.sensor->sample_count, 8);
    EXPECT_EQ(result.sensor->film_width, 320);
    EXPECT_EQ(result.sensor->film_height, 200);

    const auto* mesh_go = find_mesh_object(scene);
    ASSERT_NE(mesh_go, nullptr);
    ASSERT_TRUE(result.imported_game_object_id_by_name.contains("mesh_a"));
    EXPECT_EQ(result.imported_game_object_id_by_name.at("mesh_a"), mesh_go->id());
    ASSERT_TRUE(result.imported_game_object_id_by_name.contains("pbpt_camera"));
    const auto* camera_go = scene.find_game_object(scene.camera_manager().active_camera_owner_id());
    ASSERT_NE(camera_go, nullptr);
    EXPECT_EQ(result.imported_game_object_id_by_name.at("pbpt_camera"), camera_go->id());

    const auto* renderer = mesh_go->get_component<component::MeshRenderer>();
    const auto* pbpt_mesh = mesh_go->get_component<component::PbptMesh>();
    const auto* pbpt_light = mesh_go->get_component<component::PbptLight>();
    ASSERT_NE(renderer, nullptr);
    ASSERT_NE(pbpt_mesh, nullptr);
    ASSERT_NE(pbpt_light, nullptr);

    EXPECT_TRUE(renderer->mesh_handle().is_valid());
    EXPECT_TRUE(resources.mesh_alive(renderer->mesh_handle()));

    const auto expected_base_color = pbpt_spectrum_to_rgb({
        component::PbptSpectrumPoint{400.0f, 0.7f},
        component::PbptSpectrumPoint{500.0f, 0.7f},
        component::PbptSpectrumPoint{600.0f, 0.7f},
        component::PbptSpectrumPoint{700.0f, 0.7f},
    });
    EXPECT_NEAR(renderer->base_color().x(), expected_base_color.r, 1e-5f);
    EXPECT_NEAR(renderer->base_color().y(), expected_base_color.g, 1e-5f);
    EXPECT_NEAR(renderer->base_color().z(), expected_base_color.b, 1e-5f);
    EXPECT_NEAR(renderer->base_color().w(), 1.0f, 1e-6f);

    const auto& radiance = pbpt_light->area_emitter().radiance_spectrum;
    ASSERT_EQ(radiance.size(), 4u);
    EXPECT_FLOAT_EQ(radiance[1].lambda_nm, 500.0f);
    EXPECT_FLOAT_EQ(radiance[1].value, 8.0f);

    scene.scene_graph().update_world_transforms();
    const auto world_pos = mesh_go->node().world_position();
    EXPECT_NEAR(world_pos.x(), 1.0f, 1e-5f);
    EXPECT_NEAR(world_pos.y(), 2.0f, 1e-5f);
    EXPECT_NEAR(world_pos.z(), 3.0f, 1e-5f);

    ASSERT_NE(scene.active_camera(), nullptr);
}

TEST(FrameworkPbptSceneImporterTest, ImportsRgbReflectanceAndMapsToBaseColor) {
    TempDir temp_dir("rtr_pbpt_scene_importer_rgb_reflectance_test");

    const auto mesh_path = temp_dir.path / "meshes" / "tri.obj";
    write_text_file(mesh_path, "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");

    const auto xml_path = temp_dir.path / "scene_rgb.xml";
    write_text_file(
        xml_path,
        R"XML(<?xml version="1.0" encoding="utf-8"?>
<scene version="0.4.0">
  <bsdf type="diffuse" id="mat_rgb">
    <rgb name="reflectance" value="0.2 0.4 0.6"/>
  </bsdf>
  <shape type="obj" id="mesh_rgb">
    <string name="filename" value="meshes/tri.obj"/>
    <ref id="mat_rgb"/>
  </shape>
</scene>)XML"
    );

    core::Scene scene(1, "scene");
    resource::ResourceManager resources(2, temp_dir.path);
    const auto result = import_pbpt_scene_xml_to_scene(
        make_location(temp_dir.path, xml_path),
        scene,
        resources
    );
    EXPECT_EQ(result.imported_shape_count, 1u);

    const auto* mesh_go = find_mesh_object(scene);
    ASSERT_NE(mesh_go, nullptr);
    const auto* renderer = mesh_go->get_component<component::MeshRenderer>();
    const auto* pbpt_mesh = mesh_go->get_component<component::PbptMesh>();
    ASSERT_NE(renderer, nullptr);
    ASSERT_NE(pbpt_mesh, nullptr);
    EXPECT_NEAR(renderer->base_color().x(), 0.2f, 1e-6f);
    EXPECT_NEAR(renderer->base_color().y(), 0.4f, 1e-6f);
    EXPECT_NEAR(renderer->base_color().z(), 0.6f, 1e-6f);
    EXPECT_NEAR(renderer->base_color().w(), 1.0f, 1e-6f);
}

TEST(FrameworkPbptSceneImporterTest, ThrowsForInvalidMatrixElementCount) {
    TempDir temp_dir("rtr_pbpt_scene_importer_invalid_matrix_test");

    const auto mesh_path = temp_dir.path / "meshes" / "tri.obj";
    write_text_file(mesh_path, "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");

    const auto xml_path = temp_dir.path / "scene_invalid_matrix.xml";
    write_text_file(
        xml_path,
        R"XML(<?xml version="1.0" encoding="utf-8"?>
<scene version="0.4.0">
  <bsdf type="diffuse" id="mat_white">
    <spectrum name="reflectance" value="400:0.7, 500:0.7, 600:0.7, 700:0.7"/>
  </bsdf>
  <shape type="obj">
    <string name="filename" value="meshes/tri.obj"/>
    <transform name="toWorld">
      <matrix value="1,0,0,1"/>
    </transform>
    <ref id="mat_white"/>
  </shape>
</scene>)XML"
    );

    core::Scene scene(1, "scene");
    resource::ResourceManager resources(2, temp_dir.path);
    EXPECT_THROW(
        (void)import_pbpt_scene_xml_to_scene(make_location(temp_dir.path, xml_path), scene, resources),
        std::runtime_error
    );
}

TEST(FrameworkPbptSceneImporterTest, ThrowsForDuplicateImportedNameBetweenCameraAndShape) {
    TempDir temp_dir("rtr_pbpt_scene_importer_duplicate_name_test");

    const auto mesh_path = temp_dir.path / "meshes" / "tri.obj";
    write_text_file(mesh_path, "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");

    const auto xml_path = temp_dir.path / "scene_duplicate_name.xml";
    write_text_file(
        xml_path,
        R"XML(<?xml version="1.0" encoding="utf-8"?>
<scene version="0.4.0">
  <sensor type="perspective">
    <float name="fov" value="45"/>
    <film type="hdrfilm">
      <integer name="width" value="64"/>
      <integer name="height" value="64"/>
    </film>
  </sensor>
  <bsdf type="diffuse" id="mat_white">
    <spectrum name="reflectance" value="400:0.7, 500:0.7, 600:0.7, 700:0.7"/>
  </bsdf>
  <shape type="obj" id="pbpt_camera">
    <string name="filename" value="meshes/tri.obj"/>
    <ref id="mat_white"/>
  </shape>
</scene>)XML"
    );

    core::Scene scene(1, "scene");
    resource::ResourceManager resources(2, temp_dir.path);
    EXPECT_THROW(
        (void)import_pbpt_scene_xml_to_scene(make_location(temp_dir.path, xml_path), scene, resources),
        std::runtime_error
    );
}

TEST(FrameworkPbptSceneImporterTest, RecordsDefaultShapeNameWhenShapeIdMissing) {
    TempDir temp_dir("rtr_pbpt_scene_importer_default_name_test");

    const auto mesh_path = temp_dir.path / "meshes" / "tri_default.obj";
    write_text_file(mesh_path, "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");

    const auto xml_path = temp_dir.path / "scene_default_name.xml";
    write_text_file(
        xml_path,
        R"XML(<?xml version="1.0" encoding="utf-8"?>
<scene version="0.4.0">
  <bsdf type="diffuse" id="mat_white">
    <spectrum name="reflectance" value="400:0.7, 500:0.7, 600:0.7, 700:0.7"/>
  </bsdf>
  <shape type="obj">
    <string name="filename" value="meshes/tri_default.obj"/>
    <ref id="mat_white"/>
  </shape>
</scene>)XML"
    );

    core::Scene scene(1, "scene");
    resource::ResourceManager resources(2, temp_dir.path);
    const auto result = import_pbpt_scene_xml_to_scene(
        make_location(temp_dir.path, xml_path),
        scene,
        resources
    );

    EXPECT_EQ(result.imported_shape_count, 1u);
    ASSERT_TRUE(result.imported_game_object_id_by_name.contains("tri_default"));
    const auto imported_id = result.imported_game_object_id_by_name.at("tri_default");
    const auto* imported_go = scene.find_game_object(imported_id);
    ASSERT_NE(imported_go, nullptr);
    EXPECT_EQ(imported_go->name(), "tri_default");
}

TEST(FrameworkPbptSceneImporterTest, LookAtSensorAlignsWithRtrCameraFrontConvention) {
    TempDir temp_dir("rtr_pbpt_scene_importer_lookat_camera_test");

    const auto xml_path = temp_dir.path / "scene_lookat.xml";
    write_text_file(
        xml_path,
        R"XML(<?xml version="1.0" encoding="utf-8"?>
<scene version="0.4.0">
  <sensor type="perspective">
    <transform name="toWorld">
      <lookAt origin="0, 0, 0" target="0, 0, 1" up="0, 1, 0"/>
    </transform>
    <float name="fov" value="45"/>
    <film type="hdrfilm">
      <integer name="width" value="64"/>
      <integer name="height" value="64"/>
    </film>
  </sensor>
</scene>)XML"
    );

    core::Scene scene(1, "scene");
    resource::ResourceManager resources(2, temp_dir.path);
    (void)import_pbpt_scene_xml_to_scene(make_location(temp_dir.path, xml_path), scene, resources);

    const auto* camera = dynamic_cast<const core::PerspectiveCamera*>(scene.active_camera());
    ASSERT_NE(camera, nullptr);

    const pbpt::math::vec3 front = camera->front();
    EXPECT_NEAR(front.x(), 0.0f, 1e-5f);
    EXPECT_NEAR(front.y(), 0.0f, 1e-5f);
    EXPECT_NEAR(front.z(), 1.0f, 1e-5f);
}

TEST(FrameworkPbptSceneImporterTest, AttachesFreeLookControllerWhenInputStateProvided) {
    TempDir temp_dir("rtr_pbpt_scene_importer_freelook_test");

    const auto xml_path = temp_dir.path / "scene_with_sensor.xml";
    write_text_file(
        xml_path,
        R"XML(<?xml version="1.0" encoding="utf-8"?>
<scene version="0.4.0">
  <sensor type="perspective">
    <transform name="toWorld">
      <lookAt origin="0, 0, 0" target="0, 0, 1" up="0, 1, 0"/>
    </transform>
    <float name="fov" value="45"/>
    <film type="hdrfilm">
      <integer name="width" value="64"/>
      <integer name="height" value="64"/>
    </film>
  </sensor>
</scene>)XML"
    );

    core::Scene scene(1, "scene");
    system::input::InputState input_state{};
    resource::ResourceManager resources(2, temp_dir.path);
    PbptImportOptions options{};
    options.free_look_input_state = &input_state;
    (void)import_pbpt_scene_xml_to_scene(
        make_location(temp_dir.path, xml_path),
        scene,
        resources,
        options
    );

    const auto active_camera_owner = scene.camera_manager().active_camera_owner_id();
    ASSERT_NE(active_camera_owner, core::kInvalidGameObjectId);
    const auto* active_camera_go = scene.find_game_object(active_camera_owner);
    ASSERT_NE(active_camera_go, nullptr);
    EXPECT_NE(
        active_camera_go->get_component<component::FreeLookCameraController>(),
        nullptr
    );
}

TEST(FrameworkPbptSceneImporterTest, RelativeMeshFilenameResolvesFromXmlDirectoryWithinRoot) {
    TempDir temp_dir("rtr_pbpt_scene_importer_xml_dir_resolve_test");
    const auto resource_root = temp_dir.path / "assets";
    const auto scene_dir = resource_root / "pbpt_scene" / "cbox";
    const auto mesh_path = scene_dir / "meshes" / "tri.obj";
    const auto xml_path = scene_dir / "scene.xml";

    write_text_file(mesh_path, "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");
    write_text_file(
        xml_path,
        R"XML(<?xml version="1.0" encoding="utf-8"?>
<scene version="0.4.0">
  <bsdf type="diffuse" id="mat_white">
    <spectrum name="reflectance" value="400:0.7, 500:0.7, 600:0.7, 700:0.7"/>
  </bsdf>
  <shape type="obj" id="mesh_a">
    <string name="filename" value="meshes/tri.obj"/>
    <ref id="mat_white"/>
  </shape>
</scene>)XML"
    );

    core::Scene scene(1, "scene");
    resource::ResourceManager resources(2, resource_root);
    const auto result = import_pbpt_scene_xml_to_scene(
        make_location(resource_root, xml_path),
        scene,
        resources
    );
    EXPECT_EQ(result.imported_shape_count, 1u);
}

TEST(FrameworkPbptSceneImporterTest, AllowsRelativeSceneRootEscapingResourceRoot) {
    TempDir temp_dir("rtr_pbpt_scene_importer_outside_root_test");
    const auto resource_root = temp_dir.path / "assets";
    std::filesystem::create_directories(resource_root);
    const auto xml_path = temp_dir.path / "outside_scene" / "scene.xml";
    const auto mesh_path = temp_dir.path / "outside_scene" / "meshes" / "tri.obj";
    write_text_file(mesh_path, "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");
    write_text_file(
        xml_path,
        R"XML(<?xml version="1.0" encoding="utf-8"?>
<scene version="0.4.0">
  <bsdf type="diffuse" id="mat_white">
    <spectrum name="reflectance" value="400:0.7, 500:0.7, 600:0.7, 700:0.7"/>
  </bsdf>
  <shape type="obj">
    <string name="filename" value="meshes/tri.obj"/>
    <ref id="mat_white"/>
  </shape>
</scene>)XML"
    );

    core::Scene scene(1, "scene");
    resource::ResourceManager resources(2, resource_root);
    const auto result = import_pbpt_scene_xml_to_scene(
        make_pbpt_scene_location("../outside_scene", "scene.xml"),
        scene,
        resources
    );
    EXPECT_EQ(result.imported_shape_count, 1u);
}

TEST(FrameworkPbptSceneImporterTest, ThrowsWhenXmlFilenameContainsPathSeparator) {
    TempDir temp_dir("rtr_pbpt_scene_importer_bad_xml_filename_test");
    const auto resource_root = temp_dir.path / "assets";
    resource::ResourceManager resources(2, resource_root);
    core::Scene scene(1, "scene");

    EXPECT_THROW(
        (void)import_pbpt_scene_xml_to_scene(
            make_pbpt_scene_location("pbpt_scene/cbox", "nested/scene.xml"),
            scene,
            resources
        ),
        std::invalid_argument
    );
}

TEST(FrameworkPbptSceneImporterTest, ThrowsWhenMeshFilenameIsNotUnderMeshesDirectory) {
    TempDir temp_dir("rtr_pbpt_scene_importer_bad_mesh_dir_test");
    const auto resource_root = temp_dir.path / "assets";
    const auto scene_dir = resource_root / "pbpt_scene" / "cbox";
    const auto xml_path = scene_dir / "scene.xml";
    write_text_file(scene_dir / "models" / "tri.obj", "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");
    write_text_file(
        xml_path,
        R"XML(<?xml version="1.0" encoding="utf-8"?>
<scene version="0.4.0">
  <bsdf type="diffuse" id="mat_white">
    <spectrum name="reflectance" value="400:0.7, 500:0.7, 600:0.7, 700:0.7"/>
  </bsdf>
  <shape type="obj">
    <string name="filename" value="models/tri.obj"/>
    <ref id="mat_white"/>
  </shape>
</scene>)XML"
    );

    core::Scene scene(1, "scene");
    resource::ResourceManager resources(2, resource_root);
    EXPECT_THROW(
        (void)import_pbpt_scene_xml_to_scene(
            make_location(resource_root, xml_path),
            scene,
            resources
        ),
        std::runtime_error
    );
}

TEST(FrameworkPbptSceneImporterTest, ThrowsWhenMeshFilenameUsesParentTraversal) {
    TempDir temp_dir("rtr_pbpt_scene_importer_escape_root_test");
    const auto resource_root = temp_dir.path / "assets";
    const auto scene_dir = resource_root / "pbpt_scene" / "cbox";
    const auto xml_path = scene_dir / "scene.xml";

    write_text_file(
        xml_path,
        R"XML(<?xml version="1.0" encoding="utf-8"?>
<scene version="0.4.0">
  <bsdf type="diffuse" id="mat_white">
    <spectrum name="reflectance" value="400:0.7, 500:0.7, 600:0.7, 700:0.7"/>
  </bsdf>
  <shape type="obj">
    <string name="filename" value="../../../outside/tri.obj"/>
    <ref id="mat_white"/>
  </shape>
</scene>)XML"
    );

    core::Scene scene(1, "scene");
    resource::ResourceManager resources(2, resource_root);
    EXPECT_THROW(
        (void)import_pbpt_scene_xml_to_scene(
            make_location(resource_root, xml_path),
            scene,
            resources
        ),
        std::runtime_error
    );
}

TEST(FrameworkPbptSceneImporterTest, ThrowsWhenMeshFilenameIsAbsolutePath) {
    TempDir temp_dir("rtr_pbpt_scene_importer_abs_default_test");
    const auto mesh_path = (temp_dir.path / "meshes" / "tri_abs.obj");
    write_text_file(mesh_path, "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");

    const auto xml_path = temp_dir.path / "scene_abs_default.xml";
    const std::string xml =
        std::string("<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
                    "<scene version=\"0.4.0\">\n"
                    "  <bsdf type=\"diffuse\" id=\"mat_white\">\n"
                    "    <spectrum name=\"reflectance\" value=\"400:0.7, 500:0.7, 600:0.7, 700:0.7\"/>\n"
                    "  </bsdf>\n"
                    "  <shape type=\"obj\">\n"
                    "    <string name=\"filename\" value=\"") +
        mesh_path.string() +
        "\"/>\n"
        "    <ref id=\"mat_white\"/>\n"
        "  </shape>\n"
        "</scene>\n";
    write_text_file(xml_path, xml);

    core::Scene scene(1, "scene");
    resource::ResourceManager resources(2, temp_dir.path);
    EXPECT_THROW(
        (void)import_pbpt_scene_xml_to_scene(
            make_location(temp_dir.path, xml_path),
            scene,
            resources
        ),
        std::runtime_error
    );
}

} // namespace rtr::framework::integration::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
